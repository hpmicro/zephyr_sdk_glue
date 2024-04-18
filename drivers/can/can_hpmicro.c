/*
 * Copyright (c) 2022-2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-04-18   HPMicro         Adapt Zephyr 3.4.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>

#include "hpm_common.h"
#include "hpm_can_drv.h"
#include "hpm_clock_drv.h"

#define HPM_CAN_FILTER_NUM_MAX (16U)
#define HPM_CAN_BITRATE_MAX (1000000UL) /* 1Mbps */
#define HPM_CAN_FD_BITRATE_MAX (8000000UL) /* 8Mbps */
/* 0 - primary buffer, 1 - secondary buffer */
#define HPM_CAN_NUM_TX_BUF_ELEMENTS (2U)

/* Default baudrate: 1Mbps @80MHz CAN clock */
#define DEFAULT_HPM_CAN_CONFIG {         \
    .use_lowlevel_timing_setting = true, \
    .can_timing = {1, 60, 20, 16},       \
    .mode = can_mode_normal,  \
}


struct hpm_can_config {
    CAN_Type *base;
    clock_name_t clock_name;
    clock_source_t clock_src;
    uint32_t clock_div;
    void (*irq_config_func)(const struct device *dev);
    const struct pinctrl_dev_config *pincfg;
};

struct hpm_can_data {
    /* CAN configuration */
    can_config_t config;
    uint32_t can_filter_count;
    can_filter_config_t filter_list[HPM_CAN_FILTER_NUM_MAX];
    uint32_t filter_rtr;
    uint32_t filter_rtr_mask;

    /* RTOS misc. */
    struct k_mutex inst_mutex;
    struct k_mutex tx_mutex;
    enum can_state state;
    struct k_sem tx_sem;
    struct k_sem tx_fin_sem[HPM_CAN_NUM_TX_BUF_ELEMENTS];

    /* TX callback */
    can_tx_callback_t tx_fin_cb[HPM_CAN_NUM_TX_BUF_ELEMENTS];
    void *tx_fin_cb_arg[HPM_CAN_NUM_TX_BUF_ELEMENTS];

    /* RX callback */
    can_rx_callback_t rx_cb[HPM_CAN_FILTER_NUM_MAX];
    void *rx_cb_arg[HPM_CAN_FILTER_NUM_MAX];

    /* State change callback */
    can_state_change_callback_t state_change_cb;
    void *state_change_cb_data;
    bool started;
};

#define DT_DRV_COMPAT hpmicro_hpm_can
LOG_MODULE_REGISTER(hpmicro_hpm_can, CONFIG_CAN_LOG_LEVEL);

static uint32_t hpm_can_get_first_filter_index(const can_receive_buf_t *buf,
                                               const struct hpm_can_data *data);
static int hpm_can_get_state(const struct device *dev,
                             enum can_state *state,
                             struct can_bus_err_cnt *err_cnt);

/*
 * CAST CAN doesn't support reporting CAN filter index, this function mannually look up the filter index
 */
uint32_t hpm_can_get_first_filter_index(const can_receive_buf_t *buf,
                                        const struct hpm_can_data *data)
{
    uint32_t filter_index = -EINVAL;
    for (uint32_t i = 0; i < data->can_filter_count; i++) {
        const can_filter_config_t *filter = &data->filter_list[i];

        /* NOTE: if corresponding bit field is 1, it means "don't care" */
        if (((buf->extend_id == 1) && (filter->id_mode == can_filter_id_mode_extended_frames)) ||
            ((buf->extend_id == 0) && (filter->id_mode == can_filter_id_mode_standard_frames))) {
            if ((buf->id | filter->mask) == (filter->code | filter->mask)) {
                filter_index = i;
                break;
            }
        }
    }
    return filter_index;
}

/* Get CAN message from CAN device */
static void hpm_can_get_message(const struct device *dev)
{
    const struct hpm_can_config *cfg = dev->config;
    CAN_Type *can = cfg->base;
    struct hpm_can_data *data = dev->data;
    can_rx_callback_t cb;
    struct can_frame frame;
    void *cb_arg;
    can_receive_buf_t rx_buf;
    can_read_received_message(can, &rx_buf);

    uint32_t filter_index = hpm_can_get_first_filter_index(&rx_buf, data);
    frame.id = rx_buf.id;
    frame.flags = 0;
    if (rx_buf.canfd_frame) {
        frame.flags |= CAN_FRAME_FDF;
    }
    if (rx_buf.remote_frame) {
        frame.flags |= CAN_FRAME_RTR;
    }
    if (rx_buf.extend_id) {
        frame.flags |= CAN_FRAME_IDE;
    }
    if (rx_buf.bitrate_switch) {
        frame.flags |= CAN_FRAME_BRS;
    }
    frame.dlc = rx_buf.dlc;
    LOG_DBG("Frame on filter %d, ID: 0x%x", filter_index, frame.id);
    size_t data_len = can_dlc_to_bytes(frame.dlc);
    for (uint32_t i = 0; i < data_len; i++) {
        frame.data[i] = rx_buf.data[i];
    }

    /* Handle RX filter callback */
    if (filter_index != -EINVAL) {
        /* If RTR bit does not match filter RTR mask and bit, drop current frame */
        bool rtr_filter_mask = (data->filter_rtr_mask & BIT(filter_index)) != 0;
        bool rtr_filter = (data->filter_rtr & BIT(filter_index)) != 0;
        bool rtr_in_frame = (frame.flags && CAN_FRAME_RTR);
        if (rtr_filter_mask & (rtr_filter != rtr_in_frame)) {
            return;
        }

        cb = data->rx_cb[filter_index];
        cb_arg = data->rx_cb_arg[filter_index];
        if (cb != NULL) {
            cb(dev, &frame, cb_arg);
        } else {
            LOG_DBG("cb missing");
        }
    }
}

/*  */
static void hpm_can_state_change_handler(const struct device *dev)
{
    struct hpm_can_data *data = dev->data;
    const can_state_change_callback_t cb = data->state_change_cb;
    void *cb_data = data->state_change_cb_data;
    struct can_bus_err_cnt err_cnt;
    enum can_state state;

    (void) hpm_can_get_state(dev, &state, &err_cnt);

    if (cb != NULL) {
        cb(dev, state, err_cnt, cb_data);
    }
}


static void hpm_can_tc_event_handler(const struct device *dev, uint32_t index)
{
    struct hpm_can_data *data = dev->data;
    can_tx_callback_t tx_cb;
    k_sem_give(&data->tx_sem);
    tx_cb = data->tx_fin_cb[index];
    if (tx_cb  == NULL) {
        k_sem_give(&data->tx_fin_sem[index]);
    } else {
        tx_cb(dev, 0, data->tx_fin_cb_arg[index]);
    }
}

static void hpm_can_isr(const struct device *dev)
{
    const struct hpm_can_config *cfg = dev->config;
    CAN_Type *can = cfg->base;

    uint8_t tx_rx_flags = can_get_tx_rx_flags(can);
    uint8_t error_flags = can_get_error_interrupt_flags(can);

    /* Handle TX/RX flags */
    if ((tx_rx_flags & CAN_EVENT_RECEIVE) != 0U) {

        while(can_is_data_available_in_receive_buffer(can)) {
            hpm_can_get_message(dev);
        }
        can_clear_tx_rx_flags(can, CAN_EVENT_RECEIVE);
    }

    if ((tx_rx_flags & CAN_EVENT_TX_PRIMARY_BUF) != 0) {
        hpm_can_tc_event_handler(dev, 0);
        can_clear_tx_rx_flags(can, CAN_EVENT_TX_PRIMARY_BUF);
    }

    if ((tx_rx_flags & CAN_EVENT_TX_SECONDARY_BUF) != 0) {
        hpm_can_tc_event_handler(dev, 1);
        can_clear_tx_rx_flags(can, CAN_EVENT_TX_SECONDARY_BUF);
    }

    /* Handle error flags */
    if ((error_flags & (CAN_ERROR_WARNING_LIMIT_FLAG | CAN_ERROR_PASSIVE_MODE_ACTIVE_FLAG | CAN_ERROR_BUS_ERROR_INT_FLAG)) != 0U) {
        hpm_can_state_change_handler(dev);
        can_clear_error_interrupt_flags(can, error_flags);
    }
}

static int hpm_can_init(const struct device *dev)
{
    int ret = 0;

    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    can_config_t *config = &data->config;
    CAN_Type *can = cfg->base;

    k_mutex_init(&data->inst_mutex);
    k_mutex_init(&data->tx_mutex);
    k_sem_init(&data->tx_sem, HPM_CAN_NUM_TX_BUF_ELEMENTS, HPM_CAN_NUM_TX_BUF_ELEMENTS);

    data->can_filter_count = 0;
    for (uint32_t i= 0; i < ARRAY_SIZE(data->tx_fin_sem); i++) {
        k_sem_init(&data->tx_fin_sem[i], 1, 1);
    }

    data->filter_rtr = 0;
    data->filter_rtr_mask = 0;

    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("CAN pinctrl setup failed (%d)", ret);
        return ret;
    }

    clock_set_source_divider(cfg->clock_name, cfg->clock_src, cfg->clock_div);
#ifdef CONFIG_CAN_FD_MODE
    config->enable_canfd = true;
#endif

    /* Set Interrupt Enable Mask */
    config->irq_txrx_enable_mask = CAN_EVENT_RECEIVE | CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF | CAN_EVENT_ERROR;
    config->irq_error_enable_mask = CAN_ERROR_BUS_ERROR_INT_ENABLE | CAN_ERROR_ARBITRAITION_LOST_INT_ENABLE | CAN_ERROR_PASSIVE_INT_ENABLE;

    /* Set CAN filter list */
    config->filter_list_num = data->can_filter_count;
    config->filter_list = data->filter_list;

    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    hpm_stat_t status = can_init(can, config, can_clk_freq);
    if (status != status_success) {
        ret = -EAGAIN;
     } else {
        cfg->irq_config_func(dev);
    }

    return ret;
}


static int hpm_can_set_mode(const struct device *dev, can_mode_t mode)
{
    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    CAN_Type *can = cfg->base;

    can_config_t *can_config = &data->config;
#ifdef CONFIG_CAN_FD_MODE
    if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_FD)) != 0) {
        LOG_ERR("unsupported mode: 0x%08x", mode);
        return -ENOTSUP;
    }
#else
    if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY)) != 0) {
        LOG_ERR("unsupported mode: 0x%08x", mode);
        return -ENOTSUP;
    }
#endif /* CONFIG_CAN_FD_MODE */

    if ((mode & CAN_MODE_LOOPBACK) != 0) {
        can_config->mode = can_mode_loopback_internal;
        can_set_node_mode(can, can_mode_loopback_internal);
    }
    else if ((mode & CAN_MODE_LISTENONLY) != 0) {
        can_config->mode = can_mode_listen_only;
        can_set_node_mode(can, can_mode_listen_only);
    } else {
        can_config->mode = can_mode_normal;
        can_set_node_mode(can, can_mode_normal);
    }

    return 0;
}


/**
 * @brief Set bit timing for CAN 2.0B/ CAN-FD norminal
 */
static int hpm_can_set_timing(const struct device *dev, const struct can_timing *timing)
{
    int ret = 0;

    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    CAN_Type *can = cfg->base;

    can_config_t *config = &data->config;
    config->use_lowlevel_timing_setting = true;
    can_bit_timing_param_t *timing_param = &config->can_timing;
    timing_param->prescaler = timing->prescaler;
    /* num_seg1 in CAST_CAN = Tsync_seq + Tprop_seg + Tphase_seg1  */
    timing_param->num_seg1 = 1 + timing->prop_seg + timing->phase_seg1;
    timing_param->num_seg2 = timing->phase_seg2;
    timing_param->num_sjw = timing->sjw;

    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    hpm_stat_t status = can_init(can, config, can_clk_freq);
    if (status != status_success) {
        ret = -EAGAIN;
    }

    return ret;
}


void convert_can_frame_to_can_frame(const struct can_frame *frame,
                                     can_transmit_buf_t *tx_buf)
{
    tx_buf->buffer[0] = 0;
    tx_buf->buffer[1] = 0;

    tx_buf->id = frame->id;
    tx_buf->dlc = frame->dlc;
    tx_buf->bitrate_switch = (uint16_t)((frame->flags & CAN_FRAME_BRS) != 0);
    tx_buf->remote_frame = (uint16_t)((frame->flags & CAN_FRAME_RTR) != 0);
    tx_buf->canfd_frame = (uint16_t)((frame->flags & CAN_FRAME_FDF) != 0);
    tx_buf->extend_id = (uint16_t)((frame->flags & CAN_FRAME_IDE) != 0);

    size_t msg_len = can_dlc_to_bytes(tx_buf->dlc);

    for (size_t i = 0; i < msg_len; i++) {
        tx_buf->data[i] = frame->data[i];
    }
}


static int hpm_can_send(const struct device *dev,
                        const struct can_frame *frame,
                        k_timeout_t timeout,
                        can_tx_callback_t callback,
                        void *user_data)
{
    int ret;
    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    CAN_Type *can = cfg->base;
    enum can_state state;
    hpm_stat_t status;

    if (frame->dlc > CAN_MAX_DLC) {
        LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
        return -EINVAL;
    }

    (void) hpm_can_get_state(dev, &state, NULL);
    if (state == CAN_STATE_BUS_OFF) {
        LOG_DBG("Transmit failed, bus-off");
        return -ENETUNREACH;
    }

    can_transmit_buf_t tx_buf;
    convert_can_frame_to_can_frame(frame, &tx_buf);

    ret = k_sem_take(&data->tx_sem, timeout);
    if (ret != 0) {
        return -EAGAIN;
    }

    uint32_t fifo_idx = 0;
    if (!can_is_primary_transmit_buffer_full(can)) {
        fifo_idx = 0;
    } else {
        fifo_idx = 1;
    }

    k_mutex_lock(&data->tx_mutex, timeout);

    data->tx_fin_cb[fifo_idx] = callback;
    data->tx_fin_cb_arg[fifo_idx] = user_data;
    if (fifo_idx == 0) {
        status = can_send_high_priority_message_nonblocking(can, &tx_buf);
    } else {
        status = can_send_message_nonblocking(can, &tx_buf);
    }
    k_mutex_unlock(&data->tx_mutex);
    if (status != 0) {
        return -EIO;
    }

    if (callback == NULL) {
        LOG_DBG("Waiting for TX complete");
        k_sem_take(&data->tx_fin_sem[fifo_idx], K_FOREVER);
    }

    if (status != status_success) {
        return -EIO;
    }

    return 0;
}

/*
 *  NOTE:
 *  CAN_CTRL doesn't support change the CAN filter at runtime
 *  Software need to re-initialize the CAN controller if the changes to filter needs to be made.
 */
static int hpm_can_add_rx_filter(const struct device *dev,
                            can_rx_callback_t callback,
                            void *user_data,
                            const struct can_filter *filter)
 {
    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    CAN_Type *can = cfg->base;
    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    int filter_id;

    if (callback == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->inst_mutex, K_FOREVER);
    if (data->can_filter_count < HPM_CAN_FILTER_NUM_MAX) {
        filter_id = data->can_filter_count;
        data->can_filter_count++;
    } else {
        return -ENOSPC;
    }

    can_config_t *config = &data->config;
    can_filter_config_t *can_filter = &data->filter_list[filter_id];
    can_filter->index = filter_id;
    can_filter->id_mode = ((filter->flags & CAN_FILTER_IDE) != 0) ? can_filter_id_mode_extended_frames:
                          can_filter_id_mode_standard_frames;
    can_filter->enable = true;
    can_filter->code = filter->id;
    /* NOTE: the filter mask definition in this CAN IP is different from standard definition */
    can_filter->mask = ~filter->mask;

    config->filter_list_num = data->can_filter_count;
    config->filter_list = data->filter_list;

    if ((filter->flags & CAN_FILTER_RTR) != 0) {
		data->filter_rtr |= (1U << filter_id);
	} else {
		data->filter_rtr &= ~(1U << filter_id);
	}

	if ((filter->flags & CAN_FILTER_RTR) != 0) {
		data->filter_rtr_mask |= (1U << filter_id);
	} else {
		data->filter_rtr_mask &= ~(1U << filter_id);
	}

    (void) can_init(can, config, can_clk_freq);

    data->rx_cb[filter_id] = callback;
    data->rx_cb_arg[filter_id] = user_data;

    k_mutex_unlock(&data->inst_mutex);

    return filter_id;
}


/*
 *  NOTE:
 *  CAN_CTRL doesn't support change the CAN filter at runtime
 *  Software need to re-initialize the CAN controller if the changes to filter needs to be made.
 */
static void hpm_can_remove_rx_filter(const struct device *dev, int filter_id)
{
    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    CAN_Type *can = cfg->base;

    can_config_t *config = &data->config;
    k_mutex_lock(&data->inst_mutex, K_FOREVER);
    if (data->can_filter_count > 0) {
        can_filter_config_t *last_filter = &data->filter_list[data->can_filter_count - 1];
        for (uint32_t i = 0; i < data->can_filter_count; i++) {
            if (data->filter_list[i].index == filter_id) {

                /* Move the last filter to current filter location */
                if (i != data->can_filter_count - 1U) {
                    can_filter_config_t *current_filter = &data->filter_list[i];
                    current_filter->index = last_filter->index;
                    current_filter->id_mode = last_filter->id_mode;
                    current_filter->enable = last_filter->enable;
                    current_filter->code = last_filter->code;
                    current_filter->mask = last_filter->mask;
                }
                /* Remove the last filter */
                data->can_filter_count -= 1U;

                uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
                config->filter_list_num = data->can_filter_count;
                config->filter_list = data->filter_list;
                (void) can_init(can, config, can_clk_freq);
            }
        }
    }
    k_mutex_unlock(&data->inst_mutex);
}


static int hpm_can_get_state(const struct device *dev,
                             enum can_state *state,
                             struct can_bus_err_cnt *err_cnt)
{
     int ret = 0;

    const struct hpm_can_config *cfg = dev->config;
    CAN_Type *can = cfg->base;

    uint8_t error_flags = can_get_error_interrupt_flags(can);

    if (can_is_in_bus_off_mode(can)) {
        *state = CAN_STATE_BUS_OFF;
    } else if ((error_flags & CAN_ERROR_WARNING_LIMIT_FLAG) != 0U) {
        *state = CAN_STATE_ERROR_WARNING;
    } else if ((error_flags & CAN_ERROR_PASSIVE_MODE_ACTIVE_FLAG) != 0U) {
        *state = CAN_STATE_ERROR_PASSIVE;
    } else {
        *state = CAN_STATE_ERROR_ACTIVE;
    }

    if (err_cnt != NULL) {
        err_cnt->tx_err_cnt = can_get_transmit_error_count(can);
        err_cnt->rx_err_cnt = can_get_receive_error_count(can);
    }

    return ret;
}

#if !defined(CONFIG_CAN_AUTO_BUS_OFF_RECOVERY) || defined(__DOXYGEN__)
int hpm_can_recover(const struct device *dev, k_timeout_t timeout)
{
    return -ENOTSUP;
}
#endif


static void hpm_can_set_state_change_callback(const struct device *dev,
                                              can_state_change_callback_t callback,
                                              void *user_data)
{
    struct hpm_can_data *data = dev->data;

    data->state_change_cb = callback;
    data->state_change_cb_data = user_data;

}

static int hpm_can_get_core_clock(const struct device *dev, uint32_t *rate)
{
    const struct hpm_can_config *cfg = dev->config;
    if (rate != NULL) {
        *rate = clock_get_frequency(cfg->clock_name);
    }
    return 0;
}


static int hpm_can_get_max_filters(const struct device *dev, bool ide)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(ide);
    return HPM_CAN_FILTER_NUM_MAX;
}

static int hpm_can_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
    ARG_UNUSED(dev);
    *max_bitrate = HPM_CAN_BITRATE_MAX;

    return 0;
}

#if CONFIG_CAN_FD_MODE
static int hpm_can_set_tming_data(const struct device *dev, const struct can_timing *timing)
{
    int ret = 0;

    const struct hpm_can_config *cfg = dev->config;
    struct hpm_can_data *data = dev->data;
    can_config_t *config = &data->config;
    CAN_Type *can = cfg->base;

    config->use_lowlevel_timing_setting = true;
    can_bit_timing_param_t *timing_param = &config->canfd_timing;
    timing_param->prescaler = timing->prescaler;
    /* num_seg1 in CAST_CAN = Tsync_seq + Tprop_seg + Tphase_seg1  */
    timing_param->num_seg1 = 1 + timing->prop_seg + timing->phase_seg1;
    timing_param->num_seg2 = timing->phase_seg2;
    timing_param->num_sjw = timing->sjw;

    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    hpm_stat_t status = can_init(can, config, can_clk_freq);

    if (status != status_success) {
        ret = -EAGAIN;
    }

    return ret;
}
#endif

static int hpm_can_start(const struct device *dev)
{
    const struct hpm_can_config *config = dev->config;
    struct hpm_can_data *data = dev->data;
    if (data->started) {
        return -EALREADY;
    }

    data->started = true;

    return 0;
}

static int hpm_can_stop(const struct device *dev)
{
    const struct hpm_can_config *config = dev->config;
    struct hpm_can_data *data = dev->data;

    if (!data->started) {
		return -EALREADY;
	}

    can_deinit(config->base);

    data->started = false;

    return 0;
}

static const struct can_driver_api hpm_can_driver_api = {
    .set_mode = hpm_can_set_mode,
    .set_timing = hpm_can_set_timing,
    .send = hpm_can_send,
    .start = hpm_can_start,
    .stop = hpm_can_stop,
    .add_rx_filter = hpm_can_add_rx_filter,
    .remove_rx_filter = hpm_can_remove_rx_filter,
    .get_state = hpm_can_get_state,
    .set_state_change_callback = hpm_can_set_state_change_callback,
#if !defined(CONFIG_CAN_AUTO_BUS_OFF_RECOVERY) || defined(__DOXYGEN__)
    .recover = hpm_can_recover,
#endif
    .get_core_clock = hpm_can_get_core_clock,
    .get_max_filters = hpm_can_get_max_filters,
    .get_max_bitrate = hpm_can_get_max_bitrate,

    .timing_min = {
        .sjw = 1,
        .prop_seg = 1,
        .phase_seg1 = 1,
        .phase_seg2 = 2,
        .prescaler = 1,
    },
    .timing_max = {
        .sjw = 16,
        .prop_seg = 8,
        .phase_seg1 = 56,
        .phase_seg2 = 32,
        .prescaler = 256,
    },

#if CONFIG_CAN_FD_MODE
    .set_timing_data = hpm_can_set_timing_data,

    .timing_data_min = {
        .sjw = 1,
        .prop_seg = 0,
        .phase_seg1 = 1,
        .phase_seg2 = 2,
        .prescaler = 1,

    },
    .timing_data_max = {
        .sjw = 8,
        .prop_seg = 8,
        .phase_seg1 = 8,
        .phase_seg2 = 8,
        .sjw = 8,
        .prescaler = 256,
    }
#endif
};

#define HPM_CAN_INIT(n)                     \
\
    PINCTRL_DT_INST_DEFINE(n); \
        static void hpm_can_irq_config_func##n(const struct device *dev); \
    \
    static const struct hpm_can_config hpm_can_cfg_##n = { \
        .base = (CAN_Type*)DT_INST_REG_ADDR(n), \
        .clock_name = (clock_name_t)DT_INST_PROP(n, clk_name), \
        .clock_src = (clock_source_t)DT_INST_PROP(n, clk_source), \
        .clock_div = DT_INST_PROP(n, clk_divider), \
        .irq_config_func = hpm_can_irq_config_func##n, \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
    }; \
    \
    static struct hpm_can_data hpm_can_data_##n = {.config = DEFAULT_HPM_CAN_CONFIG, }; \
    \
    CAN_DEVICE_DT_INST_DEFINE(n, hpm_can_init, \
                         NULL, &hpm_can_data_##n, \
                         &hpm_can_cfg_##n, \
                         POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
                         &hpm_can_driver_api \
                         ); \
    static void hpm_can_irq_config_func##n(const struct device *dev) \
    {   \
        IRQ_CONNECT(DT_INST_IRQN(n), \
                DT_INST_IRQ(n, priority), hpm_can_isr,\
                DEVICE_DT_INST_GET(n), 0); \
                \
        irq_enable(DT_INST_IRQN(n)); \
    }

DT_INST_FOREACH_STATUS_OKAY(HPM_CAN_INIT)
