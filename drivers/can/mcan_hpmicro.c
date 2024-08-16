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
#include "hpm_mcan_drv.h"
#include "hpm_clock_drv.h"

#define HPM_CAN_EXT_FILTER_NUM_MAX (64U)
#define HPM_CAN_STD_FILTER_NUM_MAX (128U)
#define HPM_MCAN_BITRATE_MAX (1000000UL) /* 1Mbps */
#define HPM_MCAN_FD_BITRATE_MAX (8000000UL) /* 8Mbps */

static mcan_rx_message_t s_can_rx_buf;
static volatile mcan_tx_event_fifo_elem_t s_can_tx_evt;
static volatile bool has_sent_out;
static volatile bool has_error;
static volatile bool tx_event_occurred;
static volatile bool timeout_event_occurred;

#define HPM_MCAN_NUM_TX_BUF_ELEMENTS (32U)
#define HPM_MCAN_NUM_RX_BUF_ELEMENTS (16U)

/* Default baudrate: 1Mbps @80MHz CAN clock */
#define DEFAULT_HPM_MCAN_CONFIG {         \
    .use_lowlevel_timing_setting = false, \
    .can_timing = {2, 29, 10, 2},       \
    .mode = mcan_mode_normal,  \
}


struct hpm_mcan_config {
    /* zephyr can config*/
    const struct can_driver_config common;
    /* hpmicro config*/
    MCAN_Type *base;
    clock_name_t clock_name;
    clock_source_t clock_src;
    uint32_t clock_div;
    void (*irq_config_func)(const struct device *dev);
    const struct pinctrl_dev_config *pincfg;
};

struct hpm_mcan_data {
    struct can_driver_data common;
    /* MCAN configuration */
    mcan_config_t config;
    uint32_t ext_filter_count;
    uint16_t ext_filters_index[HPM_CAN_EXT_FILTER_NUM_MAX];
    mcan_filter_elem_t ext_filters[HPM_CAN_EXT_FILTER_NUM_MAX];
    uint32_t std_filter_count;
    uint16_t std_filters_index[HPM_CAN_STD_FILTER_NUM_MAX];
    mcan_filter_elem_t std_filters[HPM_CAN_STD_FILTER_NUM_MAX];

    uint32_t filter_rtr;
    uint32_t filter_rtr_mask;

    /* RTOS misc. */
    struct k_mutex inst_mutex;
    struct k_mutex tx_mutex;
    enum can_state state;
    struct k_sem tx_sem;
    struct k_sem tx_fin_sem[HPM_MCAN_NUM_TX_BUF_ELEMENTS];

    /* TX callback */
    can_tx_callback_t tx_fin_cb[HPM_MCAN_NUM_TX_BUF_ELEMENTS];
    void *tx_fin_cb_arg[HPM_MCAN_NUM_TX_BUF_ELEMENTS];

    /* RX callback */
    can_rx_callback_t rx_cb_ext[HPM_CAN_EXT_FILTER_NUM_MAX];
    void *rx_cb_arg_ext[HPM_CAN_EXT_FILTER_NUM_MAX];

    can_rx_callback_t rx_cb_std[HPM_CAN_STD_FILTER_NUM_MAX];
    void *rx_cb_arg_std[HPM_CAN_STD_FILTER_NUM_MAX];

    /* State change callback */
    can_state_change_callback_t state_change_cb;
    void *state_change_cb_data;
    bool started;
};

#define DT_DRV_COMPAT hpmicro_hpm_mcan
LOG_MODULE_REGISTER(hpmicro_hpm_mcan, CONFIG_CAN_LOG_LEVEL);

static int hpm_mcan_get_state(const struct device *dev,
                             enum can_state *state,
                             struct can_bus_err_cnt *err_cnt);
static void hpm_mcan_tc_event_handler(const struct device *dev, uint32_t index);
static void hpm_mcan_state_change_handler(const struct device *dev);

/*
 * CAST CAN doesn't support reporting CAN filter index, this function mannually look up the filter index
 */
static uint32_t hpm_mcan_get_first_filter_index(const mcan_rx_message_t *buf,
                                        const struct hpm_mcan_data *data)
{
    uint32_t filter_index = -EINVAL;
    uint32_t ext_filter_count = data->ext_filter_count;
    uint32_t std_filter_count = data->std_filter_count;

    if (buf->use_ext_id) {
        for (uint32_t i = 0; i < ext_filter_count; i++) {
            const mcan_filter_elem_t *filter = &data->ext_filters[i];
            if ((buf->ext_id | filter->filter_mask) == (filter->filter_id | filter->filter_mask)) {
                filter_index = i;
                break;
            }
        }
    } else {
        for (uint32_t i = 0; i < std_filter_count; i++) {
            const mcan_filter_elem_t *filter = &data->std_filters[i];
            if ((buf->std_id | filter->filter_mask) == (filter->filter_id | filter->filter_mask)) {
                filter_index = i;
                break;
            }
        }
    }
    return filter_index;
}

/* Get MCAN message from RX BUF */
static void hpm_mcan_get_message_from_rxbuf(const struct device *dev, uint32_t buf_index)
{
    const struct hpm_mcan_config *cfg = dev->config;
    MCAN_Type *can = cfg->base;
    struct hpm_mcan_data *data = dev->data;
    can_rx_callback_t cb;
    struct can_frame frame;
    void *cb_arg;
    memset(&s_can_rx_buf, 0, sizeof(s_can_rx_buf));
    mcan_read_rxbuf(can, buf_index, (mcan_rx_message_t *) &s_can_rx_buf);

    uint32_t filter_index = hpm_mcan_get_first_filter_index(&s_can_rx_buf, data);
    memset(&frame, 0, sizeof(frame));
    if (s_can_rx_buf.use_ext_id) {
        frame.id = s_can_rx_buf.ext_id;
    } else {
        frame.id = s_can_rx_buf.std_id;
    }
    frame.flags = 0;
    if (s_can_rx_buf.canfd_frame) {
        frame.flags |= CAN_FRAME_FDF;
    }
    if (s_can_rx_buf.rtr) {
        frame.flags |= CAN_FRAME_RTR;
    }
    if (s_can_rx_buf.use_ext_id) {
        frame.flags |= CAN_FRAME_IDE;
    }
    if (s_can_rx_buf.bitrate_switch) {
        frame.flags |= CAN_FRAME_BRS;
    }
    frame.dlc = s_can_rx_buf.dlc;
    LOG_DBG("Frame on filter %d, ID: 0x%x", filter_index, frame.id);
    size_t data_len = can_dlc_to_bytes(frame.dlc);
    for (uint32_t i = 0; i < data_len; i++) {
        frame.data[i] = s_can_rx_buf.data_8[i];
    }

    /* Handle RX filter callback */
    if (filter_index != -EINVAL) {
        /* If RTR bit does not match filter RTR mask and bit, drop current frame */
        bool rtr_filter_mask = (data->filter_rtr_mask & BIT(filter_index)) != 0;
        bool rtr_filter = (data->filter_rtr & BIT(filter_index)) != 0;
        bool rtr_in_frame = (frame.flags & CAN_FRAME_RTR);
        if ((rtr_filter_mask != rtr_in_frame) || (rtr_filter != rtr_in_frame)) {
            return;
        }

        if (s_can_rx_buf.use_ext_id) {
            cb = data->rx_cb_ext[filter_index];
            cb_arg = data->rx_cb_arg_ext[filter_index];
        } else {
            cb = data->rx_cb_std[filter_index];
            cb_arg = data->rx_cb_arg_std[filter_index];
        }

        if (cb != NULL) {
            cb(dev, &frame, cb_arg);
        } else {
            LOG_DBG("cb missing");
        }
    }
}

/* Get MCAN message from RX BUF */
static void hpm_mcan_get_message_from_rxfifo(const struct device *dev, uint32_t fifo_index)
{
    const struct hpm_mcan_config *cfg = dev->config;
    MCAN_Type *can = cfg->base;
    struct hpm_mcan_data *data = dev->data;
    can_rx_callback_t cb;
    struct can_frame frame;
    void *cb_arg;
    memset(&s_can_rx_buf, 0, sizeof(s_can_rx_buf));
    mcan_read_rxfifo(can, fifo_index, (mcan_rx_message_t *) &s_can_rx_buf);

    uint32_t filter_index = hpm_mcan_get_first_filter_index(&s_can_rx_buf, data);
    memset(&frame, 0, sizeof(frame));
    if (s_can_rx_buf.use_ext_id) {
        frame.id = s_can_rx_buf.ext_id;
    } else {
        frame.id = s_can_rx_buf.std_id;
    }
    frame.flags = 0;
    if (s_can_rx_buf.canfd_frame) {
        frame.flags |= CAN_FRAME_FDF;
    }
    if (s_can_rx_buf.rtr) {
        frame.flags |= CAN_FRAME_RTR;
    }
    if (s_can_rx_buf.use_ext_id) {
        frame.flags |= CAN_FRAME_IDE;
    }
    if (s_can_rx_buf.bitrate_switch) {
        frame.flags |= CAN_FRAME_BRS;
    }
    frame.dlc = s_can_rx_buf.dlc;
    LOG_DBG("Frame on filter %d, ID: 0x%x", filter_index, frame.id);
    size_t data_len = can_dlc_to_bytes(frame.dlc);
    for (uint32_t i = 0; i < data_len; i++) {
        frame.data[i] = s_can_rx_buf.data_8[i];
    }

    /* Handle RX filter callback */
    if (filter_index != -EINVAL) {
        /* If RTR bit does not match filter RTR mask and bit, drop current frame */
        bool rtr_filter_mask = (data->filter_rtr_mask & BIT(filter_index)) != 0;
        bool rtr_filter = (data->filter_rtr & BIT(filter_index)) != 0;
        bool rtr_in_frame = (frame.flags & CAN_FRAME_RTR);
        if ((rtr_filter_mask != rtr_in_frame) || (rtr_filter != rtr_in_frame)) {
            return;
        }

        if (s_can_rx_buf.use_ext_id) {
            cb = data->rx_cb_ext[filter_index];
            cb_arg = data->rx_cb_arg_ext[filter_index];
        } else {
            cb = data->rx_cb_std[filter_index];
            cb_arg = data->rx_cb_arg_std[filter_index];
        }

        if (cb != NULL) {
            cb(dev, &frame, cb_arg);
        } else {
            LOG_DBG("cb missing");
        }
    }
}

__attribute__((section(".isr")))static void hpm_mcan_isr(const struct device *dev)
{
    const struct hpm_mcan_config *cfg = dev->config;
    MCAN_Type *can = cfg->base;
    uint32_t flags = mcan_get_interrupt_flags(can);
    /* New message is available in RXFIFO0 */
    if ((flags & MCAN_INT_RXFIFO0_NEW_MSG) != 0) {
        hpm_mcan_get_message_from_rxfifo(dev, 0);
    }
    /* New message is available in RXFIFO1 */
    if ((flags & MCAN_INT_RXFIFO1_NEW_MSG) != 0U) {
        hpm_mcan_get_message_from_rxfifo(dev, 1);
    }
    /* New message is available in RXBUF */
    if ((flags & MCAN_INT_MSG_STORE_TO_RXBUF) != 0U) {
        /* NOTE: Below code is for demonstration purpose, the performance is not optimized
         *       Users should optimize the performance according to real use case.
         */
        for (uint32_t buf_index = 0; buf_index < MCAN_RXBUF_SIZE_CAN_DEFAULT; buf_index++) {
            if (mcan_is_rxbuf_data_available(can, buf_index)) {
                hpm_mcan_get_message_from_rxbuf(dev, buf_index);
                mcan_clear_rxbuf_data_available_flag(can, buf_index);
            }
        }
    }
    /* New TX Event occurred */
    if ((flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) != 0) {
        //mcan_read_tx_evt_fifo(can, (mcan_tx_event_fifo_elem_t *) &s_can_tx_evt);
    }
    /* Transmit completed */
    if ((flags & MCAN_EVENT_TRANSMIT) != 0U) {
        has_sent_out = true;
        hpm_mcan_tc_event_handler(dev, 0);
    }

    /* Error happened */
    if ((flags & MCAN_EVENT_ERROR) != 0) {
        hpm_mcan_state_change_handler(dev);
    }

    mcan_clear_interrupt_flags(can, flags);
}

static int hpm_mcan_init(const struct device *dev)
{
    int ret = 0;

    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    mcan_config_t *config = &data->config;
    MCAN_Type *can = cfg->base;

    k_mutex_init(&data->inst_mutex);
    k_mutex_init(&data->tx_mutex);
    k_sem_init(&data->tx_sem, HPM_MCAN_NUM_TX_BUF_ELEMENTS, HPM_MCAN_NUM_TX_BUF_ELEMENTS);

    for (uint32_t i= 0; i < ARRAY_SIZE(data->tx_fin_sem); i++) {
        k_sem_init(&data->tx_fin_sem[i], 1, 1);
    }

    data->filter_rtr = 0;
    data->filter_rtr_mask = 0;
    has_sent_out = true;

    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("CAN pinctrl setup failed (%d)", ret);
        return ret;
    }

    mcan_get_default_config(can, config);
    config->mode = mcan_mode_loopback_internal;

    clock_set_source_divider(cfg->clock_name, cfg->clock_src, cfg->clock_div);

    config->can_timing.num_seg1 = 60;
    config->can_timing.num_seg2 = 20;
    config->can_timing.num_sjw = 16;
    config->can_timing.prescaler = 2;

    /* Set Interrupt Enable Mask */
    uint32_t interrupt_mask = MCAN_EVENT_RECEIVE | MCAN_EVENT_TRANSMIT | MCAN_EVENT_ERROR;
    config->interrupt_mask = interrupt_mask;
    config->txbuf_trans_interrupt_mask = ~0UL;

    /* Set CAN filter list */
    config->all_filters_config.ext_id_filter_list.mcan_filter_elem_count = data->ext_filter_count;
    config->all_filters_config.ext_id_filter_list.filter_elem_list = data->ext_filters;
    config->all_filters_config.std_id_filter_list.mcan_filter_elem_count = data->std_filter_count;
    config->all_filters_config.std_id_filter_list.filter_elem_list = data->std_filters;

    mcan_get_default_ram_config(can, &config->ram_config, true);

    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    hpm_stat_t status = mcan_init(can, config, can_clk_freq);
    if (status != status_success) {
        ret = -EAGAIN;
    } else {
        cfg->irq_config_func(dev);
    }

    return ret;
}


static int hpm_mcan_set_mode(const struct device *dev, can_mode_t mode)
{
    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    mcan_config_t *config = &data->config;
    MCAN_Type *mcan = cfg->base;
    struct can_driver_data *common = &data->common;

    if (data->started) {
		return -EBUSY;
	}

#if !defined(CONFIG_CAN_FD_MODE)
    if ((mode & CAN_MODE_FD) != 0) {
        LOG_ERR("unsupported mode: 0x%08x", mode);
        return -ENOTSUP;
    }
#endif /* CONFIG_CAN_FD_MODE */

    if ((mode & CAN_MODE_LOOPBACK) != 0) {
        mcan->CCCR |= MCAN_CCCR_MON_MASK | MCAN_CCCR_TEST_MASK;
        mcan->TEST |= MCAN_TEST_LBCK_MASK;
    }
    else if ((mode & CAN_MODE_LISTENONLY) != 0) {
        mcan->CCCR |= MCAN_CCCR_MON_MASK;
    } else {
        mcan->CCCR &= ~(MCAN_CCCR_MON_MASK | MCAN_CCCR_TEST_MASK);
    }

#ifdef CONFIG_CAN_FD_MODE
    if ((mode & CAN_MODE_FD)!=0) {
        config->use_lowlevel_timing_setting = true;
        config->canfd_timing.num_seg1 = 14;
        config->canfd_timing.num_seg2  = 5;
        config->canfd_timing.num_sjw = 2;
        config->canfd_timing.prescaler = 2;
        config->enable_canfd = 1;
    }
#endif /* CONFIG_CAN_FD_MODE */

    common->mode = mode;

    return 0;
}


/**
 * @brief Set bit timing for MCAN 2.0B/ MCAN-FD norminal
 */
static int hpm_mcan_set_timing(const struct device *dev, const struct can_timing *timing)
{
    int ret = 0;

    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    MCAN_Type *mcan = cfg->base;

    if (data->started) {
		return -EBUSY;
	}

    mcan_config_t *config = &data->config;
    config->use_lowlevel_timing_setting = false;
    mcan_bit_timing_param_t *timing_param = &config->can_timing;
    timing_param->prescaler = timing->prescaler;
    /* num_seg1 in CAST_CAN = Tsync_seq + Tprop_seg + Tphase_seg1  */
    timing_param->num_seg1 = 1 + timing->prop_seg + timing->phase_seg1;
    timing_param->num_seg2 = timing->phase_seg2;
    timing_param->num_sjw = timing->sjw;

    uint32_t mcan_clk_freq = clock_get_frequency(cfg->clock_name);
    hpm_stat_t status = mcan_init(mcan, config, mcan_clk_freq);
    if (status != status_success) {
        ret = -EAGAIN;
    }

    return ret;
}


static void convert_can_frame_to_mcan_frame(const struct can_frame *frame,
                                     mcan_tx_frame_t *tx_buf)
{
    if (frame->flags & CAN_FRAME_IDE) {
        tx_buf->ext_id = frame->id;
        tx_buf->use_ext_id =1;
    } else {
        tx_buf->std_id = frame->id;
    }
    tx_buf->dlc = frame->dlc;
    tx_buf->bitrate_switch = (uint16_t)((frame->flags & CAN_FRAME_BRS) != 0);
    tx_buf->rtr = (uint16_t)((frame->flags & CAN_FRAME_RTR) != 0);
    tx_buf->canfd_frame = (uint16_t)((frame->flags & CAN_FRAME_FDF) != 0);

    size_t msg_len = can_dlc_to_bytes(tx_buf->dlc);

    for (size_t i = 0; i < msg_len; i++) {
        tx_buf->data_8[i] = frame->data[i];
    }
}


static int hpm_mcan_send(const struct device *dev,
                        const struct can_frame *frame,
                        k_timeout_t timeout,
                        can_tx_callback_t callback,
                        void *user_data)
{
    int ret;
    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    MCAN_Type *can = cfg->base;
    enum can_state state;
    hpm_stat_t status;

#ifdef CONFIG_CAN_FD_MODE
	if (((data->config.enable_canfd) != 0)) {
        if ((frame->flags & CAN_FRAME_ESI) != 0) {
            if ((frame->flags & CAN_FRAME_BRS) == 0) {
                LOG_ERR("unsupported CAN frame flags, CAN_FRAME_ESI Only valid in combination with CAN_FRAME_FDF");
                return -ENOTSUP;
            }
        }
        if ((frame->flags & (CAN_FRAME_RTR)) != 0) {
            LOG_ERR("unsupported CAN-FD frame flags 0x%02x", frame->flags);
            return -ENOTSUP;
        }
        if (frame->dlc > CANFD_MAX_DLC) {
            LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
            return -EINVAL;
        }
	} else {
        if ((frame->flags & (CAN_FRAME_FDF | CAN_FRAME_BRS | CAN_FRAME_ESI)) != 0) {
            LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
            return -ENOTSUP;
        }
        if (frame->dlc > CAN_MAX_DLC) {
            LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
            return -EINVAL;
        }
    }
#else
	if ((frame->flags & (CAN_FRAME_FDF | CAN_FRAME_BRS | CAN_FRAME_ESI)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

    if (frame->dlc > CAN_MAX_DLC) {
        LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
        return -EINVAL;
    }
#endif

    if (!data->started) {
		return -ENETDOWN;
	}

    while (!has_sent_out) {

    }

    has_sent_out = false;

    (void) hpm_mcan_get_state(dev, &state, NULL);
    if (state == CAN_STATE_BUS_OFF) {
        LOG_DBG("Transmit failed, bus-off");
        return -ENETUNREACH;
    }

    ret = k_sem_take(&data->tx_sem, timeout);
    if (ret != 0) {
        return -EAGAIN;
    }

    mcan_tx_frame_t tx_buf;
    memset(&tx_buf, 0, sizeof(tx_buf));
    convert_can_frame_to_mcan_frame(frame, &tx_buf);

    k_mutex_lock(&data->tx_mutex, timeout);

    data->tx_fin_cb[0] = callback;
    data->tx_fin_cb_arg[0] = user_data;

    status = mcan_transmit_via_txbuf_nonblocking(can, 0, &tx_buf);

    k_mutex_unlock(&data->tx_mutex);
    if (status != 0) {
        return -EIO;
    }

    if (callback == NULL) {
        LOG_DBG("Waiting for TX complete");
        k_sem_take(&data->tx_fin_sem[0], K_FOREVER);
    }


    return 0;
}

static int hpm_mcan_add_rx_filter_ext(const struct device *dev,
                            can_rx_callback_t callback,
                            void *user_data,
                            const struct can_filter *filter)
{
    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    MCAN_Type *can = cfg->base;
    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    int filter_id = -ENOSPC;

    if (callback == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->inst_mutex, K_FOREVER);
    if (data->ext_filter_count < HPM_CAN_EXT_FILTER_NUM_MAX) {
        filter_id = data->ext_filter_count;
        data->ext_filter_count++;
    } else {
        return -ENOSPC;
    }

    mcan_config_t *can_config = &data->config;
    mcan_filter_elem_t *filter_ext = &data->ext_filters[filter_id];
    data->ext_filters_index[filter_id] = filter_id;
    filter_ext->filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;
    filter_ext->filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
    filter_ext->can_id_type = MCAN_CAN_ID_TYPE_EXTENDED;
    filter_ext->filter_id = filter->id;
    filter_ext->filter_mask = ~filter->mask;

    can_config->all_filters_config.ext_id_filter_list.mcan_filter_elem_count = data->ext_filter_count;
    can_config->all_filters_config.ext_id_filter_list.filter_elem_list = data->ext_filters;

    if ((filter->flags & CAN_FRAME_RTR) != 0) {
		data->filter_rtr |= (1U << filter_id);
	} else {
		data->filter_rtr &= ~(1U << filter_id);
	}

	if ((filter->flags & CAN_FRAME_RTR) != 0) {
		data->filter_rtr_mask |= (1U << filter_id);
	} else {
		data->filter_rtr_mask &= ~(1U << filter_id);
	}

    (void) mcan_init(can, can_config, can_clk_freq);

    data->rx_cb_ext[filter_id] = callback;
    data->rx_cb_arg_ext[filter_id] = user_data;

    k_mutex_unlock(&data->inst_mutex);

    return filter_id;
}

static int hpm_mcan_add_rx_filter_std(const struct device *dev,
                            can_rx_callback_t callback,
                            void *user_data,
                            const struct can_filter *filter)
{
    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    MCAN_Type *can = cfg->base;
    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    int filter_id = -ENOSPC;

    if (callback == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data->inst_mutex, K_FOREVER);
    if (data->std_filter_count < HPM_CAN_STD_FILTER_NUM_MAX) {
        filter_id = data->std_filter_count;
        data->std_filter_count++;
    } else {
        return -ENOSPC;
    }

    mcan_config_t *can_config = &data->config;
    mcan_filter_elem_t *filter_std = &data->std_filters[filter_id];
    data->std_filters_index[filter_id] = filter_id;
    filter_std->filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;
    filter_std->filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
    filter_std->can_id_type = MCAN_CAN_ID_TYPE_STANDARD;
    filter_std->filter_id = filter->id;
    filter_std->filter_mask = ~filter->mask;

    can_config->all_filters_config.std_id_filter_list.mcan_filter_elem_count = data->std_filter_count;
    can_config->all_filters_config.std_id_filter_list.filter_elem_list = data->std_filters;

    if ((filter->flags & CAN_FRAME_RTR) != 0) {
		data->filter_rtr |= (1U << filter_id);
	} else {
		data->filter_rtr &= ~(1U << filter_id);
	}

	if ((filter->flags & CAN_FRAME_RTR) != 0) {
		data->filter_rtr_mask |= (1U << filter_id);
	} else {
		data->filter_rtr_mask &= ~(1U << filter_id);
	}

    (void) mcan_init(can, can_config, can_clk_freq);

    data->rx_cb_std[filter_id] = callback;
    data->rx_cb_arg_std[filter_id] = user_data;

    k_mutex_unlock(&data->inst_mutex);

    return filter_id;
}

/*
 *  NOTE:
 *  MCAN_CTRL doesn't support change the MCAN filter at runtime
 *  Software need to re-initialize the MCAN controller if the changes to filter needs to be made.
 */
static int hpm_mcan_add_rx_filter(const struct device *dev,
                            can_rx_callback_t callback,
                            void *user_data,
                            const struct can_filter *filter)
{
    int filter_id;

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0U) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	if ((filter->flags & CAN_FILTER_IDE) != 0U) {
		filter_id = hpm_mcan_add_rx_filter_ext(dev, callback, user_data, filter);
	} else {
		filter_id = hpm_mcan_add_rx_filter_std(dev, callback, user_data, filter);
	}

    return filter_id;
}


/*
 *  NOTE:
 *  MCAN_CTRL doesn't support change the MCAN filter at runtime
 *  Software need to re-initialize the MCAN controller if the changes to filter needs to be made.
 */
static void hpm_mcan_remove_rx_filter(const struct device *dev, int filter_id)
{
    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    MCAN_Type *can = cfg->base;

    mcan_config_t *config = &data->config;
    k_mutex_lock(&data->inst_mutex, K_FOREVER);

    if (data->ext_filter_count > 0) {
        uint16_t last_filter_index = data->ext_filters_index[data->ext_filter_count -1];
        mcan_filter_elem_t *last_filter_elem = &data->ext_filters[data->ext_filter_count -1];
        for (uint32_t i = 0; i < data->ext_filter_count; i++) {
            if (data->ext_filters_index[i] == filter_id) {
                if (i != data->ext_filter_count - 1U) {
                    mcan_filter_elem_t *current_filter_elem = &data->ext_filters[i];
                    data->ext_filters_index[i] = last_filter_index;
                    current_filter_elem->filter_id = last_filter_elem->filter_id;
                    current_filter_elem->filter_mask = last_filter_elem->filter_mask;
                    current_filter_elem->filter_type = last_filter_elem->filter_type;
                    current_filter_elem->filter_config = last_filter_elem->filter_config;
                    current_filter_elem->can_id_type = last_filter_elem->can_id_type;
                }
                data->ext_filter_count -=1;
                uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
                config->all_filters_config.ext_id_filter_list.filter_elem_list = data->ext_filters;
                config->all_filters_config.ext_id_filter_list.mcan_filter_elem_count = data->ext_filter_count;
                (void) mcan_init(can, config, can_clk_freq);
                break;
            }
        }
    }

    if (data->std_filter_count > 0) {
        uint16_t last_filter_index = data->std_filters_index[data->std_filter_count -1];
        mcan_filter_elem_t *last_filter_elem = &data->std_filters[data->std_filter_count -1];
        for (uint32_t i = 0; i < data->std_filter_count; i++) {
            if (data->std_filters_index[i] == filter_id) {
                if (i != data->std_filter_count - 1U) {
                    mcan_filter_elem_t *current_filter_elem = &data->std_filters[i];
                    data->std_filters_index[i] = last_filter_index;
                    current_filter_elem->filter_id = last_filter_elem->filter_id;
                    current_filter_elem->filter_mask = last_filter_elem->filter_mask;
                    current_filter_elem->filter_type = last_filter_elem->filter_type;
                    current_filter_elem->filter_config = last_filter_elem->filter_config;
                    current_filter_elem->can_id_type = last_filter_elem->can_id_type;
                }
                data->std_filter_count -=1;
                uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
                config->all_filters_config.std_id_filter_list.filter_elem_list = data->std_filters;
                config->all_filters_config.std_id_filter_list.mcan_filter_elem_count = data->std_filter_count;
                (void) mcan_init(can, config, can_clk_freq);
                break;
            }
        }
    }

    k_mutex_unlock(&data->inst_mutex);
}


static int hpm_mcan_get_state(const struct device *dev,
                             enum can_state *state,
                             struct can_bus_err_cnt *err_cnt)
{
    int ret = 0;
    mcan_protocol_status_t protocol_status;
    const struct hpm_mcan_config *cfg = dev->config;
    MCAN_Type *can = cfg->base;
    const struct hpm_mcan_data *data = dev->data;
    mcan_error_count_t error_count;
    memset(&error_count, 0, sizeof(mcan_error_count_t));

    if (!data->started) {
        *state = CAN_STATE_STOPPED;
    } else {
        if (state != NULL) {
            uint8_t flags = mcan_get_interrupt_flags(can);
            if ((flags & MCAN_EVENT_ERROR) != 0) {
                mcan_parse_protocol_status(can->PSR, &protocol_status);
                mcan_get_error_counter(can, &error_count);
                if (protocol_status.in_bus_off_state) {
                    *state = CAN_STATE_BUS_OFF;
                } else if (protocol_status.in_warning_state) {
                    *state = CAN_STATE_ERROR_WARNING;
                } else if (protocol_status.in_error_passive_state) {
                    *state = CAN_STATE_ERROR_PASSIVE;
                } else {
                    *state = CAN_STATE_ERROR_ACTIVE;
                }
            } else {
                *state = CAN_STATE_ERROR_ACTIVE;
            }
        }
    }

    if (err_cnt != NULL) {
        err_cnt->tx_err_cnt = error_count.transmit_error_count;
        err_cnt->rx_err_cnt = error_count.receive_error_count;
    }

    return ret;
}

#if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
int hpm_mcan_recover(const struct device *dev, k_timeout_t timeout)
{
    struct hpm_mcan_data *data = dev->data;

    ARG_UNUSED(timeout);

    if (!data->started) {
        return -ENETDOWN;
    }

    return -ENOSYS;
}
#endif

/*  */
static void hpm_mcan_state_change_handler(const struct device *dev)
{
    struct hpm_mcan_data *data = dev->data;
    const can_state_change_callback_t cb = data->state_change_cb;
    void *cb_data = data->state_change_cb_data;
    struct can_bus_err_cnt err_cnt;
    enum can_state state;

    (void) hpm_mcan_get_state(dev, &state, &err_cnt);

    if (cb != NULL) {
        cb(dev, state, err_cnt, cb_data);
    }
}

static void hpm_mcan_set_state_change_callback(const struct device *dev,
                                                can_state_change_callback_t callback,
                                                void *user_data)
{
    struct hpm_mcan_data *data = dev->data;

    data->state_change_cb = callback;
    data->state_change_cb_data = user_data;
}

static void hpm_mcan_tc_event_handler(const struct device *dev, uint32_t index)
{
    struct hpm_mcan_data *data = dev->data;
    can_tx_callback_t tx_cb;
    k_sem_give(&data->tx_sem);
    tx_cb = data->tx_fin_cb[index];
    if (tx_cb  == NULL) {
        k_sem_give(&data->tx_fin_sem[index]);
    } else {
        tx_cb(dev, 0, data->tx_fin_cb_arg[index]);
    }
}


static int hpm_mcan_get_core_clock(const struct device *dev, uint32_t *rate)
{
    const struct hpm_mcan_config *cfg = dev->config;
    if (rate != NULL) {
        *rate = clock_get_frequency(cfg->clock_name);
    }
    return 0;
}


static int hpm_mcan_get_max_filters(const struct device *dev, bool ide)
{
    ARG_UNUSED(dev);
    if (ide) {
        return HPM_CAN_EXT_FILTER_NUM_MAX;
    } else {
        return HPM_CAN_STD_FILTER_NUM_MAX;
    }
}

#if CONFIG_CAN_FD_MODE
static int hpm_mcan_set_timing_data(const struct device *dev, const struct can_timing *timing)
{
    int ret = 0;

    const struct hpm_mcan_config *cfg = dev->config;
    struct hpm_mcan_data *data = dev->data;
    mcan_config_t *config = &data->config;
    MCAN_Type *can = cfg->base;

    if (data->started) {
		return -EBUSY;
	}

    config->use_lowlevel_timing_setting = true;
    mcan_bit_timing_param_t *timing_param = &config->canfd_timing;
    timing_param->prescaler = timing->prescaler;
    /* num_seg1 in CAST_CAN = Tsync_seq + Tprop_seg + Tphase_seg1  */
    timing_param->num_seg1 = 1 + timing->prop_seg + timing->phase_seg1;
    timing_param->num_seg2 = timing->phase_seg2;
    timing_param->num_sjw = timing->sjw;
    config->enable_canfd = 1;

    uint32_t can_clk_freq = clock_get_frequency(cfg->clock_name);
    hpm_stat_t status = mcan_init(can, config, can_clk_freq);

    if (status != status_success) {
        ret = -EAGAIN;
    }

    return ret;
}
#endif

static int hpm_mcan_start(const struct device *dev)
{
    const struct hpm_mcan_config *config = dev->config;
    struct hpm_mcan_data *data = dev->data;
    mcan_config_t *can_config = &data->config;
    MCAN_Type *can = config->base;

    if (data->started) {
        return -EALREADY;
    }

    uint32_t can_clk_freq = clock_get_frequency(config->clock_name);
    mcan_init(can, can_config, can_clk_freq);

    data->started = true;

    return 0;
}

static int hpm_mcan_stop(const struct device *dev)
{
    const struct hpm_mcan_config *config = dev->config;
    struct hpm_mcan_data *data = dev->data;

    if (!data->started) {
		return -EALREADY;
	}

    mcan_deinit(config->base);

    data->started = false;

    return 0;
}

static int hpm_mcan_get_capabilities(const struct device *dev, can_mode_t *cap)
{
    struct hpm_mcan_data *data = dev->data;
    mcan_config_t *can_config = &data->config;

    *cap = can_config->mode;

#if CONFIG_CAN_FD_MODE
	*cap |= CAN_MODE_FD;
#endif

    return 0;
}

static const struct can_driver_api hpm_mcan_driver_api = {
    .get_capabilities = hpm_mcan_get_capabilities,
    .set_mode = hpm_mcan_set_mode,
    .set_timing = hpm_mcan_set_timing,
    .send = hpm_mcan_send,
    .start = hpm_mcan_start,
    .stop = hpm_mcan_stop,
    .add_rx_filter = hpm_mcan_add_rx_filter,
    .remove_rx_filter = hpm_mcan_remove_rx_filter,
    .get_state = hpm_mcan_get_state,
    .set_state_change_callback = hpm_mcan_set_state_change_callback,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
    .recover = hpm_mcan_recover,
#endif
    .get_core_clock = hpm_mcan_get_core_clock,
    .get_max_filters = hpm_mcan_get_max_filters,

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
    .set_timing_data = hpm_mcan_set_timing_data,

    .timing_data_min = {
        .sjw = 1,
        .prop_seg = 1,
        .phase_seg1 = 1,
        .phase_seg2 = 2,
        .prescaler = 2,

    },
    .timing_data_max = {
        .sjw = 4,
        .prop_seg = 8,
        .phase_seg1 = 7,
        .phase_seg2 = 5,
        .prescaler = 30,
    }
#endif
};

#define HPM_MCAN_INIT(n)                     \
\
    PINCTRL_DT_INST_DEFINE(n); \
        static void hpm_mcan_irq_config_func##n(const struct device *dev); \
    \
    static const struct hpm_mcan_config hpm_mcan_cfg_##n = { \
        .common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000), \
        .base = (MCAN_Type*)DT_INST_REG_ADDR(n), \
        .clock_name = (clock_name_t)DT_INST_PROP(n, clk_name), \
        .clock_src = (clock_source_t)DT_INST_PROP(n, clk_source), \
        .clock_div = DT_INST_PROP(n, clk_divider), \
        .irq_config_func = hpm_mcan_irq_config_func##n, \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), \
    }; \
    \
    static struct hpm_mcan_data hpm_mcan_data_##n = {.config = DEFAULT_HPM_MCAN_CONFIG, }; \
    \
    CAN_DEVICE_DT_INST_DEFINE(n, hpm_mcan_init, \
                        NULL, &hpm_mcan_data_##n, \
                        &hpm_mcan_cfg_##n, \
                        POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
                        &hpm_mcan_driver_api \
                        ); \
    static void hpm_mcan_irq_config_func##n(const struct device *dev) \
    {   \
        IRQ_CONNECT(DT_INST_IRQN(n), \
                DT_INST_IRQ(n, priority), hpm_mcan_isr,\
                DEVICE_DT_INST_GET(n), 0); \
                \
        irq_enable(DT_INST_IRQN(n)); \
    }

DT_INST_FOREACH_STATUS_OKAY(HPM_MCAN_INIT)
