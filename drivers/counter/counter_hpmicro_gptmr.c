/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT hpmicro_hpm_gptimer

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/clock_control.h>

#include "hpm_common.h"
#ifdef CONFIG_SOC_SERIES_HPM5100
#include "hpm_gptmrv2_drv.h"
#else
#include "hpm_gptmr_drv.h"
#endif
#include "hpm_clock_drv.h"
#include <drivers/clock_control/hpmicro_clock_control.h>
#include "hpm_interrupt.h"
#include "hpm_plic_drv.h"

LOG_MODULE_REGISTER(hpmicro_hpm_gptimer, CONFIG_COUNTER_LOG_LEVEL);

#define DEV_CFG(_dev) ((const struct hpm_gpt_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct hpm_gpt_data *)(_dev)->data)

#define HPM_GPT_CHAN_CNT 2

struct hpm_gpt_config {
    /* info must be first element */
    struct counter_config_info info;

    DEVICE_MMIO_NAMED_ROM(gpt_mmio);

    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    clk_src_t clock_src;
    uint32_t clock_div;
    uint32_t irq;
};

struct hpm_gpt_data {
    DEVICE_MMIO_NAMED_RAM(gpt_mmio);
    struct {
        counter_alarm_callback_t alarm_callback;
        void *alarm_user_data;
        bool force_pending;
    } alarms[HPM_GPT_CHAN_CNT];
    counter_top_callback_t top_callback;
    void *top_user_data;
    uint32_t guard_period;
};

static GPTMR_Type *get_base(const struct device *dev)
{
    return (GPTMR_Type *)DEVICE_MMIO_NAMED_GET(dev, gpt_mmio);
}

static int hpm_gpt_start(const struct device *dev)
{
    GPTMR_Type *base = get_base(dev);

    gptmr_start_counter(base, 0);

    return 0;
}

static int hpm_gpt_stop(const struct device *dev)
{
    GPTMR_Type *base = get_base(dev);

    gptmr_stop_counter(base, 0);

    return 0;
}

static int hpm_gpt_get_value(const struct device *dev, uint32_t *ticks)
{
    GPTMR_Type *base = get_base(dev);

    *ticks = gptmr_channel_get_counter(base, 0, gptmr_counter_type_normal);
    return 0;
}

static uint32_t hpm_gpt_get_top_value(const struct device *dev)
{
    GPTMR_Type *base = get_base(dev);

    return gptmr_channel_get_reload(base, 0);
}

static int hpm_gpt_set_alarm(const struct device *dev, uint8_t chan_id,
            const struct counter_alarm_cfg *alarm_cfg)
{
    GPTMR_Type *base = get_base(dev);
    struct hpm_gpt_data *data = dev->data;
    const struct hpm_gpt_config *config = dev->config;
    uint32_t ticks = alarm_cfg->ticks;

    if (chan_id >= HPM_GPT_CHAN_CNT) return -EINVAL;

    if (data->alarms[chan_id].alarm_callback) {
        return -EBUSY;
    }

    uint32_t top = hpm_gpt_get_top_value(dev);
    if (ticks > top) {
        return -EINVAL;
    }

    uint32_t now = gptmr_channel_get_counter(base, 0, gptmr_counter_type_normal);
    uint32_t time_cycle = top + 1;

    if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
        ticks = now + ticks;
    }

    uint32_t guard = data->guard_period;
    bool is_late = false;

    if (ticks >= now) {
        if (ticks - now <= guard) {
            is_late = true;
        } else {
            is_late = false;
            ticks %= time_cycle;
        }
    } else {
        if ((alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) || 
            ((now - ticks) <= guard)) {
            is_late = true;
        }
    }

    if (is_late) {
        if (alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) {
            data->alarms[chan_id].alarm_callback = alarm_cfg->callback;
            data->alarms[chan_id].alarm_user_data = alarm_cfg->user_data;
            data->alarms[chan_id].force_pending = true;

            gptmr_clear_status(base, GPTMR_CH_CMP_STAT_MASK(0, chan_id));
            gptmr_enable_irq(base, GPTMR_CH_CMP_IRQ_MASK(0, chan_id));
            __plic_set_irq_pending((uint32_t)HPM_PLIC, config->irq);
        }
        return -ETIME;
    }

    data->alarms[chan_id].alarm_callback = alarm_cfg->callback;
    data->alarms[chan_id].alarm_user_data = alarm_cfg->user_data;

    gptmr_clear_status(base, GPTMR_CH_CMP_STAT_MASK(0, chan_id));
    gptmr_update_cmp(base, 0, chan_id, ticks);
    gptmr_enable_irq(base, GPTMR_CH_CMP_IRQ_MASK(0, chan_id));

    return 0;
}

static int hpm_gpt_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
    GPTMR_Type *base = get_base(dev);
    struct hpm_gpt_data *data = dev->data;

    if (chan_id >= HPM_GPT_CHAN_CNT) {
        LOG_ERR("Invalid channel id");
        return -EINVAL;
    }

    gptmr_disable_irq(base, GPTMR_CH_CMP_IRQ_MASK(0, chan_id));

    gptmr_clear_status(base, GPTMR_CH_CMP_STAT_MASK(0, chan_id));

    data->alarms[chan_id].alarm_callback = NULL;
    data->alarms[chan_id].alarm_user_data = NULL;

    return 0;
}

void hpm_gpt_isr(const struct device *dev)
{
    GPTMR_Type *base = get_base(dev);
    struct hpm_gpt_data *data = dev->data;
    uint32_t current = gptmr_channel_get_counter(base, 0, gptmr_counter_type_normal);

    for (uint8_t chan = 0; chan < HPM_GPT_CHAN_CNT; chan++) {
        if (gptmr_check_status(base, GPTMR_CH_CMP_STAT_MASK(0, chan)) || data->alarms[chan].force_pending) {
            gptmr_clear_status(base, GPTMR_CH_CMP_STAT_MASK(0, chan));
            data->alarms[chan].force_pending = false;
            if (data->alarms[chan].alarm_callback) {
                gptmr_disable_irq(base, GPTMR_CH_CMP_IRQ_MASK(0, chan));
                counter_alarm_callback_t alarm_cb = data->alarms[chan].alarm_callback;
                data->alarms[chan].alarm_callback = NULL;
                alarm_cb(dev, chan, current, data->alarms[chan].alarm_user_data);
            }
        }
    }

    if(gptmr_check_status(base, GPTMR_CH_RLD_STAT_MASK(0))) {
        gptmr_clear_status(base, GPTMR_CH_RLD_STAT_MASK(0));
        if (data->top_callback) {
            data->top_callback(dev, data->top_user_data);
        }
    }
}

static uint32_t hpm_gpt_get_pending_int(const struct device *dev)
{
    GPTMR_Type *base = get_base(dev);
    uint32_t status =0;

    for (uint8_t chan = 0; chan < HPM_GPT_CHAN_CNT; chan++) {
        status |= gptmr_check_status(base, GPTMR_CH_CMP_STAT_MASK(0, chan));
    }

    return status | gptmr_check_status(base, GPTMR_CH_RLD_STAT_MASK(0));
}

static int hpm_gpt_set_top_value(const struct device *dev,
                  const struct counter_top_cfg *cfg)
{
    GPTMR_Type *base = get_base(dev);
    struct hpm_gpt_data *data = dev->data;
    gptmr_channel_config_t gptConfig;

    data->top_user_data = cfg->user_data;
    data->top_callback = cfg->callback;

    gptmr_channel_get_default_config(base, &gptConfig);

    if (gptConfig.cmp[1] != 0xFFFFFFFF) {
        if (cfg->ticks == 0xFFFFFFFF) {
            gptConfig.reload = cfg->ticks;
        } else {
            gptConfig.reload = cfg->ticks + 1;
        }
    } else {
        return -ENOTSUP;
    }
    hpm_gpt_stop(dev);

    gptmr_channel_config(base, 0, &gptConfig, false);
    gptmr_channel_reset_count(base, 0);
    gptmr_enable_irq(base, GPTMR_CH_RLD_IRQ_MASK(0));
    hpm_gpt_start(dev);
    return 0;
}

static uint32_t hpm_gpt_get_freq(const struct device *dev)
{
    const struct hpm_gpt_config *config = dev->config;

    return config->info.freq;
}

static uint32_t hpm_gpt_counter_api_get_guard_period(const struct device *dev,
                        uint32_t flags)
{
    ARG_UNUSED(flags);
    const struct hpm_gpt_data *data = dev->data;

    return data->guard_period;
}

static int hpm_gpt_counter_api_set_guard_period(const struct device *dev, uint32_t guard,
                    uint32_t flags)
{
    ARG_UNUSED(flags);
    struct hpm_gpt_data *data = dev->data;

    if (guard > hpm_gpt_get_top_value(dev)) {
        return -EINVAL;
    }

    data->guard_period = guard;
    return 0;
}

static int hpm_gpt_init(const struct device *dev)
{
    const struct hpm_gpt_config *config = dev->config;
    const struct hpm_clock_dt_spec gpt_clock = {
        .dev = config->clock_dev,
        .name = (clock_name_t)config->clock_subsys,
        .src = config->clock_src,
        .div = config->clock_div,
    };
    gptmr_channel_config_t gptconfig;
    uint32_t clock_freq = 0;
    GPTMR_Type *base;

    DEVICE_MMIO_NAMED_MAP(dev, gpt_mmio, K_MEM_CACHE_NONE | K_MEM_DIRECT_MAP);

    if (!device_is_ready(config->clock_dev)) {
        LOG_ERR("clock control device not ready");
        return -ENODEV;
    }

    /* Apply the source/divider from devicetree before touching the GPTMR, so the
     * counter input clock no longer depends on the SoC clock preset (and never on
     * the link-time address of this config struct).
     */
    if (clock_configure_enable(&gpt_clock) != status_success) {
        LOG_ERR("failed to configure GPT clock\n");
        return -EIO;
    }

    clock_freq = clock_get_frequency((clock_name_t)config->clock_subsys);

    /* Adjust divider to match expected freq */
    if (clock_freq % config->info.freq) {
        LOG_ERR("Cannot Adjust GPT freq to %u (input clock %u)\n",
                config->info.freq, clock_freq);
        return -EINVAL;
    }

    base = get_base(dev);
    gptmr_channel_get_default_config(base, &gptconfig);
    gptconfig.cmp_initial_polarity_high = 0;

    gptmr_channel_config(base, 0, &gptconfig, false);
    gptmr_channel_reset_count(base, 0);

    return 0;
}

static const struct counter_driver_api hpm_gpt_driver_api = {
    .start = hpm_gpt_start,
    .stop = hpm_gpt_stop,
    .get_value = hpm_gpt_get_value,
    .set_alarm = hpm_gpt_set_alarm,
    .cancel_alarm = hpm_gpt_cancel_alarm,
    .set_top_value = hpm_gpt_set_top_value,
    .get_pending_int = hpm_gpt_get_pending_int,
    .get_top_value = hpm_gpt_get_top_value,
    .get_freq = hpm_gpt_get_freq,
    .get_guard_period = hpm_gpt_counter_api_get_guard_period,
    .set_guard_period = hpm_gpt_counter_api_set_guard_period,
};

#define HPM_GPT_INIT(n)                        \
    static struct hpm_gpt_data hpm_gpt_data_ ## n;    \
                                    \
    static const struct hpm_gpt_config hpm_gpt_config_ ## n = {    \
        .info = {                        \
            .max_top_value = UINT32_MAX,            \
            .freq = DT_INST_PROP(n, gptfreq),           \
            .channels = HPM_GPT_CHAN_CNT,                    \
            .flags = COUNTER_CONFIG_INFO_COUNT_UP,        \
        },                            \
        DEVICE_MMIO_NAMED_ROM_INIT(gpt_mmio, DT_DRV_INST(n)),    \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),    \
        .clock_subsys =                        \
            (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
        .clock_src = (clk_src_t)DT_INST_CLOCKS_CELL(n, src),    \
        .clock_div = DT_INST_CLOCKS_CELL(n, div),            \
        .irq = DT_INST_IRQ(n, irq),                \
    };                                \
                                    \
    static int hpm_gpt_## n ##_init(const struct device *dev);    \
    DEVICE_DT_INST_DEFINE(n,                    \
                hpm_gpt_## n ##_init,            \
                NULL,                    \
                &hpm_gpt_data_ ## n,            \
                &hpm_gpt_config_ ## n,            \
                POST_KERNEL,                \
                CONFIG_COUNTER_INIT_PRIORITY,        \
                &hpm_gpt_driver_api);            \
                                    \
    static int hpm_gpt_## n ##_init(const struct device *dev)    \
    {                                \
        IRQ_CONNECT(DT_INST_IRQN(n),                \
                DT_INST_IRQ(n, priority),            \
                hpm_gpt_isr, DEVICE_DT_INST_GET(n), 0);    \
        irq_enable(DT_INST_IRQN(n));                \
        return hpm_gpt_init(dev);                \
    }                                \

DT_INST_FOREACH_STATUS_OKAY(HPM_GPT_INIT)
