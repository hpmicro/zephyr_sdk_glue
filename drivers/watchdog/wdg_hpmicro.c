/*
 * Copyright (c) 2026 HPMicro
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
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/clock_control.h>

#include "hpm_common.h"
#include "hpm_wdg_drv.h"
#include "hpm_clock_drv.h"
#include <drivers/clock_control/hpmicro_clock_control.h>

#define DT_DRV_COMPAT hpmicro_hpm_wdg

LOG_MODULE_REGISTER(hpmicro_hpm_wdg, CONFIG_WDT_LOG_LEVEL);

struct hpm_wdog_config {
    WDG_Type *base;
    unsigned int irq_num;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    void (*irq_config_func)(const struct device *dev);
};

struct hpm_wdog_data {
    wdt_callback_t callback;
    wdg_control_t wdog_config;
    bool timeout_valid;
    bool inited;
};

static int hpm_wdog_setup(const struct device *dev, uint8_t options)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    WDG_Type *base = config->base;

    if (!data->timeout_valid) {
        LOG_ERR("No valid timeouts installed");
        return -EINVAL;
    }

    if (data->inited) {
        return -EBUSY;
    }

    hpm_stat_t status = wdg_init(base, &data->wdog_config);
    if (status != status_success) {
        LOG_ERR(" WDG initialization failed, error_code=%d\n", status);
        return -EFAULT;
    }

    wdg_clear_status(base, WDG_ST_INTEXPIRED_MASK);
    irq_enable(config->irq_num);
    wdg_restart(base); // to ensure the watchdog starts counting from the beginning of the timeout period after initialization

    data->inited = true;

    return 0;
}

static int hpm_wdog_disable(const struct device *dev)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    WDG_Type *base = config->base;

    if (!data->timeout_valid) {
        return -EFAULT;
    }

    wdg_restart(base);
    wdg_disable(base);
    data->timeout_valid = false;
    data->inited = false;
    LOG_DBG("Disabled the watchdog");

    return 0;
}

static int hpm_wdog_configure_reset_window(wdg_control_t *wdog_config,
                       uint32_t window_max_ms)
{
    uint32_t reset_us;
    uint32_t interrupt_us;

    if (window_max_ms == 0U) {
        return -EINVAL;
    }

    /*
     * HPM WDG reset timeout is the sum of INTTIME and RSTTIME.
     * RSTTIME itself only supports up to 16K watchdog clock ticks
     * (about 500 ms with the 32.768 kHz external clock), while Zephyr
     * timeout values such as 2000 ms are valid API inputs.  Keep the
     * hardware reset stage at its largest valid value and put the
     * remaining time into the interrupt stage, with INTEN left disabled
     * unless the caller installed a callback.
     */
    reset_us = wdg_convert_reset_interval_to_us(WDG_EXT_CLK_FREQ,
                           reset_interval_max);

    if ((uint64_t)window_max_ms * USEC_PER_MSEC <= reset_us) {
        reset_us = window_max_ms * USEC_PER_MSEC;
        interrupt_us = 0U;
    } else {
        interrupt_us = window_max_ms * USEC_PER_MSEC - reset_us;
    }

    wdog_config->reset_interval =
        wdg_convert_reset_interval_from_us(WDG_EXT_CLK_FREQ, reset_us);
    wdog_config->interrupt_interval =
        wdg_convert_interrupt_interval_from_us(WDG_EXT_CLK_FREQ, interrupt_us);

    if ((wdog_config->reset_interval > reset_interval_max) ||
        (wdog_config->interrupt_interval > interrupt_interval_max)) {
        return -EINVAL;
    }

    return 0;
}

static int hpm_wdog_install_timeout(const struct device *dev,
                     const struct wdt_timeout_cfg *cfg)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;

    if (data->timeout_valid) {
        LOG_ERR("No more timeouts can be installed");
        return -ENOMEM;
    }

    if (!device_is_ready(config->clock_dev)) {
        LOG_ERR("clock control device not ready");
        return -ENODEV;
    }

    clock_enable_name((clock_name_t)config->clock_subsys);

    if (cfg->window.max == 0) {
        return -EINVAL;
    }

    switch (cfg->flags) {
        case WDT_FLAG_RESET_SOC:
            data->wdog_config.reset_enable = true;
            if (hpm_wdog_configure_reset_window(&data->wdog_config,
                                cfg->window.max) != 0) {
                return -EINVAL;
            }
            break;
        case WDT_FLAG_RESET_NONE:
            data->wdog_config.interrupt_enable = false;
            data->wdog_config.interrupt_interval = wdg_convert_interrupt_interval_from_us(WDG_EXT_CLK_FREQ, cfg->window.max * 1000);
            break;
        default:
            LOG_ERR("Unsupported watchdog config flag");
            return -ENOTSUP;
    }

    data->wdog_config.wdg_enable = true;
    data->wdog_config.clksrc = wdg_clksrc_extclk;

    if (cfg->callback != NULL) {
        data->wdog_config.interrupt_enable = true;
        data->callback = cfg->callback;
        if (cfg->window.min) {
            data->wdog_config.interrupt_interval = wdg_convert_interrupt_interval_from_us(WDG_EXT_CLK_FREQ, cfg->window.min * 1000);
        } else {
            data->wdog_config.interrupt_interval = wdg_convert_interrupt_interval_from_us(WDG_EXT_CLK_FREQ, (cfg->window.max - 1) * 1000);
        }
    }
    data->timeout_valid = true;

    return 0;
}

static int hpm_wdog_feed(const struct device *dev, int channel_id)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    WDG_Type *base = config->base;

    if (channel_id != 0 || !data->timeout_valid || !data->inited) {
        LOG_ERR("Invalid channel id");
        return -EINVAL;
    }

    wdg_restart(base);
    LOG_DBG("Fed the watchdog");

    return 0;
}

static void hpm_wdog_isr(const struct device *dev)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    WDG_Type *base = config->base;

    LOG_INF("Interrupt: Watchdog timeout expired");

    wdg_clear_status(base, WDG_ST_INTEXPIRED_MASK);

    if (data->callback) {
        data->callback(dev, 0);
    }
}

static int hpm_wdog_init(const struct device *dev)
{
    const struct hpm_wdog_config *config = dev->config;

    config->irq_config_func(dev);

    return 0;
}

static const struct wdt_driver_api hpm_wdog_api = {
    .setup = hpm_wdog_setup,
    .disable = hpm_wdog_disable,
    .install_timeout = hpm_wdog_install_timeout,
    .feed = hpm_wdog_feed,
};

#define HPM_WDOG_INIT(n)                                              \
    static struct hpm_wdog_data hpm_wdog_data_##n;                    \
                                                                        \
    static void hpm_wdog_config_func_##n(const struct device *dev)    \
    {                                                                 \
        IRQ_CONNECT(DT_INST_IRQN(n),                                  \
                    DT_INST_IRQ(n, priority),                         \
                    hpm_wdog_isr, DEVICE_DT_INST_GET(n), 0);          \
    }                                                                 \
                                                                        \
    static const struct hpm_wdog_config hpm_wdog_config_##n = {       \
        .base = (WDG_Type *)DT_INST_REG_ADDR(n),                    \
        .irq_num = DT_INST_IRQN(n),                                  \
        .irq_config_func = hpm_wdog_config_func_##n,                 \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),            \
        .clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),    \
    };                                                                \
                                                                        \
    DEVICE_DT_INST_DEFINE(n,                                          \
                        &hpm_wdog_init,                             \
                        NULL,                                       \
                        &hpm_wdog_data_##n,                         \
                        &hpm_wdog_config_##n,                       \
                        POST_KERNEL,                                \
                        CONFIG_KERNEL_INIT_PRIORITY_DEVICE,         \
                        &hpm_wdog_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_WDOG_INIT)
