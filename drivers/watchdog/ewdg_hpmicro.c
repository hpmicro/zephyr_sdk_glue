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
#include "hpm_ewdg_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_plic_drv.h"
#include <drivers/clock_control/hpmicro_clock_control.h>

#define DT_DRV_COMPAT hpmicro_hpm_ewdg
#define EWDG_CNT_CLK_FREQ 32768UL

LOG_MODULE_REGISTER(hpmicro_hpm_ewdg, CONFIG_WDT_LOG_LEVEL);

struct hpm_wdog_config {
    EWDG_Type *base;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
    bool reset_none_intr;
    void (*irq_config_func)(const struct device *dev);
    unsigned int irq_num;
};

struct hpm_wdog_data {
    wdt_callback_t callback;
    ewdg_config_t wdog_config;
    bool timeout_valid;
    bool inited;
};

static int hpm_wdog_setup(const struct device *dev, uint8_t options)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    EWDG_Type *base = config->base;

    if (!data->timeout_valid) {
        LOG_ERR("No valid timeouts installed");
        return -EINVAL;
    }

    if (data->inited) {
        return -EBUSY;
    }

    hpm_stat_t status = ewdg_init(base, &data->wdog_config);
    if (status != status_success) {
        LOG_ERR(" EWDG initialization failed, error_code=%d\n", status);
        return -EFAULT;
    }

    ewdg_clear_status_flags(base, ewdg_get_status_flags(base));
    irq_enable(config->irq_num);
    ewdg_refresh(base); // Feed the watchdog after initialization to prevent immediate timeout

    data->inited = true;

    return 0;
}

static int hpm_wdog_disable(const struct device *dev)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    EWDG_Type *base = config->base;

    if (!data->timeout_valid) {
        return -EFAULT;
    }

    ewdg_refresh(base);
    ewdg_disable(base);
    data->timeout_valid = false;
    data->inited = false;
    LOG_DBG("Disabled the watchdog");

    return 0;
}

static int hpm_wdog_install_timeout(const struct device *dev,
                     const struct wdt_timeout_cfg *cfg)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    EWDG_Type *base = config->base;

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

    ewdg_get_default_config(base, &data->wdog_config);

    switch (cfg->flags) {
        case WDT_FLAG_RESET_SOC:
            data->wdog_config.int_rst_config.enable_timeout_reset = true;
            break;
        case WDT_FLAG_RESET_NONE:
            if (config->reset_none_intr) {
                data->wdog_config.int_rst_config.enable_timeout_reset = false;
                break;
            } else {
                return -ENOTSUP;
            }
        default:
            LOG_ERR("Unsupported watchdog config flag");
            return -ENOTSUP;
    }

    data->wdog_config.enable_watchdog = true;
    data->wdog_config.ctrl_config.use_lowlevel_timeout = false;
    data->wdog_config.ctrl_config.cnt_clk_sel = ewdg_cnt_clk_src_ext_osc_clk;
    data->wdog_config.ctrl_config.timeout_reset_us = cfg->window.max * 1000U;
    data->wdog_config.cnt_src_freq = EWDG_CNT_CLK_FREQ;

    if (cfg->window.min) {
        if (config->reset_none_intr) {
            data->wdog_config.ctrl_config.enable_window_mode = true;
            data->wdog_config.ctrl_config.window_lower_limit = ewdg_window_lower_timeout_period_8_div_16;
            data->wdog_config.ctrl_config.window_upper_limit = ewdg_window_upper_timeout_period_8_div_16;
        } else {
            return -EINVAL;
        }
    } else {
        data->wdog_config.ctrl_config.enable_window_mode = false;
    }

    if (cfg->callback != NULL) {
        if (config->reset_none_intr) {
            data->wdog_config.int_rst_config.enable_timeout_interrupt = true;
            data->callback = cfg->callback;
            if (cfg->window.min) {
                data->wdog_config.ctrl_config.timeout_interrupt_us = cfg->window.min * 1000U;
            } else {
                data->wdog_config.ctrl_config.timeout_interrupt_us = (cfg->window.max - 1U) * 1000U;
            }
        } else {
            return -ENOTSUP;
        }
    }
    data->timeout_valid = true;

    return 0;
}

static int hpm_wdog_feed(const struct device *dev, int channel_id)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    EWDG_Type *base = config->base;

    if (channel_id != 0 || !data->timeout_valid || !data->inited) {
        LOG_ERR("Invalid channel id");
        return -EINVAL;
    }

    ewdg_refresh(base);
    LOG_DBG("Fed the watchdog");

    return 0;
}

static void hpm_wdog_isr(const struct device *dev)
{
    const struct hpm_wdog_config *config = dev->config;
    struct hpm_wdog_data *data = dev->data;
    EWDG_Type *base = config->base;

    uint32_t ewdg_stat = ewdg_get_status_flags(base);
    if ((ewdg_stat & EWDG_EVENT_TIMEOUT_INTERRUPT) != 0) {
        ewdg_clear_status_flags(base, ewdg_stat);
    }

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

#if defined(EWDG_TIMEOUT_INTERRUPT_REQUIRE_EDGE_TRIGGER) && \
    (EWDG_TIMEOUT_INTERRUPT_REQUIRE_EDGE_TRIGGER == 1)
#define HPM_EWDG_CONFIG_EDGE_TRIGGER(n)                                     \
    do {                                                                    \
        __plic_enable_irq_edge_trigger((uint32_t)DT_REG_ADDR(DT_INST_IRQ_INTC(n)), \
                                       DT_INST_IRQ(n, irq));                \
    } while (false)
#else
#define HPM_EWDG_CONFIG_EDGE_TRIGGER(n) do { } while (false)
#endif

#define HPM_WDOG_INIT(n)                                              \
    static struct hpm_wdog_data hpm_wdog_data_##n;                    \
                                                                        \
    static void hpm_wdog_config_func_##n(const struct device *dev)    \
    {                                                                 \
        HPM_EWDG_CONFIG_EDGE_TRIGGER(n);                              \
        IRQ_CONNECT(DT_INST_IRQN(n),                                  \
                    DT_INST_IRQ(n, priority),                         \
                    hpm_wdog_isr, DEVICE_DT_INST_GET(n), 0);          \
    }                                                                 \
                                                                        \
    static const struct hpm_wdog_config hpm_wdog_config_##n = {       \
        .base = (EWDG_Type *)DT_INST_REG_ADDR(n),                    \
        .irq_config_func = hpm_wdog_config_func_##n,                 \
        .irq_num = DT_INST_IRQN(n),                                  \
        .reset_none_intr = DT_INST_PROP(n, reset_none_intr),         \
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
