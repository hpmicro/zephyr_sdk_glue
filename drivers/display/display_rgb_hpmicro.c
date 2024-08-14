/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include "display.h"
#include "hpm_clock_drv.h"
#ifdef CONFIG_HPM_DISPLAY_PIXELMUX
#include <hpm_pixelmux_drv.h>
#endif

LOG_MODULE_REGISTER(display_hpm_rgb, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT hpmicro_hpm_display_rgb

#define HPM_DUMP_DISPLAY_INTERFACE(level, interface) \
    LOG_##level("interface: rgb");

#define HPM_DUMP_DISPLAY_PANEL_PRIVATE(level, panel)

#define DISPLAY_DATA_PRIVATE(inst) \
    .private = NULL,

#define DISPLAY_PANEL_PRIVATE(inst)
#define DISPLAY_INTERFACE(inst)

static int hpm_display_lcdc_clk_init(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    const struct hpm_lcdc *lcdc = &config->lcdc;
    const struct hpm_display_timing *timing = &config->panel.timing;
    uint32_t freq_khz;
    uint32_t div;

    freq_khz = clock_get_frequency(lcdc->pll_clock_name) / 1000;
    div      = (freq_khz + timing->pixel_clock_khz / 2) / timing->pixel_clock_khz;

    clock_add_to_group(lcdc->lcdc_clock_name, 0);
    clock_set_source_divider(lcdc->lcdc_clock_name, lcdc->lcdc_clock_src, div);
    data->pixel_clk_khz = clock_get_frequency(lcdc->lcdc_clock_name) / 1000;

    LOG_INF("pll clk        : %u KHz", freq_khz);
    LOG_INF("lcdc pixel clk : %u KHz", data->pixel_clk_khz);
    LOG_INF("panel pixel clk: %u KHz", timing->pixel_clock_khz);

    return 0;
}

#ifdef CONFIG_HPM_DISPLAY_PIXELMUX
static int hpm_display_pixelmux_router_config(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_lcdc *lcdc = &config->lcdc;
    pixelmux_rgb_select_t sel = lcdc->id == 0 ?
                                pixelmux_rgb_sel_lcdc0 :
                                pixelmux_rgb_sel_lcdc1;

    pixelmux_rgb_data_source_enable(sel);

    return 0;
}
#endif

static void hpm_display_isr(const struct device *dev)
{
    hpm_display_lcdc_isr(dev);
}

static int hpm_display_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;

    HPM_DUMP_DISPLAY_INTERFACE(DBG, config->interface);
    HPM_DUMP_DISPLAY_LCDC(DBG, &config->lcdc);
    HPM_DUMP_DISPLAY_PANEL(DBG, &config->panel);

    hpm_display_lcdc_clk_init(dev);

#ifdef CONFIG_HPM_DISPLAY_PIXELMUX
    hpm_display_pixelmux_router_config(dev);
#endif

    config->config_init(dev);
    hpm_display_pinctl_init(dev);
    hpm_display_gpio_init(dev);
    hpm_display_panel_reset(dev);
    hpm_display_lcdc_init(dev);
    hpm_display_panel_gpio_backlight(dev, 1);

    return 0;
}

static const struct display_driver_api hpm_display_api = {
    .blanking_on      = hpm_display_blanking_on,
    .blanking_off     = hpm_display_blanking_off,
    .write            = hpm_display_write,
    .read             = hpm_display_read,
    .get_framebuffer  = hpm_display_get_framebuffer,
    .set_brightness   = hpm_display_set_brightness,
    .set_contrast     = hpm_display_set_contrast,
    .get_capabilities = hpm_display_get_capabilities,
    .set_pixel_format = hpm_display_set_pixel_format,
    .set_orientation  = hpm_display_set_orientation,
};

DT_INST_FOREACH_STATUS_OKAY(HPM_DISPLAY_INIT)
