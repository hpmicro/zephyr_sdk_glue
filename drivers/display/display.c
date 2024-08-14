/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "display.h"
#include "hpm_lcdc_drv.h"
#include "hpm_l1c_drv.h"

#define HPM_LCDC_FB_ALIGNED HPM_L1C_CACHELINE_SIZE

LOG_MODULE_REGISTER(display_hpm, CONFIG_DISPLAY_LOG_LEVEL);

static void panel_para_to_lcdc(const struct hpm_display_timing *timing, lcdc_config_t *config)
{
    config->resolution_x               = timing->hactive;
    config->resolution_y               = timing->vactive;

    config->hsync.pulse_width          = timing->hsync_len;
    config->hsync.back_porch_pulse     = timing->hback_porch;
    config->hsync.front_porch_pulse    = timing->hfront_porch;

    config->vsync.pulse_width          = timing->vsync_len;
    config->vsync.back_porch_pulse     = timing->vback_porch;
    config->vsync.front_porch_pulse    = timing->vfront_porch;

    config->control.invert_hsync       = timing->hsync_pol;
    config->control.invert_vsync       = timing->vsync_pol;
    config->control.invert_href        = timing->de_pol;
    config->control.invert_pixel_data  = timing->pixel_data_pol;
    config->control.invert_pixel_clock = timing->pixel_clk_pol;
}

int hpm_display_pinctl_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    int ret;

    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(config->panel.pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("panel pinctrl setup failed (%d)", ret);
        return ret;
    }

    return ret;
}

static int hpm_display_lcdc_fb_alloc(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    int stride = panel->timing.hactive * DISPLAY_BITS_PER_PIXEL(panel->pixel_format) / 8;
    int fb_buf_size = ROUND_UP(stride * panel->timing.vactive, HPM_LCDC_FB_ALIGNED);

    data->buf = k_malloc(fb_buf_size + HPM_LCDC_FB_ALIGNED - 1);
    if (!data->buf) {
        LOG_ERR("%s: alloc fb memory (%d) bytes failed", panel->panel_name, fb_buf_size);
        return -1;
    }

    data->aligned_buf = (void *)ROUND_UP(POINTER_TO_UINT(data->buf), HPM_LCDC_FB_ALIGNED);
    data->stride_in_byte = stride;

    LOG_INF("%s: alloc fb(addr, size): %p, %d", panel->panel_name, data->aligned_buf, fb_buf_size);
    memset(data->aligned_buf, 0x00, fb_buf_size);

#define HPM_RGB888_DEBUG 0
#if HPM_RGB888_DEBUG
    int i = 0;
    for (; i < panel->timing.hactive * panel->timing.vactive / 3; i++) {
        ((uint32_t *)data->aligned_buf)[i] = 0xffff0000;
    }

    for (; i < panel->timing.hactive * panel->timing.vactive * 2 / 3; i++) {
        ((uint32_t *)data->aligned_buf)[i] = 0xff00ff00;
    }

    for (; i < panel->timing.hactive * panel->timing.vactive; i++) {
        ((uint32_t *)data->aligned_buf)[i] = 0xff0000ff;
    }
#endif

    return 0;
}

static int hpm_display_lcdc_fb_free(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    k_free(data->buf);
    LOG_INF("%s: free fb(addr, size): %p", panel->panel_name, data->buf);

    return 0;
}

int hpm_display_lcdc_init(const struct device *dev)
{
    const struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    const struct hpm_lcdc *lcdc = &config->lcdc;
    const struct hpm_panel_info *panel = &config->panel;
    display_pixel_format_t pixel_format;
    lcdc_config_t lcdc_config = {0};
    lcdc_layer_config_t layer;

    if (panel->pixel_format == PIXEL_FORMAT_ARGB_8888) {
        pixel_format = display_pixel_format_argb8888;
    } else if (panel->pixel_format == PIXEL_FORMAT_RGB_565) {
        pixel_format = display_pixel_format_rgb565;
    } else {
        LOG_ERR("HPMicro display can't support (%d) pixel format", panel->pixel_format);
        return -1;
    }

    if (hpm_display_lcdc_fb_alloc(dev) < 0) {
        return -1;
    }

    lcdc_get_default_config(lcdc->lcdc_base, &lcdc_config);
    panel_para_to_lcdc(&panel->timing, &lcdc_config);

    lcdc_init(lcdc->lcdc_base, &lcdc_config);
    lcdc_get_default_layer_config(lcdc->lcdc_base, &layer, pixel_format, 0);

    layer.position_x   = 0;
    layer.position_y   = 0;
    layer.width        = lcdc_config.resolution_x;
    layer.height       = lcdc_config.resolution_y;
    layer.buffer       = POINTER_TO_UINT(data->aligned_buf);
    layer.background.u = 0;
    layer.max_bytes    = lcdc_layer_max_bytes_512;

    if (status_success != lcdc_config_layer(lcdc->lcdc_base, 0, &layer, true)) {
        LOG_ERR("failed to configure layer");
        hpm_display_lcdc_fb_free(dev);
        return -1;
    }

    lcdc_turn_on_display(lcdc->lcdc_base);
    lcdc_enable_interrupt(lcdc->lcdc_base, LCDC_INT_EN_VS_BLANK_MASK);

    return 0;
}

void hpm_display_lcdc_isr(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_lcdc *lcdc = &config->lcdc;

    lcdc_clear_status(lcdc->lcdc_base, LCDC_ST_VS_BLANK_MASK);
}

int hpm_display_gpio_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    LOG_DBG("%s: gpio", panel->panel_name);

    if (gpio_is_ready_dt(&panel->cfg_gpio)) {
        gpio_pin_configure_dt(&panel->cfg_gpio, GPIO_OUTPUT_ACTIVE);
    } else {
        LOG_DBG("%s: no cfg_gpio", panel->panel_name);
    }

    if (gpio_is_ready_dt(&panel->reset_gpio)) {
        gpio_pin_configure_dt(&panel->reset_gpio, GPIO_OUTPUT_ACTIVE);
    } else {
        LOG_DBG("%s: no reset_gpio", panel->panel_name);
    }

    if (gpio_is_ready_dt(&panel->backlight_gpio)) {
        gpio_pin_configure_dt(&panel->backlight_gpio, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_DBG("%s: no backlight_gpio", panel->panel_name);
    }

    return 0;
}

void hpm_display_panel_gpio_backlight(const struct device *dev, int value)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    if (gpio_is_ready_dt(&panel->backlight_gpio))
        gpio_pin_set_dt(&panel->backlight_gpio, value);
}

void hpm_display_panel_gpio_reset(const struct device *dev, int value)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    if (gpio_is_ready_dt(&panel->reset_gpio))
        gpio_pin_set_dt(&panel->reset_gpio, value);
}

void hpm_display_panel_reset(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;
    /*
     * 1: active
     * 0: inactive
     */
    if (panel->reset_time_active > 0) {
        hpm_display_panel_gpio_reset(dev, 1);
        k_msleep(panel->reset_time_active);
    }

    hpm_display_panel_gpio_reset(dev, 0);

    if (panel->reset_time_inactive > 0)
        k_msleep(panel->reset_time_inactive);
}

int hpm_display_blanking_on(const struct device *dev)
{
    hpm_display_panel_gpio_backlight(dev, 1);

    return 0;
}

int hpm_display_blanking_off(const struct device *dev)
{
    hpm_display_panel_gpio_backlight(dev, 0);

    return 0;
}

int hpm_display_write(const struct device *dev, const uint16_t x,
                 const uint16_t y,
                 const struct display_buffer_descriptor *desc,
                 const void *buf)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    uint8_t *dst_start = (uint8_t *)data->aligned_buf + y * data->stride_in_byte + x * DISPLAY_BITS_PER_PIXEL(panel->pixel_format) / 8;
    uint32_t cache_buf_start = ROUND_DOWN(POINTER_TO_UINT(dst_start), HPM_L1C_CACHELINE_SIZE);
    uint32_t cache_buf_len = 0;
    const uint8_t *src_start = buf;
    int src_stride = desc->pitch * DISPLAY_BITS_PER_PIXEL(panel->pixel_format) / 8;
    int stride = src_stride < data->stride_in_byte ? src_stride : data->stride_in_byte;

    for (int i = 0; i < desc->height; i++) {
        memcpy(dst_start, src_start, stride);
        dst_start += data->stride_in_byte;
        src_start += src_stride;
        cache_buf_len += data->stride_in_byte;
    }

    cache_buf_len = ROUND_UP(cache_buf_len, HPM_L1C_CACHELINE_SIZE);
    if (l1c_dc_is_enabled())
        l1c_dc_writeback(cache_buf_start, cache_buf_len);

    return 0;
}

int hpm_display_read(const struct device *dev, const uint16_t x,
                const uint16_t y,
                const struct display_buffer_descriptor *desc,
                void *buf)
{
    LOG_ERR("Read not implemented");
    return -ENOTSUP;
}

void *hpm_display_get_framebuffer(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;

    return data->aligned_buf;
}

int hpm_display_set_brightness(const struct device *dev,
                      const uint8_t brightness)
{
    LOG_WRN("Set brightness not implemented");
    return -ENOTSUP;
}

int hpm_display_set_contrast(const struct device *dev,
                    const uint8_t contrast)
{
    LOG_ERR("Set contrast not implemented");
    return -ENOTSUP;
}

void hpm_display_get_capabilities(const struct device *dev,
                         struct display_capabilities *
                         capabilities)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    capabilities->x_resolution            = panel->timing.hactive;
    capabilities->y_resolution            = panel->timing.vactive;
    capabilities->supported_pixel_formats = panel->pixel_format;
    capabilities->screen_info             = 0;
    capabilities->current_pixel_format    = panel->pixel_format;
    capabilities->current_orientation     = DISPLAY_ORIENTATION_NORMAL;
}

int hpm_display_set_pixel_format(const struct device *dev,
                        const enum display_pixel_format
                        pixel_format)
{
    const struct hpm_display_config *config = dev->config;
    const struct hpm_panel_info *panel = &config->panel;

    if (pixel_format == panel->pixel_format) {
        return 0;
    }
    LOG_ERR("Pixel format change not implemented");
    return -ENOTSUP;
}

int hpm_display_set_orientation(const struct device *dev,
                       const enum display_orientation
                       orientation)
{
    if (orientation == DISPLAY_ORIENTATION_NORMAL) {
        return 0;
    }
    LOG_ERR("Changing display orientation not implemented");
    return -ENOTSUP;
}
