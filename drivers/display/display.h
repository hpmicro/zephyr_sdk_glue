/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <zephyr/sys/util.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>
#include <soc.h>

struct hpm_display_timing {
    uint32_t pixel_clock_khz;  /*!< pixel clocl,UINT: KHz */
    uint32_t hactive;          /*!< Horizontal active video */
    uint32_t hfront_porch;     /*!< Horizontal Front Porch */
    uint32_t hback_porch;      /*!< Horizontal Back Porch */
    uint32_t hsync_len;        /*!< Horizontal sync len */

    uint32_t vactive;          /*!< Vertical active video */
    uint32_t vfront_porch;     /*!< Vertical Front Porch */
    uint32_t vback_porch;      /*!< Vertical Back Porch */
    uint32_t vsync_len;        /*!< Vertical sync len */
    uint32_t hsync_pol :1;     /*!< Horizontal Synchronization Signal Polarity, 0: High Active, 1: Low Active */
    uint32_t vsync_pol :1;     /*!< Vertical Synchronization Signal Polarity, 0: High Active, 1: Low Active */
    uint32_t de_pol :1;        /*!< Data Enable Signal Polarity, 0: High Active, 1: Low Active */
    uint32_t pixel_clk_pol :1; /*!< Pixel Clock Signal Polarity, 0: High Active, 1: Low Active */
    uint32_t pixel_data_pol :1;/*!< Pixel Data Signal Polarity, 0: High Active, 1: Low Active */
};

struct hpm_panel_info {
    char *panel_name;
    const struct hpm_display_timing timing;
    const struct pinctrl_dev_config *pincfg;
    const struct gpio_dt_spec backlight_gpio;
    const struct gpio_dt_spec reset_gpio;
    const struct gpio_dt_spec cfg_gpio;
    const int reset_time_active;
    const int reset_time_inactive;
    int pixel_format;
    void *private;
};

struct hpm_lcdc {
    void *lcdc_base;
#ifdef CONFIG_HPM_DISPLAY_PIXELMUX
    void *pixelmux_base;
#endif
    int id;
    uint32_t pll_clock_name;
    uint32_t pll_clock_src;
    uint32_t lcdc_clock_name;
    uint32_t lcdc_clock_src;
};

struct hpm_display_config {
    struct hpm_panel_info panel;
    struct hpm_lcdc lcdc;
    void *interface;
    void (*config_init)(const struct device *dev);
};

struct hpm_display_data {
    void *buf;
    void *aligned_buf;
    int stride_in_byte;
    uint32_t pixel_clk_khz;
    struct k_sem sem;
    void *private;
};

#ifdef CONFIG_HPM_DISPLAY_PIXELMUX
#define PIXELMUX_NODE_ID(inst)    DT_PHANDLE_BY_IDX(LCDC_NODE_ID(inst), router, 0)
#define PIXELMUX_BASE(inst)      .pixelmux_base = (void *)DT_REG_ADDR_BY_IDX(PIXELMUX_NODE_ID(inst), 0),
#else
#define PIXELMUX_BASE(inst)
#endif

#define PANEL_NODE_ID(inst)       DT_DRV_INST(inst)
#define INTERFACE_NODE_ID(inst)   DT_PHANDLE_BY_IDX(PANEL_NODE_ID(inst), endpoint_in, 0)
#define LCDC_NODE_ID(inst)        DT_PHANDLE_BY_IDX(INTERFACE_NODE_ID(inst), endpoint_in, 0)

#define HPM_PANEL_TIMING(inst)                                                              \
    .timing = {                                                                             \
        .pixel_clock_khz = DT_INST_PROP(inst, pixel_clock_khz),                             \
        .hactive         = DT_INST_PROP(inst, hactive),                                     \
        .hfront_porch    = DT_INST_PROP(inst, hfront_porch),                                \
        .hback_porch     = DT_INST_PROP(inst, hback_porch),                                 \
        .hsync_len       = DT_INST_PROP(inst, hsync_len),                                   \
        .vactive         = DT_INST_PROP(inst, vactive),                                     \
        .vfront_porch    = DT_INST_PROP(inst, vfront_porch),                                \
        .vback_porch     = DT_INST_PROP(inst, vback_porch),                                 \
        .vsync_len       = DT_INST_PROP(inst, vsync_len),                                   \
        .hsync_pol       = DT_INST_PROP_OR(inst, hsync_pol, 0),                             \
        .vsync_pol       = DT_INST_PROP_OR(inst, vsync_pol, 0),                             \
        .de_pol          = DT_INST_PROP_OR(inst, de_pol, 0),                                \
        .pixel_clk_pol   = DT_INST_PROP_OR(inst, pixel_clk_pol, 0),                         \
        .pixel_data_pol  = DT_INST_PROP_OR(inst, pixel_data_pol, 0),                        \
    },

#define HPM_DISPLAY_INIT(inst)                                                              \
    PINCTRL_DT_INST_DEFINE(inst);                                                           \
    static void hpm_display_config_func_##inst(const struct device *dev);                   \
    static const struct hpm_display_config hpm_display_config_##inst = {                    \
        .panel = {                                                                          \
            .panel_name     = DT_INST_PROP(inst, panel_name),                               \
            .pixel_format   = DT_INST_PROP_OR(inst, pixel_format, 8),                       \
            .pincfg         = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                         \
            .backlight_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, backlight_gpios, {0}),         \
            .reset_gpio     = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),             \
            .cfg_gpio       = GPIO_DT_SPEC_INST_GET_OR(inst, cfg_gpios, {0}),               \
            .reset_time_active = DT_INST_PROP_OR(inst, reset_time_active, 0),               \
            .reset_time_inactive = DT_INST_PROP_OR(inst, reset_time_inactive, 0),           \
            HPM_PANEL_TIMING(inst)                                                          \
            DISPLAY_PANEL_PRIVATE(inst)                                                     \
        },                                                                                  \
        .lcdc = {                                                                           \
            .id              = DT_PROP(LCDC_NODE_ID(inst), id),                             \
            .lcdc_base       = (void *)DT_REG_ADDR_BY_IDX(LCDC_NODE_ID(inst), 0),           \
            .pll_clock_name  = DT_CLOCKS_CELL_BY_IDX(LCDC_NODE_ID(inst), 0, name),          \
            .pll_clock_src   = DT_CLOCKS_CELL_BY_IDX(LCDC_NODE_ID(inst), 0, src),           \
            .lcdc_clock_name = DT_CLOCKS_CELL_BY_IDX(LCDC_NODE_ID(inst), 1, name),          \
            .lcdc_clock_src  = DT_CLOCKS_CELL_BY_IDX(LCDC_NODE_ID(inst), 1, src),           \
            PIXELMUX_BASE(inst)                                                             \
        },                                                                                  \
        DISPLAY_INTERFACE(inst)                                                             \
        .config_init = hpm_display_config_func_##inst,                                      \
    };                                                                                      \
    static struct hpm_display_data hpm_display_data_##inst = {                              \
        DISPLAY_DATA_PRIVATE(inst)                                                          \
    };                                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                                             \
                hpm_display_init,                                                          \
                NULL,                                                                       \
                &hpm_display_data_##inst,                                                   \
                &hpm_display_config_##inst,                                                 \
                POST_KERNEL,                                                                \
                CONFIG_DISPLAY_INIT_PRIORITY,                                               \
                &hpm_display_api);                                                          \
    static void hpm_display_config_func_##inst(const struct device *dev)                    \
    {                                                                                       \
        IRQ_CONNECT(DT_IRQN(LCDC_NODE_ID(inst)),                                            \
                    DT_IRQ(LCDC_NODE_ID(inst), priority),                                   \
                    hpm_display_isr,                                                        \
                    DEVICE_DT_INST_GET(inst),                                               \
                    0);                                                                     \
        irq_enable(DT_IRQN(LCDC_NODE_ID(inst)));                                            \
    }

#define HPM_DUMP_DISPLAY_PANEL(level, panel)                                                \
    LOG_##level("panel info:");                                                             \
    LOG_##level("panel_name      = %s", (panel)->panel_name);                               \
    LOG_##level("pixel_clock_khz = %d", (panel)->timing.pixel_clock_khz);                   \
    LOG_##level("hactive         = %d", (panel)->timing.hactive);                           \
    LOG_##level("hfront_porch    = %d", (panel)->timing.hfront_porch);                      \
    LOG_##level("hback_porch     = %d", (panel)->timing.hback_porch);                       \
    LOG_##level("hsync_len       = %d", (panel)->timing.hsync_len);                         \
    LOG_##level("vactive         = %d", (panel)->timing.vactive);                           \
    LOG_##level("vfront_porch    = %d", (panel)->timing.vfront_porch);                      \
    LOG_##level("vback_porch     = %d", (panel)->timing.vback_porch);                       \
    LOG_##level("vsync_len       = %d", (panel)->timing.vsync_len);                         \
    LOG_##level("hsync_pol       = %d", (panel)->timing.hsync_pol);                         \
    LOG_##level("vsync_pol       = %d", (panel)->timing.vsync_pol);                         \
    LOG_##level("de_pol          = %d", (panel)->timing.de_pol);                            \
    LOG_##level("pixel_clk_pol   = %d", (panel)->timing.pixel_clk_pol);                     \
    LOG_##level("pixel_data_pol  = %d", (panel)->timing.pixel_data_pol);                    \
    HPM_DUMP_DISPLAY_PANEL_PRIVATE(level, panel);


#ifdef CONFIG_HPM_DISPLAY_PIXELMUX
#define HPM_PIXMUX_INFO(level, lcdc)                                                        \
    LOG_##level("pixelmux_base = 0x%x", (unsigned int)(lcdc)->pixelmux_base);
#else
#define HPM_PIXMUX_INFO(level, lcdc)
#endif

#define HPM_DUMP_DISPLAY_LCDC(level, lcdc)                                                  \
    LOG_##level("lcdc info:");                                                              \
    LOG_##level("id            = %d", (lcdc)->id);                                          \
    LOG_##level("lcdc_base     = 0x%x", (unsigned int)(lcdc)->lcdc_base);                   \
    HPM_PIXMUX_INFO(level, lcdc)

int hpm_display_lcdc_init(const struct device *dev);
void hpm_display_lcdc_isr(const struct device *dev);
int hpm_display_pinctl_init(const struct device *dev);
int hpm_display_gpio_init(const struct device *dev);
void hpm_display_panel_reset(const struct device *dev);
void hpm_display_panel_gpio_backlight(const struct device *dev, int value);
int hpm_display_blanking_on(const struct device *dev);
int hpm_display_blanking_off(const struct device *dev);
int hpm_display_write(const struct device *dev, const uint16_t x,
                 const uint16_t y,
                 const struct display_buffer_descriptor *desc,
                 const void *buf);
int hpm_display_read(const struct device *dev, const uint16_t x,
                const uint16_t y,
                const struct display_buffer_descriptor *desc,
                void *buf);
void *hpm_display_get_framebuffer(const struct device *dev);
int hpm_display_set_brightness(const struct device *dev,
                      const uint8_t brightness);
int hpm_display_set_contrast(const struct device *dev,
                    const uint8_t contrast);
void hpm_display_get_capabilities(const struct device *dev,
                         struct display_capabilities *
                         capabilities);
int hpm_display_set_pixel_format(const struct device *dev,
                        const enum display_pixel_format
                        pixel_format);
int hpm_display_set_orientation(const struct device *dev,
                       const enum display_orientation
                       orientation);
#endif