/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include "display.h"
#include <hpm_clock_drv.h>
#include <hpm_mipi_dsi_drv.h>
#include <hpm_mipi_dsi_phy_drv.h>
#include <hpm_pixelmux_drv.h>

LOG_MODULE_REGISTER(display_hpm_mipi, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT hpmicro_hpm_display_mipi

struct hpm_mipi_cfg {
    const uint8_t *mipi_cmd;
    const int mipi_cmd_len;
    const int mipi_lane_num;
    const int mipi_lane_speed;
};

struct hpm_interface_mipi_dsi {
    void *mipi_base;
    void *mipi_phy_base;
    void *pixelmux_base;
    uint32_t dsi_clock_name;
    uint32_t dsi_clock_src;
    int id;
};

#define HPM_DUMP_DISPLAY_INTERFACE(level, interface)                                                                     \
    LOG_##level("interface: mipi-dsi");                                                                                  \
    LOG_##level("mipi_base     = 0x%x", (unsigned int)((struct hpm_interface_mipi_dsi *)interface)->mipi_base);          \
    LOG_##level("mipi_phy_base = 0x%x", (unsigned int)((struct hpm_interface_mipi_dsi *)interface)->mipi_phy_base);      \
    LOG_##level("pixelmux_base = 0x%x", (unsigned int)((struct hpm_interface_mipi_dsi *)interface)->pixelmux_base);      \
    LOG_##level("id            = %d", (unsigned int)((struct hpm_interface_mipi_dsi *)interface)->id);

#define HPM_DUMP_DISPLAY_PANEL_PRIVATE(level, panel)                                                                     \
    LOG_##level("mipi_lane_num  = %d", ((struct hpm_mipi_cfg *)(panel)->private)->mipi_lane_num);                        \
    LOG_##level("mipi_lane_speed  = %d", ((struct hpm_mipi_cfg *)(panel)->private)->mipi_lane_speed);                    \
    LOG_##level("mipi_cmd_len     = %d", ((struct hpm_mipi_cfg *)(panel)->private)->mipi_cmd_len);                       \
    LOG_HEXDUMP_##level(((struct hpm_mipi_cfg *)(panel)->private)->mipi_cmd,                                             \
                                ((struct hpm_mipi_cfg *)(panel)->private)->mipi_cmd_len,                                 \
                                "mipi_cmd:")

#define DISPLAY_DATA_PRIVATE(inst)                                                                                       \
    .private = NULL,

#define DISPLAY_PANEL_PRIVATE(inst)                                                                                      \
    .private = &(struct hpm_mipi_cfg) {                                                                                  \
        .mipi_lane_num   = DT_INST_PROP(inst, mipi_lane_num),                                                            \
        .mipi_lane_speed = DT_INST_PROP(inst, mipi_lane_speed),                                                          \
        .mipi_cmd_len    = DT_INST_PROP_LEN(inst, mipi_cmd),                                                             \
        .mipi_cmd        = (const uint8_t []){DT_INST_FOREACH_PROP_ELEM_SEP(inst, mipi_cmd,                              \
                                    DT_PROP_BY_IDX, (,))}                                                                \
    },

#define DISPLAY_INTERFACE(inst)                                                                                          \
    .interface = &(struct hpm_interface_mipi_dsi) {                                                                      \
        .mipi_base     = (void *)DT_REG_ADDR_BY_NAME(INTERFACE_NODE_ID(inst), mipi_dsi),                                 \
        .mipi_phy_base = (void *)DT_REG_ADDR_BY_NAME(INTERFACE_NODE_ID(inst), mipi_dsi_phy),                             \
        .pixelmux_base = (void *)DT_REG_ADDR_BY_NAME(INTERFACE_NODE_ID(inst), pixelmux),                                 \
        .dsi_clock_name = DT_CLOCKS_CELL_BY_IDX(INTERFACE_NODE_ID(inst), 0, name),                                       \
        .dsi_clock_src  = DT_CLOCKS_CELL_BY_IDX(INTERFACE_NODE_ID(inst), 0, src),                                        \
        .id            = DT_PROP(INTERFACE_NODE_ID(inst), id),                                                           \
    },

static int hpm_display_lcdc_clk_init(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    const struct hpm_lcdc *lcdc = &config->lcdc;
    const struct hpm_display_timing *timing = &config->panel.timing;
    uint32_t freq_khz;
    uint32_t div;

    freq_khz = clock_get_frequency(lcdc->pll_clock_name) / 1000;
    div = (freq_khz + timing->pixel_clock_khz / 2) / timing->pixel_clock_khz;

    clock_add_to_group(lcdc->lcdc_clock_name, 0);
    clock_set_source_divider(lcdc->lcdc_clock_name, lcdc->lcdc_clock_src, div);
    data->pixel_clk_khz = clock_get_frequency(lcdc->lcdc_clock_name) / 1000;

    LOG_INF("pll clk        : %u KHz", freq_khz);
    LOG_INF("lcdc pixel clk : %u KHz", data->pixel_clk_khz);
    LOG_INF("panel pixel clk: %u KHz", timing->pixel_clock_khz);

    return 0;
}

static int hpm_display_pixelmux_router_config(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    struct hpm_interface_mipi_dsi *interface = config->interface;
    const struct hpm_lcdc *lcdc = &config->lcdc;

    if (interface->id == 0) {
        if (lcdc->id == 0)
            pixelmux_mipi_dsi0_data_source_enable(pixelmux_mipi_dsi0_sel_lcdc0);
        else
            pixelmux_mipi_dsi0_data_source_enable(pixelmux_mipi_dsi0_sel_lcdc1);

        pixelmux_config_tx_phy0_mode(pixelmux_tx_phy_mode_mipi);
    } else {
        if (lcdc->id == 0)
            pixelmux_mipi_dsi1_data_source_enable(pixelmux_mipi_dsi1_sel_lcdc0);
        else
            pixelmux_mipi_dsi1_data_source_enable(pixelmux_mipi_dsi1_sel_lcdc1);

        pixelmux_config_tx_phy1_mode(pixelmux_tx_phy_mode_mipi);
    }
    return 0;
}

static void hpm_display_mipi_panel_host_init(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    struct hpm_mipi_cfg *mipi_panel_cfg = config->panel.private;
    struct hpm_interface_mipi_dsi *interface = config->interface;
    const struct hpm_display_timing *timing = &config->panel.timing;
    mipi_dsi_config_t mipi_cfg;

    clock_add_to_group(interface->dsi_clock_name, 0);

    mipi_dsi_get_defconfig_on_video(&mipi_cfg);

    mipi_cfg.channel                    = 0;
    mipi_cfg.lanes                      = mipi_panel_cfg->mipi_lane_num;
    mipi_cfg.video_para.pixel_clock_khz = data->pixel_clk_khz;
    mipi_cfg.video_para.hactive         = timing->hactive;
    mipi_cfg.video_para.hsync_len       = timing->hsync_len;
    mipi_cfg.video_para.hback_porch     = timing->hback_porch;
    mipi_cfg.video_para.hfront_porch    = timing->hfront_porch;
    mipi_cfg.video_para.vsync_len       = timing->vsync_len;
    mipi_cfg.video_para.vactive         = timing->vactive;
    mipi_cfg.video_para.vback_porch     = timing->vback_porch;
    mipi_cfg.video_para.vfront_porch    = timing->vfront_porch;

    mipi_dsi_init(interface->mipi_base, &mipi_cfg);
}

static void hpm_display_mipi_panel_phy_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    struct hpm_mipi_cfg *mipi_panel_cfg = config->panel.private;
    struct hpm_interface_mipi_dsi *interface = config->interface;
    mipi_dsi_phy_config_t mipi_phy_cfg;

    mipi_phy_cfg.lanes     = mipi_panel_cfg->mipi_lane_num;
    mipi_phy_cfg.lane_mbps = mipi_panel_cfg->mipi_lane_speed;

    mipi_dsi_phy_powerdown(interface->mipi_base);
    mipi_dsi_phy_init(interface->mipi_phy_base, &mipi_phy_cfg);
    mipi_dsi_phy_poweron(interface->mipi_base);
    /*
     * Wait phy go to stable
     */
    k_msleep(2);
}

static void hpm_display_mipi_panel_cmd_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    struct hpm_mipi_cfg *mipi_panel_cfg = config->panel.private;
    struct hpm_interface_mipi_dsi *interface = config->interface;
    const uint8_t *mipi_cmd = mipi_panel_cfg->mipi_cmd;
    const int mipi_cmd_len = mipi_panel_cfg->mipi_cmd_len;
    const uint8_t *cmd_index;
    uint8_t cmd_len;
    uint8_t delay;
    int ret;

    /*
     * [len, delay, data0, data1...., ]
     * len: delay + datas. at least is 1 (only delay).
     */
    for (int i = 0; i < mipi_cmd_len;) {
        cmd_len = mipi_cmd[i] - 1;
        delay   = mipi_cmd[i + 1];
        cmd_index = mipi_cmd + i + 2;
        i += (mipi_cmd[i] + 1);

        if (cmd_len) {
            ret = mipi_dsi_dcs_write_buffer(interface->mipi_base, 0, cmd_index, cmd_len);
            if (ret <= 0) {
                LOG_HEXDUMP_ERR(cmd_index, cmd_len, "mipi_cmd failed:");
            }
        }

        if (delay)
            k_msleep(delay);
    }
    k_msleep(2);
}

static void hpm_display_mipi_panel_hs_transfer(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    struct hpm_interface_mipi_dsi *interface = config->interface;

    mipi_dsi_video_mode_hs_transfer_enable(interface->mipi_base);
}

static void hpm_display_isr(const struct device *dev)
{
    hpm_display_lcdc_isr(dev);
}

static int hpm_display_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;

    HPM_DUMP_DISPLAY_INTERFACE(DBG, config->interface);
    HPM_DUMP_DISPLAY_LCDC(DBG, &(config->lcdc));
    HPM_DUMP_DISPLAY_PANEL(DBG, &(config->panel));

    hpm_display_lcdc_clk_init(dev);
    hpm_display_pinctl_init(dev);
    hpm_display_gpio_init(dev);
    hpm_display_panel_reset(dev);
    hpm_display_pixelmux_router_config(dev);
    hpm_display_mipi_panel_host_init(dev);
    hpm_display_mipi_panel_phy_init(dev);
    hpm_display_mipi_panel_cmd_init(dev);
    hpm_display_mipi_panel_hs_transfer(dev);
    hpm_display_lcdc_init(dev);
    config->config_init(dev);
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