/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "display.h"
#include <hpm_pixelmux_drv.h>
#include <hpm_clock_drv.h>
#include <hpm_lvb_drv.h>

LOG_MODULE_REGISTER(display_hpm_lvds, CONFIG_DISPLAY_LOG_LEVEL);

#define DT_DRV_COMPAT hpmicro_hpm_display_lvds

struct hpm_lvds_cfg {
    int dual_lvds_phy;
};

struct hpm_interface_lvds {
    void *lvb_base;
    void *pixelmux_base;
    uint32_t lvb_clock_name;
    uint32_t lvb_clock_src;
    int lvds_channel;
    int rterm_enable;
    int amp_mv;
    int vcom_mv;
    int ch_mapping;
};

#define HPM_DUMP_DISPLAY_INTERFACE(level, interface)                                                               \
    LOG_##level("interface: lvds");                                                                                \
    LOG_##level("lvb_base      = 0x%x", (unsigned int)((struct hpm_interface_lvds *)interface)->lvb_base);         \
    LOG_##level("pixelmux_base = 0x%x", (unsigned int)((struct hpm_interface_lvds *)interface)->pixelmux_base);    \
    LOG_##level("lvds_channel  = %d", (unsigned int)((struct hpm_interface_lvds *)interface)->lvds_channel);       \
    LOG_##level("rterm_enable  = %d", (unsigned int)((struct hpm_interface_lvds *)interface)->rterm_enable);       \
    LOG_##level("amp_mv        = %d", (unsigned int)((struct hpm_interface_lvds *)interface)->amp_mv);             \
    LOG_##level("vcom_mv       = %d", (unsigned int)((struct hpm_interface_lvds *)interface)->vcom_mv);            \
    LOG_##level("ch_mapping    = %d", (unsigned int)((struct hpm_interface_lvds *)interface)->ch_mapping);

#define HPM_DUMP_DISPLAY_PANEL_PRIVATE(level, panel)                                                               \
    LOG_##level("dual_lvds_phy = %d", ((struct hpm_lvds_cfg *)(panel)->private)->dual_lvds_phy);

#define DISPLAY_DATA_PRIVATE(inst)                                                                                 \
    .private = NULL,

#define DISPLAY_PANEL_PRIVATE(inst)                                                                                \
    .private = &(struct hpm_lvds_cfg) {                                                                            \
        .dual_lvds_phy   = DT_INST_PROP_OR(inst, dual_lvds_phy, 0),                                                \
    }

#define DISPLAY_INTERFACE(inst)                                                                                    \
    .interface = &(struct hpm_interface_lvds) {                                                                    \
        .lvb_base       = (void *)DT_REG_ADDR_BY_NAME(DT_PARENT(INTERFACE_NODE_ID(inst)), lvb),                    \
        .pixelmux_base  = (void *)DT_REG_ADDR_BY_NAME(DT_PARENT(INTERFACE_NODE_ID(inst)), pixelmux),               \
        .lvb_clock_name = DT_CLOCKS_CELL_BY_IDX(DT_PARENT(INTERFACE_NODE_ID(inst)), 0, name),                      \
        .lvb_clock_src  = DT_CLOCKS_CELL_BY_IDX(DT_PARENT(INTERFACE_NODE_ID(inst)), 0, src),                       \
        .lvds_channel   = DT_REG_ADDR(INTERFACE_NODE_ID(inst)),                                                    \
        .rterm_enable   = DT_PROP(INTERFACE_NODE_ID(inst), rterm_enable),                                          \
        .amp_mv         = DT_PROP(INTERFACE_NODE_ID(inst), amp_mv),                                                \
        .vcom_mv        = DT_PROP(INTERFACE_NODE_ID(inst), vcom_mv),                                               \
        .ch_mapping     = DT_PROP(INTERFACE_NODE_ID(inst), ch_mapping),                                            \
    },

static int hpm_display_lcdc_clk_init(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    struct hpm_lvds_cfg *lvds_cfg = config->panel.private;
    const struct hpm_lcdc *lcdc = &config->lcdc;
    const struct hpm_display_timing *timing = &config->panel.timing;
    uint32_t freq_khz;
    uint32_t div;

    freq_khz = clock_get_frequency(lcdc->pll_clock_name) / 1000;

    if (lvds_cfg->dual_lvds_phy)
        div = (freq_khz + timing->pixel_clock_khz) / (timing->pixel_clock_khz * 2);
    else
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
    struct hpm_lvds_cfg *lvds_cfg = config->panel.private;
    struct hpm_interface_lvds *interface = config->interface;
    const struct hpm_lcdc *lcdc = &config->lcdc;

    /*
     * For convenience, Di0 must select lcdc0 and Di1 must select lcdc1.
     */
    if (lcdc->id == 0)
        pixelmux_lvb_di0_data_source_enable(pixelmux_lvb_di0_sel_lcdc0);
    else
        pixelmux_lvb_di1_data_source_enable(pixelmux_lvb_di1_sel_lcdc1);

    if (lvds_cfg->dual_lvds_phy) {
        pixelmux_config_tx_phy0_mode(pixelmux_tx_phy_mode_lvds);
        pixelmux_config_tx_phy1_mode(pixelmux_tx_phy_mode_lvds);
    } else {
        if (interface->lvds_channel == 0)
            pixelmux_config_tx_phy0_mode(pixelmux_tx_phy_mode_lvds);
        else
            pixelmux_config_tx_phy1_mode(pixelmux_tx_phy_mode_lvds);
    }

    return 0;
}

static void hpm_display_lvds_panel_lvb_init(const struct device *dev)
{
    const struct hpm_display_config *config = dev->config;
    struct hpm_interface_lvds *interface = config->interface;
    struct hpm_lvds_cfg *lvds_cfg = config->panel.private;
    const struct hpm_lcdc *lcdc = &config->lcdc;
    LVB_Type *lvb_base = interface->lvb_base;
    lvb_config_t lvb_config;
    lvb_ch_config_t lvb_ch_cfg;

    clock_add_to_group(interface->lvb_clock_name, 0);
    lvb_get_default_config(&lvb_config);

    if (lvds_cfg->dual_lvds_phy)
        lvb_config.split_mode_en = true;
    else
        lvb_config.split_mode_en = false;

    lvb_config.txclk_shift = lvb_txclk_shift_1100011;
    lvb_init(lvb_base, &lvb_config);

    /*
     * id represent lcdc and di. so :
     * if id == 0, it represent lcdc0 and di0 is selected.
     * if id == 1, it represent lcdc1 and di1 is selected.
     */
    if (lcdc->id == 0)
        lvb_ch_cfg.data_src = lvb_ch_data_source_di0;
    else
        lvb_ch_cfg.data_src = lvb_ch_data_source_di1;

    lvb_ch_cfg.map = (lvb_ch_mapping_t)interface->ch_mapping;

    if (lvds_cfg->dual_lvds_phy) {
        /*
         * In dual lvds mode, all channel must select same Di.
         */
        lvb_ch_config(lvb_base, lvb_ch_num_0, &lvb_ch_cfg);
        lvb_ch_enable(lvb_base, lvb_ch_num_0);
        lvb_ch_config(lvb_base, lvb_ch_num_1, &lvb_ch_cfg);
        lvb_ch_enable(lvb_base, lvb_ch_num_1);
    } else {
        if (interface->lvds_channel == 0) {
            lvb_ch_config(lvb_base, lvb_ch_num_0, &lvb_ch_cfg);
            lvb_ch_enable(lvb_base, lvb_ch_num_0);
        } else {
            lvb_ch_config(lvb_base, lvb_ch_num_1, &lvb_ch_cfg);
            lvb_ch_enable(lvb_base, lvb_ch_num_1);
        }
    }
}

static void hpm_display_lvds_panel_phy_init(const struct device *dev)
{
    struct hpm_display_data *data = dev->data;
    const struct hpm_display_config *config = dev->config;
    struct hpm_interface_lvds *interface = config->interface;
    struct hpm_lvds_cfg *lvds_cfg = config->panel.private;
    LVB_Type *lvb_base = interface->lvb_base;
    lvds_phy_clk_param_t param;
    uint32_t pixel_clk = data->pixel_clk_khz * 1000;

    if (lvds_cfg->dual_lvds_phy) {
        pixelmux_lvds_phy_calc_pll_cfg(pixel_clk / 2, true, &param);
        pixelmux_config_lvds_tx_phy0_clk(&param.reg);
        pixelmux_config_lvds_tx_phy1_clk(&param.reg);
    } else {
        pixelmux_lvds_phy_calc_pll_cfg(pixel_clk, false, &param);
        if (interface->lvds_channel == 0) {
            pixelmux_config_lvds_tx_phy0_clk(&param.reg);
        } else {
            pixelmux_config_lvds_tx_phy1_clk(&param.reg);
        }
    }

    lvb_lvds_phy_lane_config_t lvds_lane_cfg;
    lvb_lvds_phy_lane_get_default_config(&lvds_lane_cfg);
    lvds_lane_cfg.fvco_div4    = param.reg.data_rate_div4;
    lvds_lane_cfg.rterm_enable = interface->rterm_enable;
    lvds_lane_cfg.amp          = (lvb_lvds_lane_amp_t)interface->amp_mv;
    lvds_lane_cfg.vcom         = (lvb_lvds_lane_vcom_t)interface->vcom_mv;

    if (lvds_cfg->dual_lvds_phy) {
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx0, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx1, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx2, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx3, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_txck, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx0, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx1, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx2, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx3, &lvds_lane_cfg);
        lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_txck, &lvds_lane_cfg);
        lvb_lvds_phy0_poweron(lvb_base);
        lvb_lvds_phy1_poweron(lvb_base);

        while (lvb_lvds_phy_split_pll_is_lock(lvb_base) == false) {
        }
    } else {
        if (interface->lvds_channel == 0) {
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx0, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx1, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx2, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_tx3, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds0_txck, &lvds_lane_cfg);
            lvb_lvds_phy0_poweron(lvb_base);

            while (lvb_lvds_phy0_pll_is_lock(lvb_base) == false) {
            }
        } else {
            pixelmux_config_lvds_tx_phy1_clk(&param.reg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx0, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx1, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx2, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_tx3, &lvds_lane_cfg);
            lvb_lvds_phy_lane_init(lvb_base, lvb_lvds_lane_idx_lvds1_txck, &lvds_lane_cfg);
            lvb_lvds_phy1_poweron(lvb_base);

            while (lvb_lvds_phy1_pll_is_lock(lvb_base) == false) {
            }
        }
    }
}

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
    hpm_display_pinctl_init(dev);
    hpm_display_gpio_init(dev);
    hpm_display_panel_reset(dev);
    hpm_display_pixelmux_router_config(dev);
    hpm_display_lvds_panel_lvb_init(dev);
    hpm_display_lvds_panel_phy_init(dev);
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