/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_mipi_csi


#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/video.h>
#include <soc.h>
#include <hpm_clock_drv.h>
#include <hpm_l1c_drv.h>
#include <hpm_mipi_csi_drv.h>
#include <hpm_mipi_csi_phy_drv.h>
#include <hpm_pixelmux_drv.h>
#include <mipi_csi_hpmicro.h>

struct hpmicro_mipi_csi_config {
	MIPI_CSI_Type *base;
	MIPI_CSI_PHY_Type *phy_base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	void (*irq_config_func)(const struct device *dev);
};

struct hpmicro_mipi_csi_data {
	struct k_mutex mutex;
	uint32_t pixel_format;
};


#define DEV_BASE(dev) (((struct hpmicro_mipi_csi_config *)(dev->config))->base)

static void hpmicro_mipi_csi_isr(const struct device *dev)
{
	(void)dev;
}

static int hpmicro_mipi_csi_init(const struct device *dev, uint8_t cam_index)
{
	const struct hpmicro_mipi_csi_config *cfg = dev->config;
	struct hpmicro_mipi_csi_data *data = dev->data;

	clock_set_source_divider(cfg->clock_name, cfg->clock_src, cfg->clock_div);
	clock_add_to_group(cfg->clock_name, 0);
	uint32_t freq = clock_get_frequency(cfg->clock_name);
	printk("mipi csi clk freq: %u Hz\n", freq);
	if (cam_index == CAM0_DEVICE) {
		if (cfg->base == HPM_MIPI_CSI0) {
			/* cam0 -> mipi_csi0 */
			pixelmux_cam0_data_source_enable(pixelmux_cam0_sel_mipi_csi0);
		} else if (cfg->base == HPM_MIPI_CSI1){
			pixelmux_cam0_data_source_enable(pixelmux_cam0_sel_mipi_csi1);
		} else {
			return -EINVAL;
		}
	} else if (cam_index == CAM1_DEVICE) {
		if (cfg->base == HPM_MIPI_CSI0) {
			/* cam1-> mipi_csi0 */
			pixelmux_cam1_data_source_enable(pixelmux_cam1_sel_mipi_csi0);
		} else if (cfg->base == HPM_MIPI_CSI1) {
			pixelmux_cam1_data_source_enable(pixelmux_cam1_sel_mipi_csi1);
		}  else {
			return -EINVAL;
		}
	} else {
		return -EINVAL;
	}
	if (cfg->phy_base == HPM_MIPI_CSI_PHY0) {
		pixelmux_config_rx_phy0_mode(pixelmux_rx_phy_mode_mipi);
	} else if (cfg->phy_base == HPM_MIPI_CSI_PHY1) {
		pixelmux_config_rx_phy1_mode(pixelmux_rx_phy_mode_mipi);
	} else {
		return -EINVAL;
	}

	mipi_csi_phy_powerdown(cfg->base);

	/*phy mode: mipi */
	mipi_csi_phy_config_t mipi_phy_cfg;
	mipi_csi_phy_default_config(&mipi_phy_cfg);
	mipi_csi_phy_init(cfg->phy_base, &mipi_phy_cfg);
	mipi_csi_phy_poweron(cfg->base);

	mipi_csi_config_t mipi_cfg;
	mipi_csi_get_defconfig(&mipi_cfg);
	mipi_cfg.lanes = 2;
#if (CONFIG_DATA_TYPE_YUV == 0)
	mipi_cfg.data_type = mipi_csi_data_type_rgb565;
#else
	mipi_cfg.data_type = mipi_csi_data_type_yuv422_8bit;
#endif
	data->pixel_format = VIDEO_PIX_FMT_RGB565;
	mipi_csi_init(cfg->base, &mipi_cfg);


	/* Initialize mutex and semaphore */
	k_mutex_init(&data->mutex);
	/* Configure IRQ */
	cfg->irq_config_func(dev);
	return 0;
}

static int set_format(const struct device *dev, uint32_t pixel_format)
{
	const struct hpmicro_mipi_csi_config *cfg = dev->config;
	struct hpmicro_mipi_csi_data *data = dev->data;
	mipi_csi_config_t mipi_cfg;
	if (pixel_format == data->pixel_format) {
		return 0;
	}
	mipi_csi_get_defconfig(&mipi_cfg);
	mipi_cfg.lanes = 2;
	switch (pixel_format) {
	case VIDEO_PIX_FMT_RGB565:
		mipi_cfg.data_type = mipi_csi_data_type_rgb565;
	case VIDEO_PIX_FMT_YUYV:
		mipi_cfg.data_type = mipi_csi_data_type_yuv422_8bit;
	default:
		return -EINVAL;
	}
	mipi_csi_init(cfg->base, &mipi_cfg);
	data->pixel_format = pixel_format;
	return 0;
}

static int get_format(const struct device *dev, uint32_t *pixel_format)
{
	struct hpmicro_mipi_csi_data *data = dev->data;
	(*pixel_format) = data->pixel_format;
	return 0;
}

static const struct hpmicro_mipi_csi_driver_api mipi_csi_driver_api = {
	.init = hpmicro_mipi_csi_init,
	.set_pixel_format = set_format,
	.get_pixel_format = get_format,
};


static int _mipi_csi_init(const struct device *dev)
{
	(void)dev;
	return 0;
}

#define HPM_MIPI_CSI_INIT(idx)						      \
									      \
static void hpmicro_mipi_csi_config_##idx(const struct device *dev);	      \
									      \
static const struct hpmicro_mipi_csi_config mipi_csi_cfg_##idx = {		      \
	.base =	 (MIPI_CSI_Type *) DT_INST_REG_ADDR(idx),		      \
	.phy_base = (MIPI_CSI_PHY_Type *)DT_INST_PROP_BY_IDX(idx, mipi_csi_phy_reg, 0), \
	.irq_config_func = hpmicro_mipi_csi_config_##idx,		      \
	.clock_name = DT_INST_CLOCKS_CELL(idx, name),	\
	.clock_src = DT_INST_CLOCKS_CELL(idx, src),	\
	.clock_div = DT_INST_CLOCKS_CELL(idx, div),	\
};									      \
									      \
static struct hpmicro_mipi_csi_data mipi_csi_data_##idx;			              \
									      \
DEVICE_DT_INST_DEFINE(idx,						      \
		    _mipi_csi_init,					      \
		    NULL,						      \
		    &mipi_csi_data_##idx, &mipi_csi_cfg_##idx,			      \
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		      \
		    &mipi_csi_driver_api);						      \
									      \
static void hpmicro_mipi_csi_config_##idx(const struct device *dev)		      \
{									      \
	IRQ_CONNECT(DT_INST_IRQN(idx),					      \
		    DT_INST_IRQ(idx, priority),				      \
		    hpmicro_mipi_csi_isr, DEVICE_DT_INST_GET(idx), 0);	      \
									      \
	irq_enable(DT_INST_IRQN(idx));					      \
}

DT_INST_FOREACH_STATUS_OKAY(HPM_MIPI_CSI_INIT);
