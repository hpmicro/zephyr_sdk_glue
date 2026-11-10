/*
 * Copyright (c) 2025 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT hpmicro_hpm_cherryusb

#include <soc.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/pinctrl.h>

#include "hpm_soc.h"
#include "hpm_clock_drv.h"
#include "hpm_usb_drv.h"
#include "usb_glue_hpm.h"
#include <drivers/clock_control/hpmicro_clock_control.h>

struct cherryusb_hpm_config {
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
	uint32_t reg_base;
	uint32_t clock_name;
	const struct pinctrl_dev_config *pincfg;
	bool use_internal_vbus;
	bool power_active_high;
	uint32_t power_stable_delay_ms;
};

static void cherryusb_hpm_isr(uint32_t base)
{
	if (HPM_USB0_BASE == base) {
		hpm_isr_usb0();
	} else {
#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
		hpm_isr_usb1();
#endif
	}
}

static int cherryusb_hpm_driver_preinit(const struct device *dev)
{
	const struct cherryusb_hpm_config *config = dev->config;

	clock_enable_name((clock_name_t)config->clock_name);

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	if (config->power_active_high) {
		usb_hcd_set_power_ctrl_polarity((USB_Type *)config->reg_base, true);
	}

	if (config->power_stable_delay_ms != 0U) {
		k_sleep(K_MSEC(config->power_stable_delay_ms));
	}

	if (config->use_internal_vbus) {
		usb_phy_using_internal_vbus((USB_Type *)config->reg_base);
	}

	return 0;
}

void hpm_usb_isr_enable(uint32_t base)
{
	const struct device *dev0 = DEVICE_DT_GET(DT_NODELABEL(cherryusb_usb0));
	const struct cherryusb_hpm_config *config0 = dev0->config;

#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
	const struct device *dev1 = DEVICE_DT_GET(DT_NODELABEL(cherryusb_usb1));
	const struct cherryusb_hpm_config *config1 = dev1->config;
#endif

	if (base == config0->reg_base) {
		config0->irq_enable_func();
	} else {
#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
		if (base == config1->reg_base) {
			config1->irq_enable_func();
		}
#endif
	}
}

void hpm_usb_isr_disable(uint32_t base)
{
	const struct device *dev0 = DEVICE_DT_GET(DT_NODELABEL(cherryusb_usb0));
	const struct cherryusb_hpm_config *config0 = dev0->config;

#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
	const struct device *dev1 = DEVICE_DT_GET(DT_NODELABEL(cherryusb_usb1));
	const struct cherryusb_hpm_config *config1 = dev1->config;
#endif

	if (base == config0->reg_base) {
		config0->irq_disable_func();
	} else {
#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb1))
		if (base == config1->reg_base) {
			config1->irq_disable_func();
		}
#endif
	}
}

#define CHERRYUSB_HPM_DEFINE(n)								\
	static void cherryusb_irq_enable_func##n(void)			\
	{														\
		IRQ_CONNECT(DT_INST_IRQN(n),						\
					DT_INST_IRQ(n, priority),				\
					cherryusb_hpm_isr,						\
					DT_INST_REG_ADDR(n), 0);				\
															\
		irq_enable(DT_INST_IRQN(n));						\
	}														\
															\
	static void cherryusb_irq_disable_func##n(void)			\
	{														\
		irq_disable(DT_INST_IRQN(n));						\
	}														\
															\
	PINCTRL_DT_INST_DEFINE(n);								\
															\
	static struct cherryusb_hpm_config priv_config_##n = {	\
		.reg_base = DT_INST_REG_ADDR(n),					\
		.clock_name = DT_INST_PROP(n, clk_name),			\
		.irq_enable_func = cherryusb_irq_enable_func##n,			\
		.irq_disable_func = cherryusb_irq_disable_func##n,			\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
		.use_internal_vbus = DT_INST_PROP(n, use_internal_vbus),			\
		.power_active_high = DT_INST_PROP(n, power_active_high),		\
		.power_stable_delay_ms = DT_INST_PROP(n, power_stable_delay_ms),	\
	};																\
																	\
	DEVICE_DT_INST_DEFINE(n, cherryusb_hpm_driver_preinit, NULL,		\
						NULL, &priv_config_##n,							\
						POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
						NULL);

DT_INST_FOREACH_STATUS_OKAY(CHERRYUSB_HPM_DEFINE)
