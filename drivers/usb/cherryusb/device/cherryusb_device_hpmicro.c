/*
 * Copyright (c) 2025 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT hpmicro_hpm_cherryusb_device

#include <soc.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/pinctrl.h>

#include "hpm_clock_drv.h"
#include "usbd_core.h"

struct cherryusb_hpm_config {
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	uint32_t reg_base;
	uint32_t clock_name;
	const struct pinctrl_dev_config *pincfg;
};

static void cherryusb_device_hpm_isr(uint32_t base)
{
	if (g_usbdev_bus[0].reg_base == base) {
		USBD_IRQHandler(0);
	} else {
		USBD_IRQHandler(1);
	}
}

static int cherryusb_hpm_driver_preinit(const struct device *dev)
{
	const struct cherryusb_hpm_config *config = dev->config;

	clock_add_to_group((clock_name_t)(config->clock_name), 0);

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	config->irq_enable_func(dev);

	return 0;
}

#define CHERRYUSB_DEVICE_DEFINE(n)							\
	static void cherryusb_irq_enable_func##n(const struct device *dev)			\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(n),						\
					DT_INST_IRQ(n, priority),					\
					cherryusb_device_hpm_isr,						\
					DT_INST_REG_ADDR(n), 0);					\
											\
		irq_enable(DT_INST_IRQN(n));						\
	}										\
											\
	static void cherryusb_irq_disable_func##n(const struct device *dev)			\
	{										\
		irq_disable(DT_INST_IRQN(n));						\
	}										\
											\
	PINCTRL_DT_INST_DEFINE(n);							\
											\
	static struct cherryusb_hpm_config priv_config_##n = {				\
		.reg_base = DT_INST_REG_ADDR(n),					\
		.clock_name = DT_INST_PROP(n, clk_name),		\
		.irq_enable_func = cherryusb_irq_enable_func##n,					\
		.irq_disable_func = cherryusb_irq_disable_func##n,				\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
	};										\
											\
	DEVICE_DT_INST_DEFINE(n, cherryusb_hpm_driver_preinit, NULL,				\
						NULL, &priv_config_##n,				\
						POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
						NULL);

DT_INST_FOREACH_STATUS_OKAY(CHERRYUSB_DEVICE_DEFINE)
