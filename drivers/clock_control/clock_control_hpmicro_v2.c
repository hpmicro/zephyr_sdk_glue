/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-04-29   HPMicro         first edition
 */

#define DT_DRV_COMPAT hpmicro_hpm_pllv2
#define CLOCK_NODE DT_NODELABEL(clk)

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <drivers/clock_control/hpmicro_clock_control.h>

#include <hpm_clock_drv.h>
#include <hpm_pllctlv2_drv.h>

struct clock_hpm_cfg {
	PLLCTLV2_Type *pll_base;
	SYSCTL_Type *sysctl_base;
	uint32_t sys_core;
	uint32_t sysctl_preset;
	uint32_t ram_up_time;
};

static int clock_control_hpm_on(const struct device *dev,
				clock_control_subsys_t sys)
{
	const struct clock_hpm_cfg *cfg = dev->config;
	uint32_t clk_name = *(uint32_t *)sys;

	clock_add_to_group(clk_name, cfg->sys_core);
	return 0;
}

static int clock_control_hpm_off(const struct device *dev,
				clock_control_subsys_t sys)
{
	const struct clock_hpm_cfg *cfg = dev->config;
	uint32_t clk_name = *(uint32_t *)sys;

	clock_remove_from_group(clk_name, cfg->sys_core);
	return 0;
}

static int clock_control_hpm_get_rate(const struct device *dev,
				    clock_control_subsys_t sys,
				    uint32_t *rate)
{
	ARG_UNUSED(dev);
	uint32_t clk_name = *(uint32_t *)sys;

	*rate = clock_get_frequency(clk_name);
	return 0;
}

static enum clock_control_status
clock_control_hpm_get_status(const struct device *dev,
			    clock_control_subsys_t sys)
{
	const struct clock_hpm_cfg *cfg = dev->config;
	uint32_t clk_name = *(uint32_t *)sys;

	if (clock_check_in_group(clk_name, cfg->sys_core))
		return CLOCK_CONTROL_STATUS_ON;

	return CLOCK_CONTROL_STATUS_OFF;
}


static const struct clock_hpm_cfg config = {
	.pll_base = (PLLCTLV2_Type *)DT_REG_ADDR_BY_NAME(CLOCK_NODE, pll),
	.sysctl_base = (SYSCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_NODE, sysctl),
	.sys_core = DT_PROP(CLOCK_NODE, clock_sys_core),
	.sysctl_preset = DT_PROP(CLOCK_NODE, sysctl_present),
	.ram_up_time = DT_PROP(CLOCK_NODE, ram_up_time),
};

static struct clock_control_driver_api clock_control_hpm_api = {
	.on = clock_control_hpm_on,
	.off = clock_control_hpm_off,
	.get_rate = clock_control_hpm_get_rate,
	.get_status = clock_control_hpm_get_status,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &clock_control_hpm_api);
