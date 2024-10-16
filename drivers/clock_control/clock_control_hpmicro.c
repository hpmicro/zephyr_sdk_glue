/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-04-29   HPMicro         first edition
 */

#define CLOCK_CONTROLLER DT_NODELABEL(clk)

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <drivers/clock_control/hpmicro_clock_control.h>

#include <hpm_clock_drv.h>

struct clock_hpm_cfg {
	SYSCTL_Type *sysctl_base;
};

static int clock_control_hpm_on(const struct device *dev,
				clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	clock_name_t clk_name = *(clock_name_t *)sys;

/* sdk_glue not supported smp and low-power, put clocks in group0 */
	clock_add_to_group(clk_name, 0);
	return 0;
}

static int clock_control_hpm_off(const struct device *dev,
				clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	clock_name_t clk_name = *(clock_name_t *)sys;

	clock_remove_from_group(clk_name, 0);
	return 0;
}

static int clock_control_hpm_get_rate(const struct device *dev,
				    clock_control_subsys_t sys,
				    uint32_t *rate)
{
	ARG_UNUSED(dev);
	clock_name_t clk_name = *(clock_name_t *)sys;

	*rate = clock_get_frequency(clk_name);
	return 0;
}

static enum clock_control_status
clock_control_hpm_get_status(const struct device *dev,
			    clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	clock_name_t clk_name = *(clock_name_t *)sys;

	if (clock_check_in_group(clk_name, 0))
		return CLOCK_CONTROL_STATUS_ON;

	return CLOCK_CONTROL_STATUS_OFF;
}

static int hpm_clock_init(const struct device *dev)
{
	ARG_UNUSED(dev);	
    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);

    clock_set_source_divider(clock_ahb, clk_src_pll1_clk1, 2); /*200m hz*/
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 1);
	return 0;
}

static const struct clock_hpm_cfg config = {
	.sysctl_base = (SYSCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, sysctl),
};

static struct clock_control_driver_api clock_control_hpm_api = {
	.on = clock_control_hpm_on,
	.off = clock_control_hpm_off,
	.get_rate = clock_control_hpm_get_rate,
	.get_status = clock_control_hpm_get_status,
};

#define DT_DRV_COMPAT hpmicro_hpm_clock
DEVICE_DT_INST_DEFINE(0, hpm_clock_init, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &clock_control_hpm_api);
