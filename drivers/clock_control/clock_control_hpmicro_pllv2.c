/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-10-15   HPMicro         first edition
 */

#define CLOCK_CONTROLLER DT_NODELABEL(clk)
#define CLOCK_PLL_DIVISION_MAX (2)

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <drivers/clock_control/hpmicro_clock_control.h>

#include <hpm_clock_drv.h>
#include <hpm_pllctlv2_drv.h>
#include <hpm_sysctl_drv.h>

struct clock_div_cfg {
	uint8_t index;
	uint8_t prescale;
	uint32_t name;
};

struct clock_pllv2_cfg {
	PLLCTLV2_Type *pll_base;
	SYSCTL_Type *sysctl_base;
	struct clock_div_cfg *div[CLOCK_PLL_DIVISION_MAX];
	uint32_t freq;
	uint8_t index;
	uint8_t num_div;
};

static int clock_control_hpm_pll_on(const struct device *dev,
				clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	return 0;
}

static int clock_control_hpm_pll_off(const struct device *dev,
				clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	return 0;
}

static int clock_control_hpm_pll_get_rate(const struct device *dev,
				    clock_control_subsys_t sys,
				    uint32_t *rate)
{
	const struct clock_pllv2_cfg *cfg = dev->config;
	uint32_t index = *(uint32_t *)sys;

	*rate = pllctlv2_get_pll_freq_in_hz(cfg->base, index);
	return 0;
}

static int hpm_clock_pllv2_init(const struct device *dev)
{
	const struct clock_pllv2_cfg *cfg = dev->config;
	uint32_t status;

	sysctl_clock_set_preset(cfg->sysctl, sysctl_preset_0);
	status = pllctlv2_init_pll_with_freq(cfg->base, cfg->index, cfg->freq);

	if (status_success != status)
		return -ENOTSUP;

	for (int i = 0; i < cfg->num_div; i++) {
		pllctlv2_set_postdiv(cfg->base, cfg->index, cfg->div[i]->index, cfg->div[i]->prescale);		
	}
	return 0;
}

static struct clock_control_driver_api clock_control_hpm_pllv2_api = {
	.on = clock_control_hpm_pll_on,
	.off = clock_control_hpm_pll_off,
	.get_rate = clock_control_hpm_pll_get_rate,
};

#define DT_DRV_COMPAT hpmicro_hpm_pllv2_clock

#define PLL_DIVISION_CONFIG(n)	\
	static struct clock_div_cfg div_cfg_##n = {	\
		.index = DT_PROP_OR(n, div_i, 0),		\
		.prescale = DT_PROP_OR(n, div_p, 0),	\
		.name = DT_PROP(n, src_name),			\
	};

#define _PLL_DIVISION_PTR(n) &div_cfg_##n

#define PLL_DIVISION_PTR(n)		\
	COND_CODE_1(DT_NODE_EXISTS(n),					\
		    (_PLL_DIVISION_PTR(n)),				\
		    NULL)

#define HPM_PLL_DEFINE(n)		\
		DT_INST_FOREACH_CHILD(n, PLL_DIVISION_CONFIG);	\
														\
		static struct clock_pllv2_cfg clock_pllv2_cfg_##n = {	\
			.base = (PLLCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, pll),	\
			.sysctl = (SYSCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, sysctl),	\
			.freq = DT_INST_PROP(n, pll_frequency),	\
			.index = DT_INST_PROP(n, pll_index),	\
			.num_div = DT_INST_CHILD_NUM(n),	\
			.div = {					\
				[0] = PLL_DIVISION_PTR(DT_INST_CHILD(n, clk0)),	\
				[1] = PLL_DIVISION_PTR(DT_INST_CHILD(n, clk1)),	\
			},				\
		};		\
				\
		DEVICE_DT_INST_DEFINE(n,						\
			hpm_clock_pllv2_init, NULL, NULL, 	\
			&clock_pllv2_cfg_##n,					\
			PRE_KERNEL_1,						\
			CONFIG_CLOCK_CONTROL_INIT_PRIORITY,	\
			&clock_control_hpm_pllv2_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_PLL_DEFINE)
