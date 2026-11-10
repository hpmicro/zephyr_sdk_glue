/*
 * Copyright (c) 2024-2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#define CLOCK_CONTROLLER DT_NODELABEL(clk)

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>

#include <drivers/clock_control/hpmicro_clock_control.h>
#include <hpm_clock_drv.h>

#if defined(CONFIG_CLOCK_CONTROL_HPMICRO_PLLV1)
#include <hpm_pllctl_drv.h>
#endif

#if defined(CONFIG_CLOCK_CONTROL_HPMICRO_PLLV2)
#include <hpm_pllctlv2_drv.h>
#endif

#if defined(CONFIG_CLOCK_CONTROL_HPMICRO_CPU)
#include <hpm_soc.h>
#include <hpm_sysctl_drv.h>
#endif

struct hpm_clock_config {
	SYSCTL_Type *sysctl_base;
};

static int hpm_clock_on(const struct device *dev, clock_control_subsys_t sys)
{
	clock_name_t clock_name = (clock_name_t)(uintptr_t)sys;

	ARG_UNUSED(dev);

	/* SMP and low power are not supported yet, so use clock group 0. */
	clock_add_to_group(clock_name, 0);
	return 0;
}

static int hpm_clock_off(const struct device *dev, clock_control_subsys_t sys)
{
	clock_name_t clock_name = (clock_name_t)(uintptr_t)sys;

	ARG_UNUSED(dev);
	clock_remove_from_group(clock_name, 0);
	return 0;
}

static int hpm_clock_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	clock_name_t clock_name = (clock_name_t)(uintptr_t)sys;

	ARG_UNUSED(dev);
	*rate = clock_get_frequency(clock_name);
	return 0;
}

static enum clock_control_status hpm_clock_get_status(const struct device *dev,
						      clock_control_subsys_t sys)
{
	clock_name_t clock_name = (clock_name_t)(uintptr_t)sys;

	ARG_UNUSED(dev);
	return clock_check_in_group(clock_name, 0) ? CLOCK_CONTROL_STATUS_ON
						   : CLOCK_CONTROL_STATUS_OFF;
}

static int hpm_clock_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static const struct hpm_clock_config hpm_clock_config = {
	.sysctl_base = (SYSCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, sysctl),
};

static const struct clock_control_driver_api hpm_clock_api = {
	.on = hpm_clock_on,
	.off = hpm_clock_off,
	.get_rate = hpm_clock_get_rate,
	.get_status = hpm_clock_get_status,
};

#define DT_DRV_COMPAT hpmicro_hpm_clock

DEVICE_DT_INST_DEFINE(0, hpm_clock_init, NULL, NULL, &hpm_clock_config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &hpm_clock_api);

#undef DT_DRV_COMPAT

#if defined(CONFIG_CLOCK_CONTROL_HPMICRO_PLLV1)

#define HPM_PLLV1_DIVISION_MAX 2

struct hpm_pllv1_div_config {
	uint8_t index;
	uint8_t prescale;
	uint32_t name;
};

struct hpm_pllv1_config {
	PLLCTL_Type *base;
	struct hpm_pllv1_div_config *div[HPM_PLLV1_DIVISION_MAX];
	uint32_t frequency;
	uint8_t index;
	uint8_t num_dividers;
	bool integer_mode;
};

static int hpm_pllv1_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct hpm_pllv1_config *config = dev->config;
	uint32_t index = *(uint32_t *)sys;

	pllctl_pll_poweron(config->base, index);
	return 0;
}

static int hpm_pllv1_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct hpm_pllv1_config *config = dev->config;
	uint32_t index = *(uint32_t *)sys;

	pllctl_pll_powerdown(config->base, index);
	return 0;
}

static int hpm_pllv1_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	const struct hpm_pllv1_config *config = dev->config;
	uint32_t index = *(uint32_t *)sys;

	*rate = pllctl_get_pll_freq_in_hz(config->base, index);
	return 0;
}

static int hpm_pllv1_init(const struct device *dev)
{
	const struct hpm_pllv1_config *config = dev->config;
	uint32_t status;

	if (config->integer_mode) {
		status = pllctl_init_int_pll_with_freq(config->base, config->index,
						       config->frequency);
	} else {
		status = pllctl_init_frac_pll_with_freq(config->base, config->index,
							config->frequency);
	}

	if (status != status_success) {
		return -ENOTSUP;
	}

	for (int i = 0; i < config->num_dividers; i++) {
		pllctl_set_div(config->base, config->index, config->div[i]->index,
			       config->div[i]->prescale);
	}

	return 0;
}

static const struct clock_control_driver_api hpm_pllv1_api = {
	.on = hpm_pllv1_on,
	.off = hpm_pllv1_off,
	.get_rate = hpm_pllv1_get_rate,
};

#define DT_DRV_COMPAT hpmicro_hpm_pll_clock

#define HPM_PLLV1_DIV_CONFIG(node_id)                                                              \
	static struct hpm_pllv1_div_config hpm_pllv1_div_config_##node_id = {                      \
		.index = DT_PROP(node_id, div_i),                                                  \
		.prescale = DT_PROP(node_id, div_p),                                               \
		.name = DT_PROP(node_id, src_name),                                                \
	};

#define HPM_PLLV1_DIV_PTR(node_id) &hpm_pllv1_div_config_##node_id,

#define HPM_PLLV1_DEFINE(inst)                                                                     \
	DT_INST_FOREACH_CHILD(inst, HPM_PLLV1_DIV_CONFIG)                                          \
	static struct hpm_pllv1_config hpm_pllv1_config_##inst = {                                 \
		.base = (PLLCTL_Type *)DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, pll),                 \
		.frequency = DT_INST_PROP(inst, pll_frequency),                                    \
		.index = DT_INST_PROP(inst, pll_index),                                            \
		.num_dividers = DT_INST_CHILD_NUM(inst),                                           \
		.integer_mode = DT_INST_PROP(inst, integer_mode),                                  \
		.div = {DT_INST_FOREACH_CHILD(inst, HPM_PLLV1_DIV_PTR)},                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, hpm_pllv1_init, NULL, NULL, &hpm_pllv1_config_##inst,          \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &hpm_pllv1_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_PLLV1_DEFINE)

#undef DT_DRV_COMPAT

#endif /* CONFIG_CLOCK_CONTROL_HPMICRO_PLLV1 */

#if defined(CONFIG_CLOCK_CONTROL_HPMICRO_PLLV2)

#define HPM_PLLV2_DIVISION_MAX 4

struct hpm_pllv2_div_config {
	uint8_t index;
	uint8_t prescale;
	uint32_t name;
};

struct hpm_pllv2_config {
	PLLCTLV2_Type *base;
	struct hpm_pllv2_div_config *div[HPM_PLLV2_DIVISION_MAX];
	uint32_t frequency;
	uint8_t index;
	uint8_t num_dividers;
};

static int hpm_pllv2_on(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	return 0;
}

static int hpm_pllv2_off(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	return 0;
}

static int hpm_pllv2_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	const struct hpm_pllv2_config *config = dev->config;
	uint32_t index = *(uint32_t *)sys;

	*rate = pllctlv2_get_pll_freq_in_hz(config->base, index);
	return 0;
}

static int hpm_pllv2_init(const struct device *dev)
{
	const struct hpm_pllv2_config *config = dev->config;
	uint32_t status;

	status = pllctlv2_init_pll_with_freq(config->base, config->index, config->frequency);
	if (status != status_success) {
		return -ENOTSUP;
	}

	for (int i = 0; i < config->num_dividers; i++) {
		pllctlv2_set_postdiv(config->base, config->index, config->div[i]->index,
				     config->div[i]->prescale);
	}

	return 0;
}

static const struct clock_control_driver_api hpm_pllv2_api = {
	.on = hpm_pllv2_on,
	.off = hpm_pllv2_off,
	.get_rate = hpm_pllv2_get_rate,
};

#define DT_DRV_COMPAT hpmicro_hpm_pllv2_clock

#define HPM_PLLV2_DIV_CONFIG(node_id)                                                              \
	static struct hpm_pllv2_div_config hpm_pllv2_div_config_##node_id = {                      \
		.index = DT_PROP(node_id, div_i),                                                  \
		.prescale = DT_PROP(node_id, div_p),                                               \
		.name = DT_PROP(node_id, src_name),                                                \
	};

#define HPM_PLLV2_DIV_PTR(node_id) &hpm_pllv2_div_config_##node_id,

#define HPM_PLLV2_DEFINE(inst)                                                                     \
	DT_INST_FOREACH_CHILD(inst, HPM_PLLV2_DIV_CONFIG)                                          \
	static struct hpm_pllv2_config hpm_pllv2_config_##inst = {                                 \
		.base = (PLLCTLV2_Type *)DT_REG_ADDR_BY_NAME_OR(                                   \
			CLOCK_CONTROLLER, pllv2, DT_REG_ADDR_BY_NAME(CLOCK_CONTROLLER, pll)),      \
		.frequency = DT_INST_PROP(inst, pll_frequency),                                    \
		.index = DT_INST_PROP(inst, pll_index),                                            \
		.num_dividers = DT_INST_CHILD_NUM(inst),                                           \
		.div = {DT_INST_FOREACH_CHILD(inst, HPM_PLLV2_DIV_PTR)},                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, hpm_pllv2_init, NULL, NULL, &hpm_pllv2_config_##inst,          \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &hpm_pllv2_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_PLLV2_DEFINE)

#undef DT_DRV_COMPAT

#endif /* CONFIG_CLOCK_CONTROL_HPMICRO_PLLV2 */

#if defined(CONFIG_CLOCK_CONTROL_HPMICRO_CPU)

struct hpm_cpu_clock_config {
	clock_name_t name;
	uint8_t pll;
	uint8_t output;
	uint16_t cpu_divider;
	uint8_t axi_divider;
	uint8_t ahb_divider;
};

static const clock_source_t hpm_cpu_clock_sources[][2] = {
	{clock_source_pll0_clk0, clock_source_pll0_clk0},
	{clock_source_pll1_clk0, clock_source_pll1_clk1},
};

static const clk_src_t hpm_clock_sources[][2] = {
	{clk_src_pll0_clk0, clk_src_pll0_clk0},
	{clk_src_pll1_clk0, clk_src_pll1_clk1},
};

static int hpm_cpu_clock_get_rate(const struct device *dev, clock_control_subsys_t sys,
				  uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	*rate = clock_get_frequency(clock_cpu0);
	return 0;
}

static int hpm_cpu_clock_init(const struct device *dev)
{
	const struct hpm_cpu_clock_config *config = dev->config;
	clock_source_t source;
	hpm_stat_t status;

	if ((config->pll >= ARRAY_SIZE(hpm_cpu_clock_sources)) ||
	    (config->output >= ARRAY_SIZE(hpm_cpu_clock_sources[0]))) {
		return -EINVAL;
	}

	source = hpm_cpu_clock_sources[config->pll][config->output];
	if (config->ahb_divider != 0U) {
#if defined(CONFIG_SOC_SERIES_HPM5100) || defined(CONFIG_SOC_SERIES_HPM5300)
		status = sysctl_config_cpu0_domain_clock(HPM_SYSCTL, source, config->cpu_divider,
							 config->ahb_divider);
#elif defined(CONFIG_SOC_SERIES_HPM6200) || defined(CONFIG_SOC_SERIES_HPM6300)
		status = sysctl_config_cpu0_domain_clock(HPM_SYSCTL, source, config->cpu_divider,
							 config->axi_divider, config->ahb_divider);
#else
		return -ENOTSUP;
#endif
	} else {
		if ((config->pll >= ARRAY_SIZE(hpm_clock_sources)) ||
		    (config->output >= ARRAY_SIZE(hpm_clock_sources[0]))) {
			return -EINVAL;
		}

		status = clock_set_source_divider(config->name,
						  hpm_clock_sources[config->pll][config->output],
						  config->cpu_divider);
	}

	if ((status == status_success) && (config->name == clock_cpu0)) {
		clock_update_core_clock();
	}

	return status == status_success ? 0 : -EIO;
}

static const struct clock_control_driver_api hpm_cpu_clock_api = {
	.get_rate = hpm_cpu_clock_get_rate,
};

#define HPM_CPU_CLOCK_DEFINE(node_id)                                                              \
	static const struct hpm_cpu_clock_config hpm_cpu_clock_config_##node_id = {                \
		.name = DT_PROP(node_id, node_name),                                               \
		.pll = DT_PROP(DT_CLOCKS_CTLR(node_id), pll_index),                                \
		.output = DT_CLOCKS_CELL(node_id, clk_index),                                      \
		.cpu_divider = DT_CLOCKS_CELL(node_id, div),                                       \
		.axi_divider = DT_PROP_OR(node_id, axi_divider, 0),                                \
		.ahb_divider = DT_PROP_OR(node_id, ahb_divider, 0),                                \
	};                                                                                         \
	DEVICE_DT_DEFINE(node_id, hpm_cpu_clock_init, NULL, NULL, &hpm_cpu_clock_config_##node_id, \
			 PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_HPMICRO_CPU_INIT_PRIORITY,             \
			 &hpm_cpu_clock_api);

DT_FOREACH_STATUS_OKAY(hpmicro_hpm_cpu_clock, HPM_CPU_CLOCK_DEFINE)
DT_FOREACH_STATUS_OKAY(hpmicro_hpm_second_clock, HPM_CPU_CLOCK_DEFINE)

#endif /* CONFIG_CLOCK_CONTROL_HPMICRO_CPU */
