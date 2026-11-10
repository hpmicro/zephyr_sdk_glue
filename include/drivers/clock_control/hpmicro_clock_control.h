/*
 * Copyright 2024 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_

#if defined(CONFIG_SOC_SERIES_HPM5100)
#include <dt-bindings/clock/hpm5151-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM6700)
#include <dt-bindings/clock/hpm6750-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM5300)
#include <dt-bindings/clock/hpm5361-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM6200)
#include <dt-bindings/clock/hpm6280-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM6300)
#include <dt-bindings/clock/hpm6360-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM6800)
#include <dt-bindings/clock/hpm6880-clocks.h>
#endif /* CONFIG_SOC_SERIES_HPMXXXX */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <hpm_clock_drv.h>

struct hpm_clock_dt_spec {
	const struct device *dev;
	clock_name_t name;
	clk_src_t src;
	uint32_t div;
};

#define CLOCK_DT_SPEC_INST_GET(inst) \
	{ \
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)), \
		.name = (clock_name_t)DT_INST_CLOCKS_CELL(inst, name), \
		.src = (clk_src_t)DT_INST_CLOCKS_CELL(inst, src), \
		.div = DT_INST_CLOCKS_CELL(inst, div), \
	}

#define CLOCK_DT_SPEC_INST_GET_BY_IDX(inst, idx) \
	{ \
		.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(inst, idx)), \
		.name = (clock_name_t)DT_INST_CLOCKS_CELL_BY_IDX(inst, idx, name), \
		.src = (clk_src_t)DT_INST_CLOCKS_CELL_BY_IDX(inst, idx, src), \
		.div = DT_INST_CLOCKS_CELL_BY_IDX(inst, idx, div), \
	}

static inline void clock_enable_name_in_group(clock_name_t name, uint32_t group)
{
	clock_add_to_group(name, group);
}

static inline void clock_enable_name(clock_name_t name)
{
	clock_enable_name_in_group(name, 0);
}

static inline hpm_stat_t clock_configure_enable_in_group(const struct hpm_clock_dt_spec *spec,
							 uint32_t group)
{
	hpm_stat_t status = status_success;

	if (GET_CLK_SRC_GROUP_FROM_NAME(spec->name) == CLK_SRC_GROUP_COMMON) {
		status = clock_set_source_divider(spec->name, spec->src, spec->div);
	}

	clock_enable_name_in_group(spec->name, group);
	return status;
}

static inline hpm_stat_t clock_configure_enable(const struct hpm_clock_dt_spec *spec)
{
	return clock_configure_enable_in_group(spec, 0);
}

/**
 * @brief Obtain a reference to the HPMicro clock controller.
 *
 */
// #define HPMICRO_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(clk))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_ */
