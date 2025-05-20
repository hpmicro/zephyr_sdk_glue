/*
 * Copyright 2024 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_

#if defined(CONFIG_SOC_SERIES_HPM67XX)
#include <dt-bindings/clock/hpm6750-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM53XX)
#include <dt-bindings/clock/hpm5361-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM62XX)
#include <dt-bindings/clock/hpm6280-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM63XX)
#include <dt-bindings/clock/hpm6360-clocks.h>
#elif defined(CONFIG_SOC_SERIES_HPM68XX)
#include <dt-bindings/clock/hpm6880-clocks.h>
#endif /* CONFIG_SOC_SERIES_HPMXXXX */
#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the HPMicro clock controller.
 *
 */
// #define HPMICRO_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(clk))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_ */
