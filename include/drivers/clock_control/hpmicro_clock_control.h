/*
 * Copyright 2024 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_

#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the HPMicro clock controller.
 *
 */
#define HPMICRO_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(clk))

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_HPMICRO_H_ */
