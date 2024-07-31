# Copyright (c) 2023-2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

zephyr_include_directories(${ZEPHYR_BASE}/drivers)

add_subdirectory(clock_control)
add_subdirectory(dma)
add_subdirectory(serial)
add_subdirectory(pinctrl)
add_subdirectory(gpio)
add_subdirectory(can)
add_subdirectory(spi)
add_subdirectory(pwm)
add_subdirectory(adc)