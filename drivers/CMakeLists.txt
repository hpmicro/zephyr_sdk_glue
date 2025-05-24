# Copyright (c) 2023-2025 HPMicro
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
add_subdirectory(display)

add_subdirectory(i2c)
add_subdirectory(video)
add_subdirectory(sdhc)
add_subdirectory(usb/udc)
if (CONFIG_CHERRYUSB_HPMICRO)
    add_subdirectory(usb/cherryusb)  
endif()
add_subdirectory(ethernet) 
