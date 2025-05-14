# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0


zephyr_library()

zephyr_compile_definitions(USBD_USE_CUSTOM_ISR=1)

zephyr_include_directories(${HPM_SDK_DIR}/components/usb/device)
zephyr_library_sources(${HPM_SDK_DIR}/components/usb/device/hpm_usb_device.c)

zephyr_library_sources(cherryusb_device_hpmicro.c)
