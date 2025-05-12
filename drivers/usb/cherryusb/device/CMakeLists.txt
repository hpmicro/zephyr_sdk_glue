# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_CHERRYUSB_DEVICE_HPMICRO)
    zephyr_library()
endif() 

zephyr_compile_definitions_ifdef(CONFIG_CHERRYUSB_DEVICE_HPMICRO USBD_USE_CUSTOM_ISR=1)

zephyr_include_directories_ifdef(CONFIG_CHERRYUSB_DEVICE_HPMICRO ${HPM_SDK_DIR}/components/usb/device)
zephyr_library_sources_ifdef(CONFIG_CHERRYUSB_DEVICE_HPMICRO ${HPM_SDK_DIR}/components/usb/device/hpm_usb_device.c)

zephyr_library_sources_ifdef(CONFIG_CHERRYUSB_DEVICE_HPMICRO cherryusb_device_hpmicro.c)
