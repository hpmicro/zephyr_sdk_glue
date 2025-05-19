# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0


zephyr_library()

zephyr_compile_definitions(CONFIG_CHERRYUSB_CUSTOM_IRQ_HANDLER=1)

zephyr_include_directories_ifdef(CONFIG_CHERRYUSB_DEVICE ${HPM_SDK_DIR}/components/usb/device)
zephyr_library_sources_ifdef(CONFIG_CHERRYUSB_DEVICE ${HPM_SDK_DIR}/components/usb/device/hpm_usb_device.c)
zephyr_library_sources(cherryusb_hpmicro.c)
