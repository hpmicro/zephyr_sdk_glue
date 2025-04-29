# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_include_directories_ifdef(CONFIG_UDC_HPMICRO ${HPM_SDK_DIR}/components/usb/device)
zephyr_library_sources_ifdef(CONFIG_UDC_HPMICRO ${HPM_SDK_DIR}/components/usb/device/hpm_usb_device.c)
zephyr_library_sources_ifdef(CONFIG_UDC_HPMICRO udc_hpmicro.c)
