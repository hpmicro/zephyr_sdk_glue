# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0


zephyr_library()

zephyr_linker_sources(RODATA cherryusb_host.ld)

zephyr_compile_definitions(USBH_USE_CUSTOM_ISR=1)

zephyr_library_sources(cherryusb_host_hpmicro.c)
