# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_CHERRYUSB_HOST_HPMICRO)
    zephyr_library()
endif() 

zephyr_linker_sources_ifdef(CONFIG_CHERRYUSB_HOST_HPMICRO RODATA cherryusb_host.ld)

zephyr_compile_definitions_ifdef(CONFIG_CHERRYUSB_HOST_HPMICRO USBH_USE_CUSTOM_ISR=1)

zephyr_library_sources_ifdef(CONFIG_CHERRYUSB_HOST_HPMICRO cherryusb_host_hpmicro.c)
