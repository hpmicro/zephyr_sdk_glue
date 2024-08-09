# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

zephyr_linker_sources_ifdef(CONFIG_HPM_BOOT_HEADER ROM_START SORT_KEY 0 common/boot_header.ld)
zephyr_linker_sources(ITCM_SECTION SORT_KEY 0 common/ISR.ld)