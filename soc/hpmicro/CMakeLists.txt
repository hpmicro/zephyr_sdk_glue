# Copyright (c) 2024-2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

zephyr_linker_sources_ifdef(CONFIG_HPM_BOOT_HEADER ROM_START SORT_KEY $ common/boot_header.ld)
zephyr_linker_sources_ifdef(CONFIG_HPM_BOOT_HEADER ROM_START SORT_KEY 0x0entry common/entry.ld)

zephyr_linker_sources_ifdef(CONFIG_BOOTLOADER_MCUBOOT ROM_START SORT_KEY ! $ENV{ZEPHYR_BASE}/arch/common/rom_start_address.ld)
zephyr_linker_sources_ifdef(CONFIG_BOOTLOADER_MCUBOOT ROM_START SORT_KEY $ $ENV{ZEPHYR_BASE}/arch/common/fill_with_zeros.ld)
zephyr_linker_sources_ifdef(CONFIG_BOOTLOADER_MCUBOOT ROM_START SORT_KEY + $ENV{ZEPHYR_BASE}/arch/common/rom_start_offset.ld)
zephyr_linker_sources_ifdef(CONFIG_BOOTLOADER_MCUBOOT ROM_START SORT_KEY - common/entry.ld)
zephyr_linker_sources(ITCM_SECTION SORT_KEY 0 common/ISR.ld)
zephyr_linker_sources_ifdef(CONFIG_INIT_EXT_RAM SECTIONS common/boot_stack.ld)

# Use the HPM-specific nocache output section. HPM51/HPM53 only need normal
# word alignment because DLM is inherently uncached. PMA-based series need a
# 4-KiB minimum, naturally aligned power-of-two region.
if(CONFIG_NOCACHE_MEMORY)
  set(zephyr_nocache_ld ${ZEPHYR_BASE}/arch/common/nocache.ld)
  file(RELATIVE_PATH zephyr_nocache_relpath ${ZEPHYR_BASE}/include ${zephyr_nocache_ld})
  get_property(zephyr_nocache_snippet GLOBAL
               PROPERTY "snippet_files_used_${zephyr_nocache_relpath}")
  if(NOT zephyr_nocache_snippet)
    message(FATAL_ERROR "Zephyr nocache linker source was not registered")
  endif()

  file(STRINGS ${zephyr_nocache_snippet} linker_snippet_lines)
  list(FILTER linker_snippet_lines EXCLUDE REGEX "${zephyr_nocache_relpath}")
  string(REPLACE ";" "\n;" linker_snippet_lines "${linker_snippet_lines}")
  file(WRITE ${zephyr_nocache_snippet} ${linker_snippet_lines} "\n")

  zephyr_linker_sources(RAM_SECTIONS common/nocache.ld)
endif()
