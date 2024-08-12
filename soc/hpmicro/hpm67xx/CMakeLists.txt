# Copyright (c) 2023 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(
  start.S
  soc.c
)

if(CONFIG_SOC_ANDES_V5_HWDSP)
  zephyr_cc_option(-mext-dsp)
endif()

if(CONFIG_SOC_SERIES_HPM67XX)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
