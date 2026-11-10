# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(
  start.S
  soc.c
  ../../../../sdk_env/hpm_sdk/soc/HPM6E00/HPM6E80/hpm_otp_drv.c
)

if(CONFIG_SOC_ANDES_V5_HWDSP)
  zephyr_cc_option(-mext-dsp)
endif()

if(CONFIG_SOC_SERIES_HPM6E00)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
