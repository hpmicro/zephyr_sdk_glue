# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(
  start.S
  soc.c
  ../../../../sdk_env/hpm_sdk/soc/HPM5100/HPM5151/hpm_otp_drv.c
)

if(CONFIG_SOC_SERIES_HPM5100)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
