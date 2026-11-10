# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_sources(
  start.S
  soc.c
)

zephyr_sources_ifdef(CONFIG_SOC_HPM5301 ../../../../sdk_env/hpm_sdk/soc/HPM5300/HPM5301/hpm_otp_drv.c)
zephyr_sources_ifdef(CONFIG_SOC_HPM5361 ../../../../sdk_env/hpm_sdk/soc/HPM5300/HPM5361/hpm_otp_drv.c)

if(CONFIG_SOC_SERIES_HPM5300)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
endif()
