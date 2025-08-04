# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

if(${CONFIG_SOC_FAMILY} STREQUAL "hpmicro")

if(CONFIG_XIP)
    set(INCLUDE_BOOTHEADER 1)
endif()

set(HPM_SDK_BINARY_DIR ${CMAKE_BINARY_DIR}/hpm_sdk)
set(HPM_BOARD_DIR ${CONFIG_BOARD})
string(TOUPPER ${SOC_NAME} HPM_SOC_TOUPPER)
set(HPM_SOC ${HPM_SOC_TOUPPER})
string(REGEX REPLACE "xx" "00" SOC_SERIER_MOD "${SOC_SERIES}")
string(TOUPPER ${SOC_SERIER_MOD} HPM_SOC_SERIES_TOUPPER)
set(HPM_SOC_SERIES ${HPM_SOC_SERIES_TOUPPER})
set(TOOLCHAIN_VARIANT "zephyr")
set(ENV{GNURISCV_TOOLCHAIN_PATH} ${SYSROOT_DIR})
set(HPM_SDK_LIB ${ZEPHYR_CURRENT_LIBRARY})
set(HPM_SDK_LIB_ITF zephyr_interface)
set(HPM_SDK_GCC_LIB zephyr)
set(HPM_SDK_NDSGCC_LIB_ITF hpm_sdk_ndsgcc_lib_itf)
add_library(${HPM_SDK_NDSGCC_LIB_ITF} INTERFACE)
set(HPM_SDK_ZCC_LIB_ITF hpm_sdk_zcc_lib_itf)
add_library(${HPM_SDK_ZCC_LIB_ITF} INTERFACE)
set(HPM_SDK_GCC_STARTUP_LIB hpm_sdk_gcc_startup_lib)
add_library(${HPM_SDK_GCC_STARTUP_LIB} INTERFACE)
set(USE_CUSTOM_STARTUP 1)
set(USE_CUSTOM_LINKER 1)

file(STRINGS "${CMAKE_BINARY_DIR}/zephyr/.config" CONFIG_LINES REGEX "^CONFIG_HAS_HPMSDK_[A-Za-z0-9_]+=")

foreach(line IN LISTS CONFIG_LINES)
    string(REGEX REPLACE "(CONFIG_HAS_HPMSDK_[A-Za-z0-9_]+)=y" "\\1" config ${line})
    string(REGEX REPLACE "CONFIG_" "HPMSOC_" HPMSOC_ENABLE ${config})
    set(${HPMSOC_ENABLE} y)
endforeach()

add_subdirectory(${HPM_SDK_DIR}/arch ${HPM_SDK_BINARY_DIR}/arch)
add_subdirectory(${HPM_SDK_DIR}/soc ${HPM_SDK_BINARY_DIR}/soc)
add_subdirectory(${HPM_SDK_DIR}/drivers ${HPM_SDK_BINARY_DIR}/drivers)

sdk_inc(.)
endif()
