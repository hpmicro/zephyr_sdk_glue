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
set(ENV{GNURISCV_TOOLCHAIN_PATH} ${SYSROOT_DIR})
set(HPM_SDK_LIB ${ZEPHYR_CURRENT_LIBRARY})
set(HPM_SDK_LIB_ITF zephyr_interface)
set(HPM_SDK_GCC_LIB zephyr)
set(HPM_SDK_NDSGCC_LIB_ITF hpm_sdk_ndsgcc_lib_itf)
add_library(${HPM_SDK_NDSGCC_LIB_ITF} INTERFACE)
set(USE_CUSTOM_STARTUP 1)

add_subdirectory(${HPM_SDK_DIR}/arch ${HPM_SDK_BINARY_DIR}/arch)
add_subdirectory(${HPM_SDK_DIR}/soc ${HPM_SDK_BINARY_DIR}/soc)
add_subdirectory(${HPM_SDK_DIR}/drivers ${HPM_SDK_BINARY_DIR}/drivers)
add_subdirectory(${HPM_SDK_DIR}/components/enet_phy ${HPM_SDK_BINARY_DIR}/components)

sdk_inc(.)
endif()


