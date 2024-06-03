# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "hpmicro zephyr root directory")

set(HPM_SDK_RELATIVE ${HPM_ZEPHYR_DIR}/../sdk_env/hpm_sdk)
set(HPM_ENV_RELATIVE ${HPM_ZEPHYR_DIR}/../sdk_env)
get_filename_component(HPM_SDK_ABSOLUTE ${HPM_SDK_RELATIVE} ABSOLUTE)
get_filename_component(HPM_ENV_ABSOLUTE ${HPM_ENV_RELATIVE} ABSOLUTE)
set(HPM_SDK_DIR ${HPM_SDK_ABSOLUTE} CACHE PATH "hpmicro sdk root directory")

include(${HPM_ZEPHYR_DIR}/cmake/version.cmake)
include(${HPM_SDK_DIR}/cmake/cmake-ext.cmake)

set(generated_version_path "${CMAKE_BINARY_DIR}")

# generate hpmicro zephyr version file
execute_process(
  COMMAND ${CMAKE_COMMAND} -DHPM_ZEPHYR_DIR=${HPM_ZEPHYR_DIR}
    -DOUT_FILE=${generated_version_path}/hpm_sdk_version.h
    -DSDKVERSION=${SDKVERSION}
    -DSDK_VERSION_NUMBER=${SDK_VERSION_NUMBER}
    -DSDK_VERSION_MAJOR=${SDK_VERSION_MAJOR}
    -DSDK_VERSION_MINOR=${SDK_VERSION_MINOR}
    -DSDK_PATCHLEVEL=${SDK_PATCHLEVEL}
    -DSDK_VERSION_STRING=${SDK_VERSION_STRING}
    -P ${HPM_ZEPHYR_DIR}/cmake/gen_version_h.cmake
  WORKING_DIRECTORY ${generated_version_path}
)

zephyr_include_directories(${generated_version_path})
zephyr_include_directories(include)

add_subdirectory(modules)
add_subdirectory(drivers)
