# SPDX-License-Identifier: Apache-2.0

set(OPENOCD_CONFIG_RELATIVE ${ZEPHYR_BASE}/../sdk_env/hpm_sdk/boards/openocd)
set(HPM_TOOLS_RELATIVE ${ZEPHYR_BASE}/../sdk_env/tools)
get_filename_component(OPENOCD_CONFIG_ABSOLUTE ${OPENOCD_CONFIG_RELATIVE} ABSOLUTE)
get_filename_component(HPM_TOOLS_ABSOLUTE ${HPM_TOOLS_RELATIVE} ABSOLUTE)
set(OPENOCD_CONFIG_DIR ${OPENOCD_CONFIG_ABSOLUTE} CACHE PATH "hpmicro openocd cfg root directory")
set(HPM_TOOLS_DIR ${HPM_TOOLS_ABSOLUTE} CACHE PATH "hpmicro win tools root directory")

if(NOT CONFIG_XIP)
    board_runner_args(openocd "--use-elf")
endif()

if(${BOARD} STREQUAL "hpm6800evk")
    board_runner_args(openocd "--config=${OPENOCD_CONFIG_DIR}/probes/ft2232.cfg"
                                    "--config=${OPENOCD_CONFIG_DIR}/soc/hpm6880.cfg"
                                "--config=${OPENOCD_CONFIG_DIR}/boards/hpm6800evk.cfg")
    board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)
else()
    message(FATAL_ERROR "${BOARD} is not supported now")
endif()

if("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set(OPENOCD "/usr/local/bin/openocd" CACHE FILEPATH "" FORCE)
    set(OPENOCD_DEFAULT_PATH /usr/local/share/openocd/scripts)
elseif("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Windows")
    set(OPENOCD "${HPM_TOOLS_DIR}/openocd/openocd" CACHE FILEPATH "" FORCE)
    set(OPENOCD_DEFAULT_PATH ${HPM_TOOLS_DIR}/openocd/tcl)
else()
    message(WARNING "${CMAKE_HOST_SYSTEM_NAME} openocd is not support")
endif()
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
