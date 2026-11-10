# SPDX-License-Identifier: Apache-2.0

macro(hpmicro_setup_probe_runners board_name openocd_soc default_probe jlink_device)
	set(HPM_SDK_RELATIVE ${BOARD_DIR}/../../../../sdk_env/hpm_sdk)
	set(HPM_TOOLS_RELATIVE ${BOARD_DIR}/../../../../sdk_env/tools)
	get_filename_component(HPM_SDK_ABSOLUTE ${HPM_SDK_RELATIVE} ABSOLUTE)
	if(DEFINED HPM_SDK_DIR)
		set(HPM_SDK_ABSOLUTE ${HPM_SDK_DIR})
	endif()
	set(OPENOCD_CONFIG_RELATIVE ${HPM_SDK_ABSOLUTE}/boards/openocd)
	get_filename_component(OPENOCD_CONFIG_ABSOLUTE ${OPENOCD_CONFIG_RELATIVE} ABSOLUTE)
	get_filename_component(HPM_TOOLS_ABSOLUTE ${HPM_TOOLS_RELATIVE} ABSOLUTE)
	set(OPENOCD_CONFIG_DIR ${OPENOCD_CONFIG_ABSOLUTE} CACHE PATH "hpmicro openocd cfg root directory")
	set(HPM_TOOLS_DIR ${HPM_TOOLS_ABSOLUTE} CACHE PATH "hpmicro win tools root directory")
	set(HPM_OPENOCD_PROBE "" CACHE STRING "Override HPMicro OpenOCD probe config name")
	set(HPM_OPENOCD_TARGET_HANDLE _CHIPNAME.cpu0 CACHE STRING "HPMicro OpenOCD target handle")
	set(HPM_JLINK_DEVICE ${jlink_device} CACHE STRING "HPMicro J-Link device name")
	set(HPM_JLINK_SPEED 4000 CACHE STRING "HPMicro J-Link interface speed")
	set(HPM_JLINK_ROOT "" CACHE PATH "SEGGER J-Link install directory")

	set(HPM_JLINK_SEARCH_DIRS)
	if(HPM_JLINK_ROOT)
		list(APPEND HPM_JLINK_SEARCH_DIRS ${HPM_JLINK_ROOT})
	endif()
	if(DEFINED ENV{JLINK_ROOT})
		list(APPEND HPM_JLINK_SEARCH_DIRS $ENV{JLINK_ROOT})
	endif()
	if("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Windows")
		list(APPEND HPM_JLINK_SEARCH_DIRS
		     "C:/Program Files/SEGGER/JLink"
		     "C:/Program Files (x86)/SEGGER/JLink")
	endif()

	find_program(HPM_JLINK_COMMANDER
		     NAMES JLink.exe JLinkExe JLink
		     PATHS ${HPM_JLINK_SEARCH_DIRS})
	find_program(HPM_JLINK_GDBSERVER
		     NAMES JLinkGDBServerCL.exe JLinkGDBServer.exe JLinkGDBServerCL JLinkGDBServer
		     PATHS ${HPM_JLINK_SEARCH_DIRS})

	set(HPM_JLINK_ARGS "--device=${HPM_JLINK_DEVICE}"
			   "--iface=jtag"
			   "--speed=${HPM_JLINK_SPEED}"
			   "--tool-opt=-JTAGConf -1,-1")
	if(HPM_JLINK_COMMANDER)
		list(APPEND HPM_JLINK_ARGS "--commander=${HPM_JLINK_COMMANDER}")
	endif()
	if(HPM_JLINK_GDBSERVER)
		list(APPEND HPM_JLINK_ARGS "--gdbserver=${HPM_JLINK_GDBSERVER}")
	endif()

	if(HPM_OPENOCD_PROBE)
		set(HPM_OPENOCD_ARGS "--config=${OPENOCD_CONFIG_DIR}/probes/${HPM_OPENOCD_PROBE}.cfg"
				     "--config=${OPENOCD_CONFIG_DIR}/soc/${openocd_soc}.cfg"
				     "--config=${OPENOCD_CONFIG_DIR}/boards/${board_name}.cfg")
	else()
		set(HPM_OPENOCD_ARGS "--config=${BOARD_DIR}/support/openocd.cfg")
	endif()
	list(APPEND HPM_OPENOCD_ARGS "--openocd-search=${OPENOCD_CONFIG_DIR}"
				    "--target-handle=${HPM_OPENOCD_TARGET_HANDLE}")

	if(NOT CONFIG_XIP)
		board_runner_args(openocd "--use-elf")
	endif()

	if(${BOARD} STREQUAL ${board_name})
		board_runner_args(jlink ${HPM_JLINK_ARGS})
		board_runner_args(openocd ${HPM_OPENOCD_ARGS})
	else()
		message(FATAL_ERROR "${BOARD} is not supported now")
	endif()

	board_set_flasher(openocd)
	board_set_debugger(openocd)

	set(ENV_PATH $ENV{PATH})
	unset(OPENOCD CACHE)
	if("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
		set(SEPARATOR ":")
		string(REPLACE ${SEPARATOR} ";" ZPATH ${ENV_PATH})
		find_program(OPENOCD openocd PATHS ${ZPATH} NO_DEFAULT_PATH)
	elseif("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Windows")
		set(OPENOCD "${HPM_TOOLS_DIR}/openocd/openocd.exe" CACHE FILEPATH "" FORCE)
		set(OPENOCD_DEFAULT_PATH ${HPM_TOOLS_DIR}/openocd/tcl)
	else()
		message(WARNING "${CMAKE_HOST_SYSTEM_NAME} openocd is not support")
	endif()
	include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
	include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
endmacro()
