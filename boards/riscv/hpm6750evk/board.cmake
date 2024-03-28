# SPDX-License-Identifier: Apache-2.0

board_set_debugger_ifnset(openocd)
board_set_flasher_ifnset(openocd)

if(NOT CONFIG_XIP)
board_runner_args(openocd "--use-elf")
endif()

board_runner_args(openocd "--config=${HPM_SDK_DIR}/boards/openocd/probes/ft2232.cfg"
                                    "--config=${HPM_SDK_DIR}/boards/openocd/soc/hpm6750-single-core.cfg"
                                "--config=${HPM_SDK_DIR}/boards/openocd/boards/hpm6750evk.cfg")
board_runner_args(openocd --target-handle=_CHIPNAME.cpu0)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
