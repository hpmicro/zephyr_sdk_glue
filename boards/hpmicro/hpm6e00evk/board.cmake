# SPDX-License-Identifier: Apache-2.0

include(${BOARD_DIR}/../common/hpmicro_probe_runner.cmake)

hpmicro_setup_probe_runners(hpm6e00evk hpm6e80-single-core ft2232 HPM6E80xVMx)
