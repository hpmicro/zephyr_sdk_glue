# SPDX-License-Identifier: Apache-2.0

include(${BOARD_DIR}/../common/hpmicro_probe_runner.cmake)

hpmicro_setup_probe_runners(hpm6750evk2 hpm6750-single-core cmsis_dap HPM6750xVMx)
