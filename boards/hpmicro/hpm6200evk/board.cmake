# SPDX-License-Identifier: Apache-2.0

include(${BOARD_DIR}/../common/hpmicro_probe_runner.cmake)

hpmicro_setup_probe_runners(hpm6200evk hpm6280-single-core ft2232 HPM6280xPAx)
