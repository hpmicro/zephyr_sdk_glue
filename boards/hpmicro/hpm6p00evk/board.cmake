# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

include(${BOARD_DIR}/../common/hpmicro_probe_runner.cmake)

hpmicro_setup_probe_runners(hpm6p00evk hpm6p80-single-core ft2232 HPM6P81xRTx)
