# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

include(${BOARD_DIR}/../common/hpmicro_probe_runner.cmake)

hpmicro_setup_probe_runners(hpm6300evk hpm6360 ft2232 HPM6360xPAx)
