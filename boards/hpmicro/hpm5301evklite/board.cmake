# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

include(${BOARD_DIR}/../common/hpmicro_probe_runner.cmake)

hpmicro_setup_probe_runners(hpm5301evklite hpm5300 cmsis_dap HPM5301xEGx)
