# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_HPMICRO clock_control_hpmicro.c)
zephyr_library_sources_ifdef(CONFIG_CLOCK_CONTROL_HPMICRO_V2 clock_control_hpmicro_v2.c)