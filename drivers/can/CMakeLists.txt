# Copyright (c) 2023-2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_CAN_HPMICRO can_hpmicro.c)