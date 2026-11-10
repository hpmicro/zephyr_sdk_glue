# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_WDG_HPMICRO wdg_hpmicro.c)
zephyr_library_sources_ifdef(CONFIG_EWDG_HPMICRO ewdg_hpmicro.c)