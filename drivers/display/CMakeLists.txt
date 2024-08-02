# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_DISPLAY_HPMICRO display_rgb_hpmicro.c display.c)
zephyr_library_sources_ifdef(CONFIG_HPM_DISPLAY_LVDS display_lvds_hpmicro.c)
zephyr_library_sources_ifdef(CONFIG_HPM_DISPLAY_MIPI_DSI display_mipi_hpmicro.c)