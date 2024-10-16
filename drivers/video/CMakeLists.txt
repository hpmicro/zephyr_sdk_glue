# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_VIDEO video_hpmicro.c)
zephyr_library_sources_ifdef(CONFIG_CAMERA_HPMICRO camera_hpmicro.c)
zephyr_library_sources_ifdef(CONFIG_MIPI_CSI_HPMICRO mipi_csi_hpmicro.c)
zephyr_include_directories_ifdef(CONFIG_VIDEO .)

