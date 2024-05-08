# Copyright (c) 2023-2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_DMA_HPMICRO dma_hpmicro.c)
zephyr_library_sources_ifdef(CONFIG_DMAV2_HPMICRO dmav2_hpmicro.c)