# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_DDR_HPMICRO memc_hpmicro_ddr.c)
zephyr_library_sources_ifdef(CONFIG_FEMC_HPMICRO memc_hpmicro_femc.c)
