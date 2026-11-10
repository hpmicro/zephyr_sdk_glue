# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
if(CONFIG_CACHE_MANAGEMENT)
    zephyr_sources(cache_hpmicro.c)
endif()
