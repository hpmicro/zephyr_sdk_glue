# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_GPT_HPMICRO counter_hpmicro_gptmr.c)