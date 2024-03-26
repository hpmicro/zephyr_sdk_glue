# Copyright (c) 2023 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_PINCTRL_HPMICRO pinctrl_hpmicro.c)
