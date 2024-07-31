# Copyright (c) 2023 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_ADC_HPMICRO_ADC12 adc_hpmicro_adc12.c)
zephyr_library_sources_ifdef(CONFIG_ADC_HPMICRO_ADC16 adc_hpmicro_adc16.c)
