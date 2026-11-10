/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_pllctlv2_drv.h"

void board_early_init_hook(void)
{
	if (clock_get_frequency(clock_cpu0) == PLLCTL_SOC_PLL_REFCLK_FREQ) {
		pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);
		sysctl_clock_set_preset(HPM_SYSCTL, 2);
	}

	pcfg_dcdc_set_voltage(HPM_PCFG, 1275);
}

void init_sdram_pins(void)
{
}

void _init_ext_ram(void)
{
}

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	HPM_PPOR->RESET_ENABLE = (1UL << 31);
	HPM_PPOR->SOFTWARE_RESET = 1000U;
	while (1) {
	}
}
