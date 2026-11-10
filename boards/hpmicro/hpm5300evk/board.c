/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_usb_drv.h"

void board_init_usb_dp_dm_pins(void)
{
    /* Disconnect usb dp/dm pins pull down 45ohm resistance */

    while (sysctl_resource_any_is_busy(HPM_SYSCTL)) {
        ;
    }
    if (pllctlv2_xtal_is_stable(HPM_PLLCTLV2) && pllctlv2_xtal_is_enabled(HPM_PLLCTLV2)) {
        if (clock_check_in_group(clock_usb0, 0)) {
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        } else {
            clock_add_to_group(clock_usb0, 0);
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
            clock_remove_from_group(clock_usb0, 0);
        }
    } else {
        uint8_t tmp;
        tmp = sysctl_resource_target_get_mode(HPM_SYSCTL, sysctl_resource_xtal);
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, 0x03);    /* NOLINT */
        clock_add_to_group(clock_usb0, 0);
        usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        clock_remove_from_group(clock_usb0, 0);
        while (sysctl_resource_target_is_busy(HPM_SYSCTL, sysctl_resource_usb0)) {
            ;
        }
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, tmp);
    }
}

void board_early_init_hook(void)
{
	board_init_usb_dp_dm_pins();

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
