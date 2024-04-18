/*
 * Copyright (c) 2022-2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/init.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include "hpm_pmp_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_pcfg_drv.h"
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#include "hpm_pllctlv2_drv.h"

#ifdef CONFIG_XIP
__attribute__ ((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90001, 0x00000007, 0x0, 0x0};
uint32_t __fw_size__[] = {32768};
#endif
static void soc_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    /* group0[0] */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_rom, 0);
    clock_add_to_group(clock_can0, 0);
    clock_add_to_group(clock_can1, 0);
    clock_add_to_group(clock_can2, 0);
    clock_add_to_group(clock_can3, 0);
    clock_add_to_group(clock_ptpc, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_gptmr2, 0);
    clock_add_to_group(clock_gptmr3, 0);
    clock_add_to_group(clock_i2c0, 0);
    clock_add_to_group(clock_i2c1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_i2c3, 0);
    clock_add_to_group(clock_spi0, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_spi2, 0);
    clock_add_to_group(clock_spi3, 0);
    clock_add_to_group(clock_uart0, 0);
    clock_add_to_group(clock_uart1, 0);
    clock_add_to_group(clock_uart2, 0);
    clock_add_to_group(clock_uart3, 0);
    clock_add_to_group(clock_uart4, 0);
    clock_add_to_group(clock_uart5, 0);
    clock_add_to_group(clock_uart6, 0);
    /* group0[1] */
    clock_add_to_group(clock_uart7, 0);
    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_watchdog1, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_tsns, 0);
    clock_add_to_group(clock_crc0, 0);
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    clock_add_to_group(clock_dac0, 0);
    clock_add_to_group(clock_dac1, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_opa0, 0);
    clock_add_to_group(clock_opa1, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_rng, 0);
    clock_add_to_group(clock_sdp, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_usb0, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1175mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1175);

    /* Configure CPU to 480MHz, AXI/AHB to 160MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 3);
    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0);    /* PLL0CLK0: 960MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 1, 3);    /* PLL0CLK1: 600MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 2, 7);    /* PLL0CLK2: 400MHz */
    /* Configure PLL0 Frequency to 960MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, 960000000);

    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

static void soc_init_pmp(void)
{
}

static int hpmicro_soc_init(void)
{
	uint32_t key;

	key = irq_lock();
	soc_init_clock();
	soc_init_pmp();
	irq_unlock(key);

	return 0;
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
