/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <kernel_internal.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include "hpm_pmp_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_pcfg_drv.h"
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#ifdef CONFIG_XIP
__attribute__((section(".nor_cfg_option"))) const uint32_t option[4] = { 0xfcf90001, 0x00000007, 0x0, 0x0 };
#endif
__attribute__((weak)) void c_startup(void)
{
	uint32_t i, size;

    extern uint8_t __ramfunc_start__[], __ramfunc_end__[];
    extern uint8_t __noncacheable_bss_start__[], __noncacheable_bss_end__[];
    extern uint8_t __noncacheable_init_start__[], __noncacheable_init_end__[];
    extern uint8_t __fast_load_addr__[], __noncacheable_init_load_addr__[];
    extern uint8_t __fast_ram_bss_start__[], __fast_ram_bss_end__[];
    extern uint8_t __fast_ram_init_start__[], __fast_ram_init_end__[], __fast_ram_init_load_addr__[];
	extern uint8_t __isr_load_addr__[], __isr_entry_load_addr__[];
    extern uint8_t __isr_ram_start__[], __isr_load_size__[], __isr_entry_start__[], __isr_entry_size__[];

    /* noncacheable bss section */
    size = __noncacheable_bss_end__ - __noncacheable_bss_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_bss_start__ + i) = 0;
    }

    /* fast_ram bss section */
    size = __fast_ram_bss_end__ - __fast_ram_bss_start__;
    for (i = 0; i < size; i++) {
        *(__fast_ram_bss_start__ + i) = 0;
    }

    /* ramfunc section LMA: etext + data length */
    size = __ramfunc_end__ - __ramfunc_start__;
    for (i = 0; i < size; i++) {
        *(__ramfunc_start__ + i) = *(__fast_load_addr__ + i);
    }

    /* noncacheable init section LMA: etext + data length + ramfunc legnth + tdata length*/
    size = __noncacheable_init_end__ - __noncacheable_init_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_init_start__ + i) = *(__noncacheable_init_load_addr__ + i);
    }

    /* fast_ram init section LMA: etext + data length + ramfunc legnth + tdata length*/
    size = __fast_ram_init_end__ - __fast_ram_init_start__;
    for (i = 0; i < size; i++) {
        *(__fast_ram_init_start__ + i) = *(__fast_ram_init_load_addr__ + i);
    }

    z_early_memcpy(&__isr_ram_start__, &__isr_load_addr__, (uintptr_t) &__isr_load_size__);
    z_early_memcpy(&__isr_entry_start__, &__isr_entry_load_addr__, (uintptr_t) &__isr_entry_size__);
}

static void soc_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);
    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, DT_PROP(DT_NODELABEL(clk), ram_up_time));

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, DT_PROP(DT_NODELABEL(clk), sysctl_present));
    }
    /* Add most Clocks to group 0 */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_axiv, 0);
    clock_add_to_group(clock_axid, 0);
    clock_add_to_group(clock_axig, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_xram, 0);
    clock_add_to_group(clock_gpio, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1150mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1150);

    /* Configure PLL1_CLK0 Post Divider to 1 */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0);
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, CONFIG_MAIN_FREQUENCY);

    /* Configure axis to 200MHz */
    clock_set_source_divider(clock_axis, clk_src_pll1_clk0, 4);

    /* Configure axig/clock_gpu0 to 400MHz */
    clock_set_source_divider(clock_axig, clk_src_pll1_clk0, 2);

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

static void soc_init_pmp(void)
{
    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];

    uint32_t start_addr = (uint32_t) __noncacheable_start__;
    uint32_t end_addr = (uint32_t) __noncacheable_end__;
    uint32_t length = end_addr - start_addr;

    if (length == 0) {
        return;
    }

    /* Ensure the address and the length are power of 2 aligned */
    assert((length & (length - 1U)) == 0U);
    assert((start_addr & (length - 1U)) == 0U);

    pmp_entry_t pmp_entry[3] = { 0 };
    pmp_entry[0].pmp_addr = PMP_NAPOT_ADDR(0x0000000, 0x80000000);
    pmp_entry[0].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);


    pmp_entry[1].pmp_addr = PMP_NAPOT_ADDR(0x80000000, 0x80000000);
    pmp_entry[1].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);

    pmp_entry[2].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
    pmp_entry[2].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
    pmp_entry[2].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
    pmp_entry[2].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
    pmp_config(&pmp_entry[0], ARRAY_SIZE(pmp_entry));
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
