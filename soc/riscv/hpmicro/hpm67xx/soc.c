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
#include "hpm_pllctl_drv.h"
#include "hpm_pcfg_drv.h"
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#ifdef CONFIG_XIP
__attribute__ ((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90001, 0x00000007, 0x0, 0x0};
uint32_t __fw_size__[] = {32768};
#endif
__attribute__((weak)) void _init_noncache(void)
{
	uint32_t i, size;

	extern uint8_t __noncacheable_data_load_start[];
	extern uint8_t __noncacheable_bss_start__[], __noncacheable_bss_end__[];
	extern uint8_t __noncacheable_init_start__[], __noncacheable_init_end__[];

	/* noncacheable bss section */
	size = __noncacheable_bss_end__ - __noncacheable_bss_start__;
	for (i = 0; i < size; i++) {
		*(__noncacheable_bss_start__ + i) = 0;
	}

	/* noncacheable init section LMA: etext + data length + ramfunc legnth */
	size = __noncacheable_init_end__ - __noncacheable_init_start__;
	for (i = 0; i < size; i++) {
		*(__noncacheable_init_start__ + i) = *(__noncacheable_data_load_start + i);
	}
}

static void soc_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);
    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctl_xtal_set_rampup_time(HPM_PLLCTL, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, sysctl_preset_1);
    }

    /* Add most Clocks to group 0 */
    /* not open uart clock in this API, uart should configure pin function before opening clock */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_axi0, 0);
    clock_add_to_group(clock_axi1, 0);
    clock_add_to_group(clock_axi2, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_femc, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_xpi1, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_gptmr2, 0);
    clock_add_to_group(clock_gptmr3, 0);
    clock_add_to_group(clock_gptmr4, 0);
    clock_add_to_group(clock_gptmr5, 0);
    clock_add_to_group(clock_gptmr6, 0);
    clock_add_to_group(clock_gptmr7, 0);
    clock_add_to_group(clock_i2c0, 0);
    clock_add_to_group(clock_i2c1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_i2c3, 0);
    clock_add_to_group(clock_spi0, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_spi2, 0);
    clock_add_to_group(clock_spi3, 0);
    clock_add_to_group(clock_can0, 0);
    clock_add_to_group(clock_can1, 0);
    clock_add_to_group(clock_can2, 0);
    clock_add_to_group(clock_can3, 0);
    clock_add_to_group(clock_display, 0);
    clock_add_to_group(clock_sdxc0, 0);
    clock_add_to_group(clock_sdxc1, 0);
    clock_add_to_group(clock_camera0, 0);
    clock_add_to_group(clock_camera1, 0);
    clock_add_to_group(clock_ptpc, 0);
    clock_add_to_group(clock_ref0, 0);
    clock_add_to_group(clock_ref1, 0);
    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_eth0, 0);
    clock_add_to_group(clock_eth1, 0);
    clock_add_to_group(clock_sdp, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_ram0, 0);
    clock_add_to_group(clock_ram1, 0);
    clock_add_to_group(clock_usb0, 0);
    clock_add_to_group(clock_usb1, 0);
    clock_add_to_group(clock_jpeg, 0);
    clock_add_to_group(clock_pdma, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_rng, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_mot1, 0);
    clock_add_to_group(clock_mot2, 0);
    clock_add_to_group(clock_mot3, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_dao, 0);
    clock_add_to_group(clock_msyn, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_lmm1, 0);
    clock_add_to_group(clock_pdm, 0);

    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    clock_add_to_group(clock_adc2, 0);
    clock_add_to_group(clock_adc3, 0);

    clock_add_to_group(clock_i2s0, 0);
    clock_add_to_group(clock_i2s1, 0);
    clock_add_to_group(clock_i2s2, 0);
    clock_add_to_group(clock_i2s3, 0);
    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Add the CPU1 clock to Group1 */
    clock_add_to_group(clock_mchtmr1, 1);
    clock_add_to_group(clock_mbx1, 1);
    /* Connect Group1 to CPU1 */
    clock_connect_group_to_cpu(1, 1);

    /* Bump up DCDC voltage to 1200mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1200);
    pcfg_dcdc_switch_to_dcm_mode(HPM_PCFG);

    if (status_success != pllctl_init_int_pll_with_freq(HPM_PLLCTL, 0, CONFIG_MAIN_FREQUENCY)) {
        LOG_PRINTK("Failed to set pll0_clk0 to %uHz\n", CONFIG_MAIN_FREQUENCY);
        while (1) {
        }
    }

    clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 1);
    clock_set_source_divider(clock_cpu1, clk_src_pll0_clk0, 1);
    clock_update_core_clock();

    clock_set_source_divider(clock_ahb, clk_src_pll1_clk1, 2); /*200m hz*/
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
    clock_set_source_divider(clock_mchtmr1, clk_src_osc24m, 1);
}

static void soc_init_pmp(void)
{
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t length;
    pmp_entry_t pmp_entry[16];
    uint8_t index = 0;

    /* Init noncachable memory */
    extern uint32_t __noncacheable_start__[];
    extern uint32_t __noncacheable_end__[];
    start_addr = (uint32_t) __noncacheable_start__;
    end_addr = (uint32_t) __noncacheable_end__;
    length = end_addr - start_addr;
    if (length > 0) {
        /* Ensure the address and the length are power of 2 aligned */
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    /* Init share memory */
    extern uint32_t __share_mem_start__[];
    extern uint32_t __share_mem_end__[];
    start_addr = (uint32_t)__share_mem_start__;
    end_addr = (uint32_t)__share_mem_end__;
    length = end_addr - start_addr;
    if (length > 0) {
        /* Ensure the address and the length are power of 2 aligned */
        assert((length & (length - 1U)) == 0U);
        assert((start_addr & (length - 1U)) == 0U);
        pmp_entry[index].pmp_addr = PMP_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pmp_cfg.val = PMP_CFG(READ_EN, WRITE_EN, EXECUTE_EN, ADDR_MATCH_NAPOT, REG_UNLOCK);
        pmp_entry[index].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
        pmp_entry[index].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
        index++;
    }

    pmp_config(&pmp_entry[0], index);
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
