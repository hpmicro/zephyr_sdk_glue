/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
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
uint32_t __fw_size__[] = {32768};
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

/* TODO: DDR need to be moved in drivers/memc */
#ifdef CONFIG_INIT_EXT_RAM
#include "hpm_ddrctl_regs.h"
#include "hpm_ddrphy_regs.h"

static void _cpu_wait_ms(uint32_t cpu_freq, uint32_t ms)
{
    uint32_t ticks_per_us = (cpu_freq + 1000000UL - 1UL) / 1000000UL;
    uint64_t expected_ticks = hpm_csr_get_core_mcycle() + (uint64_t)ticks_per_us * 1000UL * ms;
    while (hpm_csr_get_core_mcycle() < expected_ticks) {
    }
}

void init_ddr2_800(void)
{
    /* Enable On-chip DCDC 1.8V output */
    HPM_PCFG->DCDCM_MODE = PCFG_DCDCM_MODE_VOLT_SET(1800) | PCFG_DCDCM_MODE_MODE_SET(1);

    /* Change DDR clock to 200MHz, namely: DDR2-800 */
    clock_set_source_divider(clock_axif, clk_src_pll1_clk0, 4);

    /* Enable DDR clock first */
    clock_add_to_group(clock_ddr0, 0);

    /* Wait until the clock is stable */
    uint32_t core_clock_freq = clock_get_frequency(clock_cpu0);
    _cpu_wait_ms(core_clock_freq, 5);

    /* Clear DFI_INIT_COMPLETE_EN bit */
    HPM_DDRCTL->DFIMISC &= ~DDRCTL_DFIMISC_DFI_INIT_COMPLETE_EN_MASK;

    /* Release DDR core reset */
    *(volatile uint32_t *) (HPM_DDRCTL_BASE + 0x3000UL) |= (1UL << 26);

    /* Enable PORT */
    HPM_DDRCTL->PCFG[0].CTRL = 1;

    /* Configure W972GG6KB parameters, configure DDRCTL first */
    HPM_DDRCTL->MSTR = DDRCTL_MSTR_ACTIVE_RANKS_SET(1)      /* RANK=1 */
                       | DDRCTL_MSTR_BURST_RDWR_SET(4)        /* Burst Length = 8 */
                       | DDRCTL_MSTR_DATA_BUS_WIDTH_SET(0)    /* Full DQ bus width */
                       | DDRCTL_MSTR_DDR3_SET(0);             /* DDR2 Device */

    /* Skip SDRAM Initialization in controller, the initialization sequence will be performed by PHY */
    HPM_DDRCTL->INIT0 = DDRCTL_INIT0_SKIP_DRAM_INIT_SET(1)
                        | DDRCTL_INIT0_POST_CKE_X1024_SET(2)    /* Default setting */
                        | DDRCTL_INIT0_PRE_CKE_X1024_SET(0x4e); /* Default setting */

    /* Configure DFI timing */
    HPM_DDRCTL->DFITMG0 = 0x03010101UL;
    HPM_DDRCTL->DFITMG1 = 0x00020101UL;
    HPM_DDRCTL->DFIUPD0 = 0x40005UL;
    HPM_DDRCTL->DFIUPD1 = 0x00020008UL;

    HPM_DDRCTL->ODTCFG = 0x06000600UL;    /* BL=8 */

    /* Configure ADDRMAP */
    HPM_DDRCTL->ADDRMAP0 = 0x001F1F1FUL;  /* RANK0 not used */
    HPM_DDRCTL->ADDRMAP1 = 0x00121212UL;  /* HIF bit[24:22] as BANK[2:0] */
    HPM_DDRCTL->ADDRMAP2 = 0;             /* HIF bit[6:3] as COL_B[6:3] */
    HPM_DDRCTL->ADDRMAP3 = 0;             /* HIF bit [10:7] as COL_B[11,9:6:7] */
    HPM_DDRCTL->ADDRMAP4 = 0xF0FUL;       /* not used */
    HPM_DDRCTL->ADDRMAP5 = 0x06030303UL;  /* HIF bit[21:11] as ROW[10:0], HIF bit[25] as ROW[11] */
    HPM_DDRCTL->ADDRMAP6 = 0x0F0F0606UL;  /* HIF bit[27:26] as ROW[13:12] */

    /* Release DDR AXI reset */
    *(volatile uint32_t *) (HPM_DDRCTL_BASE + 0x3000UL) |= (1UL << 27);

    /* Release DDR PHY */
    *(volatile uint32_t *) (HPM_DDRPHY_BASE + 0x3000UL) |= (1UL << 4);

    HPM_DDRPHY->DCR = DDRPHY_DCR_DDRMD_SET(2)       /* Set to DDR2 mode  */
                      | DDRPHY_DCR_DDR8BNK_MASK     /* BANK = 8 */
                      | DDRPHY_DCR_BYTEMASK_MASK;   /* BYTEMASK = 1 */
    HPM_DDRPHY->DSGCR |= DDRPHY_DSGCR_RRMODE_MASK;  /* Enable RRMode */

    /* Configure DDR2 registers */
    HPM_DDRPHY->MR = (3UL << 0)    /* BL = 3 */
                     | (0UL << 3)    /* BT = 0 */
                     | (6UL << 4)    /* CL = 6 */
                     | (0UL << 7)    /* Operating mode */
                     | (0UL << 8)    /* DLL Reset = 0 */
                     | (6UL << 9);   /* WR = 6  */
    HPM_DDRPHY->EMR = (1UL << 0)              /* DLL Enable */
                      | (0UL << 1)              /* Output Driver Impedance Control */
                      | (0UL << 6) | (1UL << 2) /* On Die Termination */
                      | (0UL << 3)              /* AL(Posted CAS Additive Latency) = 0 */
                      | (0UL << 7)              /* OCD = 0*/
                      | (0UL << 10)             /* DQS */
                      | (0UL << 11)             /* RDQS */
                      | (0UL << 12);            /* QOFF */
    HPM_DDRPHY->EMR2 = 0;
    HPM_DDRPHY->EMR3 = 0;
    HPM_DDRPHY->DTPR0 = (4UL << 0)
                        | (5UL << 4)
                        | (14UL << 8)
                        | (15UL << 12)
                        | (50UL << 16)
                        | (10UL << 22)
                        | (60UL << 26);
    HPM_DDRPHY->DTPR1 = (2UL << 0)
                        | (31UL << 5)
                        | (80UL << 11)
                        | (40UL << 20)
                        | (0x8 << 26);
    HPM_DDRPHY->DTPR2 = (256UL << 0)
                        | (6UL << 10)
                        | (4UL << 15)
                        | (512UL << 19);

    /* tREFPRD */
    HPM_DDRPHY->PGCR2 = 0xF06D50;

    /* Set DFI_INIT_COMPLETE_EN bit */
    HPM_DDRCTL->DFIMISC |= DDRCTL_DFIMISC_DFI_INIT_COMPLETE_EN_MASK;

    /* Start PHY Init First */
    HPM_DDRPHY->PIR |= DDRPHY_PIR_INIT_MASK;
    while ((HPM_DDRPHY->PGSR0 & DDRPHY_PGSR0_IDONE_MASK) == 0) {
    }
    /** Data training
     * RANKEN = 1, Others: default value
     */
    HPM_DDRPHY->DTCR = 0x91003587UL;

    /* Trigger PHY to do the PHY initialization and DRAM initialization */
    HPM_DDRPHY->PIR = 0xF501UL;

    /* Wait until the initialization sequence started */
    while ((HPM_DDRPHY->PGSR0 & DDRPHY_PGSR0_IDONE_MASK) != 0) {
    }
    /* Wait until the initialization sequence completed */
    while ((HPM_DDRPHY->PGSR0 & DDRPHY_PGSR0_IDONE_MASK) == 0) {
    }

    /* Wait for normal mode */
    while ((HPM_DDRCTL->STAT & DDRCTL_STAT_OPERATING_MODE_MASK) != 0x1) {
    }
}

void init_ddr3l_1333(void)
{
    /* Enable On-chip DCDC 1.4V output */
    HPM_PCFG->DCDCM_MODE = PCFG_DCDCM_MODE_VOLT_SET(1400) | PCFG_DCDCM_MODE_MODE_SET(5);

    /* Change DDR clock to 333.33MHz, namely: DDR3-1333 */
    clock_set_source_divider(clock_axif, clk_src_pll1_clk1, 2);

    /* Enable DDR clock first */
    clock_add_to_group(clock_ddr0, 0);

    /* Wait until the clock is stable */
    uint32_t core_clock_freq = clock_get_frequency(clock_cpu0);
    _cpu_wait_ms(core_clock_freq, 5);

    /* Release DDR PHY */
    *(volatile uint32_t *) (HPM_DDRPHY_BASE + 0x3000UL) |= (1UL << 4);

    /* Clear DFI_INIT_COMPLETE_EN bit */
    HPM_DDRCTL->DFIMISC &= ~DDRCTL_DFIMISC_DFI_INIT_COMPLETE_EN_MASK;

    HPM_DDRPHY->DSGCR = 0xf004641f;

    *(volatile uint32_t *) (HPM_DDRPHY_BASE + 0x3000UL) |= (1UL << 0);

    /* Release DDR core reset */
    *(volatile uint32_t *) (HPM_DDRCTL_BASE + 0x3000UL) |= (1UL << 26);

    /* Configure DDRCTL first */
    HPM_DDRCTL->MSTR = DDRCTL_MSTR_ACTIVE_RANKS_SET(1)      /* RANK=1 */
                       | DDRCTL_MSTR_BURST_RDWR_SET(4)        /* Burst Length = 8 */
                       | DDRCTL_MSTR_DATA_BUS_WIDTH_SET(0)    /* Full DQ bus width */
                       | DDRCTL_MSTR_DDR3_SET(1);             /* DDR3 Device */

    /* Enable PORT */
    HPM_DDRCTL->PCFG[0].CTRL = 1;

    /* Skip SDRAM Initialization in controller, the initialization sequence will be performed by PHY */
    HPM_DDRCTL->INIT0 = DDRCTL_INIT0_SKIP_DRAM_INIT_SET(1)
                        | DDRCTL_INIT0_POST_CKE_X1024_SET(2)    /* Default setting */
                        | DDRCTL_INIT0_PRE_CKE_X1024_SET(0x4e); /* Default setting */
    HPM_DDRCTL->DRAMTMG4 = 0x05010407;

    /* Configure DFI timing */
    HPM_DDRCTL->DFITMG0 = 0x07040102;
    HPM_DDRCTL->DFITMG1 = 0x20404;
    HPM_DDRCTL->DFIUPD1 = 0x20008;
    HPM_DDRCTL->ODTCFG = 0x06000600UL;    /* BL=8 */
    HPM_DDRCTL->ODTMAP = 0x11;

    /* Configure ADDRMAP */
    HPM_DDRCTL->ADDRMAP0 = 0x001F1F1FUL;  /* RANK0 not used */
    HPM_DDRCTL->ADDRMAP1 = 0x00121212UL;  /* HIF bit[24:22] as BANK[2:0] */
    HPM_DDRCTL->ADDRMAP2 = 0;             /* HIF bit[6:3] as COL_B[6:3] */
    HPM_DDRCTL->ADDRMAP3 = 0;             /* HIF bit [10:7] as COL_B[11,9:6:7] */
    HPM_DDRCTL->ADDRMAP4 = 0xF0FUL;       /* not used */
    HPM_DDRCTL->ADDRMAP5 = 0x06030303UL;  /* HIF bit[21:11] as ROW[10:0], HIF bit[25] as ROW[11] */
    HPM_DDRCTL->ADDRMAP6 = 0x0F060606UL;  /* HIF bit[27:26] as ROW[13:12] */

    /* Release DDR AXI reset */
    *(volatile uint32_t *) (HPM_DDRCTL_BASE + 0x3000UL) |= (1UL << 27);

    /* Configure DDR3 registers */
    HPM_DDRPHY->MR0 = 0xC70;
    HPM_DDRPHY->MR1 = 0x6;
    HPM_DDRPHY->MR2 = 0x18;
    HPM_DDRPHY->MR3 = 0;

    HPM_DDRPHY->ODTCR = 0x84210000;

    HPM_DDRPHY->DTPR0 = 0x919c8866;
    HPM_DDRPHY->DTPR1 = 0x1a838360;
    HPM_DDRPHY->DTPR2 = 0x3002d200;

    /* tREFPRD */
    HPM_DDRPHY->PGCR2 = 0xf06d28;

    /* Set DFI_INIT_COMPLETE_EN bit */
    HPM_DDRCTL->DFIMISC |= DDRCTL_DFIMISC_DFI_INIT_COMPLETE_EN_MASK;

    /* Start PHY Init First */
    HPM_DDRPHY->PIR |= DDRPHY_PIR_INIT_MASK;
    while ((HPM_DDRPHY->PGSR0 & DDRPHY_PGSR0_IDONE_MASK) == 0) {
    }
    /** Data training
     * RANKEN = 1, Others: default value
     */
    HPM_DDRPHY->DTCR = 0x930035D7;

    /* Trigger PHY to do the PHY initialization and DRAM initialization */
    HPM_DDRPHY->PIR = 0xFF81UL;

    /* Wait until the initialization sequence started */
    while ((HPM_DDRPHY->PGSR0 & DDRPHY_PGSR0_IDONE_MASK) != 0) {
    }
    /* Wait until the initialization sequence completed */
    while ((HPM_DDRPHY->PGSR0 & DDRPHY_PGSR0_IDONE_MASK) == 0) {
    }

    /* Wait for normal mode */
    while ((HPM_DDRCTL->STAT & DDRCTL_STAT_OPERATING_MODE_MASK) != 0x1) {
    }
}

void _init_ext_ram(void)
{
#if defined(CONFIG_DDR2) && (CONFIG_DDR2)
    init_ddr2_800();
#endif
#if defined(CONFIG_DDR3L) && (CONFIG_DDR3L)
    init_ddr3l_1333();
#endif
}
#endif

static int hpmicro_soc_init(void)
{
	uint32_t key;

	key = irq_lock();
#ifdef CONFIG_INIT_EXT_RAM
    _init_ext_ram();
#endif
	soc_init_clock();
	soc_init_pmp();
	irq_unlock(key);

	return 0;
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
