/*
 * Copyright (c) 2025-2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
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
#include "hpm_clock_drv.h"
#include "hpm_femc_drv.h"

void init_sdram_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PD02_FUNC_CTL_FEMC_A_00;
    HPM_IOC->PAD[IOC_PAD_PD03].FUNC_CTL = IOC_PD03_FUNC_CTL_FEMC_A_01;
    HPM_IOC->PAD[IOC_PAD_PD00].FUNC_CTL = IOC_PD00_FUNC_CTL_FEMC_A_02;
    HPM_IOC->PAD[IOC_PAD_PD01].FUNC_CTL = IOC_PD01_FUNC_CTL_FEMC_A_03;
    HPM_IOC->PAD[IOC_PAD_PC18].FUNC_CTL = IOC_PC18_FUNC_CTL_FEMC_A_04;
    HPM_IOC->PAD[IOC_PAD_PC19].FUNC_CTL = IOC_PC19_FUNC_CTL_FEMC_A_05;
    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PC20_FUNC_CTL_FEMC_A_06;
    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PC21_FUNC_CTL_FEMC_A_07;
    HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PC23_FUNC_CTL_FEMC_A_08;
    HPM_IOC->PAD[IOC_PAD_PC24].FUNC_CTL = IOC_PC24_FUNC_CTL_FEMC_A_09;
    HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PD04_FUNC_CTL_FEMC_A_10;
    HPM_IOC->PAD[IOC_PAD_PC25].FUNC_CTL = IOC_PC25_FUNC_CTL_FEMC_A_11;      /* SRAM: NWE */
    HPM_IOC->PAD[IOC_PAD_PC26].FUNC_CTL = IOC_PC26_FUNC_CTL_FEMC_A_12;      /* SRAM: NOE */

    HPM_IOC->PAD[IOC_PAD_PD31].FUNC_CTL = IOC_PD31_FUNC_CTL_FEMC_DQ_00;
    HPM_IOC->PAD[IOC_PAD_PD30].FUNC_CTL = IOC_PD30_FUNC_CTL_FEMC_DQ_01;
    HPM_IOC->PAD[IOC_PAD_PD29].FUNC_CTL = IOC_PD29_FUNC_CTL_FEMC_DQ_02;
    HPM_IOC->PAD[IOC_PAD_PD28].FUNC_CTL = IOC_PD28_FUNC_CTL_FEMC_DQ_03;
    HPM_IOC->PAD[IOC_PAD_PD27].FUNC_CTL = IOC_PD27_FUNC_CTL_FEMC_DQ_04;
    HPM_IOC->PAD[IOC_PAD_PD26].FUNC_CTL = IOC_PD26_FUNC_CTL_FEMC_DQ_05;
    HPM_IOC->PAD[IOC_PAD_PD24].FUNC_CTL = IOC_PD24_FUNC_CTL_FEMC_DQ_06;
    HPM_IOC->PAD[IOC_PAD_PD25].FUNC_CTL = IOC_PD25_FUNC_CTL_FEMC_DQ_07;
    HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_FEMC_DQ_08;
    HPM_IOC->PAD[IOC_PAD_PD17].FUNC_CTL = IOC_PD17_FUNC_CTL_FEMC_DQ_09;
    HPM_IOC->PAD[IOC_PAD_PD16].FUNC_CTL = IOC_PD16_FUNC_CTL_FEMC_DQ_10;
    HPM_IOC->PAD[IOC_PAD_PD19].FUNC_CTL = IOC_PD19_FUNC_CTL_FEMC_DQ_11;
    HPM_IOC->PAD[IOC_PAD_PD18].FUNC_CTL = IOC_PD18_FUNC_CTL_FEMC_DQ_12;
    HPM_IOC->PAD[IOC_PAD_PD21].FUNC_CTL = IOC_PD21_FUNC_CTL_FEMC_DQ_13;
    HPM_IOC->PAD[IOC_PAD_PD20].FUNC_CTL = IOC_PD20_FUNC_CTL_FEMC_DQ_14;
    HPM_IOC->PAD[IOC_PAD_PD22].FUNC_CTL = IOC_PD22_FUNC_CTL_FEMC_DQ_15;
    HPM_IOC->PAD[IOC_PAD_PC16].FUNC_CTL = IOC_PC16_FUNC_CTL_FEMC_DQ_16;
    HPM_IOC->PAD[IOC_PAD_PC17].FUNC_CTL = IOC_PC17_FUNC_CTL_FEMC_DQ_17;
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PC13_FUNC_CTL_FEMC_DQ_18;
    HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PC14_FUNC_CTL_FEMC_DQ_19;
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PC10_FUNC_CTL_FEMC_DQ_20;
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_FEMC_DQ_21;
    HPM_IOC->PAD[IOC_PAD_PC02].FUNC_CTL = IOC_PC02_FUNC_CTL_FEMC_DQ_22;
    HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PC09_FUNC_CTL_FEMC_DQ_23;
    HPM_IOC->PAD[IOC_PAD_PC00].FUNC_CTL = IOC_PC00_FUNC_CTL_FEMC_DQ_24;
    HPM_IOC->PAD[IOC_PAD_PC01].FUNC_CTL = IOC_PC01_FUNC_CTL_FEMC_DQ_25;
    HPM_IOC->PAD[IOC_PAD_PC03].FUNC_CTL = IOC_PC03_FUNC_CTL_FEMC_DQ_26;
    HPM_IOC->PAD[IOC_PAD_PC04].FUNC_CTL = IOC_PC04_FUNC_CTL_FEMC_DQ_27;
    HPM_IOC->PAD[IOC_PAD_PC05].FUNC_CTL = IOC_PC05_FUNC_CTL_FEMC_DQ_28;
    HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PC06_FUNC_CTL_FEMC_DQ_29;
    HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PC07_FUNC_CTL_FEMC_DQ_30;
    HPM_IOC->PAD[IOC_PAD_PC08].FUNC_CTL = IOC_PC08_FUNC_CTL_FEMC_DQ_31;

    HPM_IOC->PAD[IOC_PAD_PD23].FUNC_CTL = IOC_PD23_FUNC_CTL_FEMC_DM_0;     /* SRAM: NLB */
    HPM_IOC->PAD[IOC_PAD_PD15].FUNC_CTL = IOC_PD15_FUNC_CTL_FEMC_DM_1;     /* SRAM: NUB */
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_FEMC_DM_2;
    HPM_IOC->PAD[IOC_PAD_PC15].FUNC_CTL = IOC_PC15_FUNC_CTL_FEMC_DM_3;

    HPM_IOC->PAD[IOC_PAD_PX07].FUNC_CTL = IOC_PX07_FUNC_CTL_FEMC_DQS | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;

    /* SDRAM */
    HPM_IOC->PAD[IOC_PAD_PD05].FUNC_CTL = IOC_PD05_FUNC_CTL_FEMC_BA0;
    HPM_IOC->PAD[IOC_PAD_PD06].FUNC_CTL = IOC_PD06_FUNC_CTL_FEMC_BA1;      /* SRAM: NADV */
    HPM_IOC->PAD[IOC_PAD_PD10].FUNC_CTL = IOC_PD10_FUNC_CTL_FEMC_RAS;
    HPM_IOC->PAD[IOC_PAD_PD13].FUNC_CTL = IOC_PD13_FUNC_CTL_FEMC_CAS;
    HPM_IOC->PAD[IOC_PAD_PC28].FUNC_CTL = IOC_PC28_FUNC_CTL_FEMC_CKE;
    HPM_IOC->PAD[IOC_PAD_PC27].FUNC_CTL = IOC_PC27_FUNC_CTL_FEMC_CLK_0;
    HPM_IOC->PAD[IOC_PAD_PD12].FUNC_CTL = IOC_PD12_FUNC_CTL_FEMC_WE;
    HPM_IOC->PAD[IOC_PAD_PD11].FUNC_CTL = IOC_PD11_FUNC_CTL_FEMC_CS_0;
    HPM_IOC->PAD[IOC_PAD_PD08].FUNC_CTL = IOC_PD08_FUNC_CTL_FEMC_CS_1;       /* SRAM: CS0 */

    /* SRAM */
    HPM_IOC->PAD[IOC_PAD_PC29].FUNC_CTL = IOC_PC29_FUNC_CTL_FEMC_SCLK_0;
    HPM_IOC->PAD[IOC_PAD_PD07].FUNC_CTL = IOC_PD07_FUNC_CTL_FEMC_SCLK_1;
    HPM_IOC->PAD[IOC_PAD_PC30].FUNC_CTL = IOC_PC30_FUNC_CTL_FEMC_SCS_0;      /* SRAM: CS1 */
    HPM_IOC->PAD[IOC_PAD_PC31].FUNC_CTL = IOC_PC31_FUNC_CTL_FEMC_SCS_1;      /* SRAM: CS2 */
    HPM_IOC->PAD[IOC_PAD_PC22].FUNC_CTL = IOC_PC22_FUNC_CTL_FEMC_SRDY;
}

void board_init_sdram_pins(void)
{
    init_sdram_pins();
}

uint32_t board_init_femc_clock(void)
{
    clock_add_to_group(clock_femc, 0);

    /* Default FEMC clock is 166MHz */

    /* Configure the FEMC to clk_src_pll1_clk0 / div, the clk_src_pll1_clk0 default frequency is 800MHz. */
    /* clock_set_source_divider(clock_femc, clk_src_pll1_clk0, 6U); */    /* Set FEMC clock to 133MHz */

    return clock_get_frequency(clock_femc);
}

void _init_ext_ram(void)
{
    uint32_t femc_clk_in_hz;
    femc_config_t config = {0};
    femc_sdram_config_t sdram_config = {0};

    board_init_sdram_pins();
    femc_clk_in_hz = board_init_femc_clock();
    femc_default_config(HPM_FEMC, &config);
    femc_init(HPM_FEMC, &config);

    femc_get_typical_sdram_config(HPM_FEMC, &sdram_config);

    sdram_config.bank_num = FEMC_SDRAM_BANK_NUM_4;
    sdram_config.prescaler = 0x3;
    sdram_config.burst_len_in_byte = 8;
    sdram_config.auto_refresh_count_in_one_burst = 1;
    sdram_config.col_addr_bits = FEMC_SDRAM_COLUMN_ADDR_9_BITS;
    sdram_config.cas_latency = FEMC_SDRAM_CAS_LATENCY_3;

    sdram_config.refresh_to_refresh_in_ns = 60;     /* Trc */
    sdram_config.refresh_recover_in_ns = 60;        /* Trc */
    sdram_config.act_to_precharge_in_ns = 42;       /* Tras */
    sdram_config.act_to_rw_in_ns = 18;              /* Trcd */
    sdram_config.precharge_to_act_in_ns = 18;       /* Trp */
    sdram_config.act_to_act_in_ns = 12;             /* Trrd */
    sdram_config.write_recover_in_ns = 12;          /* Twr/Tdpl */
    sdram_config.self_refresh_recover_in_ns = 72;   /* Txsr */

    sdram_config.cs = FEMC_SDRAM_CS0;
    sdram_config.base_address = DT_REG_ADDR(DT_NODELABEL(dram));
    sdram_config.size_in_byte = DT_REG_SIZE(DT_NODELABEL(dram));;
    sdram_config.port_size = FEMC_SDRAM_PORT_SIZE_16_BITS;
    sdram_config.refresh_count = 8192U;
    sdram_config.refresh_in_ms = 64U;
    sdram_config.delay_cell_disable = true;
    sdram_config.delay_cell_value = 0;

    femc_config_sdram(HPM_FEMC, femc_clk_in_hz, &sdram_config);

    HPM_FEMC->SDRCTRL0 |= FEMC_SDRCTRL0_HIGHBAND_MASK;    /* use data[31:16] for 16bit SDRAM */
}

void sys_arch_reboot(int type)
{
    ARG_UNUSED(type);

    HPM_PPOR->RESET_ENABLE = (1UL << 31);
    HPM_PPOR->SOFTWARE_RESET = 1000U;
    while(1) {

    }
}
