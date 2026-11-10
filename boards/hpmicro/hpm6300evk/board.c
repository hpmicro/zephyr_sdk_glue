/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"
#include "hpm_femc_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_sdxc_drv.h"
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
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_FEMC_DQ_07;
    HPM_IOC->PAD[IOC_PAD_PA26].FUNC_CTL = IOC_PA26_FUNC_CTL_FEMC_DQ_06;
    HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_FEMC_DQ_05;
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_FEMC_DQ_04;
    HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_FEMC_DQ_03;
    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_FEMC_DQ_02;
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_FEMC_DQ_01;
    HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_FEMC_DQ_00;
    HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_FEMC_DM_0;
    HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_FEMC_DQ_08;
    HPM_IOC->PAD[IOC_PAD_PB03].FUNC_CTL = IOC_PB03_FUNC_CTL_FEMC_DQ_09;
    HPM_IOC->PAD[IOC_PAD_PB04].FUNC_CTL = IOC_PB04_FUNC_CTL_FEMC_DQ_10;
    HPM_IOC->PAD[IOC_PAD_PB05].FUNC_CTL = IOC_PB05_FUNC_CTL_FEMC_DQ_11;
    HPM_IOC->PAD[IOC_PAD_PB06].FUNC_CTL = IOC_PB06_FUNC_CTL_FEMC_DQ_12;
    HPM_IOC->PAD[IOC_PAD_PB07].FUNC_CTL = IOC_PB07_FUNC_CTL_FEMC_DQ_13;
    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PB08_FUNC_CTL_FEMC_DQ_14;
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_FEMC_DQ_15;
    HPM_IOC->PAD[IOC_PAD_PB10].FUNC_CTL = IOC_PB10_FUNC_CTL_FEMC_DM_1;
    HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PB11_FUNC_CTL_FEMC_WE;
    HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_FEMC_CAS;
    HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_FEMC_RAS;
    HPM_IOC->PAD[IOC_PAD_PB14].FUNC_CTL = IOC_PB14_FUNC_CTL_FEMC_CS_0;
    HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_FEMC_BA0;
    HPM_IOC->PAD[IOC_PAD_PB16].FUNC_CTL = IOC_PB16_FUNC_CTL_FEMC_BA1;
    HPM_IOC->PAD[IOC_PAD_PB17].FUNC_CTL = IOC_PB17_FUNC_CTL_FEMC_A_10;
    HPM_IOC->PAD[IOC_PAD_PB18].FUNC_CTL = IOC_PB18_FUNC_CTL_FEMC_A_00;
    HPM_IOC->PAD[IOC_PAD_PB19].FUNC_CTL = IOC_PB19_FUNC_CTL_FEMC_A_01;
    HPM_IOC->PAD[IOC_PAD_PB20].FUNC_CTL = IOC_PB20_FUNC_CTL_FEMC_A_02;
    HPM_IOC->PAD[IOC_PAD_PB21].FUNC_CTL = IOC_PB21_FUNC_CTL_FEMC_A_03;
    HPM_IOC->PAD[IOC_PAD_PB22].FUNC_CTL = IOC_PB22_FUNC_CTL_FEMC_CLK;
    HPM_IOC->PAD[IOC_PAD_PB23].FUNC_CTL = IOC_PB23_FUNC_CTL_FEMC_CKE;
    HPM_IOC->PAD[IOC_PAD_PB24].FUNC_CTL = IOC_PB24_FUNC_CTL_FEMC_A_12;
    HPM_IOC->PAD[IOC_PAD_PB25].FUNC_CTL = IOC_PB25_FUNC_CTL_FEMC_A_11;
    HPM_IOC->PAD[IOC_PAD_PB26].FUNC_CTL = IOC_PB26_FUNC_CTL_FEMC_A_09;
    HPM_IOC->PAD[IOC_PAD_PB27].FUNC_CTL = IOC_PB27_FUNC_CTL_FEMC_A_08;
    HPM_IOC->PAD[IOC_PAD_PB28].FUNC_CTL = IOC_PB28_FUNC_CTL_FEMC_A_07;
    HPM_IOC->PAD[IOC_PAD_PB29].FUNC_CTL = IOC_PB29_FUNC_CTL_FEMC_A_06;
    HPM_IOC->PAD[IOC_PAD_PB30].FUNC_CTL = IOC_PB30_FUNC_CTL_FEMC_A_05;
    HPM_IOC->PAD[IOC_PAD_PB31].FUNC_CTL = IOC_PB31_FUNC_CTL_FEMC_A_04;
    HPM_IOC->PAD[IOC_PAD_PA23].FUNC_CTL = IOC_PA23_FUNC_CTL_FEMC_CS_1;
    HPM_IOC->PAD[IOC_PAD_PX07].FUNC_CTL = IOC_PX07_FUNC_CTL_FEMC_DQS | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
}

void _init_ext_ram(void)
{
    uint32_t femc_clk_in_hz;
    femc_config_t config = {0};
    femc_sdram_config_t sdram_config = {0};

    clock_set_source_divider(clock_femc, clk_src_pll0_clk1, 2);
    clock_add_to_group(clock_femc, 0);
    init_sdram_pins();
    femc_clk_in_hz = clock_get_frequency(clock_femc);

    femc_default_config(HPM_FEMC, &config);
    femc_init(HPM_FEMC, &config);
    femc_get_typical_sdram_config(HPM_FEMC, &sdram_config);

    sdram_config.bank_num = FEMC_SDRAM_BANK_NUM_4;
    sdram_config.prescaler = 0x3;
    sdram_config.burst_len_in_byte = 8;
    sdram_config.auto_refresh_count_in_one_burst = 1;
    sdram_config.col_addr_bits = FEMC_SDRAM_COLUMN_ADDR_9_BITS;
    sdram_config.cas_latency = FEMC_SDRAM_CAS_LATENCY_3;

    sdram_config.refresh_to_refresh_in_ns = 60;
    sdram_config.refresh_recover_in_ns = 60;
    sdram_config.act_to_precharge_in_ns = 42;
    sdram_config.act_to_rw_in_ns = 18;
    sdram_config.precharge_to_act_in_ns = 18;
    sdram_config.act_to_act_in_ns = 12;
    sdram_config.write_recover_in_ns = 12;
    sdram_config.self_refresh_recover_in_ns = 72;

    sdram_config.cs = FEMC_SDRAM_CS0;
    sdram_config.base_address = DT_REG_ADDR(DT_NODELABEL(dram));
    sdram_config.size_in_byte = DT_REG_SIZE(DT_NODELABEL(dram));
    sdram_config.port_size = FEMC_SDRAM_PORT_SIZE_16_BITS;
    sdram_config.refresh_count = 8192UL;
    sdram_config.refresh_in_ms = 64UL;
    sdram_config.delay_cell_disable = true;
    sdram_config.delay_cell_value = 0;

    femc_config_sdram(HPM_FEMC, femc_clk_in_hz, &sdram_config);
}

uint32_t hpm_board_sd_configure_clock(SDXC_Type *ptr, uint32_t freq, bool need_inverse)
{
    uint32_t actual_freq = 0;

    if (ptr != HPM_SDXC0) {
        return 0;
    }

    sdxc_enable_inverse_clock(ptr, false);
    sdxc_enable_sd_clock(ptr, false);
    clock_set_source_divider(clock_sdxc0, clk_src_pll2_clk1, 2);

    if (freq <= 400000UL) {
        sdxc_set_clock_divider(ptr, 600);
    } else if (freq <= 26000000UL) {
        sdxc_set_clock_divider(ptr, 8);
    } else if (freq <= 52000000UL) {
        sdxc_set_clock_divider(ptr, 4);
    } else if (freq <= 100000000UL) {
        sdxc_set_clock_divider(ptr, 2);
    } else if (freq <= 208000000UL) {
        sdxc_set_clock_divider(ptr, 1);
    } else {
        sdxc_set_clock_divider(ptr, 8);
    }

    if (need_inverse) {
        sdxc_enable_inverse_clock(ptr, true);
    }

    sdxc_enable_sd_clock(ptr, true);
    actual_freq = clock_get_frequency(clock_sdxc0) / sdxc_get_clock_divider(ptr);

    return actual_freq;
}

void sys_arch_reboot(int type)
{
    ARG_UNUSED(type);

    HPM_PPOR->RESET_ENABLE = (1UL << 31);
    HPM_PPOR->SOFTWARE_RESET = 1000U;
    while(1) {

    }
}
