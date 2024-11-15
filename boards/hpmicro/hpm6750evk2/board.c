/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"
#include "hpm_femc_drv.h"
#include "hpm_sdxc_drv.h"

uint32_t board_init_femc_clock(void)
{
    clock_set_source_divider(clock_femc, clk_src_pll2_clk0, 2U); /* 166Mhz */
    /* clock_set_source_divider(clock_femc, clk_src_pll1_clk1, 2U); [> 200Mhz <] */

    return clock_get_frequency(clock_femc);
}

void init_sdram_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PC01].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC00].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB31].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB30].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB29].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB28].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB27].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB26].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB25].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB24].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB23].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB22].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB21].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB20].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB19].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB18].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);

    HPM_IOC->PAD[IOC_PAD_PD13].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD12].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD10].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD08].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD07].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD06].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD05].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD03].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD01].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD00].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC29].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC28].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC27].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);

    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC17].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC15].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC08].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC05].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC04].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);

    HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC16].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12) | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC26].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC25].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC19].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC18].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC24].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC30].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC31].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC02].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC03].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
}

uint32_t hpm_board_sd_configure_clock(SDXC_Type *ptr, uint32_t freq, bool need_inverse)
{
    uint32_t actual_freq = 0;
    do {
        if (ptr != HPM_SDXC1) {
            break;
        }
        clock_name_t sdxc_clk = (ptr == HPM_SDXC0) ? clock_sdxc0 : clock_sdxc1;
        sdxc_enable_inverse_clock(ptr, false);
        sdxc_enable_sd_clock(ptr, false);
        /* Configure the clock below 400KHz for the identification state */
        if (freq <= 400000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_osc24m, 63);
        }
            /* configure the clock to 24MHz for the SDR12/Default speed */
        else if (freq <= 26000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_osc24m, 1);
        }
            /* Configure the clock to 50MHz for the SDR25/High speed/50MHz DDR/50MHz SDR */
        else if (freq <= 52000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_pll1_clk1, 8);
        }
            /* Configure the clock to 100MHz for the SDR50 */
        else if (freq <= 100000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_pll1_clk1, 4);
        }
            /* Configure the clock to 166MHz for SDR104/HS200/HS400  */
        else if (freq <= 208000000UL) {
            clock_set_source_divider(sdxc_clk, clk_src_pll2_clk0, 2);
        }
            /* For other unsupported clock ranges, configure the clock to 24MHz */
        else {
            clock_set_source_divider(sdxc_clk, clk_src_osc24m, 1);
        }
        if (need_inverse) {
            sdxc_enable_inverse_clock(ptr, true);
        }
        sdxc_enable_sd_clock(ptr, true);
        actual_freq = clock_get_frequency(sdxc_clk);
    } while (false);

    return actual_freq;
}

void board_init_sdram_pins(void)
{
    init_sdram_pins();
}

void _init_ext_ram(void)
{
    uint32_t femc_clk_in_hz;
    clock_add_to_group(clock_femc, 0);
    board_init_sdram_pins();
    femc_clk_in_hz = board_init_femc_clock();

    femc_config_t config = {0};
    femc_sdram_config_t sdram_config = {0};

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
    sdram_config.size_in_byte = DT_REG_SIZE(DT_NODELABEL(dram));
    sdram_config.port_size = FEMC_SDRAM_PORT_SIZE_32_BITS;
    sdram_config.refresh_count = (8192UL);
    sdram_config.refresh_in_ms = (64UL);
    sdram_config.delay_cell_disable = true;
    sdram_config.delay_cell_value = 0;

    femc_config_sdram(HPM_FEMC, femc_clk_in_hz, &sdram_config);
}

void sys_arch_reboot(int type)
{
    ARG_UNUSED(type);

    HPM_PPOR->RESET_ENABLE = (1UL << 31);
    HPM_PPOR->RESET_HOT &= ~(1UL << 31);
    HPM_PPOR->RESET_COLD |= (1UL << 31);

    HPM_PPOR->SOFTWARE_RESET = 1000U;
    while(1) {

    }
}
