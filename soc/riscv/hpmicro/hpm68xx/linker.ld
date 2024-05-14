/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <zephyr/linker/linker-tool.h>
#include <zephyr/devicetree.h>

#define IS_CHOSEN_SRAM(x) (DT_DEP_ORD(DT_NODELABEL(x)) == DT_DEP_ORD(DT_CHOSEN(zephyr_sram)))

MEMORY
{
#ifdef CONFIG_XIP
	BOOT (wx) :  ORIGIN = DT_REG_ADDR(DT_CHOSEN(zephyr_flash)), LENGTH = 0x3000
#endif
#if (DT_REG_SIZE(DT_NODELABEL(core0_lm_slv)) > 0) && !IS_CHOSEN_SRAM(core0_lm_slv)
	CORE0_LM_SLV (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(core0_lm_slv)), LENGTH = DT_REG_SIZE(DT_NODELABEL(core0_lm_slv))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(core1_lm_slv)) > 0) && !IS_CHOSEN_SRAM(core1_lm_slv)
	CORE1_LM_SLV (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(core1_lm_slv)), LENGTH = DT_REG_SIZE(DT_NODELABEL(core1_lm_slv))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(sram)) > 0) && !IS_CHOSEN_SRAM(sram)
	OCRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(sram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(sram))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(noncache)) > 0) && !IS_CHOSEN_SRAM(noncache)
	NONCACHEABLE (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(noncache)), LENGTH = DT_REG_SIZE(DT_NODELABEL(noncache))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(dram)) > 0) && !IS_CHOSEN_SRAM(dram)
	DRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(dram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(dram))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(ahb_sram)) > 0) && !IS_CHOSEN_SRAM(ahb_sram)
	AHB_SRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(ahb_sram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(ahb_sram))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(apb_sram)) > 0) && !IS_CHOSEN_SRAM(apb_sram)
	APB_SRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(apb_sram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(apb_sram))
#endif
}

#include <../common/linker.ld>
#include <sections.ld>
