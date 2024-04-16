/*
 * Copyright (c) 2024 hpmicro
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
	MCU_BOOT (wx) :  ORIGIN = DT_REG_ADDR(DT_CHOSEN(zephyr_flash)), LENGTH = DT_REG_SIZE(DT_CHOSEN(zephyr_flash))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(core0_ilm_slv)) > 0) && !IS_CHOSEN_SRAM(core0_ilm_slv)
	CORE0_ILM_SLV (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(core0_ilm_slv)), LENGTH = DT_REG_SIZE(DT_NODELABEL(core0_ilm_slv))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(core0_dlm_slv)) > 0) && !IS_CHOSEN_SRAM(core0_dlm_slv)
	CORE0_DLM_SLV (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(core0_dlm_slv)), LENGTH = DT_REG_SIZE(DT_NODELABEL(core0_dlm_slv))
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
}

#include <zephyr/arch/riscv/common/linker.ld>
#include <sections.ld>
