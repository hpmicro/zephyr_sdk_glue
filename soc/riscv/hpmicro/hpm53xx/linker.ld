/*
 * Copyright (c) 2022 hpmicro
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
	MCU_BOOT (wx) :  ORIGIN = DT_REG_ADDR(DT_CHOSEN(zephyr_flash)), LENGTH = 0x3000
#endif
#if (DT_REG_SIZE(DT_NODELABEL(core0_ilm_slv)) > 0) && !IS_CHOSEN_SRAM(core0_ilm_slv)
	CORE0_ILM_SLV (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(core0_ilm_slv)), LENGTH = DT_REG_SIZE(DT_NODELABEL(core0_ilm_slv))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(core0_dlm_slv)) > 0) && !IS_CHOSEN_SRAM(core0_dlm_slv)
	CORE0_DLM_SLV (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(core0_dlm_slv)), LENGTH = DT_REG_SIZE(DT_NODELABEL(core0_dlm_slv))
#endif
}

#include <zephyr/arch/riscv/common/linker.ld>
#include <sections.ld>
