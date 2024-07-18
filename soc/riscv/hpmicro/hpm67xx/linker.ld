/*
 * Copyright (c) 2022 HPMicro
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
#if (DT_REG_SIZE(DT_NODELABEL(axi_sram)) > 0) && !IS_CHOSEN_SRAM(sram)
	SRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(axi_sram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(axi_sram))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(ext_ram)) > 0) && !IS_CHOSEN_SRAM(sdram)
	SDRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(ext_ram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(ext_ram))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(axi_noncache)) > 0) && IS_CHOSEN_SRAM(sram)
	NONCACHEABLE (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(axi_noncache)), LENGTH = DT_REG_SIZE(DT_NODELABEL(axi_noncache))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(sdram_noncache)) > 0) && IS_CHOSEN_SRAM(sdram)
	NONCACHEABLE (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(sdram_noncache)), LENGTH = DT_REG_SIZE(DT_NODELABEL(sdram_noncache))
#endif
#if (DT_REG_SIZE(DT_NODELABEL(share_ram)) > 0)
	SHARE_RAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(share_ram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(share_ram))
#endif
}

#include <../common/linker.ld>
#include <sections.ld>
