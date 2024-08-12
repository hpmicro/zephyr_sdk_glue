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

#if (DT_REG_SIZE(DT_NODELABEL(dram)) > 0) && !IS_CHOSEN_SRAM(dram)
	SDRAM (wx) : ORIGIN = DT_REG_ADDR(DT_NODELABEL(dram)), LENGTH = DT_REG_SIZE(DT_NODELABEL(dram))
#endif

}

#include <../common/linker.ld>
