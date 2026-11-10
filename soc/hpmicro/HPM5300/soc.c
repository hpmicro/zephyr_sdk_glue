/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"

#ifdef CONFIG_XIP
#include "hpm_bootheader.h"

__attribute__((section(".nor_cfg_option"), used)) const uint32_t option[4] = {
	0xfcf90002, 0x00000005, 0x00001000, 0x0
};
__attribute__((section(".last_section"))) const uint32_t rom_marker =
	CONFIG_LINKER_LAST_SECTION_ID_PATTERN;
#endif

__attribute__((weak)) void c_startup(void)
{
}

static void soc_init_clock(void)
{
	clock_add_to_group(clock_cpu0, 0);
	clock_add_to_group(clock_ahb, 0);
	clock_add_to_group(clock_lmm0, 0);
	clock_add_to_group(clock_mchtmr0, 0);
	clock_add_to_group(clock_rom, 0);
	clock_add_to_group(clock_gpio, 0);
	clock_add_to_group(clock_hdma, 0);
	clock_add_to_group(clock_xpi0, 0);
	clock_connect_group_to_cpu(0, 0);

	clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

static int hpmicro_soc_init(void)
{
	uint32_t key = irq_lock();

	soc_init_clock();
	irq_unlock(key);

	return 0;
}

void arch_cache_init(void)
{
	
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
