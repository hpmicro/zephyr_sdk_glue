/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/init.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include <board.h>
#include <zephyr/irq.h>
#include "hpm_pmp_drv.h"
#include "hpm_pllctl_drv.h"
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#ifdef CONFIG_XIP
__attribute__ ((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90001, 0x00000007, 0x0, 0x0};
uint32_t __fw_size__[] = {32768};
#endif
__attribute__((weak)) void _init_noncache(void)
{
	uint32_t i, size;

	extern uint8_t __noncacheable_data_load_start[];
	extern uint8_t __noncacheable_bss_start__[], __noncacheable_bss_end__[];
	extern uint8_t __noncacheable_init_start__[], __noncacheable_init_end__[];

	/* noncacheable bss section */
	size = __noncacheable_bss_end__ - __noncacheable_bss_start__;
	for (i = 0; i < size; i++) {
		*(__noncacheable_bss_start__ + i) = 0;
	}

	/* noncacheable init section LMA: etext + data length + ramfunc legnth */
	size = __noncacheable_init_end__ - __noncacheable_init_start__;
	for (i = 0; i < size; i++) {
		*(__noncacheable_init_start__ + i) = *(__noncacheable_data_load_start + i);
	}
}

static int hpmicro_soc_init(void)
{
	uint32_t key;

	key = irq_lock();
//TODO: fixed clock-control
	board_init_clock();
	board_init_pmp();

	irq_unlock(key);

	return 0;
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
