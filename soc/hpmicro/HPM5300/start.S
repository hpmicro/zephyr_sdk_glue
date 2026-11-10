/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/offsets.h>
#include <soc.h>
#include "hpm_csr_regs.h"

GTEXT(_start)

SECTION_FUNC(init, _start)
	/* Reset mstatus to 0. */
	csrrw x0, mstatus, x0

#ifdef CONFIG_INIT_EXT_RAM
	la sp, __hpm_boot_stack_top
	call _init_ext_ram
#endif

	/* Use Zephyr's interrupt stack after the main RAM is available. */
	la sp, z_interrupt_stacks
	li t0, __z_interrupt_stack_SIZEOF
	add sp, sp, t0

#ifdef CONFIG_ICACHE
	call l1c_ic_enable
#else
	call l1c_ic_disable
#endif

#ifdef CONFIG_DCACHE
	call l1c_dc_enable
	call l1c_dc_invalidate_all
#else
	call l1c_dc_invalidate_all
	call l1c_dc_disable
#endif

#ifdef __nds_execit
	/* Initialize EXEC.IT table. */
	la t0, _ITB_BASE_
	csrw uitb, t0
#endif
	call c_startup

	/* System reset handler. */
	j __start
