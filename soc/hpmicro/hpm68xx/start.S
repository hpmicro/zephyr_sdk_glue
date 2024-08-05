/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/toolchain.h>
#include <soc.h>
#include "hpm_csr_regs.h"

GTEXT(_start)

SECTION_FUNC(init, _start)
	/* reset mstatus to 0*/
	csrrw x0, mstatus, x0

#ifdef CONFIG_INIT_EXT_RAM
    la t0, _stack_safe
    mv sp, t0
    call _init_ext_ram
#endif

#ifdef __nds_execit
	/* Initialize EXEC.IT table */
	la t0, _ITB_BASE_
	csrw uitb, t0
#endif
	call c_startup

	/* System reset handler */
	j __start
