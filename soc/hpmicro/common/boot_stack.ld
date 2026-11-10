/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * External RAM is not available when _start begins. Reserve a small stack in
 * the always-on DTCM solely for _init_ext_ram(). The startup code switches to
 * Zephyr's interrupt stack as soon as external RAM has been initialized.
 */
SECTION_PROLOGUE(.hpm_boot_stack, (NOLOAD), SUBALIGN(16))
{
	__hpm_boot_stack_start = .;
	. += 0x400;
	__hpm_boot_stack_top = .;
} GROUP_LINK_IN(DTCM)
