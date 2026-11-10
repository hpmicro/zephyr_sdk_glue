/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* HPM51/HPM53 DLM is inherently uncached and needs no PMP/PMA boundary. */
#if defined(CONFIG_SOC_SERIES_HPM5100) || defined(CONFIG_SOC_SERIES_HPM5300)
#define HPM_NOCACHE_ALIGN(region_size) . = ALIGN(4)
#else
/*
 * Andes PMA NAPOT regions are at least 4 KiB and naturally aligned. Keep
 * these constraints in the layout because LLD evaluates ASSERT expressions
 * that use the forward-defined region size before the layout converges.
 */
#define HPM_NOCACHE_ALIGN(region_size) \
	. = ALIGN(4096); \
	. = ALIGN(1 << LOG2CEIL(region_size))
#endif

SECTION_DATA_PROLOGUE(_NOCACHE_SECTION_NAME,(NOLOAD),)
{
	HPM_NOCACHE_ALIGN(_nocache_ram_size);
	_nocache_ram_start = .;
	_nocache_noload_ram_start = .;
	*(.nocache)
	*(".nocache.*")

#include <snippets-nocache-section.ld>

	_nocache_noload_ram_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
_nocache_noload_ram_size = _nocache_noload_ram_end - _nocache_noload_ram_start;

SECTION_DATA_PROLOGUE(_NOCACHE_LOAD_SECTION_NAME,,)
{
	_nocache_load_ram_start = .;
	*(.nocache_load)
	*(".nocache_load.*")
	HPM_NOCACHE_ALIGN(_nocache_ram_size);
	_nocache_load_ram_end = .;
	_nocache_ram_end = .;
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
_nocache_load_ram_size = _nocache_load_ram_end - _nocache_load_ram_start;
_nocache_load_rom_start = LOADADDR(_NOCACHE_LOAD_SECTION_NAME);
_nocache_ram_size = _nocache_ram_end - _nocache_ram_start;
