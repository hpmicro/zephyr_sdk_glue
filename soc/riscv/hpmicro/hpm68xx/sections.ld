/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <zephyr/linker/linker-tool.h>
#include <zephyr/devicetree.h>

#ifdef CONFIG_XIP
__nor_cfg_option_load_addr__ = DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + 0x400;
__boot_header_load_addr__ = DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + 0x1000;
__app_load_addr__ = DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + 0x3000;
__boot_header_length__ = __boot_header_end__ - __boot_header_start__;
__app_offset__ = __app_load_addr__ - __boot_header_load_addr__;
#endif

SECTIONS
{
#ifdef CONFIG_XIP
	.nor_cfg_option __nor_cfg_option_load_addr__ : {
		KEEP(*(.nor_cfg_option))
	} GROUP_LINK_IN(MCU_BOOT)

	.boot_header __boot_header_load_addr__ : {
		__boot_header_start__ = .;
		KEEP(*(.boot_header))
		KEEP(*(.fw_info_table))
		KEEP(*(.dc_info))
		__boot_header_end__ = .;
	} GROUP_LINK_IN(MCU_BOOT)
#endif
	.start : {
		. = ALIGN(8);
		KEEP(*(.init.*))
		KEEP(*(.start))
	} GROUP_LINK_IN(ROMABLE_REGION)

	SECTION_PROLOGUE(.fast,,SUBALIGN(8))
	{
        PROVIDE(__ramfunc_start__ = .);
        *(.fast)
        *(.fast.*)
        PROVIDE(__ramfunc_end__ = .);
    } GROUP_LINK_IN(ITCM)

	SECTION_PROLOGUE(.noncacheable.init,,SUBALIGN(8))
	{
		__noncacheable_init_start__ = .;
		KEEP(*(.noncacheable.init))
		__noncacheable_init_end__ = .;
	} GROUP_LINK_IN(NONCACHEABLE AT> RAM)
	
	SECTION_PROLOGUE(.noncacheable.bss,(NOLOAD),SUBALIGN(8))
	{
		KEEP(*(.noncacheable))
		__noncacheable_bss_start__ = .;
		KEEP(*(.noncacheable.bss))
		__noncacheable_bss_end__ = .;
	} GROUP_LINK_IN(NONCACHEABLE AT> RAM)

	SECTION_PROLOGUE(.ahb_sram,(NOLOAD),)
	{
        KEEP(*(.ahb_sram))
	} GROUP_LINK_IN(AHB_SRAM)

	SECTION_PROLOGUE(.apb_sram,(NOLOAD),)
	{
        KEEP(*(.apb_sram))
	} GROUP_LINK_IN(APB_SRAM)

	SECTION_PROLOGUE(.fast_ram.init,,SUBALIGN(8))
	{
        __fast_ram_init_start__ = .;
        KEEP(*(.fast_ram.init))
        __fast_ram_init_end__ = .;
    } GROUP_LINK_IN(DTCM)

	SECTION_PROLOGUE(.fast_ram.bss,(NOLOAD),SUBALIGN(8))
	{
        KEEP(*(.fast_ram))
        __fast_ram_bss_start__ = .;
        KEEP(*(.fast_ram.bss))
        __fast_ram_bss_end__ = .;
    } GROUP_LINK_IN(DTCM)

    __fast_load_addr__ = LOADADDR(.fast);
    __fast_ram_init_load_addr__ = LOADADDR(.fast_ram.init);
	__noncacheable_init_load_addr__ = LOADADDR(.noncacheable.init);
	__noncacheable_start__ = ORIGIN(NONCACHEABLE);
	__noncacheable_end__ = ORIGIN(NONCACHEABLE) + LENGTH(NONCACHEABLE);
}
