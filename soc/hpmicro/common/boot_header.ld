/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

. = CONFIG_BOOT_NOR_CFG_OPTION_OFFSET;
KEEP(*(.nor_cfg_option))
. = CONFIG_BOOT_HEADER_LOAD_OFFSET;
KEEP(*(.boot_header))
KEEP(*(.fw_info_table))
KEEP(*(.dc_info))
. = CONFIG_APP_LOAD_ADDR_OFFSET;
. = ALIGN(8);
KEEP(*(.init.*))
KEEP(*(.start))