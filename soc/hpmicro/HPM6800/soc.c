/*
 * Copyright (c) 2024-2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <hpm_common.h>
#include <hpm_soc.h>
#include "hpm_clock_drv.h"
#ifdef CONFIG_NOCACHE_MEMORY
#include <zephyr/linker/linker-defs.h>
#include "hpm_pmp_drv.h"
#include "hpm_l1c_drv.h"
#endif
#ifdef CONFIG_XIP
#include "hpm_bootheader.h"
#endif

#ifdef CONFIG_XIP
__attribute__((section(".nor_cfg_option"))) const uint32_t option[4] = { 0xfcf90001, 0x00000007, 0x0, 0x0 };
__attribute__((section(".last_section"))) const uint32_t rom_marker = CONFIG_LINKER_LAST_SECTION_ID_PATTERN;
#endif
__attribute__((weak)) void c_startup(void)
{
}

static void soc_init_clock(void)
{
    /* Add most Clocks to group 0 */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_axic, 0);
    clock_add_to_group(clock_axis, 0);
    clock_add_to_group(clock_axiv, 0);
    clock_add_to_group(clock_axid, 0);
    clock_add_to_group(clock_axif, 0);
    clock_add_to_group(clock_axig, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_xram, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xdma, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_ptpc, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

#ifdef CONFIG_NOCACHE_MEMORY
static void soc_init_pma(void)
{
    volatile uint32_t start_addr = (uint32_t) &_nocache_ram_start;
    volatile uint32_t length = (uint32_t) &_nocache_ram_size;

    if (length == 0) {
        return;
    }

    /* Ensure the address and the length are power of 2 aligned */
    assert((length & (length - 1U)) == 0U);
    assert((start_addr & (length - 1U)) == 0U);

    pma_attr_t pma_attrs[1] = { 0 };
    pma_attrs[0].pma_addr = PMA_NAPOT_ADDR(start_addr, length);
    pma_attrs[0].pma_cfg.val = PMA_CFG(ADDR_MATCH_NAPOT, MEM_TYPE_MEM_NON_CACHE_BUF, AMO_EN);
    pma_config_attributes(&pma_attrs[0], ARRAY_SIZE(pma_attrs));

    /*
     * D-cache is already enabled in start.S. D45 does not update existing
     * cache states when accesses become non-cacheable. Flush dirty lines and
     * invalidate cached copies to keep the cacheability transition coherent.
     * l1c_dc_flush_all() issues L1D_WBINVAL_ALL, so it performs both operations.
     */
    l1c_dc_flush_all();
}
#endif

static int hpmicro_soc_init(void)
{
    uint32_t key;

    key = irq_lock();
    soc_init_clock();
#ifdef CONFIG_NOCACHE_MEMORY
    soc_init_pma();
#endif
    irq_unlock(key);

    return 0;
}

void arch_cache_init(void)
{
	
}

SYS_INIT(hpmicro_soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
