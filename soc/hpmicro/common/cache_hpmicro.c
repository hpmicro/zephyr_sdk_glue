/*
 * Copyright (c) 2026 Andes Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
#include "hpm_l1c_drv.h"

LOG_MODULE_REGISTER(cache_andes, CONFIG_CACHE_LOG_LEVEL);

/* micm_cfg bitfield */
#define MICM_CFG_ISET        BIT_MASK(3)
#define MICM_CFG_IWAY_SHIFT    3
#define MICM_CFG_ISZ_SHIFT    6

/* mdcm_cfg bitfield */
#define MDCM_CFG_DSZ_SHIFT    6

/* mmsc_cfg bitfield */
#define MMSC_CFG_CCTLCSR    BIT(16)
#define MMSC_CFG_VCCTL_2    BIT(19)
#define MMSC_CFG_MSC_EXT    BIT(31)
#define MMSC_CFG_RVARCH        BIT64(52)

struct cache_config {
    uint32_t instr_line_size;
    uint32_t data_line_size;
};

static struct cache_config cache_cfg;

void cache_data_enable(void)
{
    l1c_dc_enable();
}

void cache_data_disable(void)
{
    cache_data_flush_and_invd_all();

    l1c_dc_disable();
}

void cache_instr_enable(void)
{
    l1c_ic_enable();
}

void cache_instr_disable(void)
{
    l1c_ic_disable();
}

int cache_data_invd_all(void)
{
    l1c_dc_invalidate_all();

    return 0;
}

int cache_data_invd_range(void *addr, size_t size)
{
    l1c_dc_invalidate((uint32_t)addr, size);

    return 0;
}

int cache_instr_invd_all(void)
{
    unsigned long sets, ways, end;

    sets = 0x40 << (csr_read(CSR_MICM_CFG) & MICM_CFG_ISET);
    ways = ((csr_read(CSR_MICM_CFG) >> MICM_CFG_IWAY_SHIFT) & BIT_MASK(3)) + 1;
    end = ways * sets * cache_cfg.instr_line_size;

    for (int i = 0; i < end; i += cache_cfg.instr_line_size) {
        csr_write(CSR_MCCTLBEGINADDR, i);
        csr_write(CSR_MCCTLCOMMAND, HPM_L1C_CCTL_CMD_L1I_IX_INVAL);
    }

    return 0;
}

int cache_instr_invd_range(void *addr, size_t size)
{
    l1c_ic_invalidate((uint32_t)addr, size);

    return 0;
}

int cache_data_flush_all(void)
{
    l1c_dc_writeback_all();

    return 0;
}

int cache_data_flush_range(void *addr, size_t size)
{
    l1c_dc_writeback((uint32_t)addr, size);

    return 0;
}

int cache_data_flush_and_invd_all(void)
{
    l1c_dc_flush_all();

    return 0;
}

int cache_data_flush_and_invd_range(void *addr, size_t size)
{
    l1c_dc_flush((uint32_t)addr, size);

    return 0;
}

int cache_instr_flush_all(void)
{
    return -ENOTSUP;
}

int cache_instr_flush_and_invd_all(void)
{
    return -ENOTSUP;
}

int cache_instr_flush_range(void *addr, size_t size)
{
    ARG_UNUSED(addr);
    ARG_UNUSED(size);

    return -ENOTSUP;
}

int cache_instr_flush_and_invd_range(void *addr, size_t size)
{
    ARG_UNUSED(addr);
    ARG_UNUSED(size);

    return -ENOTSUP;
}

#if defined(CONFIG_DCACHE_LINE_SIZE_DETECT)
size_t cache_data_line_size_get(void)
{
    return cache_cfg.data_line_size;
}
#endif /* defined(CONFIG_DCACHE_LINE_SIZE_DETECT) */

#if defined(CONFIG_ICACHE_LINE_SIZE_DETECT)
size_t cache_instr_line_size_get(void)
{
    return cache_cfg.instr_line_size;
}
#endif /* defined(CONFIG_ICACHE_LINE_SIZE_DETECT) */

static int hpmicro_cache_init(void)
{
    unsigned long line_size;

    if (IS_ENABLED(CONFIG_ICACHE)) {
        line_size = (csr_read(CSR_MICM_CFG) >> MICM_CFG_ISZ_SHIFT) & BIT_MASK(3);

        if (line_size == 0) {
            LOG_ERR("Platform doesn't support I-cache, "
                "please disable CONFIG_ICACHE");
                        return -ENOTSUP;
        }
#if defined(CONFIG_ICACHE_LINE_SIZE_DETECT)
        /* Icache line size */
        if (line_size <= 5) {
            cache_cfg.instr_line_size = 1 << (line_size + 2);
        } else {
            LOG_ERR("Unknown line size of I-cache");
                        return -ENOTSUP;
        }
#elif (CONFIG_ICACHE_LINE_SIZE != 0)
        cache_cfg.instr_line_size = CONFIG_ICACHE_LINE_SIZE;
#elif DT_NODE_HAS_PROP(DT_PATH(cpus, cpu_0), i_cache_line_size)
        cache_cfg.instr_line_size =
            DT_PROP(DT_PATH(cpus, cpu_0), "i_cache_line_size");
#else
        LOG_ERR("Please specific the i-cache-line-size "
            "CPU0 property of the DT");
                return -ENOTSUP;
#endif /* defined(CONFIG_ICACHE_LINE_SIZE_DETECT) */
    }

    if (IS_ENABLED(CONFIG_DCACHE)) {
        line_size = (csr_read(CSR_MDCM_CFG) >> MDCM_CFG_DSZ_SHIFT) & BIT_MASK(3);
        if (line_size == 0) {
            LOG_ERR("Platform doesn't support D-cache, "
                "please disable CONFIG_DCACHE");
            return -ENOTSUP;
        }
#if defined(CONFIG_DCACHE_LINE_SIZE_DETECT)
        /* Dcache line size */
        if (line_size <= 5) {
            cache_cfg.data_line_size = 1 << (line_size + 2);
        } else {
            LOG_ERR("Unknown line size of D-cache");
            return -ENOTSUP;
        }
#elif (CONFIG_DCACHE_LINE_SIZE != 0)
        cache_cfg.data_line_size = CONFIG_DCACHE_LINE_SIZE;
#elif DT_NODE_HAS_PROP(DT_PATH(cpus, cpu_0), d_cache_line_size)
        cache_cfg.data_line_size =
            DT_PROP(DT_PATH(cpus, cpu_0), "d_cache_line_size");
#else
        LOG_ERR("Please specific the d-cache-line-size "
            "CPU0 property of the DT");
        return -ENOTSUP;
#endif /* defined(CONFIG_DCACHE_LINE_SIZE_DETECT) */
    }

    if (!(csr_read(CSR_MMSC_CFG) & MMSC_CFG_CCTLCSR)) {
        LOG_ERR("Platform doesn't support I/D cache operation");
                return -ENOTSUP;
    }

    return 0;
}

SYS_INIT(hpmicro_cache_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
