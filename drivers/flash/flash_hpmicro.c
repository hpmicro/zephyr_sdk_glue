/*
 * Copyright (c) 2023-2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_xpi
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_NOR_CFG_OPT_HDR DT_PROP(SOC_NV_FLASH_NODE, nor_cfg_opt_hdr)
#define FLASH_NOR_CFG_OPT_OPT0 DT_PROP(SOC_NV_FLASH_NODE, nor_cfg_opt_opt0)
#define FLASH_NOR_CFG_OPT_OPT1 DT_PROP(SOC_NV_FLASH_NODE, nor_cfg_opt_opt1)

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/util.h>
#include "hpm_romapi.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
LOG_MODULE_REGISTER(flash_hpmicro, CONFIG_FLASH_LOG_LEVEL);

#define HPM_STATUS_ZEPHYR_RET(x)    (x)

static xpi_nor_config_t s_xpi_nor_config;

static uint32_t flash_size;
static uint32_t sector_size;
static uint32_t page_size;
static uint32_t block_size;

struct flash_hpmicro_dev_config {
	void *controller;
};
struct flash_hpmicro_dev_data {
	XPI_Type *controller;
};

static const struct flash_parameters flash_hpmicro_parameters = {
    .write_block_size = 4,
    .erase_value = 0xff,
};

static int flash_hpmicro_init(const struct device *dev);
static bool initted = false;
static int flash_hpmicro_read(const struct device *dev, off_t offset,
                void *data,
                size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    key = irq_lock();
    if (size < 4) {
        uint32_t temp;
        status = rom_xpi_nor_read(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                     &temp, offset, 4);
        memcpy(data, &temp, size);
    } else {
        status = rom_xpi_nor_read(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                     data, offset, size);
    }
    irq_unlock(key);

    return HPM_STATUS_ZEPHYR_RET(status);
}

static int flash_hpmicro_write(const struct device *dev, off_t offset,
                 const void *data, size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    key = irq_lock();
    status = rom_xpi_nor_program(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                        data, offset, size);
    irq_unlock(key);
    return HPM_STATUS_ZEPHYR_RET(status);
}

static int flash_hpmicro_erase(const struct device *dev, off_t offset,
                 size_t size)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
    hpm_stat_t status = 0;
	unsigned int key;
    if (!initted) {
        initted = true;
        flash_hpmicro_init(dev);
    }
    if (size < 4) {
        while (1) {
        }
    }
    key = irq_lock();
    for (int i = 0; i < size; i += (s_xpi_nor_config.device_info.sector_size_kbytes * 1024)) {
        status = rom_xpi_nor_erase_sector(dev_data->controller, xpi_xfer_channel_auto, &s_xpi_nor_config,
                                   offset + i);
        if (status != status_success) {
            break;
        }
    }
    irq_unlock(key);
    return HPM_STATUS_ZEPHYR_RET(status);
}

#if CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_hpm_pages_layout[] = {
    {
        .pages_count = PARTITION_OFFSET(boot_partition) / KB(4),
        .pages_size = KB(4),
    },
    {
        .pages_count = PARTITION_SIZE(boot_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = PARTITION_SIZE(slot0_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = PARTITION_SIZE(slot1_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = PARTITION_SIZE(scratch_partition) / KB(4),
        .pages_size = KB(4)
    },
    {
        .pages_count = PARTITION_SIZE(storage_partition) / KB(4),
        .pages_size = KB(4)
    }
};

void flash_hpmicro_page_layout(const struct device *dev,
                 const struct flash_pages_layout **layout,
                 size_t *layout_size)
{
    *layout = flash_hpm_pages_layout;
    *layout_size = ARRAY_SIZE(flash_hpm_pages_layout);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_hpmicro_get_parameters(const struct device *dev)
{
    return &flash_hpmicro_parameters;
}

static int flash_hpmicro_init(const struct device *dev)
{
	struct flash_hpmicro_dev_data *const dev_data = dev->data;
	unsigned int key;

    xpi_nor_config_option_t option;
    option.header.U = FLASH_NOR_CFG_OPT_HDR;
    option.option0.U = FLASH_NOR_CFG_OPT_OPT0;
    option.option1.U = FLASH_NOR_CFG_OPT_OPT1;

    key = irq_lock();
    hpm_stat_t status = rom_xpi_nor_auto_config(dev_data->controller, &s_xpi_nor_config, &option);
    if (status != status_success) {
        irq_unlock(key);
        return status;
    }

    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_total_size,
                             &flash_size);
    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_sector_size,
                             &sector_size);
    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_block_size,
                             &block_size);
    rom_xpi_nor_get_property(dev_data->controller, &s_xpi_nor_config, xpi_nor_property_page_size, &page_size);
    irq_unlock(key);

    initted = true;
    return 0;
}

static const struct flash_driver_api flash_hpmicro_driver_api = {
    .read = flash_hpmicro_read,
    .write = flash_hpmicro_write,
    .erase = flash_hpmicro_erase,
    .get_parameters = flash_hpmicro_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
    .page_layout = flash_hpmicro_page_layout,
#endif
};

static struct flash_hpmicro_dev_data flash_hpmicro_data = {
	.controller = (XPI_Type *)DT_INST_REG_ADDR(0),
};

static const struct flash_hpmicro_dev_config flash_hpmicro_config = {
	.controller = (XPI_Type *)DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, flash_hpmicro_init,
		      NULL,
		      &flash_hpmicro_data, &flash_hpmicro_config,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_hpmicro_driver_api);
