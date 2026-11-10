/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-08-27   HPMicro         Adapt Zephyr 3.7.0
 */

#define DT_DRV_COMPAT hpmicro_hpm_rng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/random/random.h>
#include <zephyr/init.h>

#include "hpm_rng_drv.h"
#include "hpm_clock_drv.h"
#include <drivers/clock_control/hpmicro_clock_control.h>

struct hpm_entropy_config {
    RNG_Type *base;
    clock_name_t clock_name;
};

static int entropy_hpm_trng_get_entropy(const struct device *dev,
                     uint8_t *buffer,
                     uint16_t length)
{
    const struct hpm_entropy_config *config = dev->config;

    hpm_stat_t status = rng_rand_wait(config->base, buffer, length);
    if (status) {
        return 1;
    }

    return 0;
}

static const struct entropy_driver_api entropy_hpm_trng_api_funcs = {
    .get_entropy = entropy_hpm_trng_get_entropy
};

static struct hpm_entropy_config entropy_hpm_config = {
    .base = (RNG_Type *)DT_INST_REG_ADDR(0),
    .clock_name = (clock_name_t)DT_INST_PROP(0, clk_name),
};

static int entropy_hpm_trng_init(const struct device *dev)
{
    const struct hpm_entropy_config *config = dev->config;

    clock_enable_name(config->clock_name);

    hpm_stat_t status = rng_init(config->base);
    if (status != status_success) {
        return 1;
    }
    return 0;
}

DEVICE_DT_INST_DEFINE(0,
            entropy_hpm_trng_init, NULL, NULL,
            &entropy_hpm_config,
            PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
            &entropy_hpm_trng_api_funcs);
