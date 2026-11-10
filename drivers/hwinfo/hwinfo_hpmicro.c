/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-08-27   HPMicro         Adapt Zephyr 3.7.0
 */

#define DT_DRV_COMPAT hpmicro_hpm_hwinfo

#include <soc.h>
#include <zephyr/drivers/hwinfo.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include "hpm_common.h"
#include "hpm_ppor_drv.h"
#include "hpm_otp_drv.h"
#include <zephyr/devicetree.h>

#define OTP_SOC_UUID_IDX 88
#define OTP_SOC_UUID_LEN 16

#define WATCHDOG_RESET_FLAGS ( \
    ppor_reset_wdog0 | \
    ppor_reset_wdog1   \
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(wdog2), okay), (| ppor_reset_wdog2)) \
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(wdog3), okay), (| ppor_reset_wdog3)) \
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(pwatchdog), okay), (| ppor_reset_pmic_wdog)) \
)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "No hpmicro,hpm-hwinfo compatible device found");

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
    uint32_t flags = 0;
    uint32_t reason = ppor_reset_get_flags((PPOR_Type *)DT_INST_REG_ADDR(0));

    if (reason & ppor_reset_brownout) {
        flags |= RESET_BROWNOUT;
    }

    if (reason & ppor_reset_debug) {
        flags |= RESET_DEBUG;
    }

    if (reason & WATCHDOG_RESET_FLAGS) {
        flags |= RESET_WATCHDOG;
    }

    if (reason & ppor_reset_software) {
        flags |= RESET_SOFTWARE;
    }

    *cause = flags;

    return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
    uint32_t reason = -1;

    ppor_reset_clear_flags((PPOR_Type *)DT_INST_REG_ADDR(0), reason);
    return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
    uint32_t zephyr_supported_causes = 0;

    if (ppor_reset_brownout & 0xFFFFFFFF) {
        zephyr_supported_causes |= RESET_BROWNOUT;
    }

    if (ppor_reset_debug & 0xFFFFFFFF) {
        zephyr_supported_causes |= RESET_DEBUG;
    }

    if (WATCHDOG_RESET_FLAGS & 0xFFFFFFFF) {
        zephyr_supported_causes |= RESET_WATCHDOG;
    }

    if (ppor_reset_software & 0xFFFFFFFF) {
        zephyr_supported_causes |= RESET_SOFTWARE;
    }

    *supported = zephyr_supported_causes;

    return 0;
}

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
    uint32_t uuid_words[4];

    if (length <= 0 || OTP_SOC_UUID_LEN <= 0) {
        return -ENOSYS;
    }

    if (length > OTP_SOC_UUID_LEN) {
        length = OTP_SOC_UUID_LEN;
    }
    
    for (uint32_t i = 0; i < 4; i++) {
        uuid_words[i] = otp_read_from_shadow(OTP_SOC_UUID_IDX + i);
    }

    memcpy(buffer, (uint8_t *)uuid_words, length);

    return length;
}
