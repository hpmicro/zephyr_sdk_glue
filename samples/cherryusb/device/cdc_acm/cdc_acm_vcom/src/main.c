/*
 * Copyright (c) 2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <zephyr/devicetree.h>

extern void cdc_acm_init(uint8_t busid, uint32_t reg_base);

int main(void)
{
    uint32_t udc0_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_udc0));

    printf("cherry usb cdc_acm device sample.\n");

    cdc_acm_init(0, udc0_base);

    return 0;
}
