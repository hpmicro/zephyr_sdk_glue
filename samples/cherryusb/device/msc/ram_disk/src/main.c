/*
 * Copyright (c) 2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <zephyr/devicetree.h>

extern void msc_ram_init(uint8_t busid, uint32_t reg_base);

int main(void)
{
    uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));

    printf("cherry usb msc_ram sample.\n");

    msc_ram_init(0, usb_base);

    return 0;
}
