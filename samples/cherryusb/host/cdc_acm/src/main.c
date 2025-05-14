/*
 * Copyright (c) 2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <zephyr/devicetree.h>
#include "usbh_core.h"

int main(void)
{
    uint32_t uhc0_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_uhc0));

    printf("Start usb host task...\r\n");

    usbh_initialize(0, uhc0_base);

    return 0;
}