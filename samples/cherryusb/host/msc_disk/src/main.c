/*
 * Copyright (c) 2025 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include "usbh_core.h"

extern int msc_main(void);

int main(void)
{
    uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));

    printf("Start usb host task...\r\n");

    usbh_initialize(0, usb_base);

    while(usbh_find_class_instance("/dev/sda") == NULL)
    {
        k_sleep(K_MSEC(1000));
    }

    msc_main();

    return 0;
}