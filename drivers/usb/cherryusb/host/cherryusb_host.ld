/*
 * Copyright (c) 2025 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/* section information for usbh class */
. = ALIGN(8);
__usbh_class_info_start__ = .;
KEEP(*(.usbh_class_info))
__usbh_class_info_end__ = .;
