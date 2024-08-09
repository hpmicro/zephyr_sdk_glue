/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

. = ALIGN(8);
KEEP(*(.isr))
KEEP(*(".exception.entry.*"))
*(".exception.other.*")