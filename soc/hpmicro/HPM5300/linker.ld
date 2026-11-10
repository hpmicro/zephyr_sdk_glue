/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/linker/linker-tool.h>
#include <zephyr/devicetree.h>

#define HPM_RETAINED_NOINIT_REGION PGPR_REG

#include <../common/linker.ld>
