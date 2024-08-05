/*
 * Copyright (c) 2022 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef __RISCV_HPMICRO_SOC_H_
#define __RISCV_HPMICRO_SOC_H_

#include <soc_common.h>
#include <zephyr/devicetree.h>

/* Machine timer memory-mapped registers */
#define RISCV_MTIME_BASE             DT_REG_ADDR_BY_IDX(DT_NODELABEL(mtimer), 0)
#define RISCV_MTIMECMP_BASE          DT_REG_ADDR_BY_IDX(DT_NODELABEL(mtimer), 1)

#endif /* __RISCV_HPMICRO_SOC_H_ */
