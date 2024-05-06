/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_HPMICRO_COMMON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_HPMICRO_COMMON_H_

/**
 * @brief Clock source group Marcos
 */
#define MAKE_CLK_SRC(src_grp, index) (((uint8_t)(src_grp) << 4) | (index))
#define GET_CLK_SRC_GROUP(src) (((uint8_t)(src) >> 4) & 0x0FU)
#define GET_CLK_SRC_INDEX(src) ((uint8_t)(src)&0x0FU)
#define GET_CLOCK_SOURCE_FROM_CLK_SRC(clk_src) (clock_source_t)((uint32_t)(clk_src) & 0xFU)
#define RESOURCE_INVALID (0xFFFFU)

/* Clock NAME related Macros */
#define MAKE_CLOCK_NAME(resource, src_type, node) (((uint32_t)(resource) << 16) | ((uint32_t)(src_type) << 8) | ((uint32_t)node))
#define GET_CLK_SRC_GROUP_FROM_NAME(name) (((uint32_t)(name) >> 8) & 0xFFUL)
#define GET_CLK_NODE_FROM_NAME(name) ((uint32_t)(name)&0xFFUL)
#define GET_CLK_RESOURCE_FROM_NAME(name) ((uint32_t)(name) >> 16)

#endif
