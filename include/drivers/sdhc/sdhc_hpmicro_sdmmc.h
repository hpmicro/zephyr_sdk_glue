/*
 * Copyright 2022 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_SDHC_SDHC_HPMICRO_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_SDHC_SDHCL_HPMICRO_COMMON_H_

/**
 * @brief SD/MMC Bus Width definitions
 */
typedef enum {
    sdmmc_bus_width_1bit = 0,   /* Bus width: 1-bit */
    sdmmc_bus_width_4bit = 1,   /* Bus width: 4-bit */
    sdmmc_bus_width_8bit = 2,   /* Bus width: 8-bit */
} sdmmc_buswidth_t;

typedef enum {
    sdmmc_resp_none = 0,
    sdmmc_resp_r1,
    sdmmc_resp_r1b,
    sdmmc_resp_r2,
    sdmmc_resp_r3,
    sdmmc_resp_r4,
    sdmmc_resp_r5,
    sdmmc_resp_r5b,
    sdmmc_resp_r6,
    sdmmc_resp_r7,
} sdmmc_resp_type_t;

/**
 * @brief Common SD/MMC commands
 */
enum {
    sdmmc_cmd_go_idle_state = 0,
    sdmmc_cmd_all_send_cid = 2,
    sdmmc_cmd_set_dsr = 4,
    sdmmc_cmd_select_card = 7,
    sdmmc_cmd_send_csd = 9,
    sdmmc_cmd_send_cid = 10,
    sdmmc_cmd_stop_transmission = 12,
    sdmmc_cmd_send_status = 13,
    sdmmc_cmd_go_inactive_state = 15,
    sdmmc_cmd_set_block_length = 16,
    sdmmc_cmd_read_single_block = 17,
    sdmmc_cmd_read_multiple_block = 18,
    sdmmc_cmd_set_block_count = 23,
    sdmmc_cmd_write_single_block = 24,
    sdmmc_cmd_write_multiple_block = 25,
    sdmmc_cmd_program_csd = 27,
    sdmmc_cmd_set_write_protect = 29,
    sdmmc_cmd_clear_write_protect = 30,
    sdmmc_cmd_erase = 38,
    sdmmc_cmd_lock_unlock = 42,
    sdmmc_cmd_app_cmd = 55,
    sdmmc_cmd_general_cmd = 56,
    sdmmc_cmd_read_ocr = 58,
};

/**
 * @brief SD Card specific commands
 */
enum {
    sd_acmd_set_bus_width = 6,
    sd_acmd_sd_status = 13,
    sd_acmd_set_num_wr_blocks = 22,
    sd_acmd_set_wr_blk_erase_count = 23,
    sd_acmd_sd_send_op_cond = 41,
    sd_acmd_set_clear_card_detect = 42,
    sd_acmd_send_scr = 51,

    sd_cmd_all_send_cid = 2,
    sd_cmd_send_relative_addr = 3,
    sd_cmd_switch = 6,
    sd_cmd_send_if_cond = 8,
    sd_cmd_send_csd = 9,
    sd_voltage_switch = 11,
    sd_cmd_send_tuning_block = 19,
    sd_cmd_erase_start = 32,
    sd_cmd_erase_end = 33,
    sd_cmd_crc_option = 59,
};

/**
 * @brief MMC specific commands
 */
enum {
    emmc_cmd_send_op_cond = 1,
    emmc_cmd_all_send_cid = sdmmc_cmd_all_send_cid,
    emmc_cmd_set_relative_addr = 3,
    emmc_cmd_set_dsr = sdmmc_cmd_set_dsr,
    emmc_cmd_sleep_awake = 5,
    emmc_cmd_switch = 6,
    emmc_cmd_select = sdmmc_cmd_select_card,
    emmc_cmd_send_ext_csd = 8,
    emmc_cmd_send_csd = sdmmc_cmd_send_csd,
    emmc_cmd_send_cid = sdmmc_cmd_send_cid,
    emmc_cmd_stop_transmission = sdmmc_cmd_stop_transmission,
    emmc_cmd_send_status = sdmmc_cmd_send_status,
    emmc_cmd_bus_test = 14,
    emmc_cmd_go_inactive_state = sdmmc_cmd_go_inactive_state,

    emmc_cmd_set_block_length = sdmmc_cmd_set_block_length,
    emmc_cmd_read_single_block = sdmmc_cmd_read_single_block,
    emmc_cmd_read_multiple_block = sdmmc_cmd_read_multiple_block,
    emmc_cmd_send_tuning_block = 21,
    emmc_cmd_set_block_count = sdmmc_cmd_set_block_count,
    emmc_cmd_write_single_block = sdmmc_cmd_write_single_block,
    emmc_cmd_write_multiple_block = sdmmc_cmd_write_multiple_block,
    emmc_cmd_program_cid = 26,
    emmc_cmd_program_csd = sdmmc_cmd_program_csd,
    emmc_cmd_set_time = 49,

    emmc_cmd_erase_group_start = 35,
    emmc_cmd_erase_group_end = 36,
    emmc_cmd_erase = sdmmc_cmd_erase,

    emmc_cmd_set_write_prot = 28,
    emmc_cmd_clear_write_prot = 29,
    emmc_cmd_send_write_prot = 30,
    emmc_cmd_send_write_prot_type = 31,

    emmc_cmd_fast_io = 39,
    emmc_cmd_go_irq_state = 40,
    emmc_cmd_lock_unlock = sdmmc_cmd_lock_unlock,

    emmc_cmd_app_cmd = sdmmc_cmd_app_cmd,
    emmc_cmd_gen_cmd = sdmmc_cmd_general_cmd,

    emmc_cmd_protocol_read = 53,
    emmc_cmd_protocol_write = 54,

    emmc_cmd_queued_task_params = 44,
    emmc_cmd_queued_task_address = 45,
    emmc_cmd_execute_read_task = 46,
    emmc_cmd_execute_write_task = 47,
    emmc_cmd_cmdq_task_mgmt = 48,
};

typedef enum {
    sdmmc_dev_type_emmc = 0,    /* Device type is eMMC */
    sdmmc_dev_type_sd = 1,      /* Device type is SD */
    sdmmc_dev_type_sdio = 2,    /* Device type is SDIO */
} sdmmc_dev_type_t;

#endif /* ZEPHYR_INCLUDE_DRIVERS_SDHC_SDHCL_HPMICRO_COMMON_H_ */

