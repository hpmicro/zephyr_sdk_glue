/*
 * Copyright (c) 2022-2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date         Author          Notes
 * 2024-08-27   HPMicro         Adapt Zephyr 3.7.0
 */

#define DT_DRV_COMPAT hpmicro_hpm_sdhc

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>

#include "hpm_common.h"
#include "hpm_sdxc_drv.h"
#include <drivers/sdhc/sdhc_hpmicro_sdmmc.h>
#include "hpm_clock_drv.h"

LOG_MODULE_REGISTER(hpmicro_hpm_sdhc, CONFIG_SDHC_LOG_LEVEL);

#define PINCTRL_STATE_SLOW PINCTRL_STATE_PRIV_START
#define PINCTRL_STATE_MED (PINCTRL_STATE_PRIV_START + 1U)
#define PINCTRL_STATE_FAST (PINCTRL_STATE_PRIV_START + 2U)
#define PINCTRL_STATE_NOPULL (PINCTRL_STATE_PRIV_START + 3U)

/* max buffer = 256*512 + 128 adma desc; and satisfy 2^x non-cache buffer size*/
#define DUMMY_BUF_LEN 0x40000 - 128
#define CONFIG_HPM_SDHC_DMA_BUFFER_SIZE 128
__attribute__((__section__(".nocache"))) static uint32_t dummy_buffer[DUMMY_BUF_LEN/4];
extern uint32_t hpm_board_sd_configure_clock(SDXC_Type *ptr, uint32_t freq, bool need_inverse);

struct hpm_sdhc_config {
    SDXC_Type *base;
    clock_name_t clock_name;
    clock_source_t clock_src;
    uint32_t clock_div;
    const struct gpio_dt_spec pwr_gpio;
    const struct gpio_dt_spec detect_gpio;
    uint32_t power_delay_ms;
    uint32_t max_bus_freq;
    uint32_t min_bus_freq;
    bool mmc_hs200_1_8v;
    bool mmc_hs400_1_8v;
    bool pwr_3v3_support;
    bool pwr_3v0_support;
    bool pwr_1v8_support;
    bool detect_gpio_support;
    bool embedded_4_bit_support;
    const struct pinctrl_dev_config *pincfg;
    void (*irq_config_func)(const struct device *dev);
};

struct hpm_sdhc_data {
    const struct device *dev;
    struct sdhc_host_props props;
    sdmmc_dev_type_t dev_type;
    bool card_present;
    struct k_sem transfer_sem;
    volatile uint32_t transfer_status;
    struct sdhc_io host_io;
    struct k_mutex access_mutex;
    sdhc_interrupt_cb_t sdhc_cb;
    struct gpio_callback cd_callback;
    void *sdhc_cb_user_data;
    uint32_t *hpm_sdhc_dma_descriptor; /* ADMA descriptor table (noncachable) */
    uint32_t dma_descriptor_len; /* DMA descriptor table length in words */
};

static int hpm_sdhc_get_card_present(const struct device *dev);

static void card_detect_gpio_cb(const struct device *port,
                struct gpio_callback *cb,
                gpio_port_pins_t pins)
{
    struct hpm_sdhc_data *data = CONTAINER_OF(cb, struct hpm_sdhc_data, cd_callback);
    const struct device *dev = data->dev;
    const struct hpm_sdhc_config *cfg = dev->config;

    if (data->sdhc_cb) {
        if (gpio_pin_get_dt(&cfg->detect_gpio)) {
            data->sdhc_cb(dev, SDHC_INT_INSERTED, data->sdhc_cb_user_data);
        } else {
            data->sdhc_cb(dev, SDHC_INT_REMOVED, data->sdhc_cb_user_data);
        }
    }
}

/*
 * Initialize SDHC host properties for use in get_host_props api call
 */
static void hpm_sdhc_init_host_props(const struct device *dev)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    struct hpm_sdhc_data *data = dev->data;
    sdxc_capabilities_t caps;
    struct sdhc_host_props *props = &data->props;

    memset(props, 0, sizeof(struct sdhc_host_props));
    props->f_max = cfg->max_bus_freq;
    props->f_min = cfg->min_bus_freq;
    props->power_delay = cfg->power_delay_ms;
    props->is_spi = false;
    /* Read host capabilities */
    sdxc_get_capabilities(cfg->base, &caps);

    props->max_current_330 = caps.curr_capabilities1.max_current_3v3;
    props->max_current_300 = caps.curr_capabilities1.max_current_3v0;
    props->max_current_180 = caps.curr_capabilities1.max_current_1v8;

    if (cfg->pwr_1v8_support) {
        props->host_caps.vol_180_support = caps.capabilities1.voltage_1v8_support;
    } else {
        props->host_caps.vol_180_support = 0;
    }
    if (cfg->pwr_3v3_support) {
        props->host_caps.vol_330_support = caps.capabilities1.voltage_3v3_support;
    } else {
        props->host_caps.vol_330_support = 0;
    }
    if (cfg->pwr_3v0_support) {
        props->host_caps.vol_300_support = caps.capabilities1.voltage_3v0_support;
    } else {
        props->host_caps.vol_300_support = 0;
    }
    props->host_caps.suspend_res_support = caps.capabilities1.suspend_resume_support;
    props->host_caps.sdma_support = caps.capabilities1.sdma_support;
    props->host_caps.high_spd_support = caps.capabilities1.high_speed_support;
    props->host_caps.adma_2_support = caps.capabilities1.adma2_support;
    props->host_caps.max_blk_len = caps.capabilities1.max_blk_len;
    props->host_caps.ddr50_support = caps.capabilities2.ddr50_support;
    props->host_caps.sdr104_support = caps.capabilities2.sdr104_support;
    props->host_caps.sdr50_support = caps.capabilities2.sdr50_support;
    props->host_caps.bus_8_bit_support = caps.capabilities1.embedded_8_bit_support;
    props->host_caps.bus_4_bit_support = cfg->embedded_4_bit_support;
    props->host_caps.hs200_support = cfg->mmc_hs200_1_8v;
    props->host_caps.hs400_support = cfg->mmc_hs400_1_8v;
}

/*
 * Reset SDHC controller
 */
static int hpm_sdhc_reset(const struct device *dev)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    SDXC_Type *base = cfg->base;
    bool status = false;

    status = sdxc_reset(base, sdxc_reset_cmd_line, 0xFFFFU);
    if (!status) {
        return -EIO;
    }

    status = sdxc_reset(base, sdxc_reset_data_line, 0xFFFFU);
    if (!status) {
        return -EIO;
    }
    
    return 0;
}

/*
 * Set SDHC io properties
 */
static int hpm_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    struct hpm_sdhc_data *data = dev->data;
    struct sdhc_io *host_io = &data->host_io;
    uint32_t bus_clk;
    SDXC_Type *base = cfg->base;

    LOG_DBG("SDHC I/O: bus width %d, clock %dHz, card power %s, voltage %s",
        ios->bus_width,
        ios->clock,
        ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
        ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V"
        );

    if (ios->clock && (ios->clock > data->props.f_max || ios->clock < data->props.f_min)) {
        return -EINVAL;
    }

    /* Set host clock */
    if (host_io->clock != ios->clock) {
        if (ios->clock != 0) {
            /* Enable the clock output */
            if (ios->timing == SDHC_TIMING_DDR50) {
                bus_clk = hpm_board_sd_configure_clock(base, ios->clock, false);
            } else {
                bus_clk = hpm_board_sd_configure_clock(base, ios->clock, true);
            }
            LOG_DBG("BUS CLOCK: %d", bus_clk);
            if (bus_clk == 0) {
                return -ENOTSUP;
            }
        }
        host_io->clock = ios->clock;
    }

    /* Set bus width */
    if (host_io->bus_width != ios->bus_width) {
        switch (ios->bus_width) {
        case SDHC_BUS_WIDTH1BIT:
            sdxc_set_data_bus_width(base, sdxc_bus_width_1bit);
            break;
        case SDHC_BUS_WIDTH4BIT:
            sdxc_set_data_bus_width(base, sdxc_bus_width_4bit);
            break;
        case SDHC_BUS_WIDTH8BIT:
            sdxc_set_data_bus_width(base, sdxc_bus_width_8bit);
            break;
        default:
            return -ENOTSUP;
        }
        host_io->bus_width = ios->bus_width;
    }

    /* Set host signal voltage */
    if (ios->signal_voltage != host_io->signal_voltage) {
        switch (ios->signal_voltage) {
        case SD_VOL_3_3_V:
        case SD_VOL_3_0_V:
            sdxc_select_voltage(base, sdxc_bus_voltage_sd_3v3);
            break;
        case SD_VOL_1_8_V:
            /* 1. Stop providing clock to the card */
            sdxc_enable_inverse_clock(base, false);
            sdxc_enable_sd_clock(base, false);
            /* 2. Wait until DAT[3:0] are 4'b0000 */
            uint32_t data3_0_level;
            uint32_t delay_cnt = 1000000UL;
            do {
                data3_0_level = sdxc_get_data3_0_level(base);
                --delay_cnt;
            } while ((data3_0_level != 0U) && (delay_cnt > 0U));
            if (delay_cnt < 1) {
                return -EIO;
            }
            /* 3. Switch signaling to 1.8v */
            sdxc_select_voltage(base, sdxc_bus_voltage_sd_1v8);
            /* 4. delay 5ms */
            k_msleep(5);
            /* 5. Provide SD clock the card again */
            sdxc_enable_inverse_clock(base, true);
            sdxc_enable_sd_clock(base, true);
            /* 6. wait 1ms */
            k_msleep(1);
            /* 7. Check DAT[3:0], make sure the value is 4'b0000 */
            delay_cnt = 1000000UL;
            do {
                data3_0_level = sdxc_get_data3_0_level(base);
                --delay_cnt;
            } while ((data3_0_level == 0U) && (delay_cnt > 0U));
            if (delay_cnt < 1) {
                return -EIO;
            }
            break;
        default:
            return -ENOTSUP;
        }
        /* Save new host voltage */
        host_io->signal_voltage = ios->signal_voltage;
    }

    /* Set card power */
    if ((host_io->power_mode != ios->power_mode) && (cfg->pwr_gpio.port)) {
        if (ios->power_mode == SDHC_POWER_OFF) {
            gpio_pin_set_dt(&cfg->pwr_gpio, 0);
        } else if (ios->power_mode == SDHC_POWER_ON) {
            gpio_pin_set_dt(&cfg->pwr_gpio, 1);
        }
        host_io->power_mode = ios->power_mode;
    } else {
        if (ios->power_mode == SDHC_POWER_OFF) {
            sdxc_enable_power(base, false);
        } else if (ios->power_mode == SDHC_POWER_ON) {
            sdxc_enable_power(base, true);
        }
        host_io->power_mode = ios->power_mode;
    }

    /* Set I/O timing */
    if (host_io->timing != ios->timing) {
        switch (ios->timing) {
        case SDHC_TIMING_LEGACY:
        case SDHC_TIMING_HS:
            break;
        case SDHC_TIMING_DDR50:
        case SDHC_TIMING_DDR52:
        case SDHC_TIMING_SDR12:
        case SDHC_TIMING_SDR25:
            pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_SLOW);
            break;
        case SDHC_TIMING_SDR50:
            pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_MED);
            break;
        case SDHC_TIMING_HS400:
        case SDHC_TIMING_SDR104:
        case SDHC_TIMING_HS200:
            pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_FAST);
            break;
        default:
            return -ENOTSUP;
        }
        host_io->timing = ios->timing;
    }

    return 0;
}

/*
 * Return 0 if card is not busy, 1 if it is
 */
static int hpm_sdhc_card_busy(const struct device *dev)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    SDXC_Type *base = cfg->base;
    bool is_busy =0;

    uint32_t data3_0_level = sdxc_get_data3_0_level(base);
    if (!data3_0_level) {
        is_busy =1;
    } else {
        is_busy =0;
    }

    return is_busy;
}

/*
 * Execute card tuning
 */
static int hpm_sdhc_execute_tuning(const struct device *dev)
{
    struct hpm_sdhc_data *dev_data = dev->data;
    const struct hpm_sdhc_config *cfg = dev->config;
    SDXC_Type *base = cfg->base;
    hpm_stat_t status = status_invalid_argument;
    uint8_t tuning_cmd;

    /* Prepare the Auto tuning environment */
    sdxc_stop_clock_during_phase_code_change(base, true);
    sdxc_set_post_change_delay(base, 3U);
    sdxc_select_cardclk_delay_source(base, false);
    sdxc_enable_power(base, true);

    if (dev_data->dev_type == sdmmc_dev_type_sd) {
        tuning_cmd = 19U;        
    } else {    
        tuning_cmd = 21U;    
    }
    status = sdxc_perform_auto_tuning(base, tuning_cmd);

    return status;
}

static hpm_stat_t hpm_sdhc_send_command(const struct device *dev, sdxc_command_t *cmd)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    SDXC_Type *base = cfg->base;
    hpm_stat_t status;
    int ret;

    ret = hpm_sdhc_get_card_present(dev);
    if (!ret) {
        LOG_DBG("No card detected!");
        return status_fail;
    }

    status = sdxc_send_command(base, cmd);
    if (status != status_success) {
        return status;
    }

    int64_t delay_cnt = 1000000;
    uint32_t int_stat;
    bool has_done_or_error = false;
    do {
        int_stat = sdxc_get_interrupt_status(base);
        if (!IS_HPM_BITMASK_SET(int_stat, SDXC_INT_STAT_CMD_COMPLETE_MASK)) {
            delay_cnt--;

        } else {
            has_done_or_error = true;
        }

        status = sdxc_parse_interrupt_status(base);
        if (status != status_success) {
            has_done_or_error = true;
        }

    } while ((!has_done_or_error) && (delay_cnt > 0));

    if ((delay_cnt <= 0) && (!has_done_or_error)) {
        status = status_timeout;
        return status;
    }
    if (status != status_success) {
        return status;
    }
    status = sdxc_receive_cmd_response(base, cmd);
    sdxc_clear_interrupt_status(base, SDXC_INT_STAT_CMD_COMPLETE_MASK);

    if (cmd->resp_type == (sdxc_dev_resp_type_t) sdmmc_resp_r1b) {
        uint32_t delay_ms = (cmd->cmd_timeout_ms == 0) ? 100 : cmd->cmd_timeout_ms;
        delay_cnt = 10 * 1000 * delay_ms;
        has_done_or_error = false;
        do {
            int_stat = sdxc_get_interrupt_status(base);
            if (!IS_HPM_BITMASK_SET(int_stat, SDXC_INT_STAT_XFER_COMPLETE_MASK)) {
                delay_cnt--;
            } else {
                has_done_or_error = true;
            }
            status = sdxc_parse_interrupt_status(base);
            if (status != status_success) {
                has_done_or_error = true;
            }
        } while ((!has_done_or_error) && (delay_cnt > 0));

        if ((delay_cnt <= 0) && (!has_done_or_error)) {
            status = status_timeout;
        }

        sdxc_clear_interrupt_status(base, SDXC_INT_STAT_XFER_COMPLETE_MASK);
    }

    return status;
}

static hpm_stat_t hpm_sdhc_send_content(const struct device *dev, sdxc_xfer_t *content)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    SDXC_Type *base = cfg->base;
    struct hpm_sdhc_data *dev_data = dev->data;
    hpm_stat_t status;
    int ret;

    ret = hpm_sdhc_get_card_present(dev);
    if (!ret) {
        LOG_DBG("No card detected!");
        return status_fail;
    }

    sdxc_adma_config_t *config_ptr = NULL;
    sdxc_adma_config_t dma_config;

    if (content->data != NULL) {
        dma_config.dma_type = sdxc_dmasel_adma2;
        dma_config.adma_table_words = dev_data->dma_descriptor_len / sizeof(uint32_t);
        dma_config.adma_table = dev_data->hpm_sdhc_dma_descriptor;
        config_ptr = &dma_config;

        uint32_t bus_width = sdxc_get_data_bus_width(base);
        uint32_t tx_rx_bytes_per_sec = dev_data->host_io.clock * bus_width / 8;
        uint32_t block_cnt = content->data->block_cnt;
        uint32_t block_size = content->data->block_size;
        uint32_t read_write_size = block_cnt * block_size;
        uint32_t timeout_ms = (uint32_t) (1.0f * read_write_size / tx_rx_bytes_per_sec) * 1000 + 500;
        sdxc_set_data_timeout(base, timeout_ms, NULL);
    }
    status = sdxc_transfer_nonblocking(base, config_ptr, content);

    int32_t delay_cnt = 1000000U;
    uint32_t int_stat;
    bool has_done_or_error = false;
    do {
        int_stat = sdxc_get_interrupt_status(base);
        if (!IS_HPM_BITMASK_SET(int_stat, SDXC_INT_STAT_CMD_COMPLETE_MASK)) {
            delay_cnt--;
        } else {
            has_done_or_error = true;
        }

        status = sdxc_parse_interrupt_status(base);
        if (status != status_success) {
            has_done_or_error = true;
        }
    } while ((!has_done_or_error) && (delay_cnt > 0));

    if ((delay_cnt <= 0) && (!has_done_or_error)) {
        status = status_timeout;
        return status;
    }
    if (status != status_success) {
        return status;
    }
    status = sdxc_receive_cmd_response(base, content->command);

    sdxc_clear_interrupt_status(base, SDXC_INT_STAT_CMD_COMPLETE_MASK);

    if ((content->data != NULL) || (content->command->resp_type == (sdxc_dev_resp_type_t) sdmmc_resp_r1b)) {
        delay_cnt = 10000000UL; /* Delay more than 1 second based on the Bus Frequency */
        uint32_t xfer_done_or_error_mask = SDXC_INT_STAT_XFER_COMPLETE_MASK | SDXC_STS_ERROR;
        has_done_or_error = false;
        do {
            int_stat = sdxc_get_interrupt_status(base);
            if (!IS_HPM_BITMASK_SET(int_stat, xfer_done_or_error_mask)) {
                delay_cnt--;
            } else {
                has_done_or_error = true;
            }
            status = sdxc_parse_interrupt_status(base);
            if (status != status_success) {
                has_done_or_error = true;
            }
        } while ((delay_cnt > 0) && (!has_done_or_error));

        if (delay_cnt <= 0) {
            status = status_sdxc_data_timeout_error;
        }

        sdxc_clear_interrupt_status(base, SDXC_INT_STAT_XFER_COMPLETE_MASK);
    }

    return status;
}

static hpm_stat_t hpm_sdhc_error_recovery(const struct device *dev)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    SDXC_Type *base = cfg->base;
    sdxc_command_t *recovery_cmd = {0};
    recovery_cmd->cmd_index = sdmmc_cmd_stop_transmission;
    recovery_cmd->cmd_type = sdxc_cmd_type_abort_cmd;
    recovery_cmd->resp_type = (sdxc_dev_resp_type_t) sdmmc_resp_r1b;

    return sdxc_error_recovery(base, recovery_cmd);
}

/*
 * Send CMD/DATA via SDHC
 */
static hpm_stat_t hpm_sdhc_transfer(const struct device *dev, struct sdhc_command *cmd,
    struct sdhc_data *data)
{
    struct hpm_sdhc_data *dev_data = dev->data;
    volatile sdxc_command_t host_cmd = {0};
    sdxc_xfer_t host_content;
    int retries = (int)cmd->retries;
    hpm_stat_t status;
    bool need_cp =0;
    uint32_t cp_write_size =0;

    if (k_mutex_lock(&dev_data->access_mutex, K_MSEC(cmd->timeout_ms)) != 0) {
        return -EBUSY;
    }

#if defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
    if (cmd->opcode == SD_SEND_IF_COND && data == NULL) {
        return 0;
    }
#endif

    memset(&host_content, 0, sizeof(sdxc_xfer_t));
    memset(&dummy_buffer, 0, DUMMY_BUF_LEN);

    host_cmd.cmd_index = cmd->opcode;
    host_cmd.cmd_argument = cmd->arg;
    host_cmd.resp_type = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
    if (cmd->timeout_ms == SDHC_TIMEOUT_FOREVER) {
        host_cmd.cmd_timeout_ms = 0xFFFFFFFF;
    } else {
        host_cmd.cmd_timeout_ms = cmd->timeout_ms;
    }

    if (data) {
        volatile sdxc_data_t host_data = {0};
        host_data.block_size = data->block_size;
        host_data.block_cnt = data->blocks;
        switch (cmd->opcode) {
            case SD_WRITE_SINGLE_BLOCK:
                host_data.enable_auto_cmd12 = false;
                cp_write_size = host_data.block_size * host_data.block_cnt;
                memcpy(&dummy_buffer, data->data, cp_write_size);
                host_data.tx_data = (const uint32_t *) &dummy_buffer;
                break;
            case SD_WRITE_MULTIPLE_BLOCK:
                if (dev_data->host_io.timing == SDHC_TIMING_SDR104) {
                    host_data.enable_auto_cmd23 = true;
                } else {
                    host_data.enable_auto_cmd12 = true;
                }
                cp_write_size = host_data.block_size * host_data.block_cnt;
                memcpy(&dummy_buffer, data->data, cp_write_size);
                host_data.tx_data = (const uint32_t *) &dummy_buffer;
                break;
            case SD_READ_SINGLE_BLOCK:
                host_data.rx_data = (uint32_t *) &dummy_buffer;
                need_cp =1;
                break;
            case SD_READ_MULTIPLE_BLOCK:
                if (dev_data->host_io.timing == SDHC_TIMING_SDR104) {
                    host_data.enable_auto_cmd23 = true;
                } else {
                    host_data.enable_auto_cmd12 = true;
                }
                host_data.rx_data = (uint32_t *) &dummy_buffer;
                need_cp =1;
                break;
            case SD_SWITCH:
            case SD_APP_SEND_SCR:
            case MMC_SEND_EXT_CSD:
            case MMC_CHECK_BUS_TEST:
            case SD_APP_SEND_NUM_WRITTEN_BLK:
                host_data.rx_data = (uint32_t *) &dummy_buffer;
                need_cp =1;
                break;
            case SDIO_RW_EXTENDED:
                if (host_cmd.cmd_argument & BIT(SDIO_CMD_ARG_RW_SHIFT)) {
                    cp_write_size = host_data.block_size * host_data.block_cnt;
                    memcpy(&dummy_buffer, data->data, cp_write_size);
                    host_data.tx_data = (uint32_t *) &dummy_buffer;
                } else {
                    host_data.rx_data = (uint32_t *) &dummy_buffer;
                    need_cp =1;
                }
                break;
            default:
                return -ENOTSUP;
        }
        host_content.data = (sdxc_data_t *) &host_data;
    }
    host_content.command = (sdxc_command_t *) &host_cmd;
    do {
        status = hpm_sdhc_send_content(dev, &host_content);
        if (status == status_success) {
            if (data && need_cp == 1) {
                uint32_t cp_size = host_content.data->block_size * host_content.data->block_cnt;
                memcpy(data->data, &dummy_buffer, cp_size);
            }
            memcpy(cmd->response, host_content.command->response, sizeof(cmd->response));
            break;
        } else if ((status >= status_sdxc_busy) && (status <= status_sdxc_tuning_failed)) {
            hpm_stat_t error_recovery_status = hpm_sdhc_error_recovery(dev);
            if (error_recovery_status != status_success) {
                return error_recovery_status;
            }
            retries --;
        }
    } while (retries);
    k_mutex_unlock(&dev_data->access_mutex);
    return status;
}

/*
 * Send CMD via SDHC
 */
static int hpm_sdhc_cmd_send(const struct device *dev, struct sdhc_command *cmd,
    struct sdhc_data *data)
{
    hpm_stat_t status = status_invalid_argument;
    int retries = (int)cmd->retries;
    struct hpm_sdhc_data *dev_data = dev->data;
    sdxc_command_t host_cmd = {0};
    (void) memset(&host_cmd, 0, sizeof(sdxc_command_t));
    
    if (k_mutex_lock(&dev_data->access_mutex, K_MSEC(cmd->timeout_ms)) != 0) {
        return -EBUSY;
    }

    host_cmd.cmd_index = cmd->opcode;
    host_cmd.cmd_argument = cmd->arg;
    host_cmd.resp_type = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);

    do {
        status = hpm_sdhc_send_command(dev, &host_cmd);
        if (status == status_success) {
            memcpy(cmd->response, host_cmd.response, sizeof(cmd->response));
            break;
        } else if ((status >= status_sdxc_busy) && (status <= status_sdxc_tuning_failed)) {
            hpm_stat_t error_recovery_status = hpm_sdhc_error_recovery(dev);
            if (error_recovery_status != status_success) {
                return error_recovery_status;
            }
            retries --;
        }
    } while (retries);
    k_mutex_unlock(&dev_data->access_mutex);
    return status;
}

/*
 * Send CMD or CMD/DATA via SDHC
 */
static int hpm_sdhc_request(const struct device *dev, struct sdhc_command *cmd,
    struct sdhc_data *data)
{
    hpm_stat_t status = status_invalid_argument;

    switch (cmd->opcode) {
        case SD_SWITCH:
        case MMC_CHECK_BUS_TEST:
        case SD_READ_SINGLE_BLOCK:
        case SD_READ_MULTIPLE_BLOCK:
        case SD_APP_SEND_NUM_WRITTEN_BLK:
        case SD_WRITE_SINGLE_BLOCK:
        case SD_WRITE_MULTIPLE_BLOCK:
        case SD_APP_SEND_SCR:
#if defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
        case MMC_SEND_EXT_CSD:
#endif
        case SDIO_RW_EXTENDED:
            status = hpm_sdhc_transfer(dev, cmd, data);
            break;
        case SD_SPI_READ_OCR:
        case SD_SPI_CRC_ON_OFF:
            return -ENOTSUP;
        default:
            status = hpm_sdhc_cmd_send(dev, cmd, data);
            break;
    }
    if (status != status_success) {
        return -EIO;
    } else {
        return 0;
    }
}

/*
 * Get card presence
 */
static int hpm_sdhc_get_card_present(const struct device *dev)
{
    struct hpm_sdhc_data *data = dev->data;

#if !defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
    const struct hpm_sdhc_config *cfg = dev->config;
    if (cfg->detect_gpio.port) {
        data->card_present = gpio_pin_get_dt(&cfg->detect_gpio) > 0;
    } else {
        data->card_present = sdxc_is_card_inserted(cfg->base);
    }
#else
    data->card_present = true;
#endif
    return ((int)data->card_present);
}

/*
 * Get host properties
 */
static int hpm_sdhc_get_host_props(const struct device *dev,
    struct sdhc_host_props *props)
{
    struct hpm_sdhc_data *data = dev->data;

    memcpy(props, &data->props, sizeof(struct sdhc_host_props));
    return 0;
}

/*
 * Enable SDHC card interrupt
 */
static int hpm_sdhc_enable_interrupt(const struct device *dev,
                        sdhc_interrupt_cb_t callback,
                        int sources, void *user_data)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    struct hpm_sdhc_data *data = dev->data;
    int ret =0;

    /* Record SDIO callback parameters */
    data->sdhc_cb = callback;
    data->sdhc_cb_user_data = user_data;

    /* Disable all interrupts, then enable what the user requested */
    sdxc_enable_interrupt_status(cfg->base, SDXC_STS_ALL_FLAGS, false);
    sdxc_enable_interrupt_signal(cfg->base, SDXC_STS_ALL_FLAGS, false);

    if (sources & SDHC_INT_SDIO) {
        /* Enable SDIO card interrupt */
        sdxc_enable_interrupt_status(cfg->base, SDXC_INT_STAT_CARD_INTERRUPT_MASK, true);
        sdxc_enable_interrupt_signal(cfg->base, SDXC_INT_STAT_CARD_INTERRUPT_MASK, true);
    }
    if (sources & SDHC_INT_INSERTED) {
        if (cfg->detect_gpio.port) {
            /* Use GPIO interrupt */
            ret = gpio_pin_interrupt_configure_dt(&cfg->detect_gpio, GPIO_INT_EDGE_TO_ACTIVE);
            if (ret) {
                return ret;
            }
        } else {
            /* Enable card insertion interrupt */
            sdxc_enable_interrupt_status(cfg->base, SDXC_INT_STAT_CARD_INSERTION_MASK, true);
            sdxc_enable_interrupt_signal(cfg->base, SDXC_INT_STAT_CARD_INSERTION_MASK, true);
        }
    }
    if (sources & SDHC_INT_REMOVED) {
        if (cfg->detect_gpio.port) {
            /* Use GPIO interrupt */
            ret = gpio_pin_interrupt_configure_dt(&cfg->detect_gpio, GPIO_INT_EDGE_TO_INACTIVE);
            if (ret) {
                return ret;
            }
        } else {
            /* Enable card removal interrupt */
            sdxc_enable_interrupt_status(cfg->base, SDXC_INT_STAT_CARD_REMOVAL_MASK, true);
            sdxc_enable_interrupt_signal(cfg->base, SDXC_INT_STAT_CARD_REMOVAL_MASK, true);
        }
    }

    return 0;
}

static int hpm_sdhc_disable_interrupt(const struct device *dev, int sources)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    struct hpm_sdhc_data *data = dev->data;
    int ret =0;

    if (sources & SDHC_INT_SDIO) {
        /* Disable SDIO card interrupt */
        sdxc_enable_interrupt_status(cfg->base, SDXC_INT_STAT_CARD_INTERRUPT_MASK, false);
        sdxc_enable_interrupt_signal(cfg->base, SDXC_INT_STAT_CARD_INTERRUPT_MASK, false);
    }
    if (sources & SDHC_INT_INSERTED) {
        if (cfg->detect_gpio.port) {
            ret = gpio_pin_interrupt_configure_dt(&cfg->detect_gpio, GPIO_INT_DISABLE);
            if (ret) {
                return ret;
            }
        } else {
            /* Disable card insertion interrupt */
            sdxc_enable_interrupt_status(cfg->base, SDXC_INT_STAT_CARD_INSERTION_MASK, false);
            sdxc_enable_interrupt_signal(cfg->base, SDXC_INT_STAT_CARD_INSERTION_MASK, false);
        }
    }
    if (sources & SDHC_INT_REMOVED) {
        if (cfg->detect_gpio.port) {
            ret = gpio_pin_interrupt_configure_dt(&cfg->detect_gpio, GPIO_INT_DISABLE);
            if (ret) {
                return ret;
            }
        } else {
            /* Disable card removal interrupt */
            sdxc_enable_interrupt_status(cfg->base, SDXC_INT_STAT_CARD_REMOVAL_MASK, false);
            sdxc_enable_interrupt_signal(cfg->base, SDXC_INT_STAT_CARD_REMOVAL_MASK, false);
        }
    }

    /* If all interrupt flags are disabled, remove callback */
    if (((cfg->base->INT_STAT_EN) & (SDXC_INT_STAT_CARD_INTERRUPT_MASK | 
        SDXC_INT_STAT_CARD_INSERTION_MASK |
        SDXC_INT_STAT_CARD_REMOVAL_MASK)) == 0) {
        data->sdhc_cb = NULL;
        data->sdhc_cb_user_data = NULL;
    }

    return 0;
}

__attribute__((section(".isr")))static int hpm_sdhc_isr(const struct device *dev)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    struct hpm_sdhc_data *data = dev->data;
    uint32_t interrupt_status = 0;

    interrupt_status = sdxc_get_interrupt_status(cfg->base);
    if (IS_HPM_BITMASK_SET(interrupt_status, SDXC_INT_STAT_CARD_INSERTION_MASK)) {
        if (data->sdhc_cb) {
            data->sdhc_cb(dev, SDHC_INT_INSERTED, data->sdhc_cb_user_data);
        }
    }
    if (IS_HPM_BITMASK_SET(interrupt_status, SDXC_INT_STAT_CARD_REMOVAL_MASK)) {
        if (data->sdhc_cb) {
            data->sdhc_cb(dev, SDHC_INT_REMOVED, data->sdhc_cb_user_data);
        }
    }
    sdxc_clear_interrupt_status(cfg->base, interrupt_status);

    return 0;
}

/*
 * Perform early system init for SDHC
 */
static int hpm_sdhc_init(const struct device *dev)
{
    const struct hpm_sdhc_config *cfg = dev->config;
    struct hpm_sdhc_data *data = dev->data;
    SDXC_Type *base = cfg->base;
    int ret;

    ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret) {
        return ret;
    }

#if defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
    data->dev_type = sdmmc_dev_type_emmc;
#elif defined(CONFIG_HPMICRO_BOARD_SUPPORT_SDIO)
    data->dev_type = sdmmc_dev_type_sdio;
#else
    data->dev_type = sdmmc_dev_type_sd;
#endif
    cfg->irq_config_func(dev);

    hpm_board_sd_configure_clock(base, SDMMC_CLOCK_400KHZ, true);

    sdxc_config_t sdxc_config;
    sdxc_config.data_timeout = 1000; 

    sdxc_init(base, &sdxc_config);

#if defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
    sdxc_enable_emmc_support(base, true);
#endif

    sdxc_set_data_bus_width(base, (sdxc_bus_width_t) sdmmc_bus_width_1bit);

    /* Read host controller properties */
    hpm_sdhc_init_host_props(dev);

    /* Set power GPIO low, so card starts powered off */
    if (cfg->pwr_gpio.port) {
        ret = gpio_pin_configure_dt(&cfg->pwr_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret) {
            return ret;
        }
    } else {
        LOG_WRN("No power control GPIO defined. Without power control,\n"
            "the SD card may fail to communicate with the host");
    }
    if (cfg->detect_gpio.port) {
        ret = gpio_pin_configure_dt(&cfg->detect_gpio, GPIO_INPUT);
        if (ret) {
            return ret;
        }
        gpio_init_callback(&data->cd_callback, card_detect_gpio_cb, BIT(cfg->detect_gpio.pin));
        ret = gpio_add_callback_dt(&cfg->detect_gpio, &data->cd_callback);
        if (ret) {
            return ret;
        }
    }

    data->dev = dev;
    k_mutex_init(&data->access_mutex);
    /* Setup initial host IO values */
#if !defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
    data->host_io.signal_voltage = SD_VOL_3_3_V;
#else
    data->host_io.signal_voltage = SD_VOL_1_8_V;
#endif
    data->host_io.clock = 0;
    data->host_io.bus_mode = SDHC_BUSMODE_PUSHPULL;
    data->host_io.power_mode = SDHC_POWER_OFF;
    data->host_io.bus_width = SDHC_BUS_WIDTH1BIT;
    data->host_io.timing = SDHC_TIMING_LEGACY;
    data->host_io.driver_type = SD_DRIVER_TYPE_B;
    return k_sem_init(&data->transfer_sem, 0, 1);
}

static const struct sdhc_driver_api hpm_sdhc_driver_api = {
    .reset = hpm_sdhc_reset,
    .request = hpm_sdhc_request,
    .set_io = hpm_sdhc_set_io,
    .get_card_present = hpm_sdhc_get_card_present,
    .execute_tuning = hpm_sdhc_execute_tuning,
    .card_busy = hpm_sdhc_card_busy,
    .get_host_props = hpm_sdhc_get_host_props,
    .enable_interrupt = hpm_sdhc_enable_interrupt,
    .disable_interrupt = hpm_sdhc_disable_interrupt,
};

#define HPM_SDHC_NOCACHE_TAG __attribute__((__section__(".nocache")));

#define HPM_SDHC_DMA_BUFFER_DEFINE(n)                        \
    static uint32_t    __aligned(32)                        \
        hpm_sdhc_##n##_dma_descriptor[CONFIG_HPM_SDHC_DMA_BUFFER_SIZE / 4]\
        HPM_SDHC_NOCACHE_TAG;
#define HPM_SDHC_DMA_BUFFER_INIT(n)                        \
    .hpm_sdhc_dma_descriptor = hpm_sdhc_##n##_dma_descriptor,            \
    .dma_descriptor_len = CONFIG_HPM_SDHC_DMA_BUFFER_SIZE / 4,

#if defined(CONFIG_HPMICRO_BOARD_SUPPORT_EMMC)
#define HPM_SDHC_INIT(n)                            \
    PINCTRL_DT_INST_DEFINE(n);                        \
    static void hpm_sdhc_irq_config_func##n(const struct device *dev); \
                                                                        \
    static const struct hpm_sdhc_config hpm_sdhc_config_##n = {            \
        .base = (SDXC_Type *) DT_INST_REG_ADDR(n),            \
        .clock_name = (clock_name_t)DT_INST_PROP(n, clk_name), \
        .clock_src = (clock_source_t)DT_INST_PROP(n, clk_source), \
        .clock_div = DT_INST_PROP(n, clk_divider), \
        .pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwr_gpios, 0),    \
        .min_bus_freq = DT_INST_PROP(n, min_bus_freq),            \
        .max_bus_freq = DT_INST_PROP(n, max_bus_freq),            \
        .power_delay_ms = DT_INST_PROP(n, power_delay_ms),        \
        .pwr_3v3_support = DT_INST_PROP(n, pwr_3v3_support),        \
        .pwr_3v0_support = DT_INST_PROP(n, pwr_3v0_support),        \
        .pwr_1v8_support = DT_INST_PROP(n, pwr_1v8_support),        \
        .mmc_hs200_1_8v = DT_INST_PROP(n, mmc_hs200_1_8v),        \
        .mmc_hs400_1_8v = DT_INST_PROP(n, mmc_hs400_1_8v),       \
        .embedded_4_bit_support = DT_INST_PROP(n, embedded_4_bit_support),       \
        .irq_config_func = hpm_sdhc_irq_config_func##n,            \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),            \
    };                                    \
    HPM_SDHC_DMA_BUFFER_DEFINE(n)                        \
                                        \
    static struct hpm_sdhc_data hpm_sdhc_data_##n = {.card_present = false, \
        HPM_SDHC_DMA_BUFFER_INIT(n)                        \
        }; \
    \
    DEVICE_DT_INST_DEFINE(n,hpm_sdhc_init,                    \
            NULL,                            \
            &hpm_sdhc_data_##n,                    \
            &hpm_sdhc_config_##n,                    \
            POST_KERNEL,                        \
            CONFIG_SDHC_INIT_PRIORITY,                \
            &hpm_sdhc_driver_api);                    \
    static void hpm_sdhc_irq_config_func##n(const struct device *dev)    \
    {                                    \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),        \
            hpm_sdhc_isr, DEVICE_DT_INST_GET(n), 0);        \
        irq_enable(DT_INST_IRQN(n));                    \
    }    
#else
#define HPM_SDHC_INIT(n)                            \
    PINCTRL_DT_INST_DEFINE(n);                        \
    static void hpm_sdhc_irq_config_func##n(const struct device *dev); \
                                                                        \
    static const struct hpm_sdhc_config hpm_sdhc_config_##n = {            \
        .base = (SDXC_Type *) DT_INST_REG_ADDR(n),            \
        .clock_name = (clock_name_t)DT_INST_PROP(n, clk_name), \
        .clock_src = (clock_source_t)DT_INST_PROP(n, clk_source), \
        .clock_div = DT_INST_PROP(n, clk_divider), \
        .pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwr_gpios, 0),    \
        .detect_gpio = GPIO_DT_SPEC_INST_GET_OR(n, cd_gpios, 0),    \
        .pwr_3v3_support = DT_INST_PROP(n, pwr_3v3_support),        \
        .pwr_3v0_support = DT_INST_PROP(n, pwr_3v0_support),        \
        .pwr_1v8_support = DT_INST_PROP(n, pwr_1v8_support),        \
        .min_bus_freq = DT_INST_PROP(n, min_bus_freq),            \
        .max_bus_freq = DT_INST_PROP(n, max_bus_freq),            \
        .power_delay_ms = DT_INST_PROP(n, power_delay_ms),        \
        .embedded_4_bit_support = DT_INST_PROP(n, embedded_4_bit_support),       \
        .irq_config_func = hpm_sdhc_irq_config_func##n,            \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),            \
    };                                    \
    HPM_SDHC_DMA_BUFFER_DEFINE(n)                        \
                                        \
    static struct hpm_sdhc_data hpm_sdhc_data_##n = {.card_present = false, \
        HPM_SDHC_DMA_BUFFER_INIT(n)                        \
        }; \
    \
    DEVICE_DT_INST_DEFINE(n,hpm_sdhc_init,                    \
            NULL,                            \
            &hpm_sdhc_data_##n,                    \
            &hpm_sdhc_config_##n,                    \
            POST_KERNEL,                        \
            CONFIG_SDHC_INIT_PRIORITY,                \
            &hpm_sdhc_driver_api);                    \
    static void hpm_sdhc_irq_config_func##n(const struct device *dev)    \
    {                                    \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),        \
            hpm_sdhc_isr, DEVICE_DT_INST_GET(n), 0);        \
        irq_enable(DT_INST_IRQN(n));                    \
    }
#endif
DT_INST_FOREACH_STATUS_OKAY(HPM_SDHC_INIT)
