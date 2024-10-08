/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_i2c


#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <hpm_i2c_drv.h>
#include <hpm_clock_drv.h>

#define HPM_I2C_STATUS_SLAVE_IDLE          0
#define HPM_I2C_STATUS_SLAVE_BEGIN_WRITE   1
#define HPM_I2C_STATUS_SLAVE_NEXT_WRITE    2
#define HPM_I2C_STATUS_NOT_ADDRHIT         3
#define HPM_I2C_STATUS_ADDRHIT             4
struct hpmicro_i2c_config {
	I2C_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
};

struct hpmicro_i2c_current_transfer {
	struct i2c_msg *msgs;
	uint8_t *curr_buf;
	uint32_t curr_len;
	uint32_t curr_index;
	uint32_t nr_msgs;
	uint8_t addr;
	uint8_t status;
};

struct hpmicro_i2c_data {
	struct hpmicro_i2c_current_transfer transfer;
	struct i2c_target_config *slave;
	struct k_sem completion;
	struct k_mutex mutex;
};

#define DEV_BASE(dev) (((struct hpmicro_i2c_config *)(dev->config))->base)

static int hpmicro_i2c_set_bus_speed(const struct device *dev, uint32_t dev_config)
{
	hpm_stat_t stat;
	const struct hpmicro_i2c_config *cfg = dev->config;
	struct hpmicro_i2c_data *data = dev->data;
	i2c_config_t config;
	uint32_t freq;
	/* Configure clock and de-assert reset for I2Cx */
	clock_set_source_divider(cfg->clock_name, cfg->clock_src, 1U);
	clock_add_to_group(cfg->clock_name, 0);

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		config.i2c_mode = i2c_mode_normal;
		break;
	case I2C_SPEED_FAST:
		config.i2c_mode = i2c_mode_fast;
		break;
	case I2C_SPEED_FAST_PLUS:
		config.i2c_mode = i2c_mode_fast_plus;
		break;
	case I2C_SPEED_HIGH:
	case I2C_SPEED_ULTRA:
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	if (dev_config & I2C_ADDR_10_BITS) {
			config.is_10bit_addressing = true;
	} else {
			config.is_10bit_addressing = false;
	}

	freq = clock_get_frequency(cfg->clock_name);
	if (data->slave) {
		stat = i2c_init_slave(cfg->base, freq, &config, data->slave->address);
    if (stat != status_success) {
        return -EINVAL;
    }
	} else {
		stat = i2c_init_master(cfg->base, freq, &config);
    if (stat != status_success) {
        return -EINVAL;
    }
	}
	return 0;
}

static int hpmicro_i2c_configure(const struct device *dev,
				  uint32_t dev_config)
{
	struct hpmicro_i2c_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);
	ret = hpmicro_i2c_set_bus_speed(dev, dev_config);

	if (ret) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}
	k_mutex_unlock(&data->mutex);
	return 0;
}

static int hpmicro_i2c_transfer(const struct device *dev,
				 struct i2c_msg *msgs,
				 uint8_t num_msgs, uint16_t addr)
{
	const struct hpmicro_i2c_config *cfg = dev->config;
	struct hpmicro_i2c_data *data = dev->data;
	int ret = 0;
	uint32_t ctrl = 0;
	if (!num_msgs) {
		return 0;
	}
	
	if (msgs->len > I2C_SOC_TRANSFER_COUNT_MAX) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	data->transfer.msgs = msgs;
	data->transfer.curr_buf = msgs->buf;
	data->transfer.curr_len = msgs->len;
	data->transfer.curr_index = 0;
	data->transfer.nr_msgs = num_msgs;
	data->transfer.addr = addr;
	data->transfer.status = HPM_I2C_STATUS_SLAVE_IDLE;
	
	if (data->slave) {
		i2c_enable_irq(cfg->base, I2C_EVENT_ADDRESS_HIT | I2C_EVENT_TRANSACTION_COMPLETE);
	} else {
		if (data->transfer.msgs->flags & I2C_MSG_ADDR_10_BITS) {
				i2c_enable_10bit_address_mode(cfg->base, true);
		} else {
				i2c_enable_10bit_address_mode(cfg->base, false);
		}
		/* W1C, clear CMPL bit to avoid blocking the transmission */
		cfg->base->STATUS = I2C_STATUS_CMPL_MASK;
		cfg->base->CMD = I2C_CMD_CLEAR_FIFO;
		cfg->base->ADDR = I2C_ADDR_ADDR_SET(data->transfer.addr);
		if ((data->transfer.msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				ctrl |= I2C_CTRL_DIR_SET(I2C_DIR_MASTER_WRITE);
		} else {
				ctrl |= I2C_CTRL_DIR_SET(I2C_DIR_MASTER_READ);
		}
			/* start signal */
			ctrl |= I2C_CTRL_PHASE_START_SET(true) | I2C_CTRL_PHASE_ADDR_SET(true);
			/* end signal*/
			if (data->transfer.msgs->flags & I2C_MSG_STOP) {
					ctrl |= I2C_CTRL_PHASE_STOP_SET(true);
			} else {
					ctrl |= I2C_CTRL_PHASE_STOP_SET(false);
			}
			if (data->transfer.curr_len > 0) {
				ctrl |= I2C_CTRL_PHASE_DATA_SET(true)
#ifdef I2C_CTRL_DATACNT_HIGH_MASK
													| I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(data->transfer.curr_len) >> 8U)
#endif
													| I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(data->transfer.curr_len));
			}
			cfg->base->CTRL = ctrl;
			cfg->base->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
	}
	i2c_enable_irq(cfg->base, I2C_EVENT_ADDRESS_HIT | I2C_EVENT_TRANSACTION_COMPLETE);
	k_sem_take(&data->completion, K_FOREVER);
	k_mutex_unlock(&data->mutex);
	if (data->transfer.status == HPM_I2C_STATUS_NOT_ADDRHIT) {
		ret = -ENXIO;
	}
	return ret;
}

static int hpmicro_i2c_slave_register(const struct device *dev,
				       struct i2c_target_config *cfg)
{
	const struct hpmicro_i2c_config *dev_cfg = dev->config;
	struct hpmicro_i2c_data *data = dev->data;
	int ret = 0;
	hpm_stat_t stat;
	i2c_config_t config;
	uint32_t freq;

	if (!cfg) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);
	if (data->slave) {
		ret = -EBUSY;
		goto exit;
	}

	data->slave = cfg;

	if (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
			config.is_10bit_addressing = true;
	} else {
			config.is_10bit_addressing = false;
	}

	config.i2c_mode = i2c_mode_normal;
	freq = clock_get_frequency(dev_cfg->clock_name);
	stat = i2c_init_slave(dev_cfg->base, freq, &config, cfg->address);
	if (stat != status_success) {
			return -EINVAL;
	}
	return 0;

exit:
	k_mutex_unlock(&data->mutex);
	return ret;
}


static int hpmicro_i2c_slave_unregister(const struct device *dev,
					 struct i2c_target_config *cfg)
{
	struct hpmicro_i2c_data *data = dev->data;
	if (!cfg) {
		return -EINVAL;
	}
	if (data->slave != cfg) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);
	data->slave = NULL;
	k_mutex_unlock(&data->mutex);

	return 0;
}

static int hpmicro_restart_i2c(const struct device *dev, uint32_t size)
{
	struct hpmicro_i2c_data *data = dev->data;
	I2C_Type *i2c = DEV_BASE(dev);
	struct hpmicro_i2c_current_transfer *transfer = &data->transfer;
	uint32_t ctrl = 0;
	if (size > I2C_SOC_TRANSFER_COUNT_MAX) {
		return -EINVAL;
	}

	/* W1C, clear CMPL bit to avoid blocking the transmission */
	// printk("isr:%d\n", transfer->nr_msgs);
	i2c->STATUS = I2C_STATUS_CMPL_MASK;
	i2c->CMD = I2C_CMD_CLEAR_FIFO;
	i2c->ADDR = I2C_ADDR_ADDR_SET(data->transfer.addr);

	if ((transfer->msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
		ctrl |= I2C_CTRL_DIR_SET(I2C_DIR_MASTER_WRITE);
	} else {
		ctrl |= I2C_CTRL_DIR_SET(I2C_DIR_MASTER_READ);
	}
	/* start signal */
	if (transfer->msgs->flags & I2C_MSG_RESTART) {
		ctrl |= I2C_CTRL_PHASE_START_SET(true) | I2C_CTRL_PHASE_ADDR_SET(true);
	}
	/* end signal*/
	if (data->transfer.msgs->flags & I2C_MSG_STOP) {
			ctrl |= I2C_CTRL_PHASE_STOP_SET(true);
	} else {
			ctrl |= I2C_CTRL_PHASE_STOP_SET(false);
	}
	if (data->transfer.curr_len > 0) {
		ctrl |= I2C_CTRL_PHASE_DATA_SET(true)
#ifdef I2C_CTRL_DATACNT_HIGH_MASK
											| I2C_CTRL_DATACNT_HIGH_SET(I2C_DATACNT_MAP(data->transfer.curr_len) >> 8U)
#endif
											| I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(data->transfer.curr_len));
	}
	i2c->CTRL = ctrl;
	i2c->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
	if (transfer->msgs->flags & I2C_MSG_RESTART) {
		i2c_enable_irq(i2c, I2C_EVENT_ADDRESS_HIT | I2C_EVENT_TRANSACTION_COMPLETE);
	} else {
		i2c_enable_irq(i2c, I2C_EVENT_FIFO_EMPTY | I2C_EVENT_TRANSACTION_COMPLETE);
	}
	return 0;
}

__attribute__((section(".isr"))) static void hpmicro_i2c_isr(const struct device *dev)
{
	struct hpmicro_i2c_data *data = dev->data;
	I2C_Type *i2c = DEV_BASE(dev);
	struct hpmicro_i2c_current_transfer *transfer = &data->transfer;
	uint8_t val;

	volatile uint32_t status, irq;
	status = i2c_get_status(i2c);
	irq = i2c_get_irq_setting(i2c);

	/* slave address hit */
	if (((status & I2C_EVENT_ADDRESS_HIT)) && (irq & I2C_EVENT_ADDRESS_HIT)) {
		data->transfer.status = HPM_I2C_STATUS_ADDRHIT;
			i2c_clear_status(i2c, I2C_EVENT_ADDRESS_HIT);
			if (data->slave) {
				if (I2C_DIR_SLAVE_READ == i2c_get_direction(i2c)) {
						i2c_enable_irq(i2c, I2C_EVENT_FIFO_FULL);
						data->slave->callbacks->write_requested(data->slave);
				} else {
						data->transfer.status = HPM_I2C_STATUS_SLAVE_BEGIN_WRITE;
						i2c_enable_irq(i2c, I2C_EVENT_FIFO_EMPTY);
				}
				i2c_disable_irq(i2c, I2C_EVENT_ADDRESS_HIT);
				i2c_clear_status(i2c, I2C_EVENT_ADDRESS_HIT);
			} else {
				if ((data->transfer.msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
						i2c_enable_irq(i2c, I2C_EVENT_FIFO_EMPTY);
				} else {
						i2c_enable_irq(i2c, I2C_EVENT_FIFO_FULL);
				}
			}
	} else {
		if (data->transfer.status != HPM_I2C_STATUS_ADDRHIT) {
			data->transfer.status = HPM_I2C_STATUS_NOT_ADDRHIT;
			i2c_disable_irq(i2c, I2C_EVENT_FIFO_FULL | I2C_EVENT_FIFO_EMPTY);
			i2c->CTRL = I2C_CTRL_PHASE_STOP_SET(true);
      i2c->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;
			k_sem_give(&data->completion);
		}
	}

    /* transmit */
	if ((status & I2C_EVENT_FIFO_EMPTY) && (irq & I2C_EVENT_FIFO_EMPTY)) {
			if (data->slave) {
				if (data->transfer.status == HPM_I2C_STATUS_SLAVE_BEGIN_WRITE) {
					data->slave->callbacks->read_processed(data->slave, &val);
					data->transfer.status = HPM_I2C_STATUS_SLAVE_NEXT_WRITE;
				} else {
					data->slave->callbacks->read_requested(data->slave, &val);
				}
				i2c_write_byte(i2c, val);
			} else {
				if ((transfer->curr_len > 0) && (data->transfer.status != HPM_I2C_STATUS_NOT_ADDRHIT)) {
					while (!i2c_fifo_is_full(i2c)) {
							i2c_write_byte(i2c, transfer->curr_buf[transfer->curr_index++]);
							if (transfer->curr_index == transfer->curr_len) {
									i2c_disable_irq(i2c, I2C_EVENT_FIFO_EMPTY);
									break;
							}
					}
				}
			}
	}

	/* receive */
	if (status & I2C_EVENT_FIFO_FULL) {
		if (data->slave) {
			while (!i2c_fifo_is_empty(i2c)) {
				data->slave->callbacks->write_received(data->slave, i2c_read_byte(i2c));
			}
		} else {
			while (!i2c_fifo_is_empty(i2c)) {
				transfer->curr_buf[transfer->curr_index++] = i2c_read_byte(i2c);
			}
			if ((transfer->curr_index == transfer->curr_len) && (transfer->curr_len > 0)) {
				i2c_disable_irq(i2c, I2C_EVENT_FIFO_FULL);
				transfer->msgs++;
				transfer->nr_msgs--;
				transfer->curr_buf = transfer->msgs->buf;
				transfer->curr_len = transfer->msgs->len;
				transfer->curr_index = 0;
				if (transfer->nr_msgs != 0) {
					if (hpmicro_restart_i2c(dev, transfer->curr_len) < 0) {
						i2c->CTRL |= I2C_CTRL_PHASE_STOP_MASK;
					}
				}
			}
		}
	}

	/* complete */
	if (status & I2C_EVENT_TRANSACTION_COMPLETE) {
			i2c_disable_irq(i2c, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_FIFO_FULL | I2C_EVENT_FIFO_EMPTY);
			i2c_clear_status(i2c, I2C_EVENT_TRANSACTION_COMPLETE);
			if (data->slave) {
				if (I2C_DIR_MASTER_READ == i2c_get_direction(i2c)) {
					while (!i2c_fifo_is_empty(i2c)) {
						data->slave->callbacks->write_received(data->slave, i2c_read_byte(i2c));
					}
				}
				data->slave->callbacks->stop(data->slave);
				k_sem_give(&data->completion);
				return;
			}
			if (I2C_DIR_MASTER_READ == i2c_get_direction(i2c)) {
					while ((!i2c_fifo_is_empty(i2c)) && (transfer->curr_index < transfer->curr_len)) {
							transfer->curr_buf[transfer->curr_index++] = i2c_read_byte(i2c);
							if (transfer->curr_index == transfer->curr_len) {
									break;
							}
					}
			}
			if (!transfer->nr_msgs) {
				k_sem_give(&data->completion);
			} else {
				transfer->nr_msgs--;
				if ((transfer->nr_msgs > 0) && (data->transfer.status == HPM_I2C_STATUS_ADDRHIT)) {
					transfer->msgs++;
					transfer->curr_buf = transfer->msgs->buf;
					transfer->curr_len = transfer->msgs->len;
					transfer->curr_index = 0;
					hpmicro_restart_i2c(dev, data->transfer.curr_len);
				} else {
					k_sem_give(&data->completion);
				}

			}
	}
}


static int hpmicro_i2c_init(const struct device *dev)
{
	const struct hpmicro_i2c_config *cfg = dev->config;
	struct hpmicro_i2c_data *data = dev->data;
	int err;

	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	/* Configure bus speed. Default is 100KHz */
	hpmicro_i2c_set_bus_speed(dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));

	/* Initialize mutex and semaphore */
	k_mutex_init(&data->mutex);
	k_sem_init(&data->completion, 0, 1);

	/* Configure IRQ */
	cfg->irq_config_func(dev);
	return 0;
}

static const struct i2c_driver_api i2c_api = {
	.configure = hpmicro_i2c_configure,
	.transfer = hpmicro_i2c_transfer,
	.target_register = hpmicro_i2c_slave_register,
	.target_unregister = hpmicro_i2c_slave_unregister,
};

#define HPM_I2C_INIT(idx)						      \
									      \
static void hpmicro_i2c_isr_config_##idx(const struct device *dev);	      \
									      \
PINCTRL_DT_INST_DEFINE(idx);                                                  \
									      \
static const struct hpmicro_i2c_config i2c_cfg_##idx = {		      \
	.base =								      \
	(I2C_Type *) DT_INST_REG_ADDR(idx),		      \
	.irq_config_func = hpmicro_i2c_isr_config_##idx,		      \
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                        \
	.clock_name = DT_INST_CLOCKS_CELL(idx, name),	\
	.clock_src = DT_INST_CLOCKS_CELL(idx, src),	\
};									      \
									      \
static struct hpmicro_i2c_data i2c_data_##idx;			              \
									      \
I2C_DEVICE_DT_INST_DEFINE(idx,						      \
		    hpmicro_i2c_init,					      \
		    NULL,						      \
		    &i2c_data_##idx, &i2c_cfg_##idx,			      \
		    POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,		      \
		    &i2c_api);						      \
									      \
static void hpmicro_i2c_isr_config_##idx(const struct device *dev)		      \
{									      \
	IRQ_CONNECT(DT_INST_IRQN(idx),					      \
		    DT_INST_IRQ(idx, priority),				      \
		    hpmicro_i2c_isr, DEVICE_DT_INST_GET(idx), 0);	      \
									      \
	irq_enable(DT_INST_IRQN(idx));					      \
}

DT_INST_FOREACH_STATUS_OKAY(HPM_I2C_INIT);
