/*
 * Copyright (c) 2022-2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifdef defined(CONFIG_DMA_HPMICRO)
#define DT_DRV_COMPAT hpmicro_hpm_dma
#elif defined(CONFIG_DMAV2_HPMICRO)
#define DT_DRV_COMPAT hpmicro_hpm_dmav2
#endif

#include <errno.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/dma.h>

#include <hpm_common.h>
#include <hpm_dma_drv.h>
#include <hpm_dmamux_drv.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_hpm_dma);

struct dma_hpm_dma_config {
	DMA_Type *base;
	DMAMUX_Type *dmamux_base;
	int dma_channels; /* number of channels */
	void (*irq_config_func)(const struct device *dev);
};

struct hpm_dma_channel_transfer_settings {
	dma_channel_config_t ch_config;
	enum dma_channel_direction direction;
	bool valid;
	uint8_t channel_index;
};

struct call_back {
	const struct device *dev;
	void *user_data;
	dma_callback_t dma_callback;
	struct hpm_dma_channel_transfer_settings transfer_settings;
	bool busy;
};

struct dma_hpm_data {
	struct dma_context dma_ctx;
	struct call_back data_cb[DT_INST_PROP(0, dma_channels)];
	ATOMIC_DEFINE(channels_atomic, DT_INST_PROP(0, dma_channels));
	struct k_mutex dma_mutex;
};

#define DEV_CFG(dev)  ((const struct dma_hpm_dma_config *const)dev->config)
#define DEV_DATA(dev) ((struct dma_hpm_data *)dev->data)
#define DEV_BASE(dev) ((DMA_Type *)DEV_CFG(dev)->base)
#define DEV_DMAMUX_BASE(dev) ((DMAMUX_Type *)DEV_CFG(dev)->dmamux_base)

#define DEV_CHANNEL_DATA(dev, ch) \
	((struct call_back *)(&(DEV_DATA(dev)->data_cb[ch])))

static void hpm_dma_callback(const struct device *dev, void *param, uint8_t channel, bool success)
{
	int ret = -1;
	struct call_back *data = (struct call_back *)param;

	/* dma transfer termination, status may be transfer complete, error, abort */
	data->busy = 0;
	if (success) {
		ret = 0;
	}
	data->dma_callback(data->dev, data->user_data, channel, ret);
}

static void dma_hpm_dma_irq_handler(const struct device *dev)
{
	uint8_t i = 0;
	hpm_stat_t stat;

	LOG_DBG("IRQ CALLED");
	for (i = 0; i < DT_INST_PROP(0, dma_channels); i++) {
        stat = dma_check_transfer_status(DEV_BASE(dev), i);

		if ((stat & DMA_CHANNEL_STATUS_TC) != 0U) {
			LOG_DBG("channel %d IRQ OCCURRED, transfer success", i);
			hpm_dma_callback(dev, DEV_CHANNEL_DATA(dev, i), i, true);
		} else if ((stat & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) != 0U) {
			LOG_DBG("channel %d IRQ OCCURRED, transfer failed", i);
			hpm_dma_callback(dev, DEV_CHANNEL_DATA(dev, i), i, false);
		}
	}
}

static int hpm_dma_get_transfer_width(DMA_Type *base, uint32_t data_size)
{
	int ret = 0;

	switch (data_size) {
	case 1:
		ret = DMA_TRANSFER_WIDTH_BYTE;
		break;
	case 2:
		ret = DMA_TRANSFER_WIDTH_HALF_WORD;
		break;
	case 4:
		ret = DMA_TRANSFER_WIDTH_WORD;
		break;
	case 8:
		ret = DMA_TRANSFER_WIDTH_DOUBLE_WORD;
		break;
	default:
		LOG_ERR("not support transfer data width %d", data_size);
		ret = -EINVAL;
	}

	if (ret > DMA_SOC_TRANSFER_WIDTH_MAX(base)) {
		LOG_ERR("not support transfer data width for DMA@%x", (uint32_t)base);
		ret = -EINVAL;
	}
	return ret;
}

static int hpm_dma_get_transfer_burst(DMA_Type *base, uint32_t burst_size)
{
	int ret = 0;

	switch (burst_size) {
	case 1:
		ret = DMA_NUM_TRANSFER_PER_BURST_1T;
		break;
	case 2:
		ret = DMA_NUM_TRANSFER_PER_BURST_2T;
		break;
	case 4:
		ret = DMA_NUM_TRANSFER_PER_BURST_4T;
		break;
	case 8:
		ret = DMA_NUM_TRANSFER_PER_BURST_8T;
		break;
	case 16:
		ret = DMA_NUM_TRANSFER_PER_BURST_16T;
		break;
	case 32:
		ret = DMA_NUM_TRANSFER_PER_BURST_32T;
		break;
	case 64:
		ret = DMA_NUM_TRANSFER_PER_BURST_64T;
		break;
	case 128:
		ret = DMA_NUM_TRANSFER_PER_BURST_128T;
		break;
	case 256:
		ret = DMA_NUM_TRANSFER_PER_BURST_256T;
		break;
	case 512:
		ret = DMA_NUM_TRANSFER_PER_BURST_512T;
		break;
	case 1024:
		ret = DMA_NUM_TRANSFER_PER_BURST_1024T;
		break;
	default:
		LOG_ERR("not support transfer burst size %d", burst_size);
		ret = -EINVAL;
	}

	if (ret > DMA_SOC_TRANSFER_PER_BURST_MAX(base)) {
		LOG_ERR("not support transfer burst size for DMA@%x", (uint32_t)base);
		ret = -EINVAL;
	}
	return ret;
}


/* Configure a channel */
static int dma_hpm_dma_configure(const struct device *dev, uint32_t channel, struct dma_config *config)
{
	/* Check for invalid parameters before dereferencing them. */
	if (NULL == dev || NULL == config) {
		return -EINVAL;
	}

	struct call_back *data = DEV_CHANNEL_DATA(dev, channel);
	struct dma_block_config *block_config = config->head_block;
	uint32_t slot = config->dma_slot;
	unsigned int key;
	int ret = 0;
	dma_channel_config_t ch_config = {0};
	uint32_t data_size;
	uint32_t burst_size;

	if (slot > DT_INST_PROP(0, dma_requests)) {
		LOG_ERR("source number is outof scope %d", slot);
		return -ENOTSUP;
	}

	if (channel > DT_INST_PROP(0, dma_channels)) {
		LOG_ERR("out of DMA channel %d", channel);
		return -EINVAL;
	}

	data->transfer_settings.valid = false;

	data->transfer_settings.direction = config->channel_direction;

	/* Lock and page in the channel configuration */
	key = irq_lock();

	/* dmamux channel config */
	dmamux_config(DEV_DMAMUX_BASE(dev), DMA_SOC_CHN_TO_DMAMUX_CHN(DEV_BASE(dev), channel), slot, true);

	if (data->busy) {
		dma_abort_channel(DEV_BASE(dev), (1U << channel));
		dma_clear_transfer_status(DEV_BASE(dev), channel);
	}


	ch_config.priority = 0;
	ch_config.src_addr = core_local_mem_to_sys_address(HPM_CORE0, block_config->source_address);
    ch_config.dst_addr = core_local_mem_to_sys_address(HPM_CORE0, block_config->dest_address);
    ch_config.src_burst_size = config->source_burst_length;
    ch_config.src_width = config->source_data_size;
	ch_config.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
	ch_config.src_addr_ctrl = block_config->source_addr_adj;
    ch_config.dst_width = config->dest_data_size;
	ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
	ch_config.dst_addr_ctrl = block_config->dest_addr_adj;
    ch_config.size_in_byte = block_config->block_size;
	ch_config.interrupt_mask = DMA_INTERRUPT_MASK_NONE; /* enable interrupt and call callback in irq handler */
    ch_config.linked_ptr = 0;

	/* TODO use source_handshake and dest_handshake to config src_mode and dst_mode */
	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
		ch_config.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
		ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
		break;
	case MEMORY_TO_PERIPHERAL:
		ch_config.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
		ch_config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
		break;
	case PERIPHERAL_TO_MEMORY:
		ch_config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
		ch_config.dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
		break;
	case PERIPHERAL_TO_PERIPHERAL:
		ch_config.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
		ch_config.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
		break;
	default:
		LOG_ERR("not support transfer direction");
		return -EINVAL;
	}

	data_size = hpm_dma_get_transfer_width(DEV_BASE(dev), config->source_data_size);
	if (data_size < 0) {
		return -EINVAL;
	} else {
		ch_config.src_width = data_size;
	}
	data_size = hpm_dma_get_transfer_width(DEV_BASE(dev), config->dest_data_size);
	if (data_size < 0) {
		return -EINVAL;
	} else {
		ch_config.dst_width = data_size;
	}

	burst_size = hpm_dma_get_transfer_burst(DEV_BASE(dev), config->source_burst_length);
	if (burst_size < 0) {
		return -EINVAL;
	} else {
		ch_config.src_burst_size = burst_size;
	}

	data->transfer_settings.ch_config = ch_config;
	data->transfer_settings.valid = true;

	if (status_success != dma_setup_channel(DEV_BASE(dev), channel, &ch_config, false)) {
		LOG_ERR("Configure DMA channel failed.");
		ret = -EFAULT;
	}

	data->busy = false;
	if (config->dma_callback) {
		LOG_DBG("INSTALL call back on channel %d", channel);
		data->user_data = config->user_data;
		data->dma_callback = config->dma_callback;
		data->dev = dev;
	}

	irq_unlock(key);

	return ret;
}

static int dma_hpm_dma_start(const struct device *dev, uint32_t channel)
{
	struct call_back *data = DEV_CHANNEL_DATA(dev, channel);
	int ret = 0;

	if (data->transfer_settings.valid == true) {
		LOG_DBG("START DMA channel %d TRANSFER", channel);
		data->busy = true;
		dma_enable_channel(DEV_BASE(dev), channel);
	} else {
		LOG_DBG("DMA channel %d is not configured", channel);
		ret = -EFAULT;
	}
	
	return ret;
}

static int dma_hpm_dma_stop(const struct device *dev, uint32_t channel)
{
	struct dma_hpm_data *data = DEV_DATA(dev);

	data->data_cb[channel].transfer_settings.valid = false;

	if (!data->data_cb[channel].busy) {
		return 0;
	}

	dma_abort_channel(DEV_BASE(dev), (1U << channel));
	dma_clear_transfer_status(DEV_BASE(dev), channel);
	data->data_cb[channel].busy = false;
	return 0;
}

static int dma_hpm_dma_suspend(const struct device *dev, uint32_t channel)
{
	struct call_back *data = DEV_CHANNEL_DATA(dev, channel);

	if (!data->busy) {
		return -EINVAL;
	}
	dma_disable_channel(DEV_BASE(dev), channel);
	return 0;
}

static int dma_hpm_dma_resume(const struct device *dev, uint32_t channel)
{
	struct call_back *data = DEV_CHANNEL_DATA(dev, channel);

	if (!data->busy) {
		return -EINVAL;
	}
	dma_enable_channel(DEV_BASE(dev), channel);
	return 0;
}


static int dma_hpm_dma_reload(const struct device *dev, uint32_t channel,
				uint32_t src, uint32_t dst, size_t size)
{
	struct call_back *data = DEV_CHANNEL_DATA(dev, channel);

	/* Lock the channel configuration */
	const unsigned int key = irq_lock();
	int ret = 0;

	if (!data->transfer_settings.valid) {
		LOG_ERR("Invalid DMA settings on initial config. Configure DMA before reload.");
		ret = -EFAULT;
		goto cleanup;
	}

	data->transfer_settings.ch_config.src_addr = src;
	data->transfer_settings.ch_config.dst_addr = dst;
	data->transfer_settings.ch_config.size_in_byte = size;

	if (status_success != dma_setup_channel(DEV_BASE(dev), channel, &(data->transfer_settings.ch_config), false)) {
		LOG_ERR("Configure DMA channel failed.");
		ret = -EFAULT;
	}

cleanup:
	irq_unlock(key);
	return ret;
}

static uint32_t hpm_dma_get_transfer_size(DMA_Type *ptr, uint8_t ch_index)
{
	return ptr->CHCTRL[ch_index].TRANSIZE;
}

static int dma_hpm_dma_get_status(const struct device *dev, uint32_t channel,
				    struct dma_status *status)
{

	if (DEV_CHANNEL_DATA(dev, channel)->busy) {
		status->busy = true;
		status->pending_length =
			hpm_dma_get_transfer_size(DEV_BASE(dev), channel);
	} else {
		status->busy = false;
		status->pending_length = 0;
	}
	status->dir = DEV_CHANNEL_DATA(dev, channel)->transfer_settings.direction;
	return 0;
}

static const struct dma_driver_api dma_hpm_dma_api = {
	.reload = dma_hpm_dma_reload,
	.config = dma_hpm_dma_configure,
	.start = dma_hpm_dma_start,
	.stop = dma_hpm_dma_stop,
	.suspend = dma_hpm_dma_suspend,
	.resume = dma_hpm_dma_resume,
	.get_status = dma_hpm_dma_get_status,
};

static int dma_hpm_dma_init(const struct device *dev)
{
	const struct dma_hpm_dma_config *config = dev->config;
	struct dma_hpm_data *data = dev->data;

	LOG_DBG("INIT HPM DMA");
	config->irq_config_func(dev);
	memset(dev->data, 0, sizeof(struct dma_hpm_data));
	k_mutex_init(&data->dma_mutex);
	data->dma_ctx.magic = DMA_MAGIC;
	data->dma_ctx.dma_channels = config->dma_channels;
	data->dma_ctx.atomic = data->channels_atomic;
	return 0;
}


#define DMA_HPM_DMA_CONFIG_FUNC(n)				      \
	static void dma_hpm_config_func_##n(const struct device *dev) \
	{							      \
		ARG_UNUSED(dev);	\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority),			\
			    dma_hpm_dma_irq_handler, DEVICE_DT_INST_GET(n), 0);\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

#define DMA_INIT(n)						       \
	static void dma_hpm_config_func_##n(const struct device *dev); \
	static const struct dma_hpm_dma_config dma_config_##n = {    \
		.base = (DMA_Type *)DT_INST_REG_ADDR(n),	       \
		.dmamux_base =					       \
			(DMAMUX_Type *)DT_INST_REG_ADDR_BY_IDX(n, 1),  \
		.dma_channels = DT_INST_PROP(n, dma_channels),	       \
		.irq_config_func = dma_hpm_config_func_##n,	       \
	};							       \
								       \
	struct dma_hpm_data dma_data_##n;			       \
								       \
	DEVICE_DT_INST_DEFINE(n,				       \
			      &dma_hpm_dma_init, NULL,	       \
			      &dma_data_##n, &dma_config_##n,	       \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,   \
			      &dma_hpm_dma_api);		       \
								       \
	DMA_HPM_DMA_CONFIG_FUNC(n);

DT_INST_FOREACH_STATUS_OKAY(DMA_INIT)
