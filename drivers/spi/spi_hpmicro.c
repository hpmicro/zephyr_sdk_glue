/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_spi

#include <errno.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <hpm_common.h>
#include <hpm_spi_drv.h>
#include <hpm_clock_drv.h>

#ifdef CONFIG_SPI_HPM_SPI_DMA
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_hpmicro);

#include "spi/spi_context.h"

#define MAX_DATA_WIDTH		32  /* data width 1-32 bits */

#define CONFIG_SPI_INTERRUPT_DRIVEN 1

struct spi_hpm_config {
	SPI_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	void (*irq_config_func)(const struct device *dev);
	uint32_t cs2sclk;
	uint32_t csht;
	const struct pinctrl_dev_config *pincfg;
};

#ifdef CONFIG_SPI_HPM_SPI_DMA
#define SPI_HPM_SPI_DMA_ERROR_FLAG		0x01
#define SPI_HPM_SPI_DMA_RX_DONE_FLAG	0x02
#define SPI_HPM_SPI_DMA_TX_DONE_FLAG	0x04
#define SPI_HPM_SPI_DMA_DONE_FLAG		(SPI_HPM_SPI_DMA_RX_DONE_FLAG | SPI_HPM_SPI_DMA_TX_DONE_FLAG)

struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};
#endif

struct spi_hpm_data {
	const struct device *dev;
	spi_control_config_t control_config;
	struct spi_context ctx;
	size_t transfer_len;
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
	uint8_t *tx_buf;
	uint8_t *rx_buf;
#endif
#ifdef CONFIG_SPI_HPM_SPI_DMA
	volatile uint32_t status_flags;
	struct stream dma_rx;
	struct stream dma_tx;
	/* dummy value used for transferring NOP when tx buf is null */
	uint32_t dummy_tx_buffer;
	/* dummy value used to read RX data into when rx buf is null */
	uint32_t dummy_rx_buffer;
#endif
};

static void spi_hpm_master_transfer_callback(SPI_Type *base, void *userData);

static void spi_hpm_transfer_next_packet(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	struct spi_context *ctx = &data->ctx;
	spi_control_config_t control_config;
	hpm_stat_t stat;

	uint8_t *tx_data;
	uint8_t *rx_data;
	uint32_t tx_size;
	uint32_t rx_size;

	/* get SPI control config for master */
	/* addr and cmd phase are disabled */
    spi_master_get_default_control_config(&control_config);

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		/* nothing left to rx or tx, we're done! */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, dev, 0);
		return;
	}

	tx_size = MIN(data->ctx.tx_len, SPI_SOC_TRANSFER_COUNT_MAX);
	rx_size = MIN(data->ctx.rx_len, SPI_SOC_TRANSFER_COUNT_MAX);

	if (tx_size == 0) {
		/* rx only, nothing to tx */
		tx_data = NULL;
		rx_data = (uint8_t *) ctx->rx_buf;
		control_config.common_config.trans_mode = spi_trans_read_only;
	} else if (rx_size == 0) {
		/* tx only, nothing to rx */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = NULL;
		control_config.common_config.trans_mode = spi_trans_write_only;
	} else if (tx_size == rx_size) {
		/* rx and tx are the same length */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = (uint8_t *) ctx->rx_buf;
		control_config.common_config.trans_mode = spi_trans_write_read_together;
	} else if (tx_size > rx_size) {
		/* Break up the tx into multiple transfers so we don't have to
		 * rx into a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = (uint8_t *) ctx->rx_buf;
		tx_size = rx_size;
		control_config.common_config.trans_mode = spi_trans_write_read_together;
	} else {
		/* Break up the rx into multiple transfers so we don't have to
		 * tx from a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		tx_data = (uint8_t *) ctx->tx_buf;
		rx_data = (uint8_t *) ctx->rx_buf;
		rx_size = tx_size;
		control_config.common_config.trans_mode = spi_trans_write_read_together;
	}

	data->control_config = control_config;
	data->transfer_len = MAX(rx_size, tx_size);

#if CONFIG_SPI_INTERRUPT_DRIVEN
	data->tx_buf = (uint8_t *) ctx->tx_buf;
	data->rx_buf = (uint8_t *) ctx->rx_buf;
    stat = spi_control_init(base, &control_config, tx_size, rx_size);
	if (stat != status_success) {
        LOG_ERR("SPI control init failed");
		return;
    }

	/* command phase, write cmd to start transfer */
    stat = spi_write_command(base, spi_master_mode, &control_config, NULL);
    if (stat != status_success) {
        LOG_ERR("SPI write command failed");
		return;
    }

		/* enable interrupt */
		spi_enable_interrupt(base, spi_rx_fifo_threshold_int | spi_tx_fifo_threshold_int | spi_end_int);
#else
	stat = spi_transfer(base, &control_config, NULL, NULL,
                (uint8_t *)tx_data, tx_size, (uint8_t *)rx_data, rx_size);

	if (stat != status_success) {
		LOG_ERR("Transfer failed");
		return;
	}

	spi_hpm_master_transfer_callback(base, data);
#endif

}

__attribute__((section(".isr")))static void spi_hpm_isr(const struct device *dev)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	volatile uint32_t irq_status;
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
	uint8_t data_len_in_bytes;
	hpm_stat_t stat;
#endif

	irq_status = spi_get_interrupt_status(base);

	if (irq_status & spi_end_int) {
		spi_disable_interrupt(base, spi_end_int | spi_rx_fifo_threshold_int | spi_tx_fifo_threshold_int);
		spi_hpm_master_transfer_callback(base, data);
		spi_clear_interrupt_status(base, spi_end_int);
#ifdef CONFIG_SPI_INTERRUPT_DRIVEN
	} else if (irq_status & (spi_rx_fifo_threshold_int | spi_tx_fifo_threshold_int)) {
		data_len_in_bytes = spi_get_data_length_in_bytes(base);
		if(data->control_config.common_config.trans_mode == spi_trans_read_only) {
			stat = spi_read_data(base, data_len_in_bytes, data->rx_buf, 1);
			(data->rx_buf)++;
		} else if (data->control_config.common_config.trans_mode == spi_trans_write_only) {
			stat = spi_write_data(base, data_len_in_bytes, data->tx_buf, 1);
			(data->tx_buf)++;
		} else if (data->control_config.common_config.trans_mode == spi_trans_write_read_together) {
			stat = spi_write_read_data(base, data_len_in_bytes, data->tx_buf, 1, data->rx_buf, 1);
			(data->tx_buf)++;
			(data->rx_buf)++;
		} else {
			LOG_ERR("Unsupported transfer mode");
			return;
		}

		if (stat != status_success) {
			LOG_ERR("transfer failed");
			return;
		}

		spi_clear_interrupt_status(base, spi_rx_fifo_threshold_int | spi_tx_fifo_threshold_int);
#endif
	}
}

static void spi_hpm_master_transfer_callback(SPI_Type *base, void *userData)
{
	struct spi_hpm_data *data = userData;

	spi_context_update_tx(&data->ctx, 1, data->transfer_len);
	spi_context_update_rx(&data->ctx, 1, data->transfer_len);

	spi_hpm_transfer_next_packet(data->dev);
}

static int spi_hpm_configure(const struct device *dev,
			      const struct spi_config *spi_cfg)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	spi_timing_config_t timing_config = {0};
    spi_format_config_t format_config = {0};
	uint32_t word_size;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}

	word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size > MAX_DATA_WIDTH) {
		LOG_ERR("Word size %d is greater than %d",
			    word_size, MAX_DATA_WIDTH);
		return -EINVAL;
	}

	/* TODO support slave mode */
	if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("slave not supported");
		return -ENOTSUP;
	}

	/* set SPI sclk frequency for master */
	timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(config->clock_name);
	timing_config.master_config.sclk_freq_in_hz = spi_cfg->frequency;
	timing_config.master_config.cs2sclk = config->cs2sclk;
	timing_config.master_config.csht    = config->csht;
    spi_master_timing_init(base, &timing_config);

	spi_master_get_default_format_config(&format_config);
    format_config.common_config.data_len_in_bits = word_size;
    format_config.common_config.data_merge = false;
    format_config.common_config.mosi_bidir = false;
    format_config.common_config.mode = spi_master_mode;

	/* CPOL=0. Active-high LPSPI clock (idles low) */
	/* CPOL=1. Active-low LPSPI clock (idles high) */
	format_config.common_config.cpol =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
		? spi_sclk_high_idle
		: spi_sclk_low_idle;

	/* CPHA=0. Data is captured on the leading edge of the SCK and changed on the following edge. */
	/* CPHA=1. Data is changed on the leading edge of the SCK and captured on the following edge. */
	format_config.common_config.cpha =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
		? spi_sclk_sampling_even_clk_edges
		: spi_sclk_sampling_odd_clk_edges;

	format_config.common_config.lsb =
		(spi_cfg->operation & SPI_TRANSFER_LSB)
		? true
		: false;

	spi_format_init(base, &format_config);

	data->ctx.config = spi_cfg;

	return 0;
}

#ifdef CONFIG_SPI_HPM_SPI_DMA

/* This function is executed in the interrupt context */
static void spi_hpm_dma_callback(const struct device *dev, void *arg, uint32_t channel, int status)
{
	/* arg directly holds the spi device */
	struct spi_hpm_data *data = arg;

	if (status != 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
		data->status_flags |= SPI_HPM_SPI_DMA_ERROR_FLAG;
	} else {
		/* identify the origin of this callback */
		if (channel == data->dma_tx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= SPI_HPM_SPI_DMA_TX_DONE_FLAG;
			LOG_DBG("DMA TX Block Complete");
		} else if (channel == data->dma_rx.channel) {
			/* this part of the transfer ends */
			data->status_flags |= SPI_HPM_SPI_DMA_RX_DONE_FLAG;
			LOG_DBG("DMA RX Block Complete");
		} else {
			LOG_ERR("DMA callback channel %d is not valid.",
								channel);
			data->status_flags |= SPI_HPM_SPI_DMA_ERROR_FLAG;
		}
	}
	spi_context_complete(&data->ctx, dev, 0);
}

static int spi_hpm_dma_tx_load(const struct device *dev, const uint8_t *buf, size_t len)
{
	const struct spi_hpm_config *cfg = dev->config;
	struct spi_hpm_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	SPI_Type *base = cfg->base;


	/* remember active TX DMA channel (used in callback) */
	struct stream *stream = &data->dma_tx;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block for this TX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	if (buf == NULL) {
		/* Treat the transfer as a peripheral to peripheral one, so that DMA
		 * reads from this address each time
		 */
		blk_cfg->source_address = (uint32_t)&data->dummy_tx_buffer;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* tx direction has memory as source and peripheral as dest. */
		blk_cfg->source_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	}

	/* Dest is SPI DATA reg */
	blk_cfg->dest_address = (uint32_t)&(base->DATA);
	blk_cfg->block_size = len;
	/* Transfer 1 byte each DMA loop */
	stream->dma_cfg.source_burst_length = 1;

	stream->dma_cfg.head_block = &stream->dma_blk_cfg;
	/* give the client dev as arg, as the callback comes from the dma */
	stream->dma_cfg.user_data = data;
	/* pass our client origin to the dma: data->dma_tx.dma_channel */
	return dma_config(data->dma_tx.dma_dev, data->dma_tx.channel, &stream->dma_cfg);
}

static int spi_hpm_dma_rx_load(const struct device *dev, uint8_t *buf, size_t len)
{
	const struct spi_hpm_config *cfg = dev->config;
	struct spi_hpm_data *data = dev->data;
	struct dma_block_config *blk_cfg;
	SPI_Type *base = cfg->base;


	/* retrieve active RX DMA channel (used in callback) */
	struct stream *stream = &data->dma_rx;

	blk_cfg = &stream->dma_blk_cfg;

	/* prepare the block for this RX DMA channel */
	memset(blk_cfg, 0, sizeof(struct dma_block_config));

	if (buf == NULL) {
		/* Treat the transfer as a peripheral to peripheral one, so that DMA
		 * reads from this address each time
		 */
		blk_cfg->dest_address = (uint32_t)&data->dummy_rx_buffer;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_PERIPHERAL;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		/* rx direction has peripheral as source and mem as dest. */
		blk_cfg->dest_address = (uint32_t)buf;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		blk_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk_cfg->dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;
	}

	blk_cfg->block_size = len;
	/* Source is SPI DATA reg */
	blk_cfg->source_address = (uint32_t)&(base->DATA);
	stream->dma_cfg.source_burst_length = 1;

	stream->dma_cfg.head_block = blk_cfg;
	stream->dma_cfg.user_data = data;

	/* pass our client origin to the dma: data->dma_rx.channel */
	return dma_config(data->dma_rx.dma_dev, data->dma_rx.channel, &stream->dma_cfg);
}

static int wait_dma_rx_tx_done(const struct device *dev)
{
	struct spi_hpm_data *data = dev->data;
	int ret = -1;

	while (1) {
		ret = spi_context_wait_for_completion(&data->ctx);
		if (ret) {
			LOG_DBG("Timed out waiting for SPI context to complete");
			return ret;
		}
		if (data->status_flags & SPI_HPM_SPI_DMA_ERROR_FLAG) {
			return -EIO;
		}

		if ((data->status_flags & SPI_HPM_SPI_DMA_DONE_FLAG)) {
			LOG_DBG("DMA block completed");
			return 0;
		}
	}
}

static int transceive_dma(const struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
			  spi_callback_t cb,
			  void *userdata)
{
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;
	SPI_Type *base = config->base;
	int ret;
	size_t tx_dma_size, rx_dma_size;
	volatile uint32_t status;
	spi_control_config_t control_config;

	/* set SPI control config for master */
	spi_master_get_default_control_config(&control_config);
	control_config.slave_config.slave_data_only = false;

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	ret = spi_hpm_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	/* Send each spi buf via DMA, updating context as DMA completes */
	while (data->ctx.rx_len > 0 || data->ctx.tx_len > 0) {
		/* Clear status flags */
		data->status_flags = 0U;

		tx_dma_size = MIN(data->ctx.tx_len, SPI_SOC_TRANSFER_COUNT_MAX);
		rx_dma_size = MIN(data->ctx.rx_len, SPI_SOC_TRANSFER_COUNT_MAX);

		if (tx_dma_size == 0) {
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_read_only;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
			
		} else if (rx_dma_size == 0) {
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_only;
			control_config.common_config.tx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
		} else if (tx_dma_size == rx_dma_size) {
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_read_together;
			control_config.common_config.tx_dma_enable = true;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
		} else if (tx_dma_size > rx_dma_size) {
			tx_dma_size = rx_dma_size;
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_read_together;
			control_config.common_config.tx_dma_enable = true;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
		} else {
			rx_dma_size = tx_dma_size;
			ret = spi_hpm_dma_rx_load(dev, data->ctx.rx_buf, rx_dma_size);
			if (ret != 0) {
				goto out;
			}
			ret = spi_hpm_dma_tx_load(dev, data->ctx.tx_buf, tx_dma_size);
			if (ret != 0) {
				goto out;
			}
			control_config.common_config.trans_mode = spi_trans_write_read_together;
			control_config.common_config.tx_dma_enable = true;
			control_config.common_config.rx_dma_enable = true;
			ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.channel);
			if (ret != 0) {
				goto out;
			}
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.channel);
			if (ret != 0) {
				goto out;
			}
		}

		if (status_success != spi_setup_dma_transfer(base, &control_config,
                                NULL, NULL, tx_dma_size, rx_dma_size)) {
			LOG_ERR("SPI setup DMA Transfer failed");
			goto out;
		}

		/* Enable DMA Requests, already enabled in spi_setup_dma_transfer */
		/* spi_enable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable); */

		/* Wait for DMA to finish */
		ret = wait_dma_rx_tx_done(dev);
		if (ret != 0) {
			goto out;
		}

		/* wait until module is idle */
		do {
			status = base->STATUS;
		} while(status & SPI_STATUS_SPIACTIVE_MASK);


		/* Disable DMA */
		spi_disable_dma(base, spi_tx_dma_enable | spi_rx_dma_enable);

		/* Update SPI contexts with amount of data we just sent */
		spi_context_update_tx(&data->ctx, 1, tx_dma_size);
		spi_context_update_rx(&data->ctx, 1, rx_dma_size);
	}

	spi_context_cs_control(&data->ctx, false);

out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

#endif

static int transceive(const struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
			  spi_callback_t cb,
			  void *userdata)
{
	struct spi_hpm_data *data = dev->data;
	int ret;

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	ret = spi_hpm_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	spi_hpm_transfer_next_packet(dev);

	ret = spi_context_wait_for_completion(&data->ctx);
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_hpm_transceive(const struct device *dev,
			       const struct spi_config *spi_cfg,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_HPM_SPI_DMA
	return transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
#endif /* CONFIG_SPI_HPM_SPI_DMA */
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_hpm_transceive_async(const struct device *dev,
				     const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
					 spi_callback_t cb,
					 void *userdata)
{
#ifdef CONFIG_SPI_HPM_SPI_DMA
	return transceive_dma(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
#endif /* CONFIG_SPI_HPM_SPI_DMA */
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_hpm_release(const struct device *dev,
			    const struct spi_config *spi_cfg)
{
	struct spi_hpm_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_hpm_init(const struct device *dev)
{
	int err;
	const struct spi_hpm_config *config = dev->config;
	struct spi_hpm_data *data = dev->data;

	config->irq_config_func(dev);

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	data->dev = dev;

#ifdef CONFIG_SPI_HPM_SPI_DMA
	if (!device_is_ready(data->dma_tx.dma_dev)) {
		LOG_ERR("%s device is not ready", data->dma_tx.dma_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(data->dma_rx.dma_dev)) {
		LOG_ERR("%s device is not ready", data->dma_rx.dma_dev->name);
		return -ENODEV;
	}
#endif /* CONFIG_SPI_HPM_SPI_DMA */

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	clock_set_source_divider(config->clock_name, config->clock_src, config->clock_div);
	clock_add_to_group(config->clock_name, 0);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_hpm_driver_api = {
	.transceive = spi_hpm_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_hpm_transceive_async,
#endif
	.release = spi_hpm_release,
};

#ifdef CONFIG_SPI_HPM_SPI_DMA
#define SPI_DMA_CHANNELS(n)		\
	.dma_tx = {						\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)), \
		.channel =					\
			DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),	\
		.dma_cfg = {					\
			.channel_direction = MEMORY_TO_PERIPHERAL,	\
			.dma_callback = spi_hpm_dma_callback,		\
			.source_data_size = 1,				\
			.dest_data_size = 1,				\
			.block_count = 1,		\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, source) \
		}							\
	},								\
	.dma_rx = {						\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)), \
		.channel =					\
			DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),	\
		.dma_cfg = {				\
			.channel_direction = PERIPHERAL_TO_MEMORY,	\
			.dma_callback = spi_hpm_dma_callback,		\
			.source_data_size = 1,				\
			.dest_data_size = 1,				\
			.block_count = 1,		\
			.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, rx, source) \
		}							\
	}
#else
#define SPI_DMA_CHANNELS(n)
#endif /* CONFIG_SPI_HPM_SPI_DMA */


#define SPI_HPM_DEVICE_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);				\
									\
	static void spi_hpm_config_func_##n(const struct device *dev);	\
									\
	static const struct spi_hpm_config spi_hpm_config_##n = {	\
		.base = (SPI_Type *) DT_INST_REG_ADDR(n),		\
		.clock_name = DT_INST_CLOCKS_CELL(n, name),	\
		.clock_src = DT_INST_CLOCKS_CELL(n, src),	\
		.clock_div = DT_INST_CLOCKS_CELL(n, div),	\
		.irq_config_func = spi_hpm_config_func_##n,		\
		.cs2sclk = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, cs_sck_delay),	\
			DT_INST_PROP(n, cs_sck_delay)),		\
		.csht = UTIL_AND(				\
			DT_INST_NODE_HAS_PROP(n, cs_minimum_high_time),	\
			DT_INST_PROP(n, cs_minimum_high_time)),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)				\
	};								\
									\
	static struct spi_hpm_data spi_hpm_data_##n = {		\
		SPI_CONTEXT_INIT_LOCK(spi_hpm_data_##n, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_hpm_data_##n, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)	\
		SPI_DMA_CHANNELS(n)			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, spi_hpm_init, NULL,			\
			    &spi_hpm_data_##n,				\
			    &spi_hpm_config_##n, POST_KERNEL,		\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_hpm_driver_api);			\
									\
	static void spi_hpm_config_func_##n(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	\
			    spi_hpm_isr, DEVICE_DT_INST_GET(n), 0);	\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(SPI_HPM_DEVICE_INIT)
