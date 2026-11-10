/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_adc16

#include <zephyr/drivers/adc.h>
#include <hpm_adc16_drv.h>
#include <hpm_clock_drv.h>
#include <hpm_misc.h>
#include <drivers/clock_control/hpmicro_clock_control.h>
#if DT_ANY_INST_HAS_BOOL_STATUS_OKAY(en_hw_trig)
#include <hpm_trgm_drv.h>
#endif
#include <zephyr/drivers/pinctrl.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_hpmicro_adc16);

#define ADC16_MAX_RESOLUTION 16

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc/adc_context.h"

struct hpmicro_adc16_config {
	ADC16_Type *base;
	clock_name_t adc_clock_name;
	clk_src_t adc_clock_src;
	clock_name_t src_clock_name;
	clk_src_t src_clock_src;
	uint32_t src_clock_div;
	uint32_t adc_clk_div;
	uint32_t sample_time;
	uint16_t vref_mv;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
	bool trig_en;
	uint32_t trig_base;
	uint32_t trig_num;
	uint32_t trig_input_src;
};

struct hpmicro_adc16_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	uint32_t configured_channels;
	uint32_t *seq_buffer;
	uint8_t channel_id;
	uint8_t channel_num;
	uint8_t resolution;
};

#if DT_ANY_INST_HAS_BOOL_STATUS_OKAY(en_hw_trig)
static hpm_stat_t hpmicro_init_trigger_mux(TRGM_Type *ptr, uint32_t hpm_trig_input_src, uint32_t trig_num)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_pulse_at_input_rising_edge;
    trgm_output_cfg.input  = hpm_trig_input_src;
    trgm_output_config(ptr, trig_num, &trgm_output_cfg);
    return status_success;
}
#endif

static int hpmicro_adc16_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	adc16_channel_config_t channel_config;
	uint8_t channel_id = channel_cfg->channel_id;

	if (ADC16_IS_CHANNEL_INVALID(channel_id)) {
		LOG_ERR("Invalid channel %d", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported channel acquisition time");
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_EXTERNAL0 &&
	    channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported channel reference %d", channel_cfg->reference);
		return -ENOTSUP;
	}

	adc16_get_channel_default_config(&channel_config);
	channel_config.sample_cycle = config->sample_time;
	channel_config.ch = channel_id;
	if (adc16_init_channel(config->base, &channel_config) != status_success) {
		LOG_ERR("Failed to initialize channel %u", channel_id);
		return -EIO;
	}

	data->configured_channels |= BIT(channel_id);

	return 0;
}

static int hpmicro_adc16_validate_sequence(const struct device *dev,
					const struct adc_sequence *sequence)
{
	struct hpmicro_adc16_data *data = dev->data;
	uint32_t channels = sequence->channels;
	uint8_t channel_count = 0;
	size_t needed;

	if (channels == 0U) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	if (sequence->buffer == NULL) {
		LOG_ERR("Sequence buffer is NULL");
		return -EINVAL;
	}

	if (sequence->resolution == 0U ||
	    sequence->resolution > ADC16_MAX_RESOLUTION) {
		LOG_ERR("Unsupported resolution %d", sequence->resolution);
		return -EINVAL;
	}

	if ((channels & ~data->configured_channels) != 0U) {
		LOG_ERR("Channel(s) not configured: 0x%08x",
			channels & ~data->configured_channels);
		return -EINVAL;
	}

	while (channels) {
		uint8_t channel_id = find_lsb_set(channels) - 1;

		channels &= ~BIT(channel_id);
		if (ADC16_IS_CHANNEL_INVALID(channel_id)) {
			LOG_ERR("Invalid channel %u in sequence", channel_id);
			return -EINVAL;
		}
		channel_count++;
	}

	if (channel_count > ADC_SOC_SEQ_MAX_LEN) {
		LOG_ERR("Too many channels in sequence (%u/%u)", channel_count,
			ADC_SOC_SEQ_MAX_LEN);
		return -EINVAL;
	}

	needed = channel_count * sizeof(uint16_t);
	if (sequence->options != NULL) {
		needed *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		LOG_ERR("Buffer too small (%zu/%zu)", sequence->buffer_size, needed);
		return -ENOMEM;
	}

	return 0;
}

static int hpmicro_adc16_start_read(const struct device *dev,
				 const struct adc_sequence *sequence)
{
	struct hpmicro_adc16_data *data = dev->data;
	int error;

	if (sequence->oversampling != 0) {
		LOG_ERR("Unsupported oversampling");
		return -ENOTSUP;
	}

	error = hpmicro_adc16_validate_sequence(dev, sequence);
	if (error) {
		return error;
	}

	data->resolution = sequence->resolution;
	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);
	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static int hpmicro_adc16_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct hpmicro_adc16_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, async ? true : false, async);
	error = hpmicro_adc16_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

static int hpmicro_adc16_read(const struct device *dev,
			   const struct adc_sequence *sequence)
{
	return hpmicro_adc16_read_async(dev, sequence, NULL);
}

static void hpmicro_adc16_start_channel(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;

	adc16_seq_config_t seq_cfg = {0};
	adc16_dma_config_t dma_cfg = {0};
	uint32_t channels;
	uint32_t channel_id;
	uint8_t channel_num = 0;

	channels = data->channels;
	while (channels) {
		channel_id = find_lsb_set(channels) - 1;
		channels &= ~BIT(channel_id);

		seq_cfg.queue[channel_num].ch = channel_id;
		LOG_DBG("Starting channel %d", channel_id);
		channel_num++;
	}
	data->channel_num = channel_num;
	seq_cfg.seq_len = channel_num;
	seq_cfg.restart_en = false;
	seq_cfg.cont_en = true;
	seq_cfg.sw_trig_en = !config->trig_en;
	seq_cfg.hw_trig_en = config->trig_en;
	if (adc16_set_seq_config(base, &seq_cfg) != status_success) {
		LOG_ERR("Failed to configure sequence");
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	dma_cfg.start_addr = (uint32_t *)core_local_mem_to_sys_address(0, (uint32_t)data->seq_buffer);
	dma_cfg.buff_len_in_4bytes = channel_num;
	dma_cfg.stop_en = false;
	dma_cfg.stop_pos = 0;

    /* Initialize DMA for the sequence mode */
	if (adc16_init_seq_dma(base, &dma_cfg) != status_success) {
		LOG_ERR("Failed to configure sequence DMA");
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(base, adc16_event_seq_full_complete);

	if (!config->trig_en) {
		/* SW trigger */
		if (adc16_trigger_seq_by_sw(base) != status_success) {
			LOG_ERR("Failed to trigger sequence by SW");
			adc_context_complete(&data->ctx, -EIO);
		}
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct hpmicro_adc16_data *data =
		CONTAINER_OF(ctx, struct hpmicro_adc16_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	hpmicro_adc16_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct hpmicro_adc16_data *data =
		CONTAINER_OF(ctx, struct hpmicro_adc16_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

__attribute__((section(".isr")))static void hpmicro_adc16_isr(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;
	adc16_seq_dma_data_t *seq = (adc16_seq_dma_data_t *)data->seq_buffer;
	uint32_t status;
	uint8_t i;

	status = adc16_get_status_flags(base);

	if (ADC16_INT_STS_SEQ_CMPT_GET(status)) {
		adc16_clear_status_flags(base, status);
		for (i = 0; i < data->channel_num; i++) {
			*data->buffer++ = seq[i].result >> (ADC16_MAX_RESOLUTION - data->resolution);
		}
		data->channels = 0;
		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

static int hpmicro_adc16_init(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;
	adc16_config_t adc_config;
	int err;
	const struct hpm_clock_dt_spec src_clock = {
		.name = config->src_clock_name,
		.src = config->src_clock_src,
		.div = config->src_clock_div,
	};

	if (config->vref_mv == 0U) {
		LOG_ERR("vref-mv must be configured in devicetree");
		return -EINVAL;
	}

	if (clock_set_adc_source(config->adc_clock_name, config->adc_clock_src) != status_success) {
		LOG_ERR("Failed to set adc clock source");
		return -EIO;
	}
	if (clock_configure_enable(&src_clock) != status_success) {
		LOG_ERR("Failed to configure adc clock");
		return -EIO;
	}
	clock_enable_name(config->adc_clock_name);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	adc16_get_default_config(&adc_config);

	adc_config.conv_mode = adc16_conv_mode_sequence;
	adc_config.adc_clk_div = config->adc_clk_div;
#if !defined(HPM_IP_FEATURE_ADC16_FORCE_SYNC_AHB) || !HPM_IP_FEATURE_ADC16_FORCE_SYNC_AHB
#if defined(CONFIG_SOC_SERIES_HPM6800)
	adc_config.sel_sync_ahb = false;
#else
	adc_config.sel_sync_ahb = (config->adc_clock_src == clk_adc_src_ahb0);
#endif
#endif
	if (adc_config.conv_mode == adc16_conv_mode_sequence ||
	    adc_config.conv_mode == adc16_conv_mode_preemption) {
		adc_config.adc_ahb_en = true;
	}

	if (adc16_init(base, &adc_config) != status_success) {
		LOG_ERR("Failed to initialize adc");
		return -EIO;
	}

#if DT_ANY_INST_HAS_BOOL_STATUS_OKAY(en_hw_trig)
	if (config->trig_en) {
		if (config->trig_base == 0U) {
			LOG_ERR("en-hw-trig requires trig-base");
			return -EINVAL;
		}
		if (hpmicro_init_trigger_mux((TRGM_Type *)config->trig_base,
					     config->trig_input_src,
					     config->trig_num) != status_success) {
			LOG_ERR("Failed to configure trigger mux");
			return -EIO;
		}
	}
#endif

	config->irq_config_func(dev);
	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define HPMICRO_ADC16_DEVICE_API(n)					\
	static const struct adc_driver_api hpmicro_adc16_driver_api_##n = { \
		.channel_setup = hpmicro_adc16_channel_setup,		\
		.read = hpmicro_adc16_read,				\
		IF_ENABLED(CONFIG_ADC_ASYNC,				\
			   (.read_async = hpmicro_adc16_read_async,))	\
		.ref_internal = DT_INST_PROP(n, vref_mv),		\
	}

#define ADC_TRIG_CFG(n)	\
	.trig_en = DT_INST_PROP_OR(n, en_hw_trig, false),	\
	.trig_base = DT_INST_PROP_OR(n, trig_base, 0),	\
	.trig_num = DT_INST_PROP_OR(n, trig_num, 0),	\
	.trig_input_src = DT_INST_PROP_OR(n, trig_input_src, 0)

#define ADC16_HPMICRO_INIT(n)						\
	static void hpmicro_adc16_config_func_##n(const struct device *dev); \
									\
	PINCTRL_DT_INST_DEFINE(n);					\
	HPMICRO_ADC16_DEVICE_API(n);					\
	static uint32_t hpmicro_adc16_seq_buffer_##n[ADC_SOC_SEQ_MAX_LEN] \
		__attribute__((__section__(".nocache"), \
			       __aligned__(ADC_SOC_DMA_ADDR_ALIGNMENT))); \
									\
	static const struct hpmicro_adc16_config hpmicro_adc16_config_##n = {	\
		.base = (ADC16_Type *)DT_INST_REG_ADDR(n),		\
		.adc_clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, name),\
		.adc_clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, 0,  src),\
		.src_clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), name),\
		.src_clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), src),\
		.src_clock_div = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), div),\
		.adc_clk_div = DT_INST_PROP(n, adc_clk_div),	\
		.sample_time = DT_INST_PROP(n, sample_time),	\
		.vref_mv = DT_INST_PROP(n, vref_mv),	\
		.irq_config_func = hpmicro_adc16_config_func_##n,		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		ADC_TRIG_CFG(n),	\
	};								\
									\
	static struct hpmicro_adc16_data hpmicro_adc16_data_##n = {		\
		.seq_buffer = hpmicro_adc16_seq_buffer_##n, \
		ADC_CONTEXT_INIT_TIMER(hpmicro_adc16_data_##n, ctx),	\
		ADC_CONTEXT_INIT_LOCK(hpmicro_adc16_data_##n, ctx),	\
		ADC_CONTEXT_INIT_SYNC(hpmicro_adc16_data_##n, ctx),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, hpmicro_adc16_init,			\
			    NULL, &hpmicro_adc16_data_##n,			\
			    &hpmicro_adc16_config_##n, POST_KERNEL,	\
			    CONFIG_ADC_INIT_PRIORITY,			\
			    &hpmicro_adc16_driver_api_##n);			\
									\
	static void hpmicro_adc16_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority), hpmicro_adc16_isr,	\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(ADC16_HPMICRO_INIT)
