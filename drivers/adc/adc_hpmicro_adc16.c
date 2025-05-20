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
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
#include <hpm_trgm_drv.h>
#endif
#include <zephyr/drivers/pinctrl.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_hpmicro_adc16);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc/adc_context.h"

struct hpmicro_adc16_config {
	ADC16_Type *base;
	clock_name_t adc_clock_name;
	clk_src_t adc_clock_src;
	clock_name_t src_clock_name;
	clk_src_t src_clock_src;
	uint32_t src_clock_div;
	uint32_t sample_time;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	bool trig_en;
	TRGM_Type *trig_reg;
	uint32_t trig_num;
	uint32_t trig_input_src;
#endif
};

struct hpmicro_adc16_data {
	const struct device *dev;
	struct adc_context ctx;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	uint32_t seq_buffer[ADC_SOC_SEQ_MAX_LEN];
	uint8_t channel_id;
	uint8_t channel_num;
	uint8_t resolution;
};

#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
static void hpmicro_init_trigger_mux(TRGM_Type * ptr, uint32_t hpm_trig_input_src, uint32_t trig_num)
{
    trgm_output_t trgm_output_cfg;

    trgm_output_cfg.invert = false;
    trgm_output_cfg.type   = trgm_output_pulse_at_input_rising_edge;
    trgm_output_cfg.input  = hpm_trig_input_src;
    trgm_output_config(ptr, trig_num, &trgm_output_cfg);
}
#endif

static int hpmicro_adc16_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
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

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported channel reference");
		return -ENOTSUP;
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

	adc16_channel_config_t channel_config;
	adc16_seq_config_t seq_cfg;
	adc16_dma_config_t dma_cfg;
	uint32_t channels;
	uint32_t channel_id;
	uint8_t channel_num = 0;

    channel_config.sample_cycle = config->sample_time;
	channels = data->channels;
	while (channels) {
		channel_id = find_lsb_set(channels) - 1;
		channels &= ~BIT(channel_id);

		channel_config.ch           = channel_id;
        adc16_init_channel(base, &channel_config);
		seq_cfg.queue[channel_num].ch = channel_id;
		LOG_DBG("Starting channel %d", channel_id);
		channel_num ++;
	};
	data->channel_num = channel_num;
	seq_cfg.seq_len    = channel_num;
    seq_cfg.restart_en = false;
    seq_cfg.cont_en    = true;
    seq_cfg.sw_trig_en = true;
    seq_cfg.hw_trig_en = true;
	adc16_set_seq_config(base, &seq_cfg);

	 /* Set DMA config */
    dma_cfg.start_addr         = (uint32_t *)core_local_mem_to_sys_address(0, (uint32_t)data->seq_buffer);
    dma_cfg.buff_len_in_4bytes = channel_num;
    dma_cfg.stop_en            = false;
    dma_cfg.stop_pos           = 0;
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	if (config->trig_en) {
		hpmicro_init_trigger_mux(config->trig_reg, config->trig_input_src, config->trig_num);
	}
#endif
    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(base, &dma_cfg);

    /* Enable sequence complete interrupt */
    adc16_enable_interrupts(base, adc16_event_seq_full_complete);
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	if (!config->trig_en) {
#endif
    	/* SW trigger */
    	adc16_trigger_seq_by_sw(base);
#if DT_NODE_HAS_PROP(DT_NODELABEL(adc0), trig-base)
	}
#endif
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
	uint32_t status;
	uint8_t channel_id;
	uint32_t channels;
	uint16_t result;

	status = adc16_get_status_flags(base);

	if (ADC16_INT_STS_SEQ_CMPT_GET(status)) {
		adc16_clear_status_flags(base, status);
		channels = data->channels;
		while (channels) {
			channel_id = find_lsb_set(channels) - 1;
			channels &= ~BIT(channel_id);
			adc16_get_oneshot_result(base, channel_id, &result);
			result = (result & 0xffff) >> (16 - data->resolution);
			*data->buffer++ = result;
		};
		data->channels = 0;
		adc_context_on_sampling_done(&data->ctx, dev);
	}
	adc16_clear_status_flags(base, status);
}

static int hpmicro_adc16_init(const struct device *dev)
{
	const struct hpmicro_adc16_config *config = dev->config;
	struct hpmicro_adc16_data *data = dev->data;
	ADC16_Type *base = config->base;
	adc16_config_t adc_config;
	int err;

	clock_set_adc_source(config->adc_clock_name, config->adc_clock_src);
	clock_set_source_divider(config->src_clock_name, config->src_clock_src, config->src_clock_div);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	adc16_get_default_config(&adc_config);

    adc_config.conv_mode      = adc16_conv_mode_sequence;
    adc_config.adc_clk_div    = 2;
    adc_config.sel_sync_ahb   = true;
    if (adc_config.conv_mode == adc16_conv_mode_sequence ||
        adc_config.conv_mode == adc16_conv_mode_preemption) {
        adc_config.adc_ahb_en = true;
    }

	adc16_init(base, &adc_config);

	config->irq_config_func(dev);
	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api hpmicro_adc16_driver_api = {
	.channel_setup = hpmicro_adc16_channel_setup,
	.read = hpmicro_adc16_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = hpmicro_adc16_read_async,
#endif
};

#if CONFIG_ADC_TRIG
	#define ADC_TRIG_CFG(n)	\
	.trig_en = DT_INST_PROP(n, en_hw_trig),	\
	.trig_reg = (TRGM_Type *)DT_INST_PROP(n, trig_base),	\
	.trig_num = DT_INST_PROP(n, trig_num),	\
	.trig_input_src = DT_INST_PROP(n, trig_input_src)

#else
	#define ADC_TRIG_CFG(n)
#endif

#define ACD12_HPMICRO_INIT(n)						\
	static void hpmicro_adc16_config_func_##n(const struct device *dev); \
									\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static const struct hpmicro_adc16_config hpmicro_adc16_config_##n = {	\
		.base = (ADC16_Type *)DT_INST_REG_ADDR(n),		\
		.adc_clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, name),\
		.adc_clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, 0,  src),\
		.src_clock_name = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), name),\
		.src_clock_src = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), src),\
		.src_clock_div = DT_INST_CLOCKS_CELL_BY_IDX(n, DT_INST_CLOCKS_HAS_IDX(n, 1), div),\
		.sample_time = DT_INST_PROP(n, sample_time),	\
		.irq_config_func = hpmicro_adc16_config_func_##n,		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		ADC_TRIG_CFG(n)	\
	};								\
									\
	static struct hpmicro_adc16_data hpmicro_adc16_data_##n = {		\
		ADC_CONTEXT_INIT_TIMER(hpmicro_adc16_data_##n, ctx),	\
		ADC_CONTEXT_INIT_LOCK(hpmicro_adc16_data_##n, ctx),	\
		ADC_CONTEXT_INIT_SYNC(hpmicro_adc16_data_##n, ctx),	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, hpmicro_adc16_init,			\
			    NULL, &hpmicro_adc16_data_##n,			\
			    &hpmicro_adc16_config_##n, POST_KERNEL,	\
			    CONFIG_ADC_INIT_PRIORITY,			\
			    &hpmicro_adc16_driver_api);			\
									\
	static void hpmicro_adc16_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, priority), hpmicro_adc16_isr,	\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(ACD12_HPMICRO_INIT)
