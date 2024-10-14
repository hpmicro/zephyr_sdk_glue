/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_camera


#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/video.h>
#include <soc.h>
#include <hpm_cam_drv.h>
#include <hpm_clock_drv.h>
#include <hpm_l1c_drv.h>

#ifdef CONFIG_MIPI_CSI_HPMICRO
#include <mipi_csi_hpmicro.h>
#endif

#ifdef CONFIG_CAMERA_PIXELMUX_HPMICRO
#include <hpm_pixelmux_drv.h>
#endif

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hpm_camera);

typedef enum {
		camera_interface_none = 0,
		camera_interface_dvp = 1,
		camera_interface_mipi_csi = 2,
} camera_interface_e;

struct hpmicro_cam_config {
	CAM_Type *base;
	uint32_t clock_name;
	uint32_t clock_src;
	uint32_t clock_div;
	bool pixclk_sampling_falling;
	bool hsync_active_low;
	bool vsync_active_low;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
	char *interface_name;
	const struct device *mipi_csi_phy;
};

struct hpmicro_cam_data {
	struct k_sem cam_eof_sem;
	struct k_mutex mutex;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	cam_config_t cam_config;
	camera_interface_e camera_interface;
	uint32_t pitch;
	uint32_t pixelformat;
	struct k_poll_signal *signal;
	struct video_buffer *cam_buffer;
	uint8_t cam_dmafb_index;
};

// #define HPMICRO_CAM_ENABLE_EOF  1

#define DEV_BASE(dev) (((struct hpmicro_cam_config *)(dev->config))->base)

__attribute__((section(".isr"))) static void hpmicro_cam_isr(const struct device *dev)
{
	const struct hpmicro_cam_config *cfg = dev->config;
	struct hpmicro_cam_data *data = dev->data;
	enum video_signal_result result = VIDEO_BUF_DONE;
	struct video_buffer *vbuf, *vbuf_first = NULL;
	uint32_t fifo_index = 0, buffer_addr = 0;
#ifdef HPMICRO_CAM_ENABLE_EOF
	if ((cam_check_status(cfg->base, cam_status_end_of_frame) == true) &&
			(cfg->base->INT_EN & cam_irq_end_of_frame)) {
			cam_clear_status(cfg->base, cam_status_end_of_frame);
			cam_stop(cfg->base);
			k_sem_give(&data->cam_eof_sem);
	}
#endif

	if ((cam_check_status(cfg->base, cam_status_start_of_frame) == true) &&
			(cfg->base->INT_EN & cam_irq_start_of_frame)) {
			cam_clear_status(cfg->base, cam_status_start_of_frame);
			if (data->cam_buffer == NULL) {
				while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
					if ((((uint32_t)vbuf->buffer) != cfg->base->DMASA_FB1) && (data->cam_dmafb_index == 0)) {
							cam_update_buffer(cfg->base, (uint32_t)vbuf->buffer);
							k_fifo_put(&data->fifo_in, vbuf);
							break;
					} else if ((((uint32_t)vbuf->buffer) != cfg->base->DMASA_FB2) && (data->cam_dmafb_index == 1)) {
							cam_update_buffer2(cfg->base, (uint32_t)vbuf->buffer);
							k_fifo_put(&data->fifo_in, vbuf);
							break;
					}
					k_fifo_put(&data->fifo_in, vbuf);
					fifo_index++;
					if (fifo_index == 1) {
						vbuf_first = vbuf;
					} else {
						if (vbuf_first == vbuf) {
							break;
						}
					}
				}
			}
	}

	if ((cam_check_status(cfg->base, cam_status_fb1_dma_transfer_done) == true) &&
			(cfg->base->INT_EN & cam_irq_fb1_dma_transfer_done)) {
			cam_clear_status(cfg->base, cam_status_fb1_dma_transfer_done);
			buffer_addr = cfg->base->DMASA_FB1;
			if ((uint32_t)data->cam_buffer->buffer == buffer_addr) {
				if (data->cam_buffer != NULL) {
					video_buffer_release(data->cam_buffer);
					data->cam_buffer = NULL;
				}
				return;
			}
			buffer_addr = cfg->base->DMASA_FB1;
			data->cam_dmafb_index = 0;
			goto done;
	}
	if ((cam_check_status(cfg->base, cam_status_fb2_dma_transfer_done) == true) &&
			(cfg->base->INT_EN & cam_irq_fb2_dma_transfer_done)) {
			cam_clear_status(cfg->base, cam_status_fb2_dma_transfer_done);
			buffer_addr = cfg->base->DMASA_FB2;
			if ((uint32_t)data->cam_buffer->buffer == buffer_addr) {
				if (data->cam_buffer != NULL) {
					video_buffer_release(data->cam_buffer);
					data->cam_buffer = NULL;
				}
				return;
			}
			buffer_addr = cfg->base->DMASA_FB2;
			data->cam_dmafb_index = 1;
			goto done;
	}

done:
	vbuf_first = NULL;
	/* Get matching vbuf by addr */
	while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
			if ((uint32_t)vbuf->buffer == buffer_addr) {
				break;
		}
		/* should never happen on ordered stream, except on capture
			* start/restart, requeue the frame and continue looking for
			* the right buffer.
		*/
		k_fifo_put(&data->fifo_in, vbuf);

		/* prevent infinite loop */
		if (vbuf_first == NULL) {
			vbuf_first = vbuf;
		} else if (vbuf_first == vbuf) {
			vbuf = NULL;
			break;
		}
	}
	if (vbuf == NULL) {
		result = VIDEO_BUF_ERROR;
	} else {
		vbuf->timestamp = k_uptime_get_32();
#if 1
		if (l1c_dc_is_enabled()) {
				uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)vbuf->buffer);
				uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)vbuf->buffer + vbuf->bytesused);
				uint32_t aligned_size = aligned_end - aligned_start;
				l1c_dc_invalidate(aligned_start, aligned_size);
		}
#endif
		k_fifo_put(&data->fifo_out, vbuf);
	}

	/* Trigger Event */
	if (IS_ENABLED(CONFIG_POLL) && data->signal) {
		k_poll_signal_raise(data->signal, VIDEO_BUF_ERROR);
	}
}

static inline uint32_t hpmicro_get_video_pixel_format(uint32_t pixelformat)
{
	switch (pixelformat) {
	case VIDEO_PIX_FMT_BGGR8:
	case VIDEO_PIX_FMT_GBRG8:
	case VIDEO_PIX_FMT_GRBG8:
	case VIDEO_PIX_FMT_RGGB8:
		return CAM_COLOR_FORMAT_RAW8;
	case VIDEO_PIX_FMT_RGB565:
		return CAM_COLOR_FORMAT_RGB565;
	case VIDEO_PIX_FMT_YUYV:
		return CAM_COLOR_FORMAT_YCBCR422;
	default:
		return 0;
	}
}

static int hpmicro_cam_set_fmt(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_format *fmt)
{
	const struct hpmicro_cam_config *config = dev->config;
	struct hpmicro_cam_data *data = dev->data;
	uint32_t bpp = hpmicro_get_video_pixel_format(fmt->pixelformat);

	if (!bpp || ep != VIDEO_EP_OUT || (data->camera_interface == camera_interface_none)) {
		return -EINVAL;
	}
	cam_get_default_config(config->base, &data->cam_config, display_pixel_format_rgb565);
	data->cam_config.width = fmt->width;
	data->cam_config.height = fmt->height;
	data->cam_config.hsync_active_low = config->hsync_active_low;
	data->cam_config.vsync_active_low = config->vsync_active_low;
	data->cam_config.pixclk_sampling_falling = config->pixclk_sampling_falling;
	data->cam_config.color_format = hpmicro_get_video_pixel_format(fmt->pixelformat);
	if (data->camera_interface == camera_interface_mipi_csi) {
#if defined(HPM_IP_FEATURE_CAM_INV_DEN) && (HPM_IP_FEATURE_CAM_INV_DEN == 1)
		data->cam_config.de_active_low = true;
#endif
		data->cam_config.color_ext = true;
		data->cam_config.sensor_bitwidth = CAM_SENSOR_BITWIDTH_24BITS; /* For MIPI-CSI2 */
		data->pitch = 4 * data->cam_config.width;
		fmt->pitch = data->pitch;
	} else {
		data->pitch = fmt->pitch;
	}
	data->pixelformat = fmt->pixelformat;
	if (!data->cam_buffer) {
		data->cam_buffer = video_buffer_alloc(fmt->pitch * fmt->height);
	}
	data->cam_config.buffer1 = (uint32_t)data->cam_buffer->buffer;

#ifdef CONFIG_MIPI_CSI_HPMICRO
	const struct hpmicro_mipi_csi_driver_api *mipi_csi_api =
		(const struct hpmicro_mipi_csi_driver_api *)config->mipi_csi_phy->api;
	if ((data->camera_interface == camera_interface_mipi_csi) && (config->mipi_csi_phy)
				&& (mipi_csi_api->set_pixel_format(config->mipi_csi_phy, fmt->pixelformat) < 0)) {
		return -EIO;
	}
#endif

	if (cam_init(config->base, &data->cam_config) != status_success) {
		return -EIO;
	}

	cam_enable_irq(config->base, cam_irq_fb1_dma_transfer_done | cam_irq_fb2_dma_transfer_done);
#ifndef HPMICRO_CAM_ENABLE_EOF
	cam_enable_irq(config->base, cam_irq_start_of_frame);
#endif

	return 0;
}

static int hpmicro_cam_get_fmt(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_format *fmt)
{
	struct hpmicro_cam_data *data = dev->data;
	if (fmt == NULL || ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}
	if (data->camera_interface == camera_interface_mipi_csi) {
		/* mipi csi only support ARGB8888*/
		fmt->pixelformat = VIDEO_PIX_FMT_XRGB32;
	} else {
		fmt->pixelformat = data->pixelformat;
	}
	fmt->height = data->cam_config.height;
	fmt->width = data->cam_config.width;
	fmt->pitch = data->pitch;

	return 0;
}

static int hpmicro_cam_stream_start(const struct device *dev)
{
	const struct hpmicro_cam_config *config = dev->config;
	cam_start(config->base);
	return 0;
}

static int hpmicro_cam_stream_stop(const struct device *dev)
{
	const struct hpmicro_cam_config *config = dev->config;
	cam_stop_safely(config->base);
	return 0;
}

static int hpmicro_cam_flush(const struct device *dev,
				enum video_endpoint_id ep,
				bool cancel)
{
	struct hpmicro_cam_data *data = dev->data;
	struct video_buf *vbuf;

	if (!cancel) {
		/* wait for all buffer to be processed */
		do {
			k_sleep(K_MSEC(1));
		} while (!k_fifo_is_empty(&data->fifo_in));
	} else {
		while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
			k_fifo_put(&data->fifo_out, vbuf);
			if (IS_ENABLED(CONFIG_POLL) && data->signal) {
				k_poll_signal_raise(data->signal,
						    VIDEO_BUF_ABORTED);
			}
		}
	}

	return 0;
}

static int hpmicro_cam_enqueue(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_buffer *vbuf)
{
	struct hpmicro_cam_data *data = dev->data;
	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}
	k_sleep(K_MSEC(1));

#ifdef HPMICRO_CAM_ENABLE_EOF
	const struct hpmicro_cam_config *config = dev->config;
	if ((uint32_t)data->cam_buffer->buffer == data->cam_config.buffer1) {
		video_buffer_release(data->cam_buffer);
		data->cam_buffer = NULL;
	}
	if ((config->base->CR18 & CAM_CR18_CAM_ENABLE_MASK) == CAM_CR18_CAM_ENABLE_MASK) {
		cam_clear_status(config->base, cam_status_end_of_frame);
		cam_enable_irq(config->base, cam_irq_end_of_frame);
		k_sem_take(&data->cam_eof_sem, K_FOREVER);
		cam_disable_irq(config->base, cam_irq_end_of_frame);
		cam_update_buffer(config->base, (uint32_t)vbuf->buffer);
		cam_update_buffer2(config->base, (uint32_t)vbuf->buffer);
	}
#endif
	vbuf->bytesused = data->cam_config.height * data->pitch;

#ifdef HPMICRO_CAM_ENABLE_EOF
	cam_start(config->base);
#endif
	k_fifo_put(&data->fifo_in, vbuf);

	return 0;
}

static int hpmicro_cam_dequeue(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_buffer **vbuf,
				  k_timeout_t timeout)
{
	struct hpmicro_cam_data *data = dev->data;
	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static inline int hpmicro_cam_set_ctrl(const struct device *dev,
					  unsigned int cid,
					  void *value)
{
	(void)dev;
	(void)cid;
	(void)value;

	return 0;
}

static inline int hpmicro_cam_get_ctrl(const struct device *dev,
					  unsigned int cid,
					  void *value)
{
	(void)dev;
	(void)cid;
	(void)value;

	return 0;
}

static int hpmicro_cam_get_caps(const struct device *dev,
				   enum video_endpoint_id ep,
				   struct video_caps *caps)
{
	int err = 0;

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	caps->min_vbuf_count = 2;

	/* no sensor dev */
	return err;
}

#ifdef CONFIG_POLL
static int hpmicro_cam_set_signal(const struct device *dev,
				     enum video_endpoint_id ep,
				     struct k_poll_signal *signal)
{
	struct hpmicro_cam_data *data = dev->data;

	if (data->signal && signal != NULL) {
		return -EALREADY;
	}

	data->signal = signal;

	return 0;
}
#endif


static int hpmicro_cam_init(const struct device *dev)
{
	const struct hpmicro_cam_config *cfg = dev->config;
	struct hpmicro_cam_data *data = dev->data;
#ifdef CONFIG_MIPI_CSI_HPMICRO
	const struct hpmicro_mipi_csi_driver_api *mipi_csi_api =
		(const struct hpmicro_mipi_csi_driver_api *)cfg->mipi_csi_phy->api;
#endif
	int err;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	data->cam_buffer = NULL;
	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err) {
		return err;
	}

	if (strcmp(cfg->interface_name, "dvp") == 0) {
		data->camera_interface = camera_interface_dvp;
#ifdef CONFIG_CAMERA_PIXELMUX_HPMICRO
	if (cfg->base == HPM_CAM0) {
		pixelmux_cam0_data_source_enable(pixelmux_cam0_sel_dvp);
	} else {
		pixelmux_cam1_data_source_enable(pixelmux_cam1_sel_dvp);
	}
#endif
	} else if (strcmp(cfg->interface_name, "mipi") == 0) {
		data->camera_interface = camera_interface_mipi_csi;
#ifdef CONFIG_MIPI_CSI_HPMICRO
	uint8_t cam_index;
	if (cfg->base == HPM_CAM0) {
		cam_index = CAM0_DEVICE;
	} else {
		cam_index = CAM1_DEVICE;
	}
	if (data->camera_interface == camera_interface_mipi_csi) {
		if (cfg->mipi_csi_phy) {
			if (mipi_csi_api->init) {
				mipi_csi_api->init(cfg->mipi_csi_phy, cam_index);
			}
		} else {
			LOG_ERR("mipi_csi_controller node is not set in dts");
		}
	}
#endif
	} else {
		data->camera_interface = camera_interface_none;
		LOG_ERR("interface node is not set in dts");
	}

	clock_set_source_divider(cfg->clock_name, cfg->clock_src, cfg->clock_div);
	clock_add_to_group(cfg->clock_name, 0);

	LOG_INF("hpmicro cam driver interface: %s\n", cfg->interface_name);


	/* Initialize mutex and semaphore */
	k_mutex_init(&data->mutex);
	k_sem_init(&data->cam_eof_sem, 0, 1);

	/* Configure IRQ */
	cfg->irq_config_func(dev);
	return 0;
}


static const struct video_driver_api video_cam_driver_api = {
	.set_format = hpmicro_cam_set_fmt,
	.get_format = hpmicro_cam_get_fmt,
	.stream_start = hpmicro_cam_stream_start,
	.stream_stop = hpmicro_cam_stream_stop,
	.flush = hpmicro_cam_flush,
	.enqueue = hpmicro_cam_enqueue,
	.dequeue = hpmicro_cam_dequeue,
	.set_ctrl = hpmicro_cam_set_ctrl,
	.get_ctrl = hpmicro_cam_get_ctrl,
	.get_caps = hpmicro_cam_get_caps,
#ifdef CONFIG_POLL
	.set_signal = hpmicro_cam_set_signal,
#endif
};

#define HPM_CAM_CONFIG(idx) \
								\
static const struct hpmicro_cam_config cam_cfg_##idx = {		      \
	.base =	 (CAM_Type *) DT_INST_REG_ADDR(idx),		      \
	.irq_config_func = hpmicro_cam_isr_config_##idx,		      \
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                        \
	.clock_name = DT_INST_CLOCKS_CELL(idx, name),	\
	.clock_src = DT_INST_CLOCKS_CELL(idx, src),	\
	.clock_div = DT_INST_CLOCKS_CELL(idx, div),	\
	.interface_name = DT_INST_PROP(idx, interface),\
	.mipi_csi_phy =  DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(idx, mipi_csi_controller)), \
	.pixclk_sampling_falling = (bool)DT_INST_PROP(idx, pixclk_sampling_falling),\
	.hsync_active_low = (bool)DT_INST_PROP(idx, hsync_active_low),\
	.vsync_active_low = (bool)DT_INST_PROP(idx, vsync_active_low),\
};	

#define HPM_CAMERA_INIT(idx)						      \
									      \
static void hpmicro_cam_isr_config_##idx(const struct device *dev);	      \
									      \
PINCTRL_DT_INST_DEFINE(idx);                                                  \
									      \
HPM_CAM_CONFIG(idx)								      \
									      \
static struct hpmicro_cam_data cam_data_##idx;			              \
									      \
DEVICE_DT_INST_DEFINE(idx,						      \
		    hpmicro_cam_init,					      \
		    NULL,						      \
		    &cam_data_##idx, &cam_cfg_##idx,			      \
		    POST_KERNEL, CONFIG_CAMERA_HPMICRO_INIT_PRIORITY,		      \
		    &video_cam_driver_api);						      \
									      \
static void hpmicro_cam_isr_config_##idx(const struct device *dev)		      \
{									      \
	IRQ_CONNECT(DT_INST_IRQN(idx),					      \
		    DT_INST_IRQ(idx, priority),				      \
		    hpmicro_cam_isr, DEVICE_DT_INST_GET(idx), 0);	      \
									      \
	irq_enable(DT_INST_IRQN(idx));					      \
}

DT_INST_FOREACH_STATUS_OKAY(HPM_CAMERA_INIT);
