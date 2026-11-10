/*
 * Copyright (c) 2024, 2026 HPMicro
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
#include <drivers/clock_control/hpmicro_clock_control.h>

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
	struct k_mutex mutex;
	struct k_spinlock lock;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	atomic_t pending_bufs;
	cam_config_t cam_config;
	camera_interface_e camera_interface;
	uint32_t pitch;
	uint32_t pixelformat;
	struct k_poll_signal *signal;
	/* Driver-owned spare buffer: always a valid DMA target */
	struct video_buffer *spare;
	/* Sample buffer currently armed on DMASA_FB1/FB2 (or the spare) */
	struct video_buffer *fb_vbuf[2];
};

#define DEV_BASE(dev) (((struct hpmicro_cam_config *)(dev->config))->base)

/*
 * Arm a pending video buffer on one DMA frame buffer (fb: 0 = FB1, 1 = FB2).
 * The caller must hold data->lock while updating the DMASA registers and
 * fb_vbuf[] ownership.
 * If no usable buffer is pending, the driver-owned spare buffer stays armed
 * so the CAM always writes to a valid, non-consumed address.
 */
static void hpmicro_cam_arm_fb(const struct device *dev, int fb)
{
	struct hpmicro_cam_data *data = dev->data;
	const struct hpmicro_cam_config *config = dev->config;
	struct video_buffer *vbuf;

	/* A FB holding a deliverable sample buffer must not be touched */
	if (data->fb_vbuf[fb] != data->spare && data->fb_vbuf[fb] != NULL) {
		return;
	}

	vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (vbuf == NULL || vbuf == data->fb_vbuf[1 - fb]) {
		if (vbuf != NULL) {
			k_fifo_put(&data->fifo_in, vbuf);
		}
		/* No usable buffer: point this FB at the spare (safe target) */
		if (fb == 0) {
			cam_update_buffer(config->base, (uint32_t)data->spare->buffer);
		} else {
			cam_update_buffer2(config->base, (uint32_t)data->spare->buffer);
		}
		data->fb_vbuf[fb] = data->spare;
		return;
	}

	if (fb == 0) {
		cam_update_buffer(config->base, (uint32_t)vbuf->buffer);
	} else {
		cam_update_buffer2(config->base, (uint32_t)vbuf->buffer);
	}
	data->fb_vbuf[fb] = vbuf;
}

__attribute__((section(".isr"))) static void hpmicro_cam_isr(const struct device *dev)
{
	const struct hpmicro_cam_config *cfg = dev->config;
	struct hpmicro_cam_data *data = dev->data;
	k_spinlock_key_t key;
	bool delivered = false;
	bool done[2] = {false, false};

	key = k_spin_lock(&data->lock);
	if ((cam_check_status(cfg->base, cam_status_fb1_dma_transfer_done) == true) &&
			(cfg->base->INT_EN & cam_irq_fb1_dma_transfer_done)) {
			cam_clear_status(cfg->base, cam_status_fb1_dma_transfer_done);
			done[0] = true;
	}
	if ((cam_check_status(cfg->base, cam_status_fb2_dma_transfer_done) == true) &&
			(cfg->base->INT_EN & cam_irq_fb2_dma_transfer_done)) {
			cam_clear_status(cfg->base, cam_status_fb2_dma_transfer_done);
			done[1] = true;
	}

	for (int fb = 0; fb < 2; fb++) {
		struct video_buffer *vbuf;

		if (!done[fb]) {
			continue;
		}
		vbuf = data->fb_vbuf[fb];
		data->fb_vbuf[fb] = NULL;

		if (vbuf == NULL || vbuf == data->spare) {
			/* Spare frame: discard, keep a valid target armed */
			hpmicro_cam_arm_fb(dev, fb);
			continue;
		}

		vbuf->timestamp = k_uptime_get_32();
		k_fifo_put(&data->fifo_out, vbuf);
		atomic_dec(&data->pending_bufs);
		delivered = true;

		/* Re-arm with the next pending buffer, or fall back to the spare */
		hpmicro_cam_arm_fb(dev, fb);
	}
	k_spin_unlock(&data->lock, key);

	if (delivered && IS_ENABLED(CONFIG_POLL) && data->signal) {
		k_poll_signal_raise(data->signal, VIDEO_BUF_DONE);
	}
}

static inline uint32_t hpmicro_get_video_pixel_format(uint32_t pixelformat)
{
	switch (pixelformat) {
	case VIDEO_PIX_FMT_SBGGR8:
	case VIDEO_PIX_FMT_SGBRG8:
	case VIDEO_PIX_FMT_SGRBG8:
	case VIDEO_PIX_FMT_SRGGB8:
		return CAM_COLOR_FORMAT_RAW8;
	case VIDEO_PIX_FMT_RGB565:
		return CAM_COLOR_FORMAT_RGB565;
	case VIDEO_PIX_FMT_YUYV:
		return CAM_COLOR_FORMAT_YCBCR422_YUV422;
	default:
		return 0;
	}
}

static int hpmicro_cam_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct hpmicro_cam_config *config = dev->config;
	struct hpmicro_cam_data *data = dev->data;
	struct video_buffer *old_spare = data->spare;
	struct video_buffer *new_spare = old_spare;
	size_t frame_size;
	int ret;
	uint32_t bpp = hpmicro_get_video_pixel_format(fmt->pixelformat);

	if (!bpp || (data->camera_interface == camera_interface_none)) {
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
		fmt->size = fmt->height * fmt->pitch;
	} else {
		data->pitch = fmt->pitch;
		fmt->size = fmt->height * fmt->pitch;
	}
	data->pixelformat = fmt->pixelformat;
	frame_size = (size_t)data->pitch * data->cam_config.height;
	/*
	 * Keep a driver-owned spare buffer as a permanent, valid DMA target so
	 * the CAM never writes to a stale/consumed address, and use it as the
	 * initial address for both frame buffers.
	 */
	if (!new_spare || new_spare->size < frame_size) {
		new_spare = video_buffer_alloc(frame_size, K_NO_WAIT);
		if (!new_spare) {
			LOG_ERR("Failed to allocate camera spare buffer (%zu bytes)", frame_size);
			return -ENOMEM;
		}
	}
	data->cam_config.buffer1 = (uint32_t)new_spare->buffer;
	data->cam_config.buffer2 = (uint32_t)new_spare->buffer;

#ifdef CONFIG_MIPI_CSI_HPMICRO
	if ((data->camera_interface == camera_interface_mipi_csi) && (config->mipi_csi_phy)) {
		const struct hpmicro_mipi_csi_driver_api *mipi_csi_api =
			(const struct hpmicro_mipi_csi_driver_api *)config->mipi_csi_phy->api;
		if (mipi_csi_api->set_pixel_format(config->mipi_csi_phy, fmt->pixelformat) < 0) {
			ret = -EIO;
			goto fail_spare;
		}
	}
#endif

	if (cam_init(config->base, &data->cam_config) != status_success) {
		ret = -EIO;
		goto fail_spare;
	}

	data->spare = new_spare;
	data->fb_vbuf[0] = new_spare;
	data->fb_vbuf[1] = new_spare;
	if (new_spare != old_spare && old_spare != NULL) {
		if (video_buffer_release(old_spare) < 0) {
			LOG_WRN("Failed to release previous camera spare buffer");
		}
	}

	cam_enable_irq(config->base, cam_irq_fb1_dma_transfer_done | cam_irq_fb2_dma_transfer_done);

	return 0;

fail_spare:
	if (new_spare != old_spare) {
		(void)video_buffer_release(new_spare);
	}
	data->cam_config.buffer1 = old_spare ? (uint32_t)old_spare->buffer : 0;
	data->cam_config.buffer2 = data->cam_config.buffer1;
	return ret;
}

static int hpmicro_cam_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hpmicro_cam_data *data = dev->data;

	if (fmt == NULL) {
		return -EINVAL;
	}
	if (data->camera_interface == camera_interface_mipi_csi) {
		/* mipi csi only support ARGB8888*/
		fmt->pixelformat = VIDEO_PIX_FMT_BGRX32;
	} else {
		fmt->pixelformat = data->pixelformat;
	}
	fmt->height = data->cam_config.height;
	fmt->width = data->cam_config.width;
	fmt->pitch = data->pitch;
	fmt->size = fmt->height * fmt->pitch;

	return 0;
}

static int hpmicro_cam_set_stream(const struct device *dev, bool enable,
				  enum video_buf_type type)
{
	const struct hpmicro_cam_config *config = dev->config;
	struct hpmicro_cam_data *data = dev->data;
	k_spinlock_key_t key;

	if (enable) {
		key = k_spin_lock(&data->lock);
		hpmicro_cam_arm_fb(dev, 0);
		hpmicro_cam_arm_fb(dev, 1);
		cam_start(config->base);
		k_spin_unlock(&data->lock, key);
	} else {
		cam_stop_safely(config->base);
	}

	return 0;
}

static int hpmicro_cam_flush(const struct device *dev, bool cancel)
{
	const struct hpmicro_cam_config *config = dev->config;
	struct hpmicro_cam_data *data = dev->data;
	struct video_buffer *active_vbuf[2] = {NULL, NULL};
	struct video_buffer *vbuf;
	k_spinlock_key_t key;
	uint32_t enabled_irqs;
	bool aborted = false;

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (!cancel) {
		/* Wait until queued and DMA-owned buffers reach fifo_out. */
		while (atomic_get(&data->pending_bufs) != 0) {
			k_sleep(K_MSEC(1));
		}
	} else {
		key = k_spin_lock(&data->lock);
		enabled_irqs = config->base->INT_EN &
			       (cam_irq_fb1_dma_transfer_done | cam_irq_fb2_dma_transfer_done);
		cam_disable_irq(config->base,
				cam_irq_fb1_dma_transfer_done | cam_irq_fb2_dma_transfer_done);
		cam_stop_safely(config->base);
		cam_clear_status(config->base,
				 cam_status_fb1_dma_transfer_done | cam_status_fb2_dma_transfer_done);

		for (int fb = 0; fb < 2; fb++) {
			if (data->fb_vbuf[fb] != NULL && data->fb_vbuf[fb] != data->spare) {
				active_vbuf[fb] = data->fb_vbuf[fb];
			}
			data->fb_vbuf[fb] = data->spare;
		}
		if (data->spare != NULL) {
			cam_update_buffer(config->base, (uint32_t)data->spare->buffer);
			cam_update_buffer2(config->base, (uint32_t)data->spare->buffer);
		}
		if (enabled_irqs != 0U) {
			cam_enable_irq(config->base, enabled_irqs);
		}
		k_spin_unlock(&data->lock, key);

		for (int fb = 0; fb < 2; fb++) {
			vbuf = active_vbuf[fb];
			if (vbuf == NULL) {
				continue;
			}
			vbuf->bytesused = 0;
			vbuf->timestamp = 0;
			k_fifo_put(&data->fifo_out, vbuf);
			atomic_dec(&data->pending_bufs);
			aborted = true;
		}

		while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT))) {
			vbuf->bytesused = 0;
			vbuf->timestamp = 0;
			k_fifo_put(&data->fifo_out, vbuf);
			atomic_dec(&data->pending_bufs);
			aborted = true;
		}

		if (aborted && IS_ENABLED(CONFIG_POLL) && data->signal) {
			k_poll_signal_raise(data->signal, VIDEO_BUF_ABORTED);
		}
	}
	k_mutex_unlock(&data->mutex);

	return 0;
}

static int hpmicro_cam_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct hpmicro_cam_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);
	vbuf->bytesused = data->cam_config.height * data->pitch;
	vbuf->line_offset = 0;

	atomic_inc(&data->pending_bufs);
	k_fifo_put(&data->fifo_in, vbuf);
	k_mutex_unlock(&data->mutex);

	return 0;
}

static int hpmicro_cam_dequeue(const struct device *dev, struct video_buffer **vbuf,
			       k_timeout_t timeout)
{
	struct hpmicro_cam_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}
	if (l1c_dc_is_enabled() && ((*vbuf)->bytesused != 0U)) {
		uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)(*vbuf)->buffer);
		uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)(*vbuf)->buffer +
								  (*vbuf)->bytesused);

		l1c_dc_invalidate(aligned_start, aligned_end - aligned_start);
	}

	return 0;
}

static const struct video_format_cap cam_fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width_min = 32,
		.width_max = 1920,
		.height_min = 32,
		.height_max = 1080,
		.width_step = 1,
		.height_step = 1,
	},
	{
		.pixelformat = VIDEO_PIX_FMT_YUYV,
		.width_min = 32,
		.width_max = 1920,
		.height_min = 32,
		.height_max = 1080,
		.width_step = 1,
		.height_step = 1,
	},
	{
		.pixelformat = VIDEO_PIX_FMT_XRGB32,
		.width_min = 32,
		.width_max = 1920,
		.height_min = 32,
		.height_max = 1080,
		.width_step = 1,
		.height_step = 1,
	},
	{0},
};

static int hpmicro_cam_get_caps(const struct device *dev, struct video_caps *caps)
{
	if (caps == NULL) {
		return -EINVAL;
	}

	caps->format_caps = cam_fmts;
	caps->min_vbuf_count = 2;
	caps->buf_align = 0;

	return 0;
}

#ifdef CONFIG_POLL
static int hpmicro_cam_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct hpmicro_cam_data *data = dev->data;

	if (data->signal && sig != NULL) {
		return -EALREADY;
	}

	data->signal = sig;

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
	atomic_set(&data->pending_bufs, 0);
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

	const struct hpm_clock_dt_spec clock = {
		.name = (clock_name_t)cfg->clock_name,
		.src = (clk_src_t)cfg->clock_src,
		.div = cfg->clock_div,
	};

	clock_configure_enable(&clock);

	LOG_INF("hpmicro cam driver interface: %s\n", cfg->interface_name);


	/* Initialize mutex */
	k_mutex_init(&data->mutex);

	/* Configure IRQ */
	cfg->irq_config_func(dev);
	return 0;
}

static DEVICE_API(video, video_cam_driver_api) = {
	.set_format = hpmicro_cam_set_fmt,
	.get_format = hpmicro_cam_get_fmt,
	.set_stream = hpmicro_cam_set_stream,
	.flush = hpmicro_cam_flush,
	.enqueue = hpmicro_cam_enqueue,
	.dequeue = hpmicro_cam_dequeue,
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
