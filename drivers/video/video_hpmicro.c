/*
 * Copyright (c) 2024, 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_video

#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>

struct hpmicro_video_config {
	const struct device *sensor_dev;
	const struct device *camera_dev;
};

struct hpmicro_video_data {
	uint8_t cam_dmafb_index;
};

static int hpmicro_video_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct hpmicro_video_config *config = dev->config;

	if (fmt == NULL) {
		return -EINVAL;
	}

	if (config->camera_dev && video_set_format(config->camera_dev, fmt)) {
		return -EIO;
	}
	if (config->sensor_dev && video_set_format(config->sensor_dev, fmt)) {
		return -EIO;
	}
	if (config->camera_dev && video_get_format(config->camera_dev, fmt)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct hpmicro_video_config *config = dev->config;

	if (fmt == NULL) {
		return -EINVAL;
	}

	if (config->sensor_dev && !video_get_format(config->sensor_dev, fmt)) {
		if ((fmt->pixelformat == 0) || (fmt->height == 0) || (fmt->pitch == 0) ||
		    (fmt->width == 0)) {
			/* set fmt default */
			struct video_caps caps = {
				.type = VIDEO_BUF_TYPE_OUTPUT,
			};
			int err = video_get_caps(config->sensor_dev, &caps);

			if (err < 0) {
				return err;
			}
			if (caps.format_caps && caps.format_caps[0].pixelformat) {
				fmt->height = caps.format_caps[0].height_max;
				fmt->width = caps.format_caps[0].width_max;
				fmt->pixelformat = caps.format_caps[0].pixelformat;
			}
		}
		/* Fill in fmt->size (and normalize pitch) */
		return video_estimate_fmt_size(fmt);
	}
	if (config->camera_dev && !video_get_format(config->camera_dev, fmt)) {
		return video_estimate_fmt_size(fmt);
	}

	return -EIO;
}

static int hpmicro_video_set_stream(const struct device *dev, bool enable,
				    enum video_buf_type type)
{
	const struct hpmicro_video_config *config = dev->config;

	if (enable) {
		if (config->camera_dev && video_stream_start(config->camera_dev, type)) {
			return -EIO;
		}
		if (config->sensor_dev && video_stream_start(config->sensor_dev, type)) {
			return -EIO;
		}
	} else {
		if (config->sensor_dev && video_stream_stop(config->sensor_dev, type)) {
			return -EIO;
		}
		if (config->camera_dev && video_stream_stop(config->camera_dev, type)) {
			return -EIO;
		}
	}

	return 0;
}

static int hpmicro_video_flush(const struct device *dev, bool cancel)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->camera_dev && video_flush(config->camera_dev, cancel)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->camera_dev && video_enqueue(config->camera_dev, vbuf)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_dequeue(const struct device *dev, struct video_buffer **vbuf,
				 k_timeout_t timeout)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->camera_dev && video_dequeue(config->camera_dev, vbuf, timeout)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct hpmicro_video_config *config = dev->config;
	int err = -ENODEV;

	if (caps == NULL) {
		return -EINVAL;
	}

	/* Forward the sensor format capabilities */
	if (config->sensor_dev) {
		err = video_get_caps(config->sensor_dev, caps);
	}

	/* Overwrite the buffer count requirement from the camera interface */
	if (config->camera_dev) {
		struct video_caps cam_caps = {
			.type = caps->type,
		};

		if (video_get_caps(config->camera_dev, &cam_caps) == 0) {
			caps->min_vbuf_count = cam_caps.min_vbuf_count;
		}
	}

	return err;
}

static int hpmicro_video_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->sensor_dev) {
		return video_set_frmival(config->sensor_dev, frmival);
	}

	return -ENOSYS;
}

static int hpmicro_video_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->sensor_dev) {
		return video_get_frmival(config->sensor_dev, frmival);
	}

	return -ENOSYS;
}

static int hpmicro_video_enum_frmival(const struct device *dev, struct video_frmival_enum *fie)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->sensor_dev) {
		return video_enum_frmival(config->sensor_dev, fie);
	}

	return -ENOSYS;
}

#ifdef CONFIG_POLL
static int hpmicro_video_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->camera_dev && video_set_signal(config->camera_dev, sig)) {
		return -EIO;
	}

	return 0;
}
#endif

static int hpmicro_video_init(const struct device *dev)
{
	(void)dev;
	return 0;
}

static DEVICE_API(video, hpm_video_driver_api) = {
	.set_format = hpmicro_video_set_fmt,
	.get_format = hpmicro_video_get_fmt,
	.set_stream = hpmicro_video_set_stream,
	.flush = hpmicro_video_flush,
	.enqueue = hpmicro_video_enqueue,
	.dequeue = hpmicro_video_dequeue,
	.get_caps = hpmicro_video_get_caps,
	.set_frmival = hpmicro_video_set_frmival,
	.get_frmival = hpmicro_video_get_frmival,
	.enum_frmival = hpmicro_video_enum_frmival,
#ifdef CONFIG_POLL
	.set_signal = hpmicro_video_set_signal,
#endif
};

#define HPM_VIDEO_INIT(idx)						      \
									      \
static const struct hpmicro_video_config video_cfg_##idx = {		      \
	.sensor_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(idx, sensor)), \
	.camera_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(idx, camera)), \
};									      \
									      \
DEVICE_DT_INST_DEFINE(idx,						      \
		    hpmicro_video_init,					      \
		    NULL,						      \
		    NULL, &video_cfg_##idx,				      \
		    POST_KERNEL, CONFIG_VIDEO_HPMICRO_INIT_PRIORITY,	      \
		    &hpm_video_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_VIDEO_INIT);
