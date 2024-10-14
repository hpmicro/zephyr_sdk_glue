/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_video



#include <zephyr/drivers/video.h>


struct hpmicro_video_config {
	const struct device *sensor_dev;
  const struct device *camera_dev;
};

struct hpmicro_video_data {
	uint8_t cam_dmafb_index;
};


static int hpmicro_video_set_fmt(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_format *fmt)
{
	const struct hpmicro_video_config *config = dev->config;
	if (config->camera_dev && video_set_format(config->camera_dev, ep, fmt)) {
		return -EIO;
	}
	if (config->sensor_dev && video_set_format(config->sensor_dev, ep, fmt)) {
		return -EIO;
	}
	if (config->camera_dev && video_get_format(config->camera_dev, ep, fmt)) {
		return -EIO;
	}
	return 0;
}

static int hpmicro_video_get_fmt(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_format *fmt)
{
	const struct hpmicro_video_config *config = dev->config;
	struct video_caps caps;
	int err;
	if (fmt == NULL || ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	if (config->sensor_dev && !video_get_format(config->sensor_dev, ep, fmt)) {
		if ((fmt->pixelformat == 0) || (fmt->height == 0) || (fmt->pitch == 0) || (fmt->width == 0)) {
			/* set fmt default */
			err = video_get_caps(config->sensor_dev, ep, &caps);
			if (err < 0) {
				return err;
			}
			fmt->height = caps.format_caps[2].height_max;
			fmt->width = caps.format_caps[2].width_max;
			fmt->pixelformat = caps.format_caps[2].pixelformat;
			fmt->pitch = fmt->width * 2;
		}
		return hpmicro_video_set_fmt(dev, ep, fmt);
	}
	if (config->camera_dev && video_get_format(config->camera_dev, ep, fmt)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_stream_start(const struct device *dev)
{
	const struct hpmicro_video_config *config = dev->config;
	if (config->camera_dev && video_stream_start(config->camera_dev)) {
		return -EIO;
	}

	if (config->sensor_dev && video_stream_start(config->sensor_dev)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_stream_stop(const struct device *dev)
{
	const struct hpmicro_video_config *config = dev->config;

	if (config->sensor_dev && video_stream_stop(config->sensor_dev)) {
		return -EIO;
	}

	if (config->camera_dev && video_stream_stop(config->camera_dev)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_flush(const struct device *dev,
				enum video_endpoint_id ep,
				bool cancel)
{
	const struct hpmicro_video_config *config = dev->config;
	if (config->camera_dev && video_flush(config->camera_dev, ep, cancel)) {
		return -EIO;
	}
	return 0;
}

static int hpmicro_video_enqueue(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_buffer *vbuf)
{
	const struct hpmicro_video_config *config = dev->config;
	if (config->camera_dev && video_enqueue(config->camera_dev, ep, vbuf)) {
		return -EIO;
	}

	return 0;
}

static int hpmicro_video_dequeue(const struct device *dev,
				  enum video_endpoint_id ep,
				  struct video_buffer **vbuf,
				  k_timeout_t timeout)
{
	const struct hpmicro_video_config *config = dev->config;
	if (config->camera_dev && video_dequeue(config->camera_dev, ep, vbuf, timeout)) {
		return -EIO;
	}
	return 0;
}

static inline int hpmicro_video_set_ctrl(const struct device *dev,
					  unsigned int cid,
					  void *value)
{
	const struct hpmicro_video_config *config = dev->config;
	int ret = -ENOTSUP;

	/* Forward to sensor dev if any */
	if (config->sensor_dev) {
		ret = video_set_ctrl(config->sensor_dev, cid, value);
	}

	return ret;
}

static inline int hpmicro_video_get_ctrl(const struct device *dev,
					  unsigned int cid,
					  void *value)
{
	const struct hpmicro_video_config *config = dev->config;
	int ret = -ENOTSUP;

	/* Forward to sensor dev if any */
	if (config->sensor_dev) {
		ret = video_get_ctrl(config->sensor_dev, cid, value);
	}

	return ret;
}

static int hpmicro_video_get_caps(const struct device *dev,
				   enum video_endpoint_id ep,
				   struct video_caps *caps)
{
	const struct hpmicro_video_config *config = dev->config;
	int err = -ENODEV;

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	/* Just forward to sensor dev for now */
	if (config->sensor_dev) {
		err = video_get_caps(config->sensor_dev, ep, caps);
	}

	if (config->camera_dev && video_get_caps(config->camera_dev, ep, caps)) {
		return -EIO;
	}
	/* no sensor dev */
	return err;
}

#ifdef CONFIG_POLL
static int hpmicro_video_set_signal(const struct device *dev,
				     enum video_endpoint_id ep,
				     struct k_poll_signal *signal)
{
	const struct hpmicro_video_config *config = dev->config;
	if (config->camera_dev && video_set_signal(config->camera_dev, ep, signal)) {
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


static const struct video_driver_api hpm_video_driver_api = {
	.set_format = hpmicro_video_set_fmt,
	.get_format = hpmicro_video_get_fmt,
	.stream_start = hpmicro_video_stream_start,
	.stream_stop = hpmicro_video_stream_stop,
	.flush = hpmicro_video_flush,
	.enqueue = hpmicro_video_enqueue,
	.dequeue = hpmicro_video_dequeue,
	.set_ctrl = hpmicro_video_set_ctrl,
	.get_ctrl = hpmicro_video_get_ctrl,
	.get_caps = hpmicro_video_get_caps,
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
		    NULL, &video_cfg_##idx,			      \
		    POST_KERNEL, CONFIG_VIDEO_HPMICRO_INIT_PRIORITY,		      \
		    &hpm_video_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_VIDEO_INIT);