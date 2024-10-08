/*
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/video.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define VIDEO_DEV_SW "VIDEO_SW_GENERATOR"

#if DT_HAS_CHOSEN(zephyr_display)
#include <zephyr/drivers/display.h>
#endif


#if DT_HAS_CHOSEN(zephyr_display)
static inline void video_display_frame(const struct device *const display_dev,
				       const struct video_buffer *const vbuf,
				       const struct video_format fmt)
{
	struct display_buffer_descriptor buf_desc;

	buf_desc.buf_size = vbuf->bytesused;
	buf_desc.width = fmt.width;
	buf_desc.pitch = buf_desc.width;
	buf_desc.height = fmt.height;

	display_write(display_dev, 50, 50, &buf_desc, vbuf->buffer);
}
#endif

int main(void)
{
	struct video_buffer *buffers[2], *vbuf;
	struct video_format fmt;
	struct video_caps caps;
	const struct device *video;
	unsigned int frame = 0;
	size_t bsize;
	int i = 0;

	/* Default to software video pattern generator */
	video = device_get_binding(VIDEO_DEV_SW);
	if (video == NULL) {
		LOG_ERR("Video device %s not found", VIDEO_DEV_SW);
		return 0;
	}

	/* But would be better to use a real video device if any */
#if DT_HAS_CHOSEN(zephyr_camera)
	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	if (video == NULL) {
		LOG_ERR("Video device not found");
		return 0;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready.\n", dev->name);
		return 0;
	}

	video = dev;
#endif

#if DT_HAS_CHOSEN(zephyr_display)
	const struct device *display  = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	struct display_capabilities disp_capabilities;
	if (display == NULL) {
		LOG_ERR("display device not found");
		return 0;
	}

	if (!device_is_ready(display)) {
		LOG_ERR("%s: device not ready.\n", display->name);
		return 0;
	}

	display_get_capabilities(display, &disp_capabilities);

#endif

	printk("- Device name: %s\n", video->name);

	/* Get capabilities */
	if (video_get_caps(video, VIDEO_EP_OUT, &caps)) {
		LOG_ERR("Unable to retrieve video capabilities");
		return 0;
	}

	printk("- Capabilities:\n");
	while (caps.format_caps[i].pixelformat) {
		const struct video_format_cap *fcap = &caps.format_caps[i];
		/* fourcc to string */
		printk("  %c%c%c%c width [%u; %u; %u] height [%u; %u; %u]\n",
		       (char)fcap->pixelformat,
		       (char)(fcap->pixelformat >> 8),
		       (char)(fcap->pixelformat >> 16),
		       (char)(fcap->pixelformat >> 24),
		       fcap->width_min, fcap->width_max, fcap->width_step,
		       fcap->height_min, fcap->height_max, fcap->height_step);
		i++;
	}
	fmt.height = 480;
	fmt.width = 640;
	fmt.pitch = 640 * 2;
	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
	if (video_set_format(video, VIDEO_EP_OUT, &fmt)) {
		LOG_ERR("Unable to set video format");
		return 0;
	}
	/* Get default/native format */
	if (video_get_format(video, VIDEO_EP_OUT, &fmt)) {
		LOG_ERR("Unable to retrieve video format");
		return 0;
	}

	printk("- Default format: %c%c%c%c %ux%u\n", (char)fmt.pixelformat,
	       (char)(fmt.pixelformat >> 8),
	       (char)(fmt.pixelformat >> 16),
	       (char)(fmt.pixelformat >> 24),
	       fmt.width, fmt.height);

	/* Size to allocate for each buffer */
	bsize = fmt.pitch * fmt.height;

	/* Alloc video buffers and enqueue for capture */
	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
		buffers[i] = video_buffer_alloc(bsize);
		if (buffers[i] == NULL) {
			LOG_ERR("Unable to alloc video buffer");
			return 0;
		}

		video_enqueue(video, VIDEO_EP_OUT, buffers[i]);
	}

	/* Start video capture */
	if (video_stream_start(video)) {
		LOG_ERR("Unable to start capture (interface)");
		return 0;
	}

	printk("Capture started\n");

	/* Grab video frames */
	while (1) {
		int err;
		// k_sleep(K_MSEC(10));
		err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			return 0;
		}

		printk("\rGot frame %u! size: %u; timestamp %u ms\n",
		       frame++, vbuf->bytesused, vbuf->timestamp);
#if DT_HAS_CHOSEN(zephyr_display)
		video_display_frame(display, vbuf, fmt);
#endif

		err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
			return 0;
		}
	}
}
