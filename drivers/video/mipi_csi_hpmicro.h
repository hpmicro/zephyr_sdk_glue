/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef MIPI_CSI_HPMICRO_H_
#define MIPI_CSI_HPMICRO_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CAM0_DEVICE      0
#define CAM1_DEVICE      1

typedef int (*hpmicro_mipi_csi_api_init)(const struct device *dev, uint8_t cam_index);
typedef int (*hpmicro_mipi_csi_api_set_pixel_format)(const struct device *dev, uint32_t pixel_format);
typedef int (*hpmicro_mipi_csi_api_get_pixel_format)(const struct device *dev, uint32_t *pixel_format);

struct hpmicro_mipi_csi_driver_api {
	/* mandatory callbacks */
	hpmicro_mipi_csi_api_init init;
	hpmicro_mipi_csi_api_set_pixel_format set_pixel_format;
	hpmicro_mipi_csi_api_get_pixel_format get_pixel_format;

};

#ifdef __cplusplus
}
#endif

#endif
