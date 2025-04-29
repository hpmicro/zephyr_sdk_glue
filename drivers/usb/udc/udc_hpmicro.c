/*
 * Copyright (c) 2025 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#define DT_DRV_COMPAT hpmicro_hpm_udc

#include <soc.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/pinctrl.h>

#include "../udc/udc_common.h"
#include "hpm_clock_drv.h"
#include "hpm_usb_device.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_hpmicro, CONFIG_UDC_DRIVER_LOG_LEVEL);

/*
 * There is no real advantage to change control endpoint size
 * but we can use it for testing UDC driver API and higher layers.
 */
#define USB_HPM_MPS0		UDC_MPS0_64
#define USB_HPM_EP0_SIZE	64

struct udc_hpm_config {
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	dcd_data_t *dcd_data;
	USB_Type *base;
	clock_name_t clock_name;
	const struct pinctrl_dev_config *pincfg;
};

struct udc_hpm_data {
	usb_device_handle_t handle;
};

/* TODO: implement the cache maintenance
 * solution1: Use the non-cached buf to do memcpy before/after giving buffer to usb controller.
 * solution2: Use cache API to flush/invalid cache. but it needs the given buffer is
 * cache line size aligned and the buffer range cover multiple of cache line size block.
 * Need to change the usb stack to implement it, will try to implement it later.
 */
#if defined(CONFIG_NOCACHE_MEMORY)
K_HEAP_DEFINE_NOCACHE(hpm_packet_alloc_pool, 16u * 2u * 1024u);

/* allocate non-cached buffer for usb */
static void *udc_hpm_nocache_alloc(uint32_t size)
{
	void *p = (void *)k_heap_alloc(&hpm_packet_alloc_pool, size, K_NO_WAIT);

	if (p != NULL) {
		(void)memset(p, 0, size);
	}

	return p;
}

/* free the allocated non-cached buffer */
static void udc_hpm_nocache_free(void *p)
{
	if (p == NULL) {
		return;
	}
	k_heap_free(&hpm_packet_alloc_pool, p);
}
#endif

/* Index to bit position in register */
static inline uint8_t ep_idx2bit(uint8_t ep_idx)
{
	return ep_idx / 2 + ((ep_idx % 2) ? 16 : 0);
}

/* If ep is busy, return busy. Otherwise feed the buf to controller */
static int udc_hpm_ep_feed(const struct device *dev,
			struct udc_ep_config *const cfg,
			struct net_buf *const buf)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	bool status = true;
	uint8_t *data;
	uint32_t len;
	unsigned int key;

	key = irq_lock();
	if (!udc_ep_is_busy(dev, cfg->addr)) {
		udc_ep_set_busy(dev, cfg->addr, true);
		irq_unlock(key);

		if (USB_EP_DIR_IS_OUT(cfg->addr)) {
			len = net_buf_tailroom(buf);
#if defined(CONFIG_NOCACHE_MEMORY)
			data = (len == 0 ? NULL : udc_hpm_nocache_alloc(len));
#else
			data = net_buf_tail(buf);
#endif
			status = usb_device_edpt_xfer(handle, cfg->addr, data, len);
		} else {
			len = buf->len;
#if defined(CONFIG_NOCACHE_MEMORY)
			data = (len == 0 ? NULL : udc_hpm_nocache_alloc(len));
			memcpy(data, buf->data, len);
#else
			data = buf->data;
#endif
			status = usb_device_edpt_xfer(handle, cfg->addr, data, len);
		}

		key = irq_lock();
		if (!status) {
			udc_ep_set_busy(dev, cfg->addr, false);
		}
		irq_unlock(key);
	} else {
		irq_unlock(key);
		return -EBUSY;
	}

	return (status ? 0 : -EIO);
}

/* return success if the ep is busy or stalled. */
static int udc_hpm_ep_try_feed(const struct device *dev,
			struct udc_ep_config *const cfg)
{
	struct net_buf *feed_buf;

	feed_buf = udc_buf_peek(dev, cfg->addr);
	if (feed_buf) {
		int ret = udc_hpm_ep_feed(dev, cfg, feed_buf);

		return ((ret == -EBUSY || ret == -EACCES || ret == 0) ? 0 : -EIO);
	}

	return 0;
}

/*
 * Allocate buffer and initiate a new control OUT transfer.
 */
static int udc_hpm_ctrl_feed_dout(const struct device *dev,
				   const size_t length)
{
	struct net_buf *buf;
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	int ret;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_put(&cfg->fifo, buf);

	ret = udc_hpm_ep_feed(dev, cfg, buf);

	if (ret) {
		net_buf_unref(buf);
		return ret;
	}

	return 0;
}

static void udc_hpm_handler_setup(const struct device *dev, struct usb_setup_packet *setup)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	int err;
	struct net_buf *buf;

	LOG_DBG("setup packet");
	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT,
			sizeof(struct usb_setup_packet));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate for setup");
		return ;
	}

	udc_ep_buf_set_setup(buf);
	memcpy(buf->data, setup, 8);
	net_buf_add(buf, 8);

	if (setup->RequestType.type == USB_REQTYPE_TYPE_STANDARD &&
	    setup->RequestType.direction == USB_REQTYPE_DIR_TO_DEVICE &&
	    setup->bRequest == USB_SREQ_SET_ADDRESS &&
	    setup->wLength == 0) {
		usb_dcd_set_address(handle->regs, setup->wValue);
	}

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	if (!buf->len) {
		return ;
	}

	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		LOG_DBG("s:%p|feed for -out-", buf);
		err = udc_hpm_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			err = udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		err = udc_ctrl_submit_s_in_status(dev);
	} else {
		err = udc_ctrl_submit_s_status(dev);
	}

	return ;
}

static int udc_hpm_handler_ctrl_out(const struct device *dev, struct net_buf *buf,
				uint8_t *hpm_buf, uint16_t hpm_len)
{
	int err = 0;
	uint32_t len;

	len = MIN(net_buf_tailroom(buf), hpm_len);
#if defined(CONFIG_NOCACHE_MEMORY)
	memcpy(net_buf_tail(buf), hpm_buf, len);
	udc_hpm_nocache_free(hpm_buf);
#endif
	net_buf_add(buf, len);
	if (udc_ctrl_stage_is_status_out(dev)) {
		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);
		/* Status stage finished, notify upper layer */
		err = udc_ctrl_submit_status(dev, buf);
	} else {
		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);
	}

	if (udc_ctrl_stage_is_status_in(dev)) {
		err = udc_ctrl_submit_s_out_status(dev, buf);
	}

	return err;
}

static int udc_hpm_handler_ctrl_in(const struct device *dev, struct net_buf *buf,
				uint8_t *hpm_buf, uint16_t hpm_len)
{
	int err = 0;
	uint32_t len;

	len = MIN(buf->len, hpm_len);
	buf->data += len;
	buf->len -= len;
#if defined(CONFIG_NOCACHE_MEMORY)
	udc_hpm_nocache_free(hpm_buf);
#endif

	if (udc_ctrl_stage_is_status_in(dev) ||
	udc_ctrl_stage_is_no_data(dev)) {
		/* Status stage finished, notify upper layer */
		err = udc_ctrl_submit_status(dev, buf);
	}

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_status_out(dev)) {
		/*
		 * IN transfer finished, release buffer,
		 * control OUT buffer should be already fed.
		 */
		net_buf_unref(buf);
		err = udc_hpm_ctrl_feed_dout(dev, 0u);
	}

	return err;
}

static int udc_hpm_handler_non_ctrl_in(const struct device *dev, uint8_t ep,
			struct net_buf *buf, uint8_t *hpm_buf, uint16_t hpm_len)
{
	int err;
	uint32_t len;

	len = MIN(buf->len, hpm_len);
	buf->data += len;
	buf->len -= len;

#if defined(CONFIG_NOCACHE_MEMORY)
	udc_hpm_nocache_free(hpm_buf);
#endif
	err = udc_submit_ep_event(dev, buf, 0);
	udc_hpm_ep_try_feed(dev, udc_get_ep_cfg(dev, ep));

	return err;
}

static int udc_hpm_handler_non_ctrl_out(const struct device *dev, uint8_t ep,
			struct net_buf *buf, uint8_t *hpm_buf, uint16_t hpm_len)
{
	int err;
	uint32_t len;

	len = MIN(net_buf_tailroom(buf), hpm_len);
#if defined(CONFIG_NOCACHE_MEMORY)
	memcpy(net_buf_tail(buf), hpm_buf, len);
#endif
	net_buf_add(buf, len);

#if defined(CONFIG_NOCACHE_MEMORY)
	udc_hpm_nocache_free(hpm_buf);
#endif
	err = udc_submit_ep_event(dev, buf, 0);
	udc_hpm_ep_try_feed(dev, udc_get_ep_cfg(dev, ep));

	return err;
}

static int udc_hpm_handler_out(const struct device *dev, uint8_t ep,
				uint8_t *hpm_buf, uint16_t hpm_len)
{
	int err;
	struct net_buf *buf;
	unsigned int key;

	buf = udc_buf_get(dev, ep);

	key = irq_lock();
	udc_ep_set_busy(dev, ep, false);
	irq_unlock(key);

	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	if (ep == USB_CONTROL_EP_OUT) {
		err = udc_hpm_handler_ctrl_out(dev, buf, hpm_buf, hpm_len);
	} else {
		err = udc_hpm_handler_non_ctrl_out(dev, ep, buf, hpm_buf, hpm_len);
	}

	return err;
}

/* return true - zlp is feed; false - no zlp */
static bool udc_hpm_handler_zlt(const struct device *dev, uint8_t ep, struct net_buf *buf,
				uint16_t hpm_len)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	/* The whole transfer is already done by HPM controller driver. */
	if (hpm_len >= buf->len) {
		if (udc_ep_buf_has_zlp(buf)) {
			bool status;

			udc_ep_buf_clear_zlp(buf);
			status = usb_device_edpt_xfer(handle, ep, NULL, 0);
			if (status != true) {
				udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
				return false;
			}
			return true;
		}
	}

	return false;
}

static int udc_hpm_handler_in(const struct device *dev, uint8_t ep,
				uint8_t *hpm_buf, uint16_t hpm_len)
{
	int err;
	struct net_buf *buf;
	unsigned int key;

	buf = udc_buf_peek(dev, ep);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	if (udc_hpm_handler_zlt(dev, ep, buf, hpm_len)) {
		return 0;
	}

	buf = udc_buf_get(dev, ep);

	key = irq_lock();
	udc_ep_set_busy(dev, ep, false);
	irq_unlock(key);

	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}
	if (ep == USB_CONTROL_EP_IN) {
		err = udc_hpm_handler_ctrl_in(dev, buf, hpm_buf, hpm_len);
	} else {
		err = udc_hpm_handler_non_ctrl_in(dev, ep, buf, hpm_buf, hpm_len);
	}

	return err;
}

static void udc_hpm_isr(const struct device *dev)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	uint32_t int_status;
	uint32_t transfer_len;
	bool ep_cb_req;

	/* Acknowledge handled interrupt */
	int_status = usb_device_status_flags(handle);
	int_status &= usb_device_interrupts(handle);
	usb_device_clear_status_flags(handle, int_status);

	if (int_status & USB_USBINTR_UEE_MASK) {
		LOG_DBG("usbd intr error!\r\n");
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
	}

	if (int_status & USB_USBINTR_URE_MASK) {
		struct udc_ep_config *cfg;
		usb_device_bus_reset(handle, USB_HPM_EP0_SIZE);
		cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
		if (cfg->stat.enabled) {
			udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
		}
		cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
		if (cfg->stat.enabled) {
			udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
		}
		if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
					USB_EP_TYPE_CONTROL,
					USB_HPM_EP0_SIZE, 0)) {
			LOG_ERR("Failed to enable control endpoint");
			return ;
		}

		if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
					USB_EP_TYPE_CONTROL,
					USB_HPM_EP0_SIZE, 0)) {
			LOG_ERR("Failed to enable control endpoint");
			return ;
		}
		udc_submit_event(dev, UDC_EVT_RESET, 0);
	}

	if (int_status & USB_USBINTR_SLE_MASK) {
		if (usb_device_get_suspend_status(handle)) {
			/* Note: Host may delay more than 3 ms before and/or after bus reset before doing enumeration. */
			if (usb_device_get_address(handle)) {
				udc_set_suspended(dev, true);
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}
		} else {
		}
	}

	if (int_status & USB_USBINTR_PCE_MASK) {
		if (!usb_device_get_port_ccs(handle)) {
			udc_submit_event(dev, UDC_EVT_VBUS_REMOVED, 0);
		} else {
			udc_set_suspended(dev, false);
			udc_submit_event(dev, UDC_EVT_RESUME, 0);
			udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
		}
	}

	if (int_status & USB_USBINTR_UE_MASK) {
		uint32_t const edpt_complete = usb_device_get_edpt_complete_status(handle);
		uint32_t const edpt_setup_status = usb_device_get_setup_status(handle);

		if (edpt_complete) {
			usb_device_clear_edpt_complete_status(handle, edpt_complete);
			for (uint8_t ep_idx = 0; ep_idx < USB_SOS_DCD_MAX_QHD_COUNT; ep_idx++) {
				if (edpt_complete & (1 << ep_idx2bit(ep_idx))) {
					transfer_len = 0;
					ep_cb_req = true;

					/* Failed QTD also get ENDPTCOMPLETE set */
					dcd_qhd_t *p_qhd = usb_device_qhd_get(handle, ep_idx);
					dcd_qtd_t *p_qtd = p_qhd->attached_qtd;
					while (1) {
						if (p_qtd->halted || p_qtd->xact_err || p_qtd->buffer_err) {
							LOG_DBG("usbd transfer error!\r\n");
							ep_cb_req = false;
							break;
						} else if (p_qtd->active) {
							ep_cb_req = false;
							break;
						} else {
							transfer_len += p_qtd->expected_bytes - p_qtd->total_bytes;
						}
						
						if (p_qtd->next == USB_SOC_DCD_QTD_NEXT_INVALID) {
							break;
						} else {
							p_qtd = (dcd_qtd_t *)p_qtd->next;
						}
					}
					
					if (ep_cb_req) {
						uint8_t const ep_addr = (ep_idx / 2) | ((ep_idx & 0x01) ? 0x80 : 0);
						if (ep_addr & 0x80) {
							udc_hpm_handler_in(dev, ep_addr, (uint8_t *)p_qhd->attached_buffer, transfer_len);
						} else {
							udc_hpm_handler_out(dev, ep_addr, (uint8_t *)p_qhd->attached_buffer, transfer_len);
						}
					}
				}
			}
		}

		if (edpt_setup_status) {
			/*------------- Set up Received -------------*/
			usb_device_clear_setup_status(handle, edpt_setup_status);
			dcd_qhd_t *qhd0 = usb_device_qhd_get(handle, 0);
			udc_hpm_handler_setup(dev, (struct usb_setup_packet *)&qhd0->setup_request);
		}
	}
}

/* Return actual USB device speed */
static enum udc_bus_speed udc_hpm_device_speed(const struct device *dev)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	uint8_t hpm_speed;

	hpm_speed = usb_get_port_speed(handle->regs);

	switch (hpm_speed) {
	case 0x02:
		return UDC_BUS_SPEED_HS;
	case 0x01:
		__ASSERT(false, "Low speed mode not supported");
		__fallthrough;
	case 0x00:
		__fallthrough;
	default:
		return UDC_BUS_SPEED_FS;
	}
}

static int udc_hpm_ep_enqueue(const struct device *dev,
			struct udc_ep_config *const cfg,
			struct net_buf *const buf)
{
	udc_buf_put(cfg, buf);
	if (cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted", cfg->addr);
		return 0;
	}

	return udc_hpm_ep_try_feed(dev, cfg);
}

static int udc_hpm_ep_dequeue(const struct device *dev,
			struct udc_ep_config *const cfg)
{
	struct net_buf *buf;
	unsigned int key;

	cfg->stat.halted = false;
	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	key = irq_lock();
	udc_ep_set_busy(dev, cfg->addr, false);
	irq_unlock(key);

	return 0;
}

static int udc_hpm_ep_set_halt(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	usb_device_edpt_stall(handle, cfg->addr);

	return 0;
}

static int udc_hpm_ep_clear_halt(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	usb_device_edpt_clear_stall(handle, cfg->addr);

	/* transfer is enqueued after stalled */
	return udc_hpm_ep_try_feed(dev, cfg);
}

static int udc_hpm_ep_enable(const struct device *dev,
			struct udc_ep_config *const cfg)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	usb_endpoint_config_t tmp_ep_cfg;

	LOG_DBG("Enable ep 0x%02x", cfg->addr);

	tmp_ep_cfg.xfer = cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;
	tmp_ep_cfg.ep_addr = cfg->addr;
	tmp_ep_cfg.max_packet_size = cfg->mps;

	usb_device_edpt_open(handle, &tmp_ep_cfg);

	return 0;
}

static int udc_hpm_ep_disable(const struct device *dev,
			struct udc_ep_config *const cfg)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	LOG_DBG("Disable ep 0x%02x", cfg->addr);

	usb_device_edpt_close(handle, cfg->addr);

	return 0;
}

static int udc_hpm_host_wakeup(const struct device *dev)
{
	return -ENOTSUP;
}

static int udc_hpm_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	usb_dcd_set_address(handle->regs, addr);

	return 0;
}

static int udc_hpm_enable(const struct device *dev)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	handle->regs->USBCMD |= USB_USBCMD_RS_MASK;

	return 0;
}

static int udc_hpm_disable(const struct device *dev)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	handle->regs->USBCMD &= ~USB_USBCMD_RS_MASK;

	return 0;
}

static int udc_hpm_init(const struct device *dev)
{
	const struct udc_hpm_config *config = dev->config;
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	uint32_t int_mask;

	int_mask = (USB_USBINTR_UE_MASK | USB_USBINTR_UEE_MASK | USB_USBINTR_SLE_MASK |
				USB_USBINTR_PCE_MASK | USB_USBINTR_URE_MASK);

	usb_device_init(handle, int_mask);

	/* enable USB interrupt */
	config->irq_enable_func(dev);

	LOG_DBG("Initialized USB controller %x", (uint32_t)config->base);

	return 0;
}

static int udc_hpm_shutdown(const struct device *dev)
{
	const struct udc_hpm_config *config = dev->config;
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	/* Disable interrupt */
	config->irq_disable_func(dev);

	/* De-init HPM USB device driver. */
	usb_device_deinit(handle);

	return 0;
}

static int udc_hpm_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_hpm_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static int udc_hpm_driver_preinit(const struct device *dev)
{
	const struct udc_hpm_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct udc_hpm_data *priv = data->priv;
	usb_device_handle_t *handle = &priv->handle;
	int err;

	k_mutex_init(&data->mutex);

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = 1024;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = 1024;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	/* Requires udc_hpm_host_wakeup() implementation */
	data->caps.rwup = false;
	data->caps.mps0 = USB_HPM_MPS0;
	data->caps.hs = true;

	handle->regs = config->base;
	handle->dcd_data = config->dcd_data;

	clock_add_to_group(config->clock_name, 0);

	pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	return 0;
}

static const struct udc_api udc_hpm_api = {
	.device_speed = udc_hpm_device_speed,
	.ep_enqueue = udc_hpm_ep_enqueue,
	.ep_dequeue = udc_hpm_ep_dequeue,
	.ep_set_halt = udc_hpm_ep_set_halt,
	.ep_clear_halt = udc_hpm_ep_clear_halt,
	.ep_try_config = NULL,
	.ep_enable = udc_hpm_ep_enable,
	.ep_disable = udc_hpm_ep_disable,
	.host_wakeup = udc_hpm_host_wakeup,
	.set_address = udc_hpm_set_address,
	.enable = udc_hpm_enable,
	.disable = udc_hpm_disable,
	.init = udc_hpm_init,
	.shutdown = udc_hpm_shutdown,
	.lock = udc_hpm_lock,
	.unlock = udc_hpm_unlock,
};

#define USB_HPM_EHCI_DEVICE_DEFINE(n)							\
	static void udc_irq_enable_func##n(const struct device *dev)			\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(n),						\
					DT_INST_IRQ(n, priority),					\
					udc_hpm_isr,						\
					DEVICE_DT_INST_GET(n), 0);					\
											\
		irq_enable(DT_INST_IRQN(n));						\
	}										\
											\
	static void udc_irq_disable_func##n(const struct device *dev)			\
	{										\
		irq_disable(DT_INST_IRQN(n));						\
	}										\
											\
	static struct udc_ep_config							\
		ep_cfg_out##n[DT_INST_PROP(n, num_bidir_endpoints)];			\
	static struct udc_ep_config							\
		ep_cfg_in##n[DT_INST_PROP(n, num_bidir_endpoints)];			\
											\
	static __attribute__((__section__(".nocache"))) ATTR_ALIGN(USB_SOC_DCD_DATA_RAM_ADDRESS_ALIGNMENT) dcd_data_t _dcd_data##n;	\
											\
	PINCTRL_DT_INST_DEFINE(n);							\
											\
	static struct udc_hpm_config priv_config_##n = {				\
		.base = (USB_Type *)DT_INST_REG_ADDR(n),					\
		.clock_name = (clock_name_t)DT_INST_PROP(n, clk_name),		\
		.irq_enable_func = udc_irq_enable_func##n,					\
		.irq_disable_func = udc_irq_disable_func##n,				\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),			\
		.ep_cfg_in = ep_cfg_in##n,						\
		.ep_cfg_out = ep_cfg_out##n,						\
		.dcd_data = &_dcd_data##n,						\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				\
	};										\
											\
	static struct udc_hpm_data priv_data_##n = {					\
	};										\
											\
	static struct udc_data udc_data_##n = {						\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),			\
		.priv = &priv_data_##n,							\
	};										\
											\
	DEVICE_DT_INST_DEFINE(n, udc_hpm_driver_preinit, NULL,				\
						&udc_data_##n, &priv_config_##n,				\
						POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
						&udc_hpm_api);

DT_INST_FOREACH_STATUS_OKAY(USB_HPM_EHCI_DEVICE_DEFINE)
