/*
 * Copyright (c) 2025-2026 hpmicro
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
#include "hpm_usb_drv.h"
#include "hpm_usb_device.h"
#include <drivers/clock_control/hpmicro_clock_control.h>

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
	bool use_internal_vbus;
	bool power_active_high;
	uint32_t power_stable_delay_ms;
};

enum udc_hpm_event_type {
	UDC_HPM_EVT_ERROR,
	UDC_HPM_EVT_RESET,
	UDC_HPM_EVT_SUSPEND,
	UDC_HPM_EVT_RESUME,
	UDC_HPM_EVT_VBUS_REMOVED,
	UDC_HPM_EVT_VBUS_READY,
	UDC_HPM_EVT_XFER,
	UDC_HPM_EVT_SETUP,
};

struct udc_hpm_event {
	sys_snode_t node;
	const struct device *dev;
	enum udc_hpm_event_type type;
	union {
		int status;
		struct {
			uint8_t ep;
			uint16_t len;
		} xfer;
		struct usb_setup_packet setup;
	};
};

struct udc_hpm_data {
	usb_device_handle_t handle;
	struct k_work work;
	struct k_fifo fifo;
};

K_MEM_SLAB_DEFINE(udc_hpm_event_slab, sizeof(struct udc_hpm_event),
		  CONFIG_UDC_HPMICRO_EVENT_QUEUE_SIZE, sizeof(void *));

#if defined(CONFIG_NOCACHE_MEMORY)
#define USB_HPM_DCD_DATA_SECTION __attribute__((__section__(".nocache")))
#else
#define USB_HPM_DCD_DATA_SECTION
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
	if (!udc_ep_is_busy(cfg)) {
		udc_ep_set_busy(cfg, true);
		irq_unlock(key);

		if (USB_EP_DIR_IS_OUT(cfg->addr)) {
			len = net_buf_tailroom(buf);
			data = net_buf_tail(buf);
			status = usb_device_edpt_xfer(handle, cfg->addr, data, len);
		} else {
			len = buf->len;
			data = buf->data;
			status = usb_device_edpt_xfer(handle, cfg->addr, data, len);
		}

		key = irq_lock();
		if (!status) {
			udc_ep_set_busy(cfg, false);
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

	feed_buf = udc_buf_peek(cfg);
	if (feed_buf) {
		int ret = udc_hpm_ep_feed(dev, cfg, feed_buf);

		return ((ret == -EBUSY || ret == -EACCES || ret == 0) ? 0 : -EIO);
	}

	return 0;
}

static void udc_hpm_event_submit(const struct device *dev, const struct udc_hpm_event *event)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	struct udc_hpm_event *queued_event;
	int err;

	err = k_mem_slab_alloc(&udc_hpm_event_slab, (void **)&queued_event, K_NO_WAIT);
	if (unlikely(err)) {
		LOG_ERR("USB event slab exhausted");
		udc_submit_event(dev, UDC_EVT_ERROR, err);
		return;
	}

	queued_event->dev = dev;
	queued_event->type = event->type;

	switch (event->type) {
	case UDC_HPM_EVT_ERROR:
		queued_event->status = event->status;
		break;
	case UDC_HPM_EVT_XFER:
		queued_event->xfer = event->xfer;
		break;
	case UDC_HPM_EVT_SETUP:
		queued_event->setup = event->setup;
		break;
	default:
		break;
	}

	k_fifo_put(&priv->fifo, queued_event);
	k_work_submit_to_queue(udc_get_work_q(), &priv->work);
}

static void udc_hpm_handler_reset(const struct device *dev)
{
	struct udc_ep_config *cfg;

	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	if (cfg->stat.enabled) {
		udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
	}

	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	if (cfg->stat.enabled) {
		udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
				   USB_HPM_EP0_SIZE, 0)) {
		LOG_ERR("Failed to enable control OUT endpoint");
		return;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				   USB_HPM_EP0_SIZE, 0)) {
		LOG_ERR("Failed to enable control IN endpoint");
		return;
	}

	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void udc_hpm_handler_setup(const struct device *dev, struct usb_setup_packet *setup)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	LOG_DBG("setup packet");

	if (setup->RequestType.type == USB_REQTYPE_TYPE_STANDARD &&
	    setup->RequestType.direction == USB_REQTYPE_DIR_TO_DEVICE &&
	    setup->bRequest == USB_SREQ_SET_ADDRESS &&
	    setup->wLength == 0) {
		usb_dcd_set_address(handle->regs, setup->wValue);
	}

	udc_setup_received(dev, setup);
}

static int udc_hpm_handler_ctrl_out(const struct device *dev, struct net_buf *buf,
				uint16_t hpm_len)
{
	uint32_t len;

	len = MIN(net_buf_tailroom(buf), hpm_len);
	net_buf_add(buf, len);

	return udc_submit_ep_event(dev, buf, 0);
}

static int udc_hpm_handler_ctrl_in(const struct device *dev, struct net_buf *buf,
				uint16_t hpm_len)
{
	uint32_t len;

	len = MIN(buf->len, hpm_len);
	buf->data += len;
	buf->len -= len;

	return udc_submit_ep_event(dev, buf, 0);
}

static int udc_hpm_handler_non_ctrl_in(const struct device *dev, uint8_t ep,
			struct net_buf *buf, uint16_t hpm_len)
{
	int err;
	uint32_t len;

	len = MIN(buf->len, hpm_len);
	buf->data += len;
	buf->len -= len;

	err = udc_submit_ep_event(dev, buf, 0);
	udc_hpm_ep_try_feed(dev, udc_get_ep_cfg(dev, ep));

	return err;
}

static int udc_hpm_handler_non_ctrl_out(const struct device *dev, uint8_t ep,
			struct net_buf *buf, uint16_t hpm_len)
{
	int err;
	uint32_t len;

	len = MIN(net_buf_tailroom(buf), hpm_len);
	net_buf_add(buf, len);

	err = udc_submit_ep_event(dev, buf, 0);
	udc_hpm_ep_try_feed(dev, udc_get_ep_cfg(dev, ep));

	return err;
}

static int udc_hpm_handler_out(const struct device *dev, uint8_t ep, uint16_t hpm_len)
{
	int err;
	struct net_buf *buf;
	struct udc_ep_config *const cfg = udc_get_ep_cfg(dev, ep);
	unsigned int key;

	buf = udc_buf_get(cfg);

	key = irq_lock();
	udc_ep_set_busy(cfg, false);
	irq_unlock(key);

	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	if (ep == USB_CONTROL_EP_OUT) {
		err = udc_hpm_handler_ctrl_out(dev, buf, hpm_len);
	} else {
		err = udc_hpm_handler_non_ctrl_out(dev, ep, buf, hpm_len);
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

static int udc_hpm_handler_in(const struct device *dev, uint8_t ep, uint16_t hpm_len)
{
	int err;
	struct net_buf *buf;
	struct udc_ep_config *const cfg = udc_get_ep_cfg(dev, ep);
	unsigned int key;

	buf = udc_buf_peek(cfg);
	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	if (udc_hpm_handler_zlt(dev, ep, buf, hpm_len)) {
		return 0;
	}

	buf = udc_buf_get(cfg);

	key = irq_lock();
	udc_ep_set_busy(cfg, false);
	irq_unlock(key);

	if (buf == NULL) {
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}
	if (ep == USB_CONTROL_EP_IN) {
		err = udc_hpm_handler_ctrl_in(dev, buf, hpm_len);
	} else {
		err = udc_hpm_handler_non_ctrl_in(dev, ep, buf, hpm_len);
	}

	return err;
}

static void udc_hpm_work_handler(struct k_work *item)
{
	struct udc_hpm_data *priv = CONTAINER_OF(item, struct udc_hpm_data, work);
	struct udc_hpm_event *event;
	int err;

	while ((event = k_fifo_get(&priv->fifo, K_NO_WAIT)) != NULL) {
		const struct device *dev = event->dev;

		switch (event->type) {
		case UDC_HPM_EVT_ERROR:
			udc_submit_event(dev, UDC_EVT_ERROR, event->status);
			break;
		case UDC_HPM_EVT_RESET:
			udc_hpm_handler_reset(dev);
			break;
		case UDC_HPM_EVT_SUSPEND:
			udc_set_suspended(dev, true);
			udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			break;
		case UDC_HPM_EVT_RESUME:
			if (udc_is_suspended(dev)) {
				udc_set_suspended(dev, false);
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			}
			break;
		case UDC_HPM_EVT_VBUS_REMOVED:
			udc_submit_event(dev, UDC_EVT_VBUS_REMOVED, 0);
			break;
		case UDC_HPM_EVT_VBUS_READY:
			udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
			break;
		case UDC_HPM_EVT_XFER:
			if (USB_EP_DIR_IS_IN(event->xfer.ep)) {
				err = udc_hpm_handler_in(dev, event->xfer.ep, event->xfer.len);
			} else {
				err = udc_hpm_handler_out(dev, event->xfer.ep, event->xfer.len);
			}
			if (unlikely(err)) {
				udc_submit_event(dev, UDC_EVT_ERROR, err);
			}
			break;
		case UDC_HPM_EVT_SETUP:
			udc_hpm_handler_setup(dev, &event->setup);
			break;
		}

		k_mem_slab_free(&udc_hpm_event_slab, event);
	}
}

static void udc_hpm_isr(const struct device *dev)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	struct udc_hpm_event event;
	uint32_t int_status;
	uint32_t transfer_len;
	bool ep_cb_req;

	/* Acknowledge handled interrupt and keep UDC state changes in the event thread. */
	int_status = usb_device_status_flags(handle);
	int_status &= usb_device_interrupts(handle);
	usb_device_clear_status_flags(handle, int_status);

	if (int_status & USB_USBINTR_UEE_MASK) {
		LOG_DBG("usbd intr error!\r\n");
		event.type = UDC_HPM_EVT_ERROR;
		event.status = -EIO;
		udc_hpm_event_submit(dev, &event);
	}

	if (IS_ENABLED(CONFIG_UDC_ENABLE_SOF) &&
	    (int_status & USB_USBINTR_SRE_MASK)) {
		udc_submit_sof_event(dev);
	}

	if (int_status & USB_USBINTR_URE_MASK) {
		usb_device_bus_reset(handle, USB_HPM_EP0_SIZE);
		event.type = UDC_HPM_EVT_RESET;
		udc_hpm_event_submit(dev, &event);
	}

	if (int_status & USB_USBINTR_SLE_MASK) {
		if (usb_device_get_suspend_status(handle)) {
			/* Note: Host may delay more than 3 ms before and/or after bus reset before doing enumeration. */
			if (usb_device_get_address(handle)) {
				event.type = UDC_HPM_EVT_SUSPEND;
				udc_hpm_event_submit(dev, &event);
			}
		} else {
		}
	}

	if (int_status & USB_USBINTR_PCE_MASK) {
		if (!usb_device_get_port_ccs(handle)) {
			event.type = UDC_HPM_EVT_VBUS_REMOVED;
			udc_hpm_event_submit(dev, &event);
		} else {
			event.type = UDC_HPM_EVT_RESUME;
			udc_hpm_event_submit(dev, &event);
			event.type = UDC_HPM_EVT_VBUS_READY;
			udc_hpm_event_submit(dev, &event);
		}
	}

	if ((int_status & USB_USBINTR_UE_MASK) && !(int_status & USB_USBINTR_URE_MASK)) {
		uint32_t const edpt_complete = usb_device_get_edpt_complete_status(handle);
		uint32_t const edpt_setup_status = usb_device_get_setup_status(handle);

		if (edpt_complete) {
			usb_device_clear_edpt_complete_status(handle, edpt_complete);
			for (uint8_t ep_idx = 0; ep_idx < USB_SOS_DCD_MAX_QHD_COUNT; ep_idx++) {
				if (edpt_complete & (1 << ep_idx2bit(ep_idx))) {
					dcd_qhd_t *p_qhd;
					dcd_qtd_t *p_qtd;
					uint8_t ep_addr;

					transfer_len = 0;
					ep_cb_req = true;

					/* Failed QTD also get ENDPTCOMPLETE set */
					p_qhd = usb_device_qhd_get(handle, ep_idx);
					p_qtd = p_qhd->attached_qtd;
					if (p_qtd == NULL) {
						LOG_ERR("Endpoint %u completed without a QTD", ep_idx);
						event.type = UDC_HPM_EVT_ERROR;
						event.status = -EIO;
						udc_hpm_event_submit(dev, &event);
						continue;
					}

					while (1) {
						if (p_qtd->halted || p_qtd->xact_err || p_qtd->buffer_err) {
							LOG_DBG("usbd transfer error!\r\n");
							event.type = UDC_HPM_EVT_ERROR;
							event.status = -EIO;
							udc_hpm_event_submit(dev, &event);
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
						ep_addr = (ep_idx / 2) | ((ep_idx & 0x01) ? 0x80 : 0);
						event.type = UDC_HPM_EVT_XFER;
						event.xfer.ep = ep_addr;
						event.xfer.len = transfer_len;
						udc_hpm_event_submit(dev, &event);
					}
				}
			}
		}

		if (edpt_setup_status) {
			/*------------- Setup Received -------------*/
			dcd_qhd_t *qhd0 = usb_device_qhd_get(handle, 0);

			usb_device_clear_setup_status(handle, edpt_setup_status);
			do {
				usb_dcd_set_sutw(handle->regs, true);
				memcpy(&event.setup, (const void *)&qhd0->setup_request,
				       sizeof(event.setup));
			} while (!usb_dcd_get_sutw(handle->regs));
			usb_dcd_set_sutw(handle->regs, false);

			event.type = UDC_HPM_EVT_SETUP;
			udc_hpm_event_submit(dev, &event);
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
	if (cfg->addr == USB_CONTROL_EP_OUT) {
		struct udc_buf_info *bi = udc_get_buf_info(buf);

		if (bi->setup) {
			udc_buf_put(cfg, buf);

			/* SETUP can be received without arming control OUT. */
			return 0;
		}
	}

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
	unsigned int key;

	cfg->stat.halted = false;
	udc_ep_cancel_queued(dev, cfg);

	key = irq_lock();
	udc_ep_set_busy(cfg, false);
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

	return usb_device_edpt_open(handle, &tmp_ep_cfg) ? 0 : -EIO;
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
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;

	usb_dcd_remote_wakeup(handle->regs);

	return 0;
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

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				USB_EP_TYPE_CONTROL, USB_HPM_EP0_SIZE, 0)) {
		LOG_ERR("Failed to enable control endpoint");
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				USB_EP_TYPE_CONTROL, USB_HPM_EP0_SIZE, 0)) {
		LOG_ERR("Failed to enable control endpoint");
	}

	handle->regs->USBCMD |= USB_USBCMD_RS_MASK;

	return 0;
}

static int udc_hpm_disable(const struct device *dev)
{
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	struct udc_ep_config *cfg;

	handle->regs->USBCMD &= ~USB_USBCMD_RS_MASK;

	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	if (cfg->stat.enabled) {
		udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
	}

	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	if (cfg->stat.enabled) {
		udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
	}

	return 0;
}

static int udc_hpm_init(const struct device *dev)
{
	const struct udc_hpm_config *config = dev->config;
	struct udc_hpm_data *priv = udc_get_private(dev);
	usb_device_handle_t *handle = &priv->handle;
	uint32_t int_mask;

	if (config->power_active_high) {
		usb_hcd_set_power_ctrl_polarity(config->base, true);
	}

	if (config->power_stable_delay_ms != 0U) {
		k_busy_wait(config->power_stable_delay_ms * USEC_PER_MSEC);
	}

	if (config->use_internal_vbus) {
		usb_phy_using_internal_vbus(config->base);
	}

	int_mask = (USB_USBINTR_UE_MASK | USB_USBINTR_UEE_MASK | USB_USBINTR_SLE_MASK |
				USB_USBINTR_PCE_MASK | USB_USBINTR_URE_MASK);
	if (IS_ENABLED(CONFIG_UDC_ENABLE_SOF)) {
		int_mask |= USB_USBINTR_SRE_MASK;
	}

	if (!usb_device_init(handle, int_mask)) {
		return -EIO;
	}

	if (!IS_ENABLED(CONFIG_UDC_DRIVER_HIGH_SPEED_SUPPORT_ENABLED)) {
		config->base->PORTSC1 |= USB_PORTSC1_PFSC_MASK;
	}

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

static void udc_hpm_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_hpm_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static int udc_hpm_driver_preinit(const struct device *dev)
{
	const struct udc_hpm_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct udc_hpm_data *priv = data->priv;
	usb_device_handle_t *handle = &priv->handle;
	int err;

	k_mutex_init(&data->mutex);
	k_fifo_init(&priv->fifo);
	k_work_init(&priv->work, udc_hpm_work_handler);

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.high_bandwidth = 1;
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
			config->ep_cfg_in[i].caps.high_bandwidth = 1;
			config->ep_cfg_in[i].caps.mps = 1024;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	data->caps.rwup = true;
	data->caps.mps0 = USB_HPM_MPS0;
	data->caps.hs = true;

	handle->regs = config->base;
	handle->dcd_data = config->dcd_data;

	clock_enable_name(config->clock_name);

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
	static USB_HPM_DCD_DATA_SECTION ATTR_ALIGN(USB_SOC_DCD_DATA_RAM_ADDRESS_ALIGNMENT) dcd_data_t _dcd_data##n;	\
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
		.use_internal_vbus = DT_INST_PROP(n, use_internal_vbus),		\
		.power_active_high = DT_INST_PROP(n, power_active_high),		\
		.power_stable_delay_ms = DT_INST_PROP(n, power_stable_delay_ms),	\
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
