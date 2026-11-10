/*
 * Copyright (c) 2024, sakumisu
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_msc.h"
#include "usbh_serial.h"

#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>

#include <ff.h>

#define SERIAL_TEST_LEN 1024U
#define HID_REPORT_BUFFER_SIZE 128U

#if SERIAL_TEST_LEN >= CONFIG_USBHOST_SERIAL_RX_SIZE
#error "SERIAL_TEST_LEN must be smaller than CONFIG_USBHOST_SERIAL_RX_SIZE"
#endif

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t
    hid_buffer[CONFIG_USBHOST_MAX_HID_CLASS][HID_REPORT_BUFFER_SIZE];

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t
    serial_tx_buffer[CONFIG_USBHOST_MAX_SERIAL_CLASS][SERIAL_TEST_LEN];
static uint8_t serial_rx_data[CONFIG_USBHOST_MAX_SERIAL_CLASS][SERIAL_TEST_LEN];

struct usb_host_serial_context {
    struct usbh_serial *serial;
    bool stop_requested;
    bool running;
    usb_osal_sem_t done_sem;
};

static struct usb_host_serial_context
    serial_context[CONFIG_USBHOST_MAX_SERIAL_CLASS];

/* The MSC class disconnect callback runs in the USB host thread.  Keep the
 * filesystem worker alive until it has finished all FatFs operations so the
 * class object and disk interface cannot be released underneath it. */
struct usb_host_msc_context {
    struct usbh_msc *msc_class;
    FATFS fat_fs;
    struct fs_mount_t mount;
    char mount_point[CONFIG_USBHOST_DEV_NAMELEN];
    bool stop_requested;
    bool running;
    bool mounted;
    usb_osal_sem_t done_sem;
};

static struct usb_host_msc_context
    msc_context[CONFIG_USBHOST_MAX_MSC_CLASS];

static void usbh_serial_thread(CONFIG_USB_OSAL_THREAD_SET_ARGV)
{
    int ret;
    bool test_success = false;
    struct usb_host_serial_context *ctx =
        (struct usb_host_serial_context *)CONFIG_USB_OSAL_THREAD_GET_ARGV;
    struct usbh_serial *serial = ctx->serial;
    struct usbh_serial_termios termios;
    uint32_t serial_rx_bytes = 0;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;

    if (ctx->stop_requested || serial == NULL || serial->hport == NULL ||
        !serial->hport->connected) {
        goto done;
    }

    tx_buffer = serial_tx_buffer[serial->minor];
    rx_buffer = serial_rx_data[serial->minor];

    serial = usbh_serial_open(serial->hport->config.intf[serial->intf].devname,
                              USBH_SERIAL_O_RDWR | USBH_SERIAL_O_NONBLOCK);
    if (serial == NULL) {
        USB_LOG_RAW("no serial device found\r\n");
        goto done;
    }

    memset(&termios, 0, sizeof(termios));
    termios.baudrate = 115200;
    termios.stopbits = USBH_SERIAL_STOPBITS_1;
    termios.parity = USBH_SERIAL_PARITY_NONE;
    termios.databits = USBH_SERIAL_DATABITS_8;
    ret = usbh_serial_control(serial, USBH_SERIAL_CMD_SET_ATTR, &termios);
    if (ret < 0) {
        USB_LOG_RAW("set serial attr error, ret:%d\r\n", ret);
        goto close;
    }

    memset(tx_buffer, 0xA5, SERIAL_TEST_LEN);
    USB_LOG_RAW("start serial%u loopback test, len: %d\r\n",
                (unsigned int)serial->minor, SERIAL_TEST_LEN);

    ret = usbh_serial_write(serial, tx_buffer, SERIAL_TEST_LEN);
    if (ret != SERIAL_TEST_LEN) {
        USB_LOG_RAW("serial write error, ret:%d\r\n", ret);
        goto close;
    }

    for (uint32_t wait_timeout = 0; wait_timeout <= 500; wait_timeout++) {
        if (ctx->stop_requested) {
            goto close;
        }

        ret = usbh_serial_read(serial, &rx_buffer[serial_rx_bytes],
                               SERIAL_TEST_LEN - serial_rx_bytes);
        if (ret < 0) {
            USB_LOG_RAW("serial read error, ret:%d\r\n", ret);
            goto close;
        }

        serial_rx_bytes += ret;
        if (serial_rx_bytes == SERIAL_TEST_LEN) {
            test_success = memcmp(rx_buffer, tx_buffer, SERIAL_TEST_LEN) == 0;
            break;
        }

        usb_osal_msleep(10);
    }

close:
    USB_LOG_RAW("serial loopback test %s\r\n", test_success ? "success" : "failed");
    usbh_serial_close(serial);
done:
    if (ctx->done_sem != NULL) {
        usb_osal_sem_give(ctx->done_sem);
    }
    ctx->running = false;
    usb_osal_thread_delete(NULL);
}

static void usbh_hid_callback(void *arg, int nbytes)
{
    struct usbh_hid *hid_class = (struct usbh_hid *)arg;
    uint8_t *buffer;

    if (hid_class == NULL || hid_class->minor >= CONFIG_USBHOST_MAX_HID_CLASS) {
        return;
    }

    buffer = hid_buffer[hid_class->minor];

    if (nbytes > 0) {
        for (int i = 0; i < nbytes; i++) {
            USB_LOG_RAW("0x%02x ", buffer[i]);
        }
        USB_LOG_RAW("nbytes:%d\r\n", (unsigned int)nbytes);
    }

    if (nbytes >= 0 || nbytes == -USB_ERR_NAK) {
        usbh_int_urb_fill(&hid_class->intin_urb, hid_class->hport, hid_class->intin,
                          buffer, hid_class->intin->wMaxPacketSize, 0,
                          usbh_hid_callback, hid_class);
        usbh_submit_urb(&hid_class->intin_urb);
    }
}

static void usbh_hid_thread(CONFIG_USB_OSAL_THREAD_SET_ARGV)
{
    struct usbh_hid *hid_class = (struct usbh_hid *)CONFIG_USB_OSAL_THREAD_GET_ARGV;
    uint8_t *buffer;

    if (hid_class == NULL || hid_class->minor >= CONFIG_USBHOST_MAX_HID_CLASS ||
        hid_class->intin == NULL) {
        usb_osal_thread_delete(NULL);
        return;
    }

    if (hid_class->intin->wMaxPacketSize > HID_REPORT_BUFFER_SIZE) {
        USB_LOG_RAW("hid%u report size %u exceeds sample buffer size %u\r\n",
                    hid_class->minor, hid_class->intin->wMaxPacketSize,
                    HID_REPORT_BUFFER_SIZE);
        usb_osal_thread_delete(NULL);
        return;
    }

    buffer = hid_buffer[hid_class->minor];

    usbh_int_urb_fill(&hid_class->intin_urb, hid_class->hport, hid_class->intin,
                      buffer, hid_class->intin->wMaxPacketSize, 0,
                      usbh_hid_callback, hid_class);
    if (usbh_submit_urb(&hid_class->intin_urb) < 0) {
        USB_LOG_RAW("hid interrupt transfer start failed\r\n");
    }

    usb_osal_thread_delete(NULL);
}

static int usb_host_list_directory(const char *path)
{
    int ret;
    int count = 0;
    struct fs_dir_t dir;
    struct fs_dirent entry;

    fs_dir_t_init(&dir);
    ret = fs_opendir(&dir, path);
    if (ret < 0) {
        USB_LOG_RAW("open directory %s failed, ret:%d\r\n", path, ret);
        return ret;
    }

    USB_LOG_RAW("\r\nListing dir %s ...\r\n", path);
    while (true) {
        ret = fs_readdir(&dir, &entry);
        if (ret < 0 || entry.name[0] == '\0') {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR) {
            USB_LOG_RAW("[DIR ] %s\r\n", entry.name);
        } else {
            USB_LOG_RAW("[FILE] %s (size = %zu)\r\n", entry.name, entry.size);
        }
        count++;
    }

    fs_closedir(&dir);
    return ret < 0 ? ret : count;
}

static void usb_host_msc_thread(CONFIG_USB_OSAL_THREAD_SET_ARGV)
{
    int ret;
    struct usb_host_msc_context *ctx =
        (struct usb_host_msc_context *)CONFIG_USB_OSAL_THREAD_GET_ARGV;

    if (ctx->stop_requested) {
        goto done;
    }

    ret = fs_mount(&ctx->mount);
    if (ret < 0) {
        USB_LOG_RAW("mount %s failed, ret:%d\r\n", ctx->mount_point, ret);
        goto done;
    }
    ctx->mounted = true;

    if (!ctx->stop_requested) {
        ret = usb_host_list_directory(ctx->mount_point);
        if (ret < 0) {
            USB_LOG_RAW("list directory failed, ret:%d\r\n", ret);
        }
    }

    ret = fs_unmount(&ctx->mount);
    if (ret < 0) {
        USB_LOG_RAW("unmount %s failed, ret:%d\r\n", ctx->mount_point, ret);
    }
    ctx->mounted = false;

done:
    if (ctx->mounted) {
        fs_unmount(&ctx->mount);
        ctx->mounted = false;
    }
    if (ctx->done_sem != NULL) {
        usb_osal_sem_give(ctx->done_sem);
    }
    ctx->running = false;
    usb_osal_thread_delete(NULL);
}

void usbh_serial_run(struct usbh_serial *serial)
{
    struct usb_host_serial_context *ctx;

    if (serial == NULL || serial->minor < 0 ||
        serial->minor >= CONFIG_USBHOST_MAX_SERIAL_CLASS) {
        return;
    }

    ctx = &serial_context[serial->minor];
    if (ctx->running) {
        USB_LOG_RAW("serial%u thread is still running\r\n",
                    (unsigned int)serial->minor);
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->serial = serial;
    ctx->done_sem = usb_osal_sem_create(0);
    if (ctx->done_sem == NULL) {
        USB_LOG_RAW("serial%u completion semaphore creation failed\r\n",
                    (unsigned int)serial->minor);
        return;
    }

    ctx->running = true;
    if (usb_osal_thread_create("usbh_serial", 2048, CONFIG_USBHOST_PSC_PRIO + 1,
                               usbh_serial_thread, ctx) == NULL) {
        ctx->running = false;
        usb_osal_sem_delete(ctx->done_sem);
        ctx->done_sem = NULL;
        USB_LOG_RAW("serial thread creation failed\r\n");
    }
}

void usbh_serial_stop(struct usbh_serial *serial)
{
    struct usb_host_serial_context *ctx;
    usb_osal_sem_t done;

    if (serial == NULL || serial->minor < 0 ||
        serial->minor >= CONFIG_USBHOST_MAX_SERIAL_CLASS) {
        return;
    }

    ctx = &serial_context[serial->minor];
    ctx->stop_requested = true;
    done = ctx->done_sem;
    if (done != NULL) {
        usb_osal_sem_take(done, USB_OSAL_WAITING_FOREVER);
        usb_osal_sem_delete(done);
        ctx->done_sem = NULL;
    }
    ctx->serial = NULL;
}

void usbh_hid_run(struct usbh_hid *hid_class)
{
    if (usb_osal_thread_create("usbh_hid", 2048, CONFIG_USBHOST_PSC_PRIO + 1,
                               usbh_hid_thread, hid_class) == NULL) {
        USB_LOG_RAW("hid thread creation failed\r\n");
    }
}

void usbh_hid_stop(struct usbh_hid *hid_class)
{
    ARG_UNUSED(hid_class);
}

void usbh_msc_app_run(struct usbh_msc *msc_class)
{
    uint8_t index;
    struct usb_host_msc_context *ctx;

    if (msc_class == NULL || msc_class->sdchar < 'a' ||
        msc_class->sdchar >= 'a' + CONFIG_USBHOST_MAX_MSC_CLASS) {
        return;
    }

    index = msc_class->sdchar - 'a';
    ctx = &msc_context[index];
    if (ctx->running) {
        USB_LOG_RAW("msc%u thread is still running\r\n", index);
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->msc_class = msc_class;
    snprintf(ctx->mount_point, sizeof(ctx->mount_point), "/USB%u:", index);
    ctx->mount.type = FS_FATFS;
    ctx->mount.fs_data = &ctx->fat_fs;
    ctx->mount.mnt_point = ctx->mount_point;
    ctx->done_sem = usb_osal_sem_create(0);
    if (ctx->done_sem == NULL) {
        USB_LOG_RAW("msc%u completion semaphore creation failed\r\n", index);
        return;
    }

    ctx->running = true;
    if (usb_osal_thread_create("usbh_msc", 4096, CONFIG_USBHOST_PSC_PRIO + 1,
                               usb_host_msc_thread, ctx) == NULL) {
        ctx->running = false;
        usb_osal_sem_delete(ctx->done_sem);
        ctx->done_sem = NULL;
        USB_LOG_RAW("msc%u thread creation failed\r\n", index);
    }
}

void usbh_msc_app_stop(struct usbh_msc *msc_class)
{
    uint8_t index;
    struct usb_host_msc_context *ctx;
    usb_osal_sem_t done;

    if (msc_class == NULL || msc_class->sdchar < 'a' ||
        msc_class->sdchar >= 'a' + CONFIG_USBHOST_MAX_MSC_CLASS) {
        return;
    }

    index = msc_class->sdchar - 'a';
    ctx = &msc_context[index];
    ctx->stop_requested = true;
    done = ctx->done_sem;
    if (done != NULL) {
        /* URBs have already been killed by CherryUSB before this callback,
         * so a worker blocked in FatFs/MSC I/O can make progress to exit. */
        usb_osal_sem_take(done, USB_OSAL_WAITING_FOREVER);
        usb_osal_sem_delete(done);
        ctx->done_sem = NULL;
    }
    ctx->msc_class = NULL;
}
