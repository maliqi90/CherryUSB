/*
 * Copyright (c) 2025, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include <nuttx/mm/circbuf.h>

#ifndef CONFIG_USBDEV_CDCACM_RXBUFSIZE
#define CONFIG_USBDEV_CDCACM_RXBUFSIZE 512
#endif

#ifndef CONFIG_USBDEV_CDCACM_TXBUFSIZE
#define CONFIG_USBDEV_CDCACM_TXBUFSIZE 512
#endif

struct usbdev_serial_s {
    char name[16];
    struct circbuf_s circ;
    uint8_t inep;
    uint8_t outep;
    sem_t txdone_sem;
    sem_t rxdone_sem;
    struct usbd_interface ctrl_intf;
    struct usbd_interface data_intf;
    __attribute__((aligned(32))) uint8_t cache_rxbuffer[CONFIG_USBDEV_CDCACM_RXBUFSIZE];
    __attribute__((aligned(32))) uint8_t cache_txbuffer[CONFIG_USBDEV_CDCACM_TXBUFSIZE];
};

struct usbdev_serial_s *g_usb_cdcacm_serial[8];

static int nuttx_errorcode(int error)
{
    int err = 0;

    switch (error) {
        case -USB_ERR_NOMEM:
            err = -EIO;
            break;
        case -USB_ERR_INVAL:
            err = -EINVAL;
            break;
        case -USB_ERR_NODEV:
            err = -ENODEV;
            break;
        case -USB_ERR_NOTCONN:
            err = -ENOTCONN;
            break;
        case -USB_ERR_NOTSUPP:
            err = -EIO;
            break;
        case -USB_ERR_BUSY:
            err = -EBUSY;
            break;
        case -USB_ERR_RANGE:
            err = -ERANGE;
            break;
        case -USB_ERR_STALL:
            err = -EPERM;
            break;
        case -USB_ERR_NAK:
            err = -EAGAIN;
            break;
        case -USB_ERR_DT:
            err = -EIO;
            break;
        case -USB_ERR_IO:
            err = -EIO;
            break;
        case -USB_ERR_SHUTDOWN:
            err = -ESHUTDOWN;
            break;
        case -USB_ERR_TIMEOUT:
            err = -ETIMEDOUT;
            break;

        default:
            break;
    }
    return err;
}

void usbd_cdc_acm_bulk_out1(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    nxsem_post(&g_usb_cdcacm_serial[0]->rxdone_sem);
}

void usbd_cdc_acm_bulk_in1(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(busid, ep, NULL, 0);
    } else {
        nxsem_post(&g_usb_cdcacm_serial[0]->txdone_sem);
    }
}

void usbd_cdc_acm_bulk_out2(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    nxsem_post(&g_usb_cdcacm_serial[1]->rxdone_sem);
}

void usbd_cdc_acm_bulk_in2(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(busid, ep, NULL, 0);
    } else {
        nxsem_post(&g_usb_cdcacm_serial[1]->txdone_sem);
    }
}

void usbd_cdc_acm_bulk_out3(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    nxsem_post(&g_usb_cdcacm_serial[2]->rxdone_sem);
}

void usbd_cdc_acm_bulk_in3(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(busid, ep, NULL, 0);
    } else {
        nxsem_post(&g_usb_cdcacm_serial[2]->txdone_sem);
    }
}

/* Character driver methods */

static int usbdev_open(FAR struct file *filep);
static int usbdev_close(FAR struct file *filep);
static ssize_t usbdev_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t usbdev_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_usbdevops = {
    usbdev_open,  /* open */
    usbdev_close, /* close */
    usbdev_read,  /* read */
    usbdev_write, /* write */
    NULL,         /* seek */
    NULL,         /* ioctl */
    NULL,         /* mmap */
    NULL,         /* truncate */
    NULL          /* poll */
};

static int usbdev_open(FAR struct file *filep)
{
    FAR struct inode *inode = filep->f_inode;

    DEBUGASSERT(inode->i_private);

    if (usb_device_is_configured(0)) {
        return OK;
    } else {
        return -ENODEV;
    }
}

static int usbdev_close(FAR struct file *filep)
{
    FAR struct inode *inode = filep->f_inode;

    DEBUGASSERT(inode->i_private);

    if (!usb_device_is_configured(0)) {
        return -ENODEV;
    }

    return 0;
}

static ssize_t usbdev_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
    FAR struct inode *inode = filep->f_inode;
    struct usbdev_serial_s *serial;
    __attribute__((aligned(32))) uint8_t cache_tempbuffer[512];
    int ret;

    DEBUGASSERT(inode->i_private);
    serial = (struct usbdev_serial_s *)inode->i_private;

    if (!usb_device_is_configured(0)) {
        return -ENODEV;
    }

    while (circbuf_used(&serial->circ) < buflen) {
        nxsem_trywait(&serial->rxdone_sem);
        usbd_ep_start_read(0, serial->outep, cache_tempbuffer, usbd_get_ep_mps(0, serial->outep));
        ret = nxsem_wait(&serial->rxdone_sem);
        if (ret < 0) {
            return nuttx_errorcode(ret);
        }
#if defined(CONFIG_ARCH_DCACHE) && !defined(CONFIG_USB_DCACHE_ENABLE)
        up_invalidate_dcache((uintptr_t)cache_tempbuffer, (uintptr_t)(cache_tempbuffer + USB_ALIGN_UP(ret, 64)));
#endif
        circbuf_overwrite(&serial->circ, cache_tempbuffer, ret);
    }
    circbuf_read(&serial->circ, buffer, buflen);
    return buflen;
}

static ssize_t usbdev_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
    FAR struct inode *inode = filep->f_inode;
    struct usbdev_serial_s *serial;
    int ret;

    DEBUGASSERT(inode->i_private);
    serial = (struct usbdev_serial_s *)inode->i_private;

    if (!usb_device_is_configured(0)) {
        return -ENODEV;
    }

#ifdef CONFIG_ARCH_DCACHE
    uint32_t write_len = 0;

    while (write_len < buflen) {
        uint32_t len = buflen - write_len;
        if (len > CONFIG_USBDEV_CDCACM_TXBUFSIZE) {
            len = CONFIG_USBDEV_CDCACM_TXBUFSIZE;
        }
        memcpy(serial->cache_txbuffer, buffer + write_len, len);
#ifndef CONFIG_USB_DCACHE_ENABLE
        up_clean_dcache((uintptr_t)serial->cache_txbuffer, (uintptr_t)(serial->cache_txbuffer + USB_ALIGN_UP(len, 64)));
#endif
        nxsem_trywait(&serial->txdone_sem);
        usbd_ep_start_write(0, serial->inep, serial->cache_txbuffer, usbd_get_ep_mps(0, serial->inep));
        ret = nxsem_wait(&serial->txdone_sem);
        if (ret < 0) {
            return nuttx_errorcode(ret);
        } else {
            write_len += len;
        }
    }
    return buflen;
#else
    nxsem_trywait(&serial->txdone_sem);
    usbd_ep_start_write(0, serial->inep, buffer, buflen);
    ret = nxsem_wait(&serial->txdone_sem);
    if (ret < 0) {
        return nuttx_errorcode(ret);
    } else {
        return buflen;
    }
#endif
}

static struct usbd_endpoint cdc_out_ep[8] = {
    { .ep_addr = 0,
      .ep_cb = usbd_cdc_acm_bulk_out1 },
    { .ep_addr = 0,
      .ep_cb = usbd_cdc_acm_bulk_out2 },
    { .ep_addr = 0,
      .ep_cb = usbd_cdc_acm_bulk_out3 }
};

static struct usbd_endpoint cdc_in_ep[8] = {
    { .ep_addr = 0,
      .ep_cb = usbd_cdc_acm_bulk_in1 },
    { .ep_addr = 0,
      .ep_cb = usbd_cdc_acm_bulk_in2 },
    { .ep_addr = 0,
      .ep_cb = usbd_cdc_acm_bulk_in3 }
};

static void cdcacm_notify_handler1(uint8_t busid, uint8_t event, void *arg)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;

        case USBD_EVENT_CONFIGURED:
            nxsem_post(&g_usb_cdcacm_serial[0]->txdone_sem);
            nxsem_post(&g_usb_cdcacm_serial[0]->rxdone_sem);
            break;
        default:
            break;
    }
}

static void cdcacm_notify_handler2(uint8_t busid, uint8_t event, void *arg)
{
    switch (event) {
        case USBD_EVENT_RESET:

            break;

        case USBD_EVENT_CONFIGURED:
            nxsem_post(&g_usb_cdcacm_serial[1]->txdone_sem);
            nxsem_post(&g_usb_cdcacm_serial[1]->rxdone_sem);
            break;
        default:
            break;
    }
}

static void cdcacm_notify_handler3(uint8_t busid, uint8_t event, void *arg)
{
    switch (event) {
        case USBD_EVENT_RESET:

            break;

        case USBD_EVENT_CONFIGURED:
            nxsem_post(&g_usb_cdcacm_serial[2]->txdone_sem);
            nxsem_post(&g_usb_cdcacm_serial[2]->rxdone_sem);
            break;
        default:
            break;
    }
}

usbd_notify_handler cdcacm_notify_handler[8] = {
    cdcacm_notify_handler1,
    cdcacm_notify_handler2,
    cdcacm_notify_handler3
};

void usbd_cdcacm_init(uint8_t busid, uint8_t id, const char *name, uint8_t inep, uint8_t outep)
{
    g_usb_cdcacm_serial[id] = kmm_malloc(sizeof(struct usbdev_serial_s));
    DEBUGASSERT(g_usb_cdcacm_serial[id]);

    memset(g_usb_cdcacm_serial[id], 0, sizeof(struct usbdev_serial_s));
    memcpy(g_usb_cdcacm_serial[id]->name, name, strlen(name));

    circbuf_init(&g_usb_cdcacm_serial[id]->circ, g_usb_cdcacm_serial[id]->cache_rxbuffer, CONFIG_USBDEV_CDCACM_RXBUFSIZE);

    nxsem_init(&g_usb_cdcacm_serial[id]->rxdone_sem, 0, 0);
    nxsem_init(&g_usb_cdcacm_serial[id]->txdone_sem, 0, 0);

    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &g_usb_cdcacm_serial[id]->ctrl_intf));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &g_usb_cdcacm_serial[id]->data_intf));
    g_usb_cdcacm_serial[id]->ctrl_intf.notify_handler = cdcacm_notify_handler[id];

    cdc_out_ep[id].ep_addr = outep;
    cdc_in_ep[id].ep_addr = inep;
    usbd_add_endpoint(busid, &cdc_out_ep[id]);
    usbd_add_endpoint(busid, &cdc_in_ep[id]);

    register_driver(name, &g_usbdevops, 0666, g_usb_cdcacm_serial[id]);
}

void usbd_cdcacm_deinit(uint8_t busid, uint8_t id)
{
    unregister_driver(g_usb_cdcacm_serial[id]->name);

    kmm_free(g_usb_cdcacm_serial[id]);
}