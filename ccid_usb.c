#include "ccid_usb.h"

#include <furi_hal.h>
#include <furi_hal_usb.h>
#include <usb_std.h>
#include <string.h>

#define TAG "CcidUsb"

#ifndef USB_EP0_SIZE
#define USB_EP0_SIZE 8
#endif

#define CCID_STR_MANUF   1
#define CCID_STR_PRODUCT 2
#define CCID_STR_SERIAL  3
/* furi_hal_usb only serves string indices 0..3; interface names are served by
 * a descriptor handler chained in front of the stock one. */
#define CCID_STR_IFACE0 4
#define CCID_STR_IFACE1 5

#define CCID_EP_SIZE  64
#define CCID_INT_SIZE 8

static const uint8_t ep_out[CCID_MAX_IFACES] = {0x01, 0x04};
static const uint8_t ep_in[CCID_MAX_IFACES] = {0x82, 0x85};
static const uint8_t ep_int[CCID_MAX_IFACES] = {0x83, 0x86};

/* Class requests */
#define CCID_REQ_ABORT                 0x01
#define CCID_REQ_GET_CLOCK_FREQUENCIES 0x02
#define CCID_REQ_GET_DATA_RATES        0x03

struct CcidClassDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdCCID;
    uint8_t bMaxSlotIndex;
    uint8_t bVoltageSupport;
    uint32_t dwProtocols;
    uint32_t dwDefaultClock;
    uint32_t dwMaximumClock;
    uint8_t bNumClockSupported;
    uint32_t dwDataRate;
    uint32_t dwMaxDataRate;
    uint8_t bNumDataRatesSupported;
    uint32_t dwMaxIFSD;
    uint32_t dwSynchProtocols;
    uint32_t dwMechanical;
    uint32_t dwFeatures;
    uint32_t dwMaxCCIDMessageLength;
    uint8_t bClassGetResponse;
    uint8_t bClassEnvelope;
    uint16_t wLcdLayout;
    uint8_t bPINSupport;
    uint8_t bMaxCCIDBusySlots;
} __attribute__((packed));

struct CcidInterfaceBlock {
    struct usb_interface_descriptor intf;
    struct CcidClassDescriptor ccid;
    struct usb_endpoint_descriptor ep_out;
    struct usb_endpoint_descriptor ep_in;
    struct usb_endpoint_descriptor ep_int;
} __attribute__((packed));

struct CcidConfigDescriptor {
    struct usb_config_descriptor config;
    struct CcidInterfaceBlock iface[CCID_MAX_IFACES];
} __attribute__((packed));

static struct usb_device_descriptor ccid_device_desc = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DTYPE_DEVICE,
    .bcdUSB = VERSION_BCD(2, 0, 0),
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = USB_EP0_SIZE,
    .idVendor = 0,
    .idProduct = 0,
    .bcdDevice = 0,
    .iManufacturer = CCID_STR_MANUF,
    .iProduct = CCID_STR_PRODUCT,
    .iSerialNumber = CCID_STR_SERIAL,
    .bNumConfigurations = 1,
};

static struct CcidConfigDescriptor ccid_cfg_desc;

typedef struct {
    /* OUT: message assembly */
    uint8_t rx[CCID_MAX_MSG];
    volatile size_t rx_len;
    volatile size_t msg_len;
    volatile bool msg_ready;
    volatile bool rx_deferred;
    /* IN: response */
    uint8_t tx[CCID_MAX_MSG];
    volatile size_t tx_len;
    volatile size_t tx_off;
    volatile bool tx_need_zlp;
    volatile bool tx_busy;
    /* Interrupt */
    uint8_t int_buf[2];
    volatile bool int_busy;
    volatile bool int_pending;
    volatile uint8_t int_pending_state;
} CcidIface;

typedef struct {
    usbd_device* dev;
    FuriHalUsbInterface* prev_config;
    usbd_dsc_callback prev_descriptor;
    FuriThreadId thread;
    bool started;
    uint8_t interfaces;
    const void* str_interface[CCID_MAX_IFACES];
    volatile bool configured;
    CcidIface iface[CCID_MAX_IFACES];
} Ccid;

static Ccid ccid;

static void ccid_notify(uint32_t flags) {
    if(ccid.thread) furi_thread_flags_set(ccid.thread, flags);
}

static uint32_t ccid_le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int ccid_iface_of_ep(uint8_t ep) {
    for(uint8_t i = 0; i < ccid.interfaces; i++) {
        if((ep & 0x7F) == (ep_out[i] & 0x7F) || (ep & 0x7F) == (ep_in[i] & 0x7F) ||
           (ep & 0x7F) == (ep_int[i] & 0x7F)) {
            return i;
        }
    }
    return -1;
}

static void ccid_read_out(usbd_device* dev, uint8_t i) {
    CcidIface* c = &ccid.iface[i];
    if(c->msg_ready) {
        c->rx_deferred = true; /* NAK until the current message is consumed */
        return;
    }
    uint8_t pkt[CCID_EP_SIZE];
    int32_t len = usbd_ep_read(dev, ep_out[i], pkt, CCID_EP_SIZE);
    if(len <= 0) return;
    if(c->rx_len + (size_t)len > sizeof(c->rx)) {
        c->rx_len = 0; /* oversize: drop */
        return;
    }
    memcpy(&c->rx[c->rx_len], pkt, (size_t)len);
    c->rx_len += (size_t)len;
    if(c->rx_len >= CCID_HEADER_SIZE) {
        size_t need = CCID_HEADER_SIZE + ccid_le32(&c->rx[1]);
        if(need > sizeof(c->rx)) {
            c->rx_len = 0;
            return;
        }
        if(c->rx_len >= need) {
            c->msg_len = need;
            c->msg_ready = true;
            ccid_notify(CCID_FLAG_MSG);
        }
    }
}

static void ccid_tx_next(usbd_device* dev, uint8_t i) {
    CcidIface* c = &ccid.iface[i];
    if(c->tx_off < c->tx_len) {
        size_t n = c->tx_len - c->tx_off;
        if(n > CCID_EP_SIZE) n = CCID_EP_SIZE;
        size_t off = c->tx_off;
        c->tx_off = off + n;
        c->tx_need_zlp = (n == CCID_EP_SIZE) && (c->tx_off == c->tx_len);
        usbd_ep_write(dev, ep_in[i], &c->tx[off], (uint16_t)n);
    } else if(c->tx_need_zlp) {
        c->tx_need_zlp = false;
        usbd_ep_write(dev, ep_in[i], NULL, 0);
    } else {
        c->tx_busy = false;
        ccid_notify(CCID_FLAG_TX);
    }
}

static void ccid_int_send(usbd_device* dev, uint8_t i, uint8_t state) {
    CcidIface* c = &ccid.iface[i];
    c->int_buf[0] = 0x50; /* RDR_to_PC_NotifySlotChange */
    c->int_buf[1] = state;
    c->int_busy = true;
    usbd_ep_write(dev, ep_int[i], c->int_buf, 2);
}

static void ccid_ep_callback(usbd_device* dev, uint8_t event, uint8_t ep) {
    int i = ccid_iface_of_ep(ep);
    if(i < 0) return;
    CcidIface* c = &ccid.iface[i];
    if(event == usbd_evt_eprx) {
        ccid_read_out(dev, (uint8_t)i);
    } else if(event == usbd_evt_eptx) {
        if((ep & 0x7F) == (ep_int[i] & 0x7F)) {
            c->int_busy = false;
            if(c->int_pending) {
                c->int_pending = false;
                ccid_int_send(dev, (uint8_t)i, c->int_pending_state);
            }
        } else {
            ccid_tx_next(dev, (uint8_t)i);
        }
    }
}

static void ccid_reset_ifaces(void) {
    for(uint8_t i = 0; i < CCID_MAX_IFACES; i++) {
        CcidIface* c = &ccid.iface[i];
        c->tx_busy = false;
        c->int_busy = false;
        c->int_pending = false;
        c->msg_ready = false;
        c->rx_deferred = false;
        c->rx_len = 0;
    }
}

static usbd_respond ccid_ep_config(usbd_device* dev, uint8_t cfg) {
    switch(cfg) {
    case 0:
        for(uint8_t i = 0; i < ccid.interfaces; i++) {
            usbd_ep_deconfig(dev, ep_out[i]);
            usbd_ep_deconfig(dev, ep_in[i]);
            usbd_ep_deconfig(dev, ep_int[i]);
            usbd_reg_endpoint(dev, ep_out[i], 0);
            usbd_reg_endpoint(dev, ep_in[i], 0);
            usbd_reg_endpoint(dev, ep_int[i], 0);
        }
        ccid.configured = false;
        ccid_reset_ifaces();
        ccid_notify(CCID_FLAG_CONFIG);
        return usbd_ack;
    case 1:
        for(uint8_t i = 0; i < ccid.interfaces; i++) {
            usbd_ep_config(dev, ep_out[i], USB_EPTYPE_BULK, CCID_EP_SIZE);
            usbd_ep_config(dev, ep_in[i], USB_EPTYPE_BULK, CCID_EP_SIZE);
            usbd_ep_config(dev, ep_int[i], USB_EPTYPE_INTERRUPT, CCID_INT_SIZE);
            usbd_reg_endpoint(dev, ep_out[i], ccid_ep_callback);
            usbd_reg_endpoint(dev, ep_in[i], ccid_ep_callback);
            usbd_reg_endpoint(dev, ep_int[i], ccid_ep_callback);
        }
        ccid_reset_ifaces();
        ccid.configured = true;
        ccid_notify(CCID_FLAG_CONFIG);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

static usbd_respond ccid_control(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback) {
    UNUSED(dev);
    UNUSED(callback);
    if((req->bmRequestType & 0x7F) != 0x21) return usbd_fail; /* class, interface */
    switch(req->bRequest) {
    case CCID_REQ_ABORT:
        return usbd_ack;
    case CCID_REQ_GET_CLOCK_FREQUENCIES:
    case CCID_REQ_GET_DATA_RATES:
        /* bNumClockSupported / bNumDataRatesSupported are 0: not supported */
        return usbd_fail;
    default:
        return usbd_fail;
    }
}

static usbd_respond ccid_descriptor(usbd_ctlreq* req, void** address, uint16_t* length) {
    const uint8_t dtype = (uint8_t)(req->wValue >> 8);
    const uint8_t dnumber = (uint8_t)(req->wValue & 0xFF);
    if(dtype == USB_DTYPE_STRING && dnumber >= CCID_STR_IFACE0 &&
       dnumber < CCID_STR_IFACE0 + CCID_MAX_IFACES) {
        const void* s = ccid.str_interface[dnumber - CCID_STR_IFACE0];
        if(!s) return usbd_fail;
        *address = (void*)s;
        *length = ((const struct usb_string_descriptor*)s)->bLength;
        return usbd_ack;
    }
    return ccid.prev_descriptor ? ccid.prev_descriptor(req, address, length) : usbd_fail;
}

static void ccid_init(usbd_device* dev, FuriHalUsbInterface* intf, void* ctx) {
    UNUSED(intf);
    UNUSED(ctx);
    ccid.dev = dev;
    ccid.prev_descriptor = dev->descriptor_callback;
    usbd_reg_descr(dev, ccid_descriptor);
    usbd_reg_config(dev, ccid_ep_config);
    usbd_reg_control(dev, ccid_control);
    usbd_connect(dev, true);
}

static void ccid_deinit(usbd_device* dev) {
    usbd_reg_config(dev, NULL);
    usbd_reg_control(dev, NULL);
    if(ccid.prev_descriptor) usbd_reg_descr(dev, ccid.prev_descriptor);
    ccid.configured = false;
}

static void ccid_wakeup(usbd_device* dev) {
    UNUSED(dev);
}

static void ccid_suspend(usbd_device* dev) {
    UNUSED(dev);
}

static FuriHalUsbInterface ccid_usb_interface = {
    .init = ccid_init,
    .deinit = ccid_deinit,
    .wakeup = ccid_wakeup,
    .suspend = ccid_suspend,
    .dev_descr = &ccid_device_desc,
    .str_manuf_descr = NULL,
    .str_prod_descr = NULL,
    .str_serial_descr = NULL,
    .cfg_descr = (void*)&ccid_cfg_desc,
};

static void ccid_build_descriptors(const CcidUsbConfig* config) {
    uint8_t n = config->interfaces;
    if(n < 1) n = 1;
    if(n > CCID_MAX_IFACES) n = CCID_MAX_IFACES;
    ccid.interfaces = n;

    ccid_device_desc.idVendor = config->vid;
    ccid_device_desc.idProduct = config->pid;
    ccid_device_desc.bcdDevice = config->bcd_device;
    ccid_device_desc.bcdUSB = config->bcd_usb;
    ccid_device_desc.iSerialNumber = config->str_serial ? CCID_STR_SERIAL : 0;
    ccid_usb_interface.str_manuf_descr = (void*)config->str_manufacturer;
    ccid_usb_interface.str_prod_descr = (void*)config->str_product;
    ccid_usb_interface.str_serial_descr = (void*)config->str_serial;

    memset(&ccid_cfg_desc, 0, sizeof(ccid_cfg_desc));
    struct usb_config_descriptor* c = &ccid_cfg_desc.config;
    c->bLength = sizeof(struct usb_config_descriptor);
    c->bDescriptorType = USB_DTYPE_CONFIGURATION;
    c->wTotalLength =
        (uint16_t)(sizeof(struct usb_config_descriptor) + n * sizeof(struct CcidInterfaceBlock));
    c->bNumInterfaces = n;
    c->bConfigurationValue = 1;
    c->iConfiguration = 0;
    c->bmAttributes = USB_CFG_ATTR_RESERVED;
    c->bMaxPower = (uint8_t)USB_CFG_POWER_MA(config->max_power_ma);

    for(uint8_t i = 0; i < n; i++) {
        struct CcidInterfaceBlock* b = &ccid_cfg_desc.iface[i];
        ccid.str_interface[i] = config->str_interface[i];

        b->intf.bLength = sizeof(struct usb_interface_descriptor);
        b->intf.bDescriptorType = USB_DTYPE_INTERFACE;
        b->intf.bInterfaceNumber = i;
        b->intf.bAlternateSetting = 0;
        b->intf.bNumEndpoints = 3;
        b->intf.bInterfaceClass = 0x0B; /* Smart Card (CCID) */
        b->intf.bInterfaceSubClass = 0x00;
        b->intf.bInterfaceProtocol = 0x00;
        b->intf.iInterface = config->str_interface[i] ? (uint8_t)(CCID_STR_IFACE0 + i) : 0;

        struct CcidClassDescriptor* d = &b->ccid;
        d->bLength = sizeof(struct CcidClassDescriptor);
        d->bDescriptorType = 0x21;
        d->bcdCCID = 0x0110;
        d->bMaxSlotIndex = 0;
        d->bVoltageSupport = config->voltage_support;
        d->dwProtocols = config->protocols;
        d->dwDefaultClock = config->default_clock;
        d->dwMaximumClock = config->max_clock;
        d->bNumClockSupported = 0;
        d->dwDataRate = config->data_rate;
        d->dwMaxDataRate = config->max_data_rate;
        d->bNumDataRatesSupported = 0;
        d->dwMaxIFSD = config->max_ifsd;
        d->dwFeatures = config->features;
        d->dwMaxCCIDMessageLength = config->max_message_length;
        d->bClassGetResponse = 0xFF;
        d->bClassEnvelope = 0xFF;
        d->bMaxCCIDBusySlots = 1;

        struct usb_endpoint_descriptor* e[3] = {&b->ep_out, &b->ep_in, &b->ep_int};
        const uint8_t addr[3] = {ep_out[i], ep_in[i], ep_int[i]};
        for(uint8_t k = 0; k < 3; k++) {
            e[k]->bLength = sizeof(struct usb_endpoint_descriptor);
            e[k]->bDescriptorType = USB_DTYPE_ENDPOINT;
            e[k]->bEndpointAddress = addr[k];
            e[k]->bmAttributes = k == 2 ? USB_EPTYPE_INTERRUPT : USB_EPTYPE_BULK;
            e[k]->wMaxPacketSize = k == 2 ? CCID_INT_SIZE : CCID_EP_SIZE;
            e[k]->bInterval = k == 2 ? 0x10 : 0x00;
        }
    }
}

bool ccid_usb_start(const CcidUsbConfig* config, FuriThreadId notify_thread) {
    memset(&ccid, 0, sizeof(ccid));
    ccid.thread = notify_thread;
    ccid_build_descriptors(config);

    ccid.prev_config = furi_hal_usb_get_config();
    furi_hal_usb_unlock();
    if(!furi_hal_usb_set_config(&ccid_usb_interface, NULL)) {
        FURI_LOG_E(TAG, "USB config switch failed");
        ccid.thread = NULL;
        return false;
    }
    ccid.started = true;
    return true;
}

void ccid_usb_stop(void) {
    if(!ccid.started) return;
    furi_hal_usb_set_config(ccid.prev_config, NULL);
    ccid.thread = NULL;
    ccid.started = false;
}

bool ccid_usb_is_configured(void) {
    return ccid.configured;
}

size_t ccid_usb_take_message(uint8_t i, uint8_t* buf, size_t max) {
    if(i >= ccid.interfaces) return 0;
    CcidIface* c = &ccid.iface[i];
    if(!c->msg_ready) return 0;
    size_t n = c->msg_len;
    if(n > max) n = max;
    memcpy(buf, c->rx, n);

    FURI_CRITICAL_ENTER();
    size_t extra = c->rx_len - c->msg_len;
    memmove(c->rx, &c->rx[c->msg_len], extra);
    c->rx_len = extra;
    c->msg_ready = false;
    bool deferred = c->rx_deferred;
    c->rx_deferred = false;
    FURI_CRITICAL_EXIT();

    if(deferred && ccid.dev) ccid_read_out(ccid.dev, i);
    return n;
}

bool ccid_usb_tx_busy(uint8_t i) {
    return i < ccid.interfaces && ccid.iface[i].tx_busy;
}

bool ccid_usb_send(uint8_t i, const uint8_t* msg, size_t len) {
    if(i >= ccid.interfaces) return false;
    CcidIface* c = &ccid.iface[i];
    if(!ccid.configured || c->tx_busy || !ccid.dev || len > sizeof(c->tx)) return false;
    memcpy(c->tx, msg, len);
    c->tx_len = len;
    c->tx_off = 0;
    c->tx_need_zlp = false;
    c->tx_busy = true;
    ccid_tx_next(ccid.dev, i);
    return true;
}

void ccid_usb_notify_slot(uint8_t i, bool present, bool changed) {
    if(i >= ccid.interfaces || !ccid.configured || !ccid.dev) return;
    CcidIface* c = &ccid.iface[i];
    uint8_t state = (uint8_t)((present ? 0x01 : 0x00) | (changed ? 0x02 : 0x00));
    FURI_CRITICAL_ENTER();
    bool busy = c->int_busy;
    if(busy) {
        /* Merge: keep the "changed" bit of an unsent notification */
        c->int_pending_state =
            (uint8_t)((c->int_pending ? (c->int_pending_state & 0x02) : 0) | state);
        c->int_pending = true;
    }
    FURI_CRITICAL_EXIT();
    if(!busy) ccid_int_send(ccid.dev, i, state);
}
