#include "cp210x_usb.h"

#include <furi_hal.h>
#include <furi_hal_usb.h>
#include <usb_std.h>
#include <string.h>

#define TAG "Cp210xUsb"

#ifndef USB_EP0_SIZE
#define USB_EP0_SIZE 8
#endif

#define CP_STR_MANUF   1
#define CP_STR_PRODUCT 2
#define CP_STR_SERIAL  3

/* Separate endpoint numbers per direction (the STM32 USB driver pattern used by
 * the stock CDC implementation). Host drivers discover them from descriptors. */
#define CP_EP_OUT  0x01
#define CP_EP_IN   0x82
#define CP_EP_SIZE 64

#define CP_RX_STREAM_SIZE 1024

/* CP210x vendor requests (AN571) */
#define CP210X_IFC_ENABLE      0x00
#define CP210X_SET_BAUDDIV     0x01
#define CP210X_GET_BAUDDIV     0x02
#define CP210X_SET_LINE_CTL    0x03
#define CP210X_GET_LINE_CTL    0x04
#define CP210X_SET_BREAK       0x05
#define CP210X_IMM_CHAR        0x06
#define CP210X_SET_MHS         0x07
#define CP210X_GET_MDMSTS      0x08
#define CP210X_SET_XON         0x09
#define CP210X_SET_XOFF        0x0A
#define CP210X_SET_EVENTMASK   0x0B
#define CP210X_GET_EVENTMASK   0x0C
#define CP210X_SET_CHAR        0x0D
#define CP210X_GET_CHARS       0x0E
#define CP210X_GET_PROPS       0x0F
#define CP210X_GET_COMM_STATUS 0x10
#define CP210X_RESET           0x11
#define CP210X_PURGE           0x12
#define CP210X_SET_FLOW        0x13
#define CP210X_GET_FLOW        0x14
#define CP210X_EMBED_EVENTS    0x15
#define CP210X_GET_EVENTSTATE  0x16
#define CP210X_SET_CHARS       0x19
#define CP210X_GET_BAUDRATE    0x1D
#define CP210X_SET_BAUDRATE    0x1E
#define CP210X_VENDOR_SPECIFIC 0xFF

#define CP210X_GET_PARTNUM     0x370B
#define CP210X_PARTNUM_CP2102  0x02

#define CP210X_BAUD_CLOCK 3686400U

/* ------------------------------------------------------------------------ */
/* Descriptors                                                               */
/* ------------------------------------------------------------------------ */

struct CpConfigDescriptor {
    struct usb_config_descriptor config;
    struct usb_interface_descriptor intf;
    struct usb_endpoint_descriptor ep_in;
    struct usb_endpoint_descriptor ep_out;
} __attribute__((packed));

static struct usb_device_descriptor cp_device_desc = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DTYPE_DEVICE,
    .bcdUSB = VERSION_BCD(1, 1, 0),
    .bDeviceClass = 0x00, /* defined per interface */
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = USB_EP0_SIZE,
    .idVendor = 0x10C4,
    .idProduct = 0xEA60,
    .bcdDevice = VERSION_BCD(1, 0, 0),
    .iManufacturer = CP_STR_MANUF,
    .iProduct = CP_STR_PRODUCT,
    .iSerialNumber = CP_STR_SERIAL,
    .bNumConfigurations = 1,
};

static const struct CpConfigDescriptor cp_cfg_desc = {
    .config =
        {
            .bLength = sizeof(struct usb_config_descriptor),
            .bDescriptorType = USB_DTYPE_CONFIGURATION,
            .wTotalLength = sizeof(struct CpConfigDescriptor),
            .bNumInterfaces = 1,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = USB_CFG_ATTR_RESERVED,
            .bMaxPower = USB_CFG_POWER_MA(100),
        },
    .intf =
        {
            .bLength = sizeof(struct usb_interface_descriptor),
            .bDescriptorType = USB_DTYPE_INTERFACE,
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 0xFF, /* vendor specific */
            .bInterfaceSubClass = 0x00,
            .bInterfaceProtocol = 0x00,
            .iInterface = CP_STR_PRODUCT,
        },
    .ep_in =
        {
            .bLength = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = CP_EP_IN,
            .bmAttributes = USB_EPTYPE_BULK,
            .wMaxPacketSize = CP_EP_SIZE,
            .bInterval = 0x00,
        },
    .ep_out =
        {
            .bLength = sizeof(struct usb_endpoint_descriptor),
            .bDescriptorType = USB_DTYPE_ENDPOINT,
            .bEndpointAddress = CP_EP_OUT,
            .bmAttributes = USB_EPTYPE_BULK,
            .wMaxPacketSize = CP_EP_SIZE,
            .bInterval = 0x00,
        },
};

static const struct usb_string_descriptor cp_str_manuf = USB_STRING_DESC("Silicon Labs");
static const struct usb_string_descriptor cp_str_product =
    USB_STRING_DESC("CP2102 USB to UART Bridge Controller");
static const struct usb_string_descriptor cp_str_serial = USB_STRING_DESC("0001");

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

typedef struct {
    usbd_device* dev;
    FuriHalUsbInterface* prev_config;
    FuriThreadId thread;
    FuriStreamBuffer* rx_stream;

    volatile bool configured;
    volatile bool tx_busy;
    volatile bool rx_deferred;

    /* Line state (written from the USB interrupt) */
    volatile bool enabled;
    volatile uint32_t baud;
    volatile uint16_t line_ctl;
    volatile bool dtr;
    volatile bool rts;
    volatile bool brk;
    volatile uint8_t embed_escape;
    uint16_t event_mask;
    uint8_t special_chars[6]; /* EOF, error, break, event, XON, XOFF */
    uint8_t flow[16];

    volatile bool imm_pending;
    volatile uint8_t imm_char;
    volatile uint32_t purge;
    volatile uint32_t errors;
    volatile uint32_t queue_to_host;
    volatile uint32_t queue_to_uart;

    uint8_t ctl_buf[72];
} Cp210x;

static Cp210x cp;

static void cp_notify(uint32_t flags) {
    if(cp.thread) furi_thread_flags_set(cp.thread, flags);
}

static void cp_put_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t cp_get_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void cp_line_defaults(void) {
    cp.enabled = false;
    cp.baud = 9600;
    cp.line_ctl = 0x0800; /* 8 data bits, no parity, 1 stop bit */
    cp.dtr = false;
    cp.rts = false;
    cp.brk = false;
    cp.embed_escape = 0;
    cp.event_mask = 0;
    /* EOF, error, break, event, XON (DC1), XOFF (DC3) */
    const uint8_t chars[6] = {0x00, 0x00, 0x00, 0x00, 0x11, 0x13};
    memcpy(cp.special_chars, chars, sizeof(chars));
    memset(cp.flow, 0, sizeof(cp.flow));
    cp_put_u32(&cp.flow[0], 0x00000001); /* ulControlHandshake: DTR control */
    cp_put_u32(&cp.flow[4], 0x00000040); /* ulFlowReplace: RTS control */
    cp_put_u32(&cp.flow[8], 0x00000080); /* ulXonLimit */
    cp_put_u32(&cp.flow[12], 0x00000080); /* ulXoffLimit */
    cp.errors = 0;
}

/* ------------------------------------------------------------------------ */
/* Endpoints                                                                 */
/* ------------------------------------------------------------------------ */

static void cp_read_out_ep(usbd_device* dev) {
    uint8_t buf[CP_EP_SIZE];
    int32_t len = usbd_ep_read(dev, CP_EP_OUT, buf, CP_EP_SIZE);
    if(len > 0 && cp.enabled) {
        furi_stream_buffer_send(cp.rx_stream, buf, (size_t)len, 0);
    }
}

static void cp_ep_callback(usbd_device* dev, uint8_t event, uint8_t ep) {
    UNUSED(ep);
    if(event == usbd_evt_eptx) {
        cp.tx_busy = false;
        cp_notify(CP210X_FLAG_TX);
    } else if(event == usbd_evt_eprx) {
        /* Leave the endpoint NAKing when there is no room: USB backpressure */
        if(furi_stream_buffer_spaces_available(cp.rx_stream) >= CP_EP_SIZE) {
            cp_read_out_ep(dev);
        } else {
            cp.rx_deferred = true;
        }
        cp_notify(CP210X_FLAG_RX);
    }
}

static usbd_respond cp_ep_config(usbd_device* dev, uint8_t cfg) {
    switch(cfg) {
    case 0:
        usbd_ep_deconfig(dev, CP_EP_IN);
        usbd_ep_deconfig(dev, CP_EP_OUT);
        usbd_reg_endpoint(dev, CP_EP_IN, 0);
        usbd_reg_endpoint(dev, CP_EP_OUT, 0);
        cp.configured = false;
        cp.tx_busy = false;
        cp.rx_deferred = false;
        cp_line_defaults();
        cp_notify(CP210X_FLAG_CONFIG);
        return usbd_ack;
    case 1:
        usbd_ep_config(dev, CP_EP_IN, USB_EPTYPE_BULK, CP_EP_SIZE);
        usbd_ep_config(dev, CP_EP_OUT, USB_EPTYPE_BULK, CP_EP_SIZE);
        usbd_reg_endpoint(dev, CP_EP_IN, cp_ep_callback);
        usbd_reg_endpoint(dev, CP_EP_OUT, cp_ep_callback);
        cp.configured = true;
        cp.tx_busy = false;
        cp_notify(CP210X_FLAG_CONFIG);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

/* ------------------------------------------------------------------------ */
/* Vendor control requests                                                   */
/* ------------------------------------------------------------------------ */

static usbd_respond cp_reply(usbd_device* dev, usbd_ctlreq* req, const void* data, uint16_t len) {
    if(len > req->wLength) len = req->wLength;
    if(len > sizeof(cp.ctl_buf)) len = sizeof(cp.ctl_buf);
    memmove(cp.ctl_buf, data, len);
    dev->status.data_ptr = cp.ctl_buf;
    dev->status.data_count = len;
    return usbd_ack;
}

static void cp_build_props(uint8_t* p) {
    memset(p, 0, 66);
    p[0] = 66; /* wLength */
    p[2] = 0x00; /* bcdVersion 1.00 */
    p[3] = 0x01;
    cp_put_u32(&p[4], 0x00000001); /* ulServiceMask: SP_SERIALCOMM */
    cp_put_u32(&p[12], 640); /* ulMaxTxQueue */
    cp_put_u32(&p[16], 640); /* ulMaxRxQueue */
    cp_put_u32(&p[20], 0x10000000); /* ulMaxBaud: BAUD_USER */
    cp_put_u32(&p[24], 0x00000001); /* ulProvSubType: RS232 */
    cp_put_u32(&p[28], 0x0000013F); /* ulProvCapabilities */
    cp_put_u32(&p[32], 0x0000007F); /* ulSettableParams */
    cp_put_u32(&p[36], 0x1007FFFF); /* ulSettableBaud (incl. BAUD_USER) */
    p[40] = 0x0F; /* wSettableData: 5,6,7,8 bits */
    p[42] = 0x07; /* wSettableStopParity: stop 1/1.5/2 */
    p[43] = 0x1F; /* parity none/odd/even/mark/space */
    cp_put_u32(&p[44], 640); /* ulCurrentTxQueue */
    cp_put_u32(&p[48], 640); /* ulCurrentRxQueue */
}

static usbd_respond cp_control_vendor(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback) {
    UNUSED(callback);
    const uint8_t type = req->bmRequestType & 0x60;
    const uint8_t recipient = req->bmRequestType & 0x1F;
    const bool dev_to_host = (req->bmRequestType & 0x80) != 0;

    if(type != 0x40) return usbd_fail; /* vendor requests only */
    if(recipient != 0x00 && recipient != 0x01) return usbd_fail;

    switch(req->bRequest) {
    case CP210X_IFC_ENABLE:
        cp.enabled = (req->wValue & 0x0001) != 0;
        if(!cp.enabled) cp.purge |= CP210X_PURGE_TX | CP210X_PURGE_RX;
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;

    case CP210X_SET_BAUDDIV:
        if(req->wValue) cp.baud = CP210X_BAUD_CLOCK / req->wValue;
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;

    case CP210X_GET_BAUDDIV: {
        if(!dev_to_host) return usbd_fail;
        uint32_t b = cp.baud ? cp.baud : 1;
        uint16_t div = (uint16_t)(CP210X_BAUD_CLOCK / b);
        uint8_t d[2] = {(uint8_t)div, (uint8_t)(div >> 8)};
        return cp_reply(dev, req, d, 2);
    }

    case CP210X_SET_LINE_CTL: {
        /* AN571: an invalid setting gets a procedural stall */
        uint8_t stop = (uint8_t)(req->wValue & 0x0F);
        uint8_t parity = (uint8_t)((req->wValue >> 4) & 0x0F);
        uint8_t bits = (uint8_t)(req->wValue >> 8);
        if(stop > 2 || parity > 4 || bits < 5 || bits > 8) return usbd_fail;
        cp.line_ctl = req->wValue;
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;
    }

    case CP210X_GET_LINE_CTL: {
        if(!dev_to_host) return usbd_fail;
        uint8_t d[2] = {(uint8_t)cp.line_ctl, (uint8_t)(cp.line_ctl >> 8)};
        return cp_reply(dev, req, d, 2);
    }

    case CP210X_SET_BREAK:
        cp.brk = (req->wValue & 0x0001) != 0;
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;

    case CP210X_IMM_CHAR:
        cp.imm_char = (uint8_t)req->wValue;
        cp.imm_pending = true;
        cp_notify(CP210X_FLAG_RX);
        return usbd_ack;

    case CP210X_SET_MHS:
        if(req->wValue & 0x0100) cp.dtr = (req->wValue & 0x0001) != 0;
        if(req->wValue & 0x0200) cp.rts = (req->wValue & 0x0002) != 0;
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;

    case CP210X_GET_MDMSTS: {
        if(!dev_to_host) return usbd_fail;
        /* CTS/DSR/RI/DCD are not wired to the MFRC522: inputs idle (inactive) */
        uint8_t d = (uint8_t)((cp.dtr ? 0x01 : 0) | (cp.rts ? 0x02 : 0));
        return cp_reply(dev, req, &d, 1);
    }

    case CP210X_SET_XON:
    case CP210X_SET_XOFF:
    case CP210X_RESET:
        return usbd_ack;

    case CP210X_SET_EVENTMASK:
        cp.event_mask = req->wValue;
        return usbd_ack;

    case CP210X_GET_EVENTMASK: {
        if(!dev_to_host) return usbd_fail;
        uint8_t d[2] = {(uint8_t)cp.event_mask, (uint8_t)(cp.event_mask >> 8)};
        return cp_reply(dev, req, d, 2);
    }

    case CP210X_SET_CHAR: {
        /* AN571: bits 7:0 = index (EOF, error, break, event, XON, XOFF),
         * bits 15:8 = character value */
        uint8_t idx = (uint8_t)req->wValue;
        if(idx >= sizeof(cp.special_chars)) return usbd_fail;
        cp.special_chars[idx] = (uint8_t)(req->wValue >> 8);
        return usbd_ack;
    }

    case CP210X_GET_CHARS:
        if(!dev_to_host) return usbd_fail;
        return cp_reply(dev, req, cp.special_chars, sizeof(cp.special_chars));

    case CP210X_SET_CHARS:
        /* AN571: anything other than six bytes is an error */
        if(req->wLength != sizeof(cp.special_chars)) return usbd_fail;
        memcpy(cp.special_chars, req->data, sizeof(cp.special_chars));
        return usbd_ack;

    case CP210X_GET_PROPS: {
        if(!dev_to_host) return usbd_fail;
        uint8_t p[66];
        cp_build_props(p);
        return cp_reply(dev, req, p, sizeof(p));
    }

    case CP210X_GET_COMM_STATUS: {
        if(!dev_to_host) return usbd_fail;
        uint8_t s[19];
        memset(s, 0, sizeof(s));
        cp_put_u32(&s[0], cp.errors);
        cp.errors = 0; /* reading clears latched errors */
        cp_put_u32(&s[8], cp.queue_to_host); /* ulAmountInInQueue */
        cp_put_u32(&s[12], cp.queue_to_uart); /* ulAmountInOutQueue */
        return cp_reply(dev, req, s, sizeof(s));
    }

    case CP210X_PURGE:
        /* bits 0/2: transmit queue (to UART), bits 1/3: receive queue (to host) */
        if(req->wValue & 0x0005) cp.purge |= CP210X_PURGE_TX;
        if(req->wValue & 0x000A) cp.purge |= CP210X_PURGE_RX;
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;

    case CP210X_SET_FLOW: {
        uint16_t n = req->wLength;
        if(n > sizeof(cp.flow)) n = sizeof(cp.flow);
        memcpy(cp.flow, req->data, n);
        return usbd_ack;
    }

    case CP210X_GET_FLOW:
        if(!dev_to_host) return usbd_fail;
        return cp_reply(dev, req, cp.flow, sizeof(cp.flow));

    case CP210X_EMBED_EVENTS:
        cp.embed_escape = (uint8_t)req->wValue;
        return usbd_ack;

    case CP210X_GET_EVENTSTATE: {
        if(!dev_to_host) return usbd_fail;
        uint8_t d[2] = {0, 0};
        return cp_reply(dev, req, d, 2);
    }

    case CP210X_GET_BAUDRATE: {
        if(!dev_to_host) return usbd_fail;
        uint8_t d[4];
        cp_put_u32(d, cp.baud);
        return cp_reply(dev, req, d, 4);
    }

    case CP210X_SET_BAUDRATE:
        if(req->wLength >= 4) {
            uint32_t b = cp_get_u32(req->data);
            /* Simulated UART: every rate is accepted exactly */
            if(b) cp.baud = b;
        }
        cp_notify(CP210X_FLAG_LINE);
        return usbd_ack;

    case CP210X_VENDOR_SPECIFIC:
        if(dev_to_host && req->wValue == CP210X_GET_PARTNUM) {
            uint8_t d = CP210X_PARTNUM_CP2102;
            return cp_reply(dev, req, &d, 1);
        }
        return usbd_fail;

    default:
        return usbd_fail;
    }
}

static Cp210xCtlLog cp_log[CP210X_LOG_SIZE];
static volatile uint32_t cp_log_total;
static volatile uint32_t cp_log_stalls;

static usbd_respond cp_control(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback) {
    usbd_respond r = cp_control_vendor(dev, req, callback);
    if((req->bmRequestType & 0x60) == 0x40) {
        Cp210xCtlLog* e = &cp_log[cp_log_total % CP210X_LOG_SIZE];
        e->request = req->bRequest;
        e->in = (req->bmRequestType & 0x80) != 0;
        e->stalled = (r == usbd_fail);
        e->value = req->wValue;
        e->length = req->wLength;
        cp_log_total++;
        if(r == usbd_fail) cp_log_stalls++;
    }
    return r;
}

size_t cp210x_usb_get_log(Cp210xCtlLog* out, size_t max) {
    FURI_CRITICAL_ENTER();
    uint32_t total = cp_log_total;
    size_t n = total < CP210X_LOG_SIZE ? total : CP210X_LOG_SIZE;
    if(n > max) n = max;
    for(size_t i = 0; i < n; i++) {
        out[i] = cp_log[(total - n + i) % CP210X_LOG_SIZE];
    }
    FURI_CRITICAL_EXIT();
    return n;
}

void cp210x_usb_get_counts(uint32_t* total, uint32_t* stalled) {
    *total = cp_log_total;
    *stalled = cp_log_stalls;
}

/* ------------------------------------------------------------------------ */
/* FuriHal USB interface                                                     */
/* ------------------------------------------------------------------------ */

static void cp_init(usbd_device* dev, FuriHalUsbInterface* intf, void* ctx) {
    UNUSED(intf);
    UNUSED(ctx);
    cp.dev = dev;
    usbd_reg_config(dev, cp_ep_config);
    usbd_reg_control(dev, cp_control);
    usbd_connect(dev, true);
}

static void cp_deinit(usbd_device* dev) {
    usbd_reg_config(dev, NULL);
    usbd_reg_control(dev, NULL);
    cp.configured = false;
}

static void cp_wakeup(usbd_device* dev) {
    UNUSED(dev);
}

static void cp_suspend(usbd_device* dev) {
    UNUSED(dev);
}

static FuriHalUsbInterface cp210x_usb_interface = {
    .init = cp_init,
    .deinit = cp_deinit,
    .wakeup = cp_wakeup,
    .suspend = cp_suspend,
    .dev_descr = &cp_device_desc,
    .str_manuf_descr = (void*)&cp_str_manuf,
    .str_prod_descr = (void*)&cp_str_product,
    .str_serial_descr = (void*)&cp_str_serial,
    .cfg_descr = (void*)&cp_cfg_desc,
};

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

bool cp210x_usb_start(FuriThreadId notify_thread) {
    memset(&cp, 0, sizeof(cp));
    cp_line_defaults();
    memset(cp_log, 0, sizeof(cp_log));
    cp_log_total = 0;
    cp_log_stalls = 0;
    cp.thread = notify_thread;
    cp.rx_stream = furi_stream_buffer_alloc(CP_RX_STREAM_SIZE, 1);
    cp.prev_config = furi_hal_usb_get_config();
    furi_hal_usb_unlock();
    if(!furi_hal_usb_set_config(&cp210x_usb_interface, NULL)) {
        FURI_LOG_E(TAG, "USB config switch failed");
        furi_stream_buffer_free(cp.rx_stream);
        cp.rx_stream = NULL;
        cp.thread = NULL;
        return false;
    }
    return true;
}

void cp210x_usb_stop(void) {
    if(!cp.rx_stream) return;
    furi_hal_usb_set_config(cp.prev_config, NULL);
    cp.thread = NULL;
    furi_stream_buffer_free(cp.rx_stream);
    cp.rx_stream = NULL;
}

void cp210x_usb_get_line(Cp210xLine* line) {
    FURI_CRITICAL_ENTER();
    line->configured = cp.configured;
    line->enabled = cp.enabled;
    line->baud = cp.baud;
    line->line_ctl = cp.line_ctl;
    line->dtr = cp.dtr;
    line->rts = cp.rts;
    line->brk = cp.brk;
    line->embed_escape = cp.embed_escape;
    FURI_CRITICAL_EXIT();
}

size_t cp210x_usb_read(uint8_t* buf, size_t max) {
    if(!cp.rx_stream) return 0;
    size_t n = furi_stream_buffer_receive(cp.rx_stream, buf, max, 0);
    if(cp.rx_deferred && cp.dev &&
       furi_stream_buffer_spaces_available(cp.rx_stream) >= CP_EP_SIZE) {
        cp.rx_deferred = false;
        cp_read_out_ep(cp.dev);
    }
    return n;
}

bool cp210x_usb_take_imm_char(uint8_t* c) {
    bool pending = false;
    FURI_CRITICAL_ENTER();
    if(cp.imm_pending) {
        *c = cp.imm_char;
        cp.imm_pending = false;
        pending = true;
    }
    FURI_CRITICAL_EXIT();
    return pending;
}

uint32_t cp210x_usb_take_purge(void) {
    FURI_CRITICAL_ENTER();
    uint32_t p = cp.purge;
    cp.purge = 0;
    FURI_CRITICAL_EXIT();
    if(p & CP210X_PURGE_TX) {
        uint8_t tmp[32];
        while(cp.rx_stream && furi_stream_buffer_receive(cp.rx_stream, tmp, sizeof(tmp), 0)) {
        }
    }
    return p;
}

bool cp210x_usb_tx_ready(void) {
    return cp.configured && !cp.tx_busy;
}

bool cp210x_usb_write(const uint8_t* buf, size_t len) {
    if(!cp210x_usb_tx_ready() || !cp.dev) return false;
    if(len > CP_EP_SIZE) len = CP_EP_SIZE;
    cp.tx_busy = true;
    if(usbd_ep_write(cp.dev, CP_EP_IN, buf, (uint16_t)len) < 0) {
        cp.tx_busy = false;
        return false;
    }
    return true;
}

void cp210x_usb_set_queue_levels(uint32_t to_host, uint32_t to_uart) {
    cp.queue_to_host = to_host;
    cp.queue_to_uart = to_uart;
}

void cp210x_usb_report_error(uint32_t error_bits) {
    FURI_CRITICAL_ENTER();
    cp.errors |= error_bits;
    FURI_CRITICAL_EXIT();
}
