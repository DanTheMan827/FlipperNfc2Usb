/**
 * Generic USB CDC-ACM ("USB serial") adapter personality. Every current OS
 * binds its built-in driver to it by class (Windows 10+ usbser.sys, macOS
 * AppleUSBCDCACM, Linux cdc_acm), so no driver install is needed.
 *
 * Interface protocol is 0 (no AT command set) so modem managers leave the
 * port alone.
 */
#include "usb_uart.h"

#include <furi_hal.h>
#include <furi_hal_usb.h>
#include <usb_std.h>
#include <string.h>

#define TAG "CdcUsb"

#ifndef USB_EP0_SIZE
#define USB_EP0_SIZE 8
#endif

/* pid.codes test VID/PID: a generic id that no vendor driver claims */
#define CDC_VID 0x1209
#define CDC_PID 0x0001

#define CDC_STR_MANUF   1
#define CDC_STR_PRODUCT 2
#define CDC_STR_SERIAL  3

#define CDC_EP_OUT      0x01
#define CDC_EP_IN       0x82
#define CDC_EP_NOTIFY   0x83
#define CDC_EP_SIZE     64
#define CDC_NOTIFY_SIZE 16

#define CDC_RX_STREAM_SIZE 1024

/* CDC PSTN class requests (CDC PSTN 1.2 table 13) */
#define CDC_SET_LINE_CODING        0x20
#define CDC_GET_LINE_CODING        0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK             0x23

/* SERIAL_STATE notification bits (CDC PSTN 1.2 table 31) */
#define CDC_STATE_DCD     (1U << 0) /* bRxCarrier */
#define CDC_STATE_DSR     (1U << 1) /* bTxCarrier */
#define CDC_STATE_BREAK   (1U << 2)
#define CDC_STATE_RING    (1U << 3)
#define CDC_STATE_FRAMING (1U << 4)
#define CDC_STATE_PARITY  (1U << 5)
#define CDC_STATE_OVERRUN (1U << 6)

/* ------------------------------------------------------------------------ */
/* Descriptors                                                               */
/* ------------------------------------------------------------------------ */

static struct usb_device_descriptor cdc_device_desc = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DTYPE_DEVICE,
    .bcdUSB = VERSION_BCD(2, 0, 0),
    .bDeviceClass = 0xEF, /* miscellaneous: function described by an IAD */
    .bDeviceSubClass = 0x02,
    .bDeviceProtocol = 0x01,
    .bMaxPacketSize0 = USB_EP0_SIZE,
    .idVendor = CDC_VID,
    .idProduct = CDC_PID,
    .bcdDevice = VERSION_BCD(1, 0, 0),
    .iManufacturer = CDC_STR_MANUF,
    .iProduct = CDC_STR_PRODUCT,
    .iSerialNumber = CDC_STR_SERIAL,
    .bNumConfigurations = 1,
};

#define CDC_CFG_LEN 75

static const uint8_t cdc_cfg_desc[CDC_CFG_LEN] __attribute__((aligned(4))) = {
    /* Configuration */
    0x09, 0x02, CDC_CFG_LEN, 0x00, 0x02, 0x01, 0x00, 0x80, 50 /* 100 mA */,
    /* Interface association: CDC ACM, interfaces 0-1 */
    0x08, 0x0B, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,
    /* Interface 0: communications, ACM, no protocol */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x00, CDC_STR_PRODUCT,
    /* Header functional descriptor, CDC 1.10 */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* Call management: none, data interface 1 */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* ACM: line coding, control line state and SEND_BREAK supported */
    0x04, 0x24, 0x02, 0x06,
    /* Union: control 0, data 1 */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* Notification endpoint */
    0x07, 0x05, CDC_EP_NOTIFY, 0x03, CDC_NOTIFY_SIZE, 0x00, 0x10,
    /* Interface 1: data */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* Bulk OUT */
    0x07, 0x05, CDC_EP_OUT, 0x02, CDC_EP_SIZE, 0x00, 0x00,
    /* Bulk IN */
    0x07, 0x05, CDC_EP_IN, 0x02, CDC_EP_SIZE, 0x00, 0x00,
};

static const struct usb_string_descriptor cdc_str_manuf = USB_STRING_DESC("NFC2USB");
static const struct usb_string_descriptor cdc_str_product = USB_STRING_DESC("MFRC522 UART");
static const struct usb_string_descriptor cdc_str_serial = USB_STRING_DESC("MFRC522");

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

    volatile uint32_t baud;
    volatile uint16_t line_ctl;
    volatile bool dtr;
    volatile bool rts;
    volatile bool brk; /* SEND_BREAK with no duration, or a timed break running */
    volatile uint32_t brk_until; /* tick a timed break ends, 0 = untimed */

    volatile uint8_t pending_state; /* SERIAL_STATE error bits to report */
    volatile bool notify_busy;
    volatile bool notify_pending;
    uint8_t notify_buf[10];

    uint8_t ctl_buf[8];
} CdcAcm;

static CdcAcm cdc;

static UsbUartCtlLog cdc_log[USB_UART_LOG_SIZE];
static volatile uint32_t cdc_log_total;
static volatile uint32_t cdc_log_stalls;

static void cdc_notify(uint32_t flags) {
    if(cdc.thread) furi_thread_flags_set(cdc.thread, flags);
}

static void cdc_line_defaults(void) {
    cdc.baud = 9600;
    cdc.line_ctl = 0x0800; /* 8N1 */
    cdc.dtr = false;
    cdc.rts = false;
    cdc.brk = false;
    cdc.brk_until = 0;
    cdc.pending_state = 0;
}

/* ------------------------------------------------------------------------ */
/* Endpoints                                                                 */
/* ------------------------------------------------------------------------ */

static void cdc_read_out_ep(usbd_device* dev) {
    uint8_t buf[CDC_EP_SIZE];
    int32_t len = usbd_ep_read(dev, CDC_EP_OUT, buf, CDC_EP_SIZE);
    if(len > 0) furi_stream_buffer_send(cdc.rx_stream, buf, (size_t)len, 0);
}

/* SERIAL_STATE: DCD and DSR always on (the adapter is "connected"), plus any
 * error bits since the last report. Called from thread and interrupt. */
static void cdc_send_state(usbd_device* dev) {
    FURI_CRITICAL_ENTER();
    if(cdc.notify_busy || !cdc.configured) {
        cdc.notify_pending = true;
        FURI_CRITICAL_EXIT();
        return;
    }
    uint8_t bits = (uint8_t)(CDC_STATE_DCD | CDC_STATE_DSR | cdc.pending_state);
    if(cdc.brk) bits |= CDC_STATE_BREAK;
    cdc.pending_state = 0;
    cdc.notify_pending = false;
    cdc.notify_busy = true;
    FURI_CRITICAL_EXIT();

    uint8_t* n = cdc.notify_buf;
    n[0] = 0xA1; /* class, interface, device-to-host */
    n[1] = 0x20; /* SERIAL_STATE */
    n[2] = 0;
    n[3] = 0;
    n[4] = 0; /* interface 0 */
    n[5] = 0;
    n[6] = 2; /* wLength */
    n[7] = 0;
    n[8] = bits;
    n[9] = 0;
    if(usbd_ep_write(dev, CDC_EP_NOTIFY, n, sizeof(cdc.notify_buf)) < 0) cdc.notify_busy = false;
}

static void cdc_ep_callback(usbd_device* dev, uint8_t event, uint8_t ep) {
    if(ep == CDC_EP_NOTIFY) {
        if(event == usbd_evt_eptx) {
            cdc.notify_busy = false;
            if(cdc.notify_pending) cdc_send_state(dev);
        }
        return;
    }
    if(event == usbd_evt_eptx) {
        cdc.tx_busy = false;
        cdc_notify(USB_UART_FLAG_TX);
    } else if(event == usbd_evt_eprx) {
        /* Leave the endpoint NAKing when there is no room: USB backpressure */
        if(furi_stream_buffer_spaces_available(cdc.rx_stream) >= CDC_EP_SIZE) {
            cdc_read_out_ep(dev);
        } else {
            cdc.rx_deferred = true;
        }
        cdc_notify(USB_UART_FLAG_RX);
    }
}

static usbd_respond cdc_ep_config(usbd_device* dev, uint8_t cfg) {
    switch(cfg) {
    case 0:
        usbd_ep_deconfig(dev, CDC_EP_IN);
        usbd_ep_deconfig(dev, CDC_EP_OUT);
        usbd_ep_deconfig(dev, CDC_EP_NOTIFY);
        usbd_reg_endpoint(dev, CDC_EP_IN, 0);
        usbd_reg_endpoint(dev, CDC_EP_OUT, 0);
        usbd_reg_endpoint(dev, CDC_EP_NOTIFY, 0);
        cdc.configured = false;
        cdc.tx_busy = false;
        cdc.rx_deferred = false;
        cdc.notify_busy = false;
        cdc_line_defaults();
        cdc_notify(USB_UART_FLAG_CONFIG);
        return usbd_ack;
    case 1:
        usbd_ep_config(dev, CDC_EP_IN, USB_EPTYPE_BULK, CDC_EP_SIZE);
        usbd_ep_config(dev, CDC_EP_OUT, USB_EPTYPE_BULK, CDC_EP_SIZE);
        usbd_ep_config(dev, CDC_EP_NOTIFY, USB_EPTYPE_INTERRUPT, CDC_NOTIFY_SIZE);
        usbd_reg_endpoint(dev, CDC_EP_IN, cdc_ep_callback);
        usbd_reg_endpoint(dev, CDC_EP_OUT, cdc_ep_callback);
        usbd_reg_endpoint(dev, CDC_EP_NOTIFY, cdc_ep_callback);
        cdc.configured = true;
        cdc.tx_busy = false;
        cdc.notify_busy = false;
        cdc_notify(USB_UART_FLAG_CONFIG);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

/* ------------------------------------------------------------------------ */
/* Class requests                                                            */
/* ------------------------------------------------------------------------ */

static usbd_respond cdc_class_request(usbd_device* dev, usbd_ctlreq* req) {
    if((req->bmRequestType & 0x7F) != 0x21) return usbd_fail; /* class, interface */
    if((req->wIndex & 0xFF) != 0) return usbd_fail; /* communications interface only */

    switch(req->bRequest) {
    case CDC_SET_LINE_CODING: {
        if(req->wLength < 7) return usbd_fail;
        const uint8_t* d = req->data;
        uint32_t baud = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) |
                        ((uint32_t)d[3] << 24);
        uint8_t stop = d[4]; /* 0 = 1, 1 = 1.5, 2 = 2 stop bits */
        uint8_t parity = d[5]; /* none, odd, even, mark, space */
        uint8_t bits = d[6];
        if(baud == 0 || stop > 2 || parity > 4 || bits < 5 || bits > 8) return usbd_fail;
        /* Same encoding as CP210x SET_LINE_CTL */
        cdc.baud = baud;
        cdc.line_ctl = (uint16_t)(((uint16_t)bits << 8) | ((uint16_t)parity << 4) | stop);
        cdc_notify(USB_UART_FLAG_LINE);
        return usbd_ack;
    }
    case CDC_GET_LINE_CODING: {
        uint32_t baud = cdc.baud;
        uint16_t lc = cdc.line_ctl;
        cdc.ctl_buf[0] = (uint8_t)baud;
        cdc.ctl_buf[1] = (uint8_t)(baud >> 8);
        cdc.ctl_buf[2] = (uint8_t)(baud >> 16);
        cdc.ctl_buf[3] = (uint8_t)(baud >> 24);
        cdc.ctl_buf[4] = (uint8_t)(lc & 0x0F);
        cdc.ctl_buf[5] = (uint8_t)((lc >> 4) & 0x0F);
        cdc.ctl_buf[6] = (uint8_t)(lc >> 8);
        uint16_t len = req->wLength < 7 ? req->wLength : 7;
        dev->status.data_ptr = cdc.ctl_buf;
        dev->status.data_count = len;
        return usbd_ack;
    }
    case CDC_SET_CONTROL_LINE_STATE: {
        bool dtr = (req->wValue & 0x01) != 0;
        bool open = dtr && !cdc.dtr;
        cdc.dtr = dtr;
        cdc.rts = (req->wValue & 0x02) != 0;
        cdc_notify(USB_UART_FLAG_LINE);
        /* Report carrier when the host opens the port */
        if(open) cdc.notify_pending = true;
        return usbd_ack;
    }
    case CDC_SEND_BREAK:
        if(req->wValue == 0) {
            cdc.brk = false;
            cdc.brk_until = 0;
        } else {
            cdc.brk = true;
            cdc.brk_until = req->wValue == 0xFFFF ?
                                0 :
                                furi_get_tick() + furi_ms_to_ticks(req->wValue) + 1;
        }
        cdc_notify(USB_UART_FLAG_LINE);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

static usbd_respond cdc_control(usbd_device* dev, usbd_ctlreq* req, usbd_rqc_callback* callback) {
    UNUSED(callback);
    usbd_respond r = cdc_class_request(dev, req);
    if((req->bmRequestType & 0x60) == 0x20) {
        UsbUartCtlLog* e = &cdc_log[cdc_log_total % USB_UART_LOG_SIZE];
        e->request = req->bRequest;
        e->in = (req->bmRequestType & 0x80) != 0;
        e->stalled = (r == usbd_fail);
        e->value = req->wValue;
        e->length = req->wLength;
        cdc_log_total++;
        if(r == usbd_fail) cdc_log_stalls++;
    }
    return r;
}

/* ------------------------------------------------------------------------ */
/* FuriHal USB interface                                                     */
/* ------------------------------------------------------------------------ */

static void cdc_init(usbd_device* dev, FuriHalUsbInterface* intf, void* ctx) {
    UNUSED(intf);
    UNUSED(ctx);
    cdc.dev = dev;
    usbd_reg_config(dev, cdc_ep_config);
    usbd_reg_control(dev, cdc_control);
    usbd_connect(dev, true);
}

static void cdc_deinit(usbd_device* dev) {
    usbd_reg_config(dev, NULL);
    usbd_reg_control(dev, NULL);
    cdc.configured = false;
}

static void cdc_wakeup(usbd_device* dev) {
    UNUSED(dev);
}

static void cdc_suspend(usbd_device* dev) {
    UNUSED(dev);
}

static FuriHalUsbInterface cdc_usb_interface = {
    .init = cdc_init,
    .deinit = cdc_deinit,
    .wakeup = cdc_wakeup,
    .suspend = cdc_suspend,
    .dev_descr = &cdc_device_desc,
    .str_manuf_descr = (void*)&cdc_str_manuf,
    .str_prod_descr = (void*)&cdc_str_product,
    .str_serial_descr = (void*)&cdc_str_serial,
    .cfg_descr = (void*)cdc_cfg_desc,
};

/* ------------------------------------------------------------------------ */
/* Backend                                                                   */
/* ------------------------------------------------------------------------ */

static bool cdc_start(FuriThreadId notify_thread) {
    memset(&cdc, 0, sizeof(cdc));
    cdc_line_defaults();
    memset(cdc_log, 0, sizeof(cdc_log));
    cdc_log_total = 0;
    cdc_log_stalls = 0;
    cdc.thread = notify_thread;
    cdc.rx_stream = furi_stream_buffer_alloc(CDC_RX_STREAM_SIZE, 1);
    cdc.prev_config = furi_hal_usb_get_config();
    furi_hal_usb_unlock();
    if(!furi_hal_usb_set_config(&cdc_usb_interface, NULL)) {
        FURI_LOG_E(TAG, "USB config switch failed");
        furi_stream_buffer_free(cdc.rx_stream);
        cdc.rx_stream = NULL;
        cdc.thread = NULL;
        return false;
    }
    return true;
}

static void cdc_stop(void) {
    if(!cdc.rx_stream) return;
    furi_hal_usb_set_config(cdc.prev_config, NULL);
    cdc.thread = NULL;
    furi_stream_buffer_free(cdc.rx_stream);
    cdc.rx_stream = NULL;
}

static void cdc_get_line(UsbUartLine* line) {
    /* End a timed break */
    if(cdc.brk && cdc.brk_until && (int32_t)(furi_get_tick() - cdc.brk_until) >= 0) {
        cdc.brk = false;
        cdc.brk_until = 0;
    }
    FURI_CRITICAL_ENTER();
    line->configured = cdc.configured;
    line->enabled = cdc.configured; /* a CDC adapter's UART runs whenever it is configured */
    line->port_open = cdc.dtr; /* hosts raise DTR when they open the port */
    line->baud = cdc.baud;
    line->line_ctl = cdc.line_ctl;
    line->dtr = cdc.dtr;
    line->rts = cdc.rts;
    line->brk = cdc.brk;
    line->embed_escape = 0;
    FURI_CRITICAL_EXIT();
    if(cdc.notify_pending && cdc.dev) cdc_send_state(cdc.dev);
}

static size_t cdc_read(uint8_t* buf, size_t max) {
    if(!cdc.rx_stream) return 0;
    size_t n = furi_stream_buffer_receive(cdc.rx_stream, buf, max, 0);
    if(cdc.rx_deferred && cdc.dev &&
       furi_stream_buffer_spaces_available(cdc.rx_stream) >= CDC_EP_SIZE) {
        cdc.rx_deferred = false;
        cdc_read_out_ep(cdc.dev);
    }
    return n;
}

static bool cdc_take_imm_char(uint8_t* c) {
    UNUSED(c);
    return false; /* CDC has no transmit-immediate request */
}

static uint32_t cdc_take_purge(void) {
    return 0; /* CDC has no purge request */
}

static bool cdc_tx_ready(void) {
    return cdc.configured && !cdc.tx_busy;
}

static bool cdc_write(const uint8_t* buf, size_t len) {
    if(!cdc_tx_ready() || !cdc.dev) return false;
    if(len > CDC_EP_SIZE - 1) len = CDC_EP_SIZE - 1;
    cdc.tx_busy = true;
    if(usbd_ep_write(cdc.dev, CDC_EP_IN, buf, (uint16_t)len) < 0) {
        cdc.tx_busy = false;
        return false;
    }
    return true;
}

static void cdc_set_queue_levels(uint32_t to_host, uint32_t to_uart) {
    UNUSED(to_host);
    UNUSED(to_uart);
}

static void cdc_report_error(uint32_t error_bits) {
    uint8_t bits = 0;
    if(error_bits & USB_UART_ERROR_FRAMING) bits |= CDC_STATE_FRAMING;
    if(error_bits & USB_UART_ERROR_PARITY) bits |= CDC_STATE_PARITY;
    if(error_bits & (USB_UART_ERROR_OVERRUN | USB_UART_ERROR_QUEUE)) bits |= CDC_STATE_OVERRUN;
    if(!bits) return;
    FURI_CRITICAL_ENTER();
    cdc.pending_state |= bits;
    cdc.notify_pending = true;
    FURI_CRITICAL_EXIT();
}

static size_t cdc_get_log(UsbUartCtlLog* out, size_t max) {
    FURI_CRITICAL_ENTER();
    uint32_t total = cdc_log_total;
    size_t n = total < USB_UART_LOG_SIZE ? total : USB_UART_LOG_SIZE;
    if(n > max) n = max;
    for(size_t i = 0; i < n; i++) {
        out[i] = cdc_log[(total - n + i) % USB_UART_LOG_SIZE];
    }
    FURI_CRITICAL_EXIT();
    return n;
}

static void cdc_get_counts(uint32_t* total, uint32_t* stalled) {
    *total = cdc_log_total;
    *stalled = cdc_log_stalls;
}

static const char* cdc_request_name(uint8_t r) {
    switch(r) {
    case CDC_SET_LINE_CODING:
        return "SLINECOD";
    case CDC_GET_LINE_CODING:
        return "GLINECOD";
    case CDC_SET_CONTROL_LINE_STATE:
        return "SCTLLINE";
    case CDC_SEND_BREAK:
        return "BREAK";
    default:
        return "?";
    }
}

const UsbUartBackend usb_uart_cdc = {
    .name = "CDC",
    .max_write = CDC_EP_SIZE - 1,
    .start = cdc_start,
    .stop = cdc_stop,
    .get_line = cdc_get_line,
    .read = cdc_read,
    .take_imm_char = cdc_take_imm_char,
    .take_purge = cdc_take_purge,
    .tx_ready = cdc_tx_ready,
    .write = cdc_write,
    .set_queue_levels = cdc_set_queue_levels,
    .report_error = cdc_report_error,
    .get_log = cdc_get_log,
    .get_counts = cdc_get_counts,
    .request_name = cdc_request_name,
};
