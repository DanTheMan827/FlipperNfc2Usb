/**
 * Common interface of the USB-to-UART adapter personalities the simulated
 * MFRC522 can sit behind (Silicon Labs CP2102, generic USB CDC-ACM).
 */
#pragma once

#include <furi.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Thread flags raised on the notify thread */
#define USB_UART_FLAG_RX     (1U << 0) /* data from host available */
#define USB_UART_FLAG_TX     (1U << 1) /* IN endpoint free again */
#define USB_UART_FLAG_CONFIG (1U << 2) /* USB configured / deconfigured */
#define USB_UART_FLAG_LINE   (1U << 3) /* line settings / purge / enable changed */

/* Purge bits as seen by the UART side */
#define USB_UART_PURGE_TX (1U << 0) /* host -> UART queue */
#define USB_UART_PURGE_RX (1U << 1) /* UART -> host queue */

/* Line error bits */
#define USB_UART_ERROR_BREAK   0x01
#define USB_UART_ERROR_FRAMING 0x02
#define USB_UART_ERROR_OVERRUN 0x04
#define USB_UART_ERROR_QUEUE   0x08
#define USB_UART_ERROR_PARITY  0x10

typedef struct {
    bool configured; /* USB configuration selected by host */
    bool enabled; /* adapter UART running (data flows) */
    bool port_open; /* host has the port open, as far as the adapter can tell */
    uint32_t baud;
    uint16_t line_ctl; /* [15:8] data bits, [7:4] parity, [3:0] stop bits (CP210x layout) */
    bool dtr;
    bool rts;
    bool brk;
    uint8_t embed_escape; /* CP210x embedded events, 0 = off */
} UsbUartLine;

/* Diagnostics: the last class / vendor control requests the host sent */
#define USB_UART_LOG_SIZE 8
typedef struct {
    uint8_t request; /* bRequest */
    bool in; /* device-to-host */
    bool stalled; /* answered with a STALL */
    uint16_t value;
    uint16_t length;
} UsbUartCtlLog;

typedef struct {
    const char* name; /* short name for menus, e.g. "CDC" */
    /* Bytes per write: 63 keeps every CDC packet short, so hosts that read
     * with large buffers get data at once instead of waiting for more */
    uint8_t max_write;
    /** Switch the Flipper USB port to this personality. */
    bool (*start)(FuriThreadId notify_thread);
    /** Restore the previous USB personality. */
    void (*stop)(void);
    void (*get_line)(UsbUartLine* line);
    /** Read bytes the host sent. */
    size_t (*read)(uint8_t* buf, size_t max);
    /** Pending transmit-immediate character, if the adapter has such a request. */
    bool (*take_imm_char)(uint8_t* c);
    /** Purge bits requested since the last call (USB_UART_PURGE_*). */
    uint32_t (*take_purge)(void);
    bool (*tx_ready)(void);
    /** Send up to 64 bytes to the host. */
    bool (*write)(const uint8_t* buf, size_t len);
    /** Queue depths the adapter reports to the host. */
    void (*set_queue_levels)(uint32_t to_host, uint32_t to_uart);
    /** Line errors to report to the host (USB_UART_ERROR_*). */
    void (*report_error)(uint32_t error_bits);
    /** Copy the request log, oldest first. */
    size_t (*get_log)(UsbUartCtlLog* out, size_t max);
    void (*get_counts)(uint32_t* total, uint32_t* stalled);
    /** Short name of a logged request. */
    const char* (*request_name)(uint8_t request);
} UsbUartBackend;

extern const UsbUartBackend usb_uart_cp2102;
extern const UsbUartBackend usb_uart_cdc;
