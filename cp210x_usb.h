/**
 * USB device personality of a Silicon Labs CP2102 USB-to-UART bridge
 * (VID 10C4, PID EA60), including its vendor-specific control protocol (AN571).
 */
#pragma once

#include <furi.h>
#include <stdint.h>
#include <stdbool.h>

/* Thread flags raised on the notify thread */
#define CP210X_FLAG_RX     (1U << 0) /* data from host available */
#define CP210X_FLAG_TX     (1U << 1) /* IN endpoint free again */
#define CP210X_FLAG_CONFIG (1U << 2) /* USB configured / deconfigured */
#define CP210X_FLAG_LINE   (1U << 3) /* line settings / purge / enable changed */

/* PURGE bits as seen by the UART side */
#define CP210X_PURGE_TX (1U << 0) /* host -> UART queue */
#define CP210X_PURGE_RX (1U << 1) /* UART -> host queue */

/* Line status bits for embedded events / comm status */
#define CP210X_ERROR_BREAK   0x01
#define CP210X_ERROR_FRAMING 0x02
#define CP210X_ERROR_OVERRUN 0x04
#define CP210X_ERROR_QUEUE   0x08
#define CP210X_ERROR_PARITY  0x10

typedef struct {
    bool configured; /* USB configuration selected by host */
    bool enabled; /* IFC_ENABLE */
    uint32_t baud;
    uint16_t line_ctl; /* [15:8] data bits, [7:4] parity, [3:0] stop bits */
    bool dtr;
    bool rts;
    bool brk;
    uint8_t embed_escape; /* 0 = embedded events disabled */
} Cp210xLine;

/** Switch the Flipper USB port to the CP2102 personality. */
bool cp210x_usb_start(FuriThreadId notify_thread);
/** Restore the previous USB personality. */
void cp210x_usb_stop(void);

void cp210x_usb_get_line(Cp210xLine* line);

/** Read bytes the host sent (OUT endpoint). */
size_t cp210x_usb_read(uint8_t* buf, size_t max);
/** Pending IMM_CHAR request, returns true and the char if there is one. */
bool cp210x_usb_take_imm_char(uint8_t* c);
/** PURGE bits requested since the last call (CP210X_PURGE_*). */
uint32_t cp210x_usb_take_purge(void);

bool cp210x_usb_tx_ready(void);
/** Send up to 64 bytes on the IN endpoint. */
bool cp210x_usb_write(const uint8_t* buf, size_t len);

/** Queue depths reported by GET_COMM_STATUS. */
void cp210x_usb_set_queue_levels(uint32_t to_host, uint32_t to_uart);
/** Latch line errors reported by GET_COMM_STATUS (CP210X_ERROR_*). */
void cp210x_usb_report_error(uint32_t error_bits);

/* Diagnostics: the last vendor control requests the host sent. */
#define CP210X_LOG_SIZE 8
typedef struct {
    uint8_t request; /* bRequest */
    bool in; /* device-to-host */
    bool stalled; /* answered with a STALL */
    uint16_t value;
    uint16_t length;
} Cp210xCtlLog;

/** Copy the log, oldest first. Returns the number of entries. */
size_t cp210x_usb_get_log(Cp210xCtlLog* out, size_t max);
/** Total vendor requests and how many were stalled since start. */
void cp210x_usb_get_counts(uint32_t* total, uint32_t* stalled);
