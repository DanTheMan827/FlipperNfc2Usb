/**
 * USB device personality of a Silicon Labs CP2102 USB-to-UART bridge
 * (VID 10C4, PID EA60), including its vendor-specific control protocol (AN571).
 * Used through the generic UsbUartBackend interface (usb_uart_cp2102).
 */
#pragma once

#include "usb_uart.h"

/* CP210x names for the shared flags and error bits */
#define CP210X_FLAG_RX     USB_UART_FLAG_RX
#define CP210X_FLAG_TX     USB_UART_FLAG_TX
#define CP210X_FLAG_CONFIG USB_UART_FLAG_CONFIG
#define CP210X_FLAG_LINE   USB_UART_FLAG_LINE

#define CP210X_PURGE_TX USB_UART_PURGE_TX
#define CP210X_PURGE_RX USB_UART_PURGE_RX

#define CP210X_ERROR_BREAK   USB_UART_ERROR_BREAK
#define CP210X_ERROR_FRAMING USB_UART_ERROR_FRAMING
#define CP210X_ERROR_OVERRUN USB_UART_ERROR_OVERRUN
#define CP210X_ERROR_QUEUE   USB_UART_ERROR_QUEUE
#define CP210X_ERROR_PARITY  USB_UART_ERROR_PARITY

#define CP210X_LOG_SIZE USB_UART_LOG_SIZE
typedef UsbUartLine Cp210xLine;
typedef UsbUartCtlLog Cp210xCtlLog;

bool cp210x_usb_start(FuriThreadId notify_thread);
void cp210x_usb_stop(void);
void cp210x_usb_get_line(Cp210xLine* line);
size_t cp210x_usb_read(uint8_t* buf, size_t max);
bool cp210x_usb_take_imm_char(uint8_t* c);
uint32_t cp210x_usb_take_purge(void);
bool cp210x_usb_tx_ready(void);
bool cp210x_usb_write(const uint8_t* buf, size_t len);
void cp210x_usb_set_queue_levels(uint32_t to_host, uint32_t to_uart);
void cp210x_usb_report_error(uint32_t error_bits);
size_t cp210x_usb_get_log(Cp210xCtlLog* out, size_t max);
void cp210x_usb_get_counts(uint32_t* total, uint32_t* stalled);
