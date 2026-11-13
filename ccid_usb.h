/**
 * USB CCID (smart card reader class) device with one or two CCID interfaces
 * (e.g. PICC + SAM), each with one slot. Descriptors are parameterised so the
 * device can present itself as a specific commercial reader.
 */
#pragma once

#include <furi.h>
#include <stdint.h>
#include <stdbool.h>

#define CCID_FLAG_MSG    (1U << 0) /* a complete PC_to_RDR message is available */
#define CCID_FLAG_TX     (1U << 1) /* a response finished sending */
#define CCID_FLAG_CONFIG (1U << 2) /* configured / deconfigured */

#define CCID_HEADER_SIZE (10)
#define CCID_MAX_DATA    (300)
#define CCID_MAX_MSG     (CCID_HEADER_SIZE + CCID_MAX_DATA)
#define CCID_MAX_IFACES  (2)

typedef struct {
    uint16_t vid;
    uint16_t pid;
    uint16_t bcd_device;
    uint16_t bcd_usb;
    uint16_t max_power_ma;
    const void* str_manufacturer; /* struct usb_string_descriptor* */
    const void* str_product;
    const void* str_serial; /* may be NULL */
    uint8_t interfaces; /* 1 or 2 */
    const void* str_interface[CCID_MAX_IFACES]; /* iInterface strings, may be NULL */
    /* CCID class descriptor fields (shared by all interfaces) */
    uint8_t voltage_support;
    uint32_t protocols;
    uint32_t default_clock;
    uint32_t max_clock;
    uint32_t data_rate;
    uint32_t max_data_rate;
    uint32_t max_ifsd;
    uint32_t features;
    uint32_t max_message_length;
} CcidUsbConfig;

bool ccid_usb_start(const CcidUsbConfig* config, FuriThreadId notify_thread);
void ccid_usb_stop(void);

bool ccid_usb_is_configured(void);

/** Copy out a complete PC_to_RDR message (header + data) of an interface. 0 if none. */
size_t ccid_usb_take_message(uint8_t iface, uint8_t* buf, size_t max);

/** Queue a full RDR_to_PC message. Fails while a previous one is still sending. */
bool ccid_usb_send(uint8_t iface, const uint8_t* msg, size_t len);
bool ccid_usb_tx_busy(uint8_t iface);

/** RDR_to_PC_NotifySlotChange on the interface's interrupt endpoint. */
void ccid_usb_notify_slot(uint8_t iface, bool present, bool changed);
