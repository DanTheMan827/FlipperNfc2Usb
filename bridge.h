/**
 * Glue between the CP2102 USB personality, the simulated UART wire and the
 * MFRC522 model. Runs its own worker thread.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t version; /* VersionReg value */
    bool strict_uart; /* drop bytes when host rate/framing differs from the chip's */
    bool realtime; /* pace bytes at the configured baud rate */
} BridgeSettings;

typedef struct {
    bool usb_ok;
    bool usb_configured;
    bool port_open;
    uint32_t host_baud;
    uint16_t line_ctl;
    uint32_t chip_baud;
    uint8_t serial_speed;
    bool link_ok;
    bool strict; /* Strict UART: a rate/format mismatch garbles bytes */
    bool over_spec; /* host or chip rate above the MFRC522's 1228.8 kBd maximum */
    bool field_on;
    bool crypto_on;
    bool soft_power_down;
    uint8_t command;
    uint32_t bytes_from_host;
    uint32_t bytes_to_host;
    uint32_t framing_errors;
    uint32_t rf_frames;
    uint32_t rf_timeouts;
    char last_rf[40];
} BridgeStatus;

typedef struct Bridge Bridge;

Bridge* bridge_alloc(const BridgeSettings* settings);
/** Starts the worker and switches USB to the CP2102 personality. */
bool bridge_start(Bridge* bridge);
void bridge_free(Bridge* bridge);

void bridge_get_status(Bridge* bridge, BridgeStatus* status);
void bridge_set_settings(Bridge* bridge, const BridgeSettings* settings);
/** Equivalent of pulsing NRSTPD low (hard power-down / reset). */
void bridge_power_cycle(Bridge* bridge);

/** Format CP210x line control as e.g. "8N1". */
void bridge_format_line(uint16_t line_ctl, char* out, uint32_t out_size);
