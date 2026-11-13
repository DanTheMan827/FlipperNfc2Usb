/**
 * Behavioural model of an NXP MFRC522 operating with its UART host interface.
 *
 * Single-threaded: every function must be called from the same thread.
 * RF operations are asynchronous: the model calls hooks.rf_submit() and the
 * owner later feeds the outcome back with mfrc522_rf_complete().
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "rf.h"

typedef struct {
    /** One byte leaves the MFRC522 TX pin (towards the host). */
    void (*uart_tx)(uint8_t byte, void* ctx);
    /** Antenna drivers on/off. */
    void (*set_field)(bool on, void* ctx);
    /** Start an RF exchange. */
    void (*rf_submit)(const RfRequest* req, void* ctx);
    /** Monotonic microsecond clock. */
    uint64_t (*now_us)(void* ctx);
    void* ctx;
} Mfrc522Hooks;

typedef struct {
    uint8_t command;
    uint8_t serial_speed;
    uint32_t uart_baud;
    bool field_on;
    bool crypto_on;
    bool soft_power_down;
    uint32_t reg_reads;
    uint32_t reg_writes;
    uint32_t rf_frames;
    uint32_t rf_timeouts;
    char last_rf[40];
} Mfrc522Status;

typedef struct Mfrc522 Mfrc522;

Mfrc522* mfrc522_alloc(const Mfrc522Hooks* hooks);
void mfrc522_free(Mfrc522* m);

/** Value returned by VersionReg (0x91 = v1.0, 0x92 = v2.0, 0x88 = FM17522 ...). */
void mfrc522_set_version(Mfrc522* m, uint8_t version);

/** Power-on reset (NRSTPD toggled / supply cycled). */
void mfrc522_hard_reset(Mfrc522* m);

/** A byte arrived on the MFRC522 RX pin. */
void mfrc522_uart_rx(Mfrc522* m, uint8_t byte);

/** Outcome of a request issued through hooks.rf_submit(). */
void mfrc522_rf_complete(Mfrc522* m, const RfResult* result);

/** Advance timers. Call regularly and whenever mfrc522_next_event_us() is reached. */
void mfrc522_tick(Mfrc522* m);

/** Absolute time (hooks.now_us base) of the next internal event, UINT64_MAX if none. */
uint64_t mfrc522_next_event_us(Mfrc522* m);

/** Current UART speed selected by SerialSpeedReg, in baud (rounded). */
uint32_t mfrc522_uart_baud(Mfrc522* m);

void mfrc522_get_status(Mfrc522* m, Mfrc522Status* status);

const char* mfrc522_command_name(uint8_t command);
