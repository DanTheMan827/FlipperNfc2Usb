#include "bridge.h"
#include "cp210x_usb.h"
#include "mfrc522.h"
#include "rf.h"

#include <furi.h>
#include <furi_hal_cortex.h>
#include <string.h>
#include <stdio.h>

#define TAG "Mfrc522Bridge"

#define BF_RF_DONE  (1U << 4)
#define BF_POWER    (1U << 5)
#define BF_SETTINGS (1U << 6)
#define BF_STOP     (1U << 7)
#define BF_ALL      (CP210X_FLAG_RX | CP210X_FLAG_TX | CP210X_FLAG_CONFIG | CP210X_FLAG_LINE | \
                BF_RF_DONE | BF_POWER | BF_SETTINGS | BF_STOP)

#define WIRE_SIZE        256
#define IDLE_WAIT_MS     50
#define BAUD_TOLERANCE_PERMILLE 30 /* +/-3 % between host and chip clocks */
/* Fastest UART rate in the MFRC522 datasheet: 1228.8 kBd nominal, which is
 * SerialSpeedReg = 15h -> 27.12 MHz / 22 = 1 232 727 Bd actual. */
#define MFRC522_UART_MAX_BAUD 1232727U

/* One direction of the simulated serial line. Each byte carries the time at
 * which its stop bit has been shifted out completely. */
typedef struct {
    uint8_t data[WIRE_SIZE];
    uint8_t err[WIRE_SIZE];
    uint64_t due_ns[WIRE_SIZE];
    uint16_t head;
    uint16_t count;
    uint64_t last_due_ns;
} Wire;

struct Bridge {
    FuriThread* thread;
    FuriMutex* mutex;
    FuriMessageQueue* rf_results;
    Mfrc522* chip;
    Rf* rf;

    BridgeSettings settings; /* shared, guarded by mutex */
    BridgeSettings active; /* worker copy */
    BridgeStatus status; /* shared, guarded by mutex */

    Wire to_chip;
    Wire to_host;
    Cp210xLine line;

    uint32_t last_cyc;
    uint32_t acc_cyc;
    uint64_t now_us;

    bool usb_ok;
    bool was_configured;
    uint32_t bytes_from_host;
    uint32_t bytes_to_host;
    uint32_t framing_errors;
};

/* ------------------------------------------------------------------------ */
/* Time                                                                      */
/* ------------------------------------------------------------------------ */

static uint64_t bridge_clock_us(Bridge* b) {
    uint32_t cyc = furi_hal_cortex_timer_get(0).start; /* DWT cycle counter */
    uint32_t delta = cyc - b->last_cyc;
    b->last_cyc = cyc;
    b->acc_cyc += delta;
    uint32_t per_us = furi_hal_cortex_instructions_per_microsecond();
    uint32_t add = b->acc_cyc / per_us;
    b->acc_cyc -= add * per_us;
    b->now_us += add;
    return b->now_us;
}

/* ------------------------------------------------------------------------ */
/* Wire helpers                                                              */
/* ------------------------------------------------------------------------ */

static void wire_clear(Wire* w) {
    w->head = 0;
    w->count = 0;
    w->last_due_ns = 0;
}

static bool wire_push(Wire* w, uint8_t data, bool err, uint64_t now_ns, uint64_t char_ns) {
    if(w->count >= WIRE_SIZE) return false;
    uint64_t start = (w->last_due_ns > now_ns) ? w->last_due_ns : now_ns;
    uint64_t due = start + char_ns;
    uint16_t idx = (uint16_t)((w->head + w->count) % WIRE_SIZE);
    w->data[idx] = data;
    w->err[idx] = err ? 1 : 0;
    w->due_ns[idx] = due;
    w->last_due_ns = due;
    w->count++;
    return true;
}

static void wire_pop(Wire* w) {
    w->head = (uint16_t)((w->head + 1) % WIRE_SIZE);
    w->count--;
}

static uint8_t line_data_bits(uint16_t line_ctl) {
    uint8_t d = (uint8_t)(line_ctl >> 8);
    return (d >= 5 && d <= 8) ? d : 8;
}

static uint8_t line_parity(uint16_t line_ctl) {
    return (uint8_t)((line_ctl >> 4) & 0x0F);
}

static uint8_t line_stop(uint16_t line_ctl) {
    return (uint8_t)(line_ctl & 0x0F);
}

/* Duration of one character on the wire at the host's configured format. */
static uint64_t bridge_char_ns(Bridge* b) {
    if(!b->active.realtime || b->line.baud == 0) return 0;
    uint16_t lc = b->line.line_ctl;
    uint32_t half_bits = 2U * (1U + line_data_bits(lc) + (line_parity(lc) ? 1U : 0U));
    uint8_t stop = line_stop(lc);
    half_bits += (stop == 0) ? 2U : (stop == 1) ? 3U : 4U;
    return (uint64_t)half_bits * 500000000ULL / b->line.baud;
}

/* Would a real UART pair at these settings understand each other? */
static bool bridge_link_ok(Bridge* b) {
    uint16_t lc = b->line.line_ctl;
    if(line_data_bits(lc) != 8 || line_parity(lc) != 0) return false;
    uint32_t chip = mfrc522_uart_baud(b->chip);
    uint32_t host = b->line.baud;
    uint32_t diff = (host > chip) ? host - chip : chip - host;
    return (uint64_t)diff * 1000U <= (uint64_t)chip * BAUD_TOLERANCE_PERMILLE;
}

/* Is either end of the wire running faster than the MFRC522 is specified for?
 * The host side is checked in both modes: even when the chip is assumed to
 * follow the host's rate, it cannot follow it past its maximum. The chip's own
 * SerialSpeedReg setting only matters in strict mode. */
static bool bridge_over_spec(Bridge* b) {
    uint64_t host_limit =
        (uint64_t)MFRC522_UART_MAX_BAUD * (1000U + BAUD_TOLERANCE_PERMILLE) / 1000U;
    if(b->line.baud > host_limit) return true;
    if(b->active.strict_uart && mfrc522_uart_baud(b->chip) > MFRC522_UART_MAX_BAUD) return true;
    return false;
}

/* Can a byte cross the wire intact right now? */
static bool bridge_line_works(Bridge* b) {
    if(bridge_over_spec(b)) return false;
    if(b->active.strict_uart && !bridge_link_ok(b)) return false;
    return true;
}

/* ------------------------------------------------------------------------ */
/* MFRC522 hooks                                                             */
/* ------------------------------------------------------------------------ */

static void hook_uart_tx(uint8_t byte, void* ctx) {
    Bridge* b = ctx;
    if(!b->line.configured || !b->line.enabled) return; /* adapter UART is off */
    bool err = !bridge_line_works(b);
    uint64_t now_ns = bridge_clock_us(b) * 1000U;
    if(!wire_push(&b->to_host, byte, err, now_ns, bridge_char_ns(b))) {
        cp210x_usb_report_error(CP210X_ERROR_QUEUE);
    }
}

static void hook_set_field(bool on, void* ctx) {
    Bridge* b = ctx;
    rf_set_field(b->rf, on);
}

static void hook_rf_submit(const RfRequest* req, void* ctx) {
    Bridge* b = ctx;
    rf_submit(b->rf, req);
}

static uint64_t hook_now_us(void* ctx) {
    return bridge_clock_us((Bridge*)ctx);
}

static void rf_done_callback(const RfResult* result, void* ctx) {
    Bridge* b = ctx;
    if(furi_message_queue_put(b->rf_results, result, 100) != FuriStatusOk) {
        FURI_LOG_E(TAG, "RF result dropped");
    }
    furi_thread_flags_set(furi_thread_get_id(b->thread), BF_RF_DONE);
}

/* ------------------------------------------------------------------------ */
/* Worker                                                                    */
/* ------------------------------------------------------------------------ */

static void bridge_power_cycle_chip(Bridge* b) {
    mfrc522_hard_reset(b->chip);
    wire_clear(&b->to_chip);
    wire_clear(&b->to_host);
}

static void bridge_pump_from_usb(Bridge* b, uint64_t now_ns) {
    uint64_t char_ns = bridge_char_ns(b);
    uint8_t c;
    if(cp210x_usb_take_imm_char(&c) && b->line.enabled) {
        wire_push(&b->to_chip, c, false, now_ns, char_ns);
    }
    uint8_t buf[64];
    while(b->to_chip.count < WIRE_SIZE - sizeof(buf)) {
        size_t n = cp210x_usb_read(buf, sizeof(buf));
        if(n == 0) break;
        for(size_t i = 0; i < n; i++) {
            wire_push(&b->to_chip, buf[i], false, now_ns, char_ns);
        }
    }
}

static void bridge_deliver_to_chip(Bridge* b) {
    while(b->to_chip.count) {
        uint64_t now_ns = bridge_clock_us(b) * 1000U;
        if(b->to_chip.due_ns[b->to_chip.head] > now_ns) break;
        uint8_t byte = b->to_chip.data[b->to_chip.head];
        wire_pop(&b->to_chip);
        b->bytes_from_host++;
        if(b->line.brk || !bridge_line_works(b)) {
            /* The MFRC522 UART sees a corrupted frame and discards it */
            b->framing_errors++;
            continue;
        }
        mfrc522_uart_rx(b->chip, byte);
    }
}

static void bridge_pump_to_usb(Bridge* b) {
    if(!b->to_host.count || !cp210x_usb_tx_ready()) return;
    uint64_t now_ns = bridge_clock_us(b) * 1000U;
    uint8_t esc = b->line.embed_escape;
    uint8_t pkt[64];
    size_t n = 0;
    while(b->to_host.count) {
        uint16_t h = b->to_host.head;
        if(b->to_host.due_ns[h] > now_ns) break;
        uint8_t d = b->to_host.data[h];
        bool err = b->to_host.err[h] != 0;
        size_t need = err ? (esc ? 4U : 0U) : ((esc && d == esc) ? 2U : 1U);
        if(n + need > sizeof(pkt)) break;
        if(err) {
            b->framing_errors++;
            cp210x_usb_report_error(CP210X_ERROR_FRAMING);
            if(esc) {
                /* Embedded event: ESC, 01, LSR (framing error), data */
                pkt[n++] = esc;
                pkt[n++] = 0x01;
                pkt[n++] = 0x08;
                pkt[n++] = 0x00;
            }
        } else if(esc && d == esc) {
            pkt[n++] = esc;
            pkt[n++] = 0x00;
        } else {
            pkt[n++] = d;
        }
        if(!err) b->bytes_to_host++;
        wire_pop(&b->to_host);
    }
    if(n) cp210x_usb_write(pkt, n);
}

static void bridge_publish_status(Bridge* b) {
    Mfrc522Status cs;
    mfrc522_get_status(b->chip, &cs);
    furi_mutex_acquire(b->mutex, FuriWaitForever);
    BridgeStatus* s = &b->status;
    s->usb_ok = b->usb_ok;
    s->usb_configured = b->line.configured;
    s->port_open = b->line.enabled;
    s->host_baud = b->line.baud;
    s->line_ctl = b->line.line_ctl;
    s->chip_baud = cs.uart_baud;
    s->serial_speed = cs.serial_speed;
    s->link_ok = bridge_link_ok(b);
    s->strict = b->active.strict_uart;
    s->over_spec = bridge_over_spec(b);
    s->field_on = cs.field_on;
    s->crypto_on = cs.crypto_on;
    s->soft_power_down = cs.soft_power_down;
    s->command = cs.command;
    s->bytes_from_host = b->bytes_from_host;
    s->bytes_to_host = b->bytes_to_host;
    s->framing_errors = b->framing_errors;
    s->rf_frames = cs.rf_frames;
    s->rf_timeouts = cs.rf_timeouts;
    memcpy(s->last_rf, cs.last_rf, sizeof(s->last_rf));
    furi_mutex_release(b->mutex);
}

static uint32_t bridge_wait_ms(Bridge* b) {
    uint64_t now_us = bridge_clock_us(b);
    uint64_t next_us = UINT64_MAX;
    if(b->to_chip.count) {
        next_us = b->to_chip.due_ns[b->to_chip.head] / 1000U;
    }
    if(b->to_host.count && cp210x_usb_tx_ready()) {
        uint64_t t = b->to_host.due_ns[b->to_host.head] / 1000U;
        if(t < next_us) next_us = t;
    }
    uint64_t chip_next = mfrc522_next_event_us(b->chip);
    if(chip_next < next_us) next_us = chip_next;

    if(next_us == UINT64_MAX) return IDLE_WAIT_MS;
    if(next_us <= now_us) return 0;
    uint64_t ms = (next_us - now_us + 999U) / 1000U;
    return ms > IDLE_WAIT_MS ? IDLE_WAIT_MS : (uint32_t)ms;
}

static int32_t bridge_worker(void* context) {
    Bridge* b = context;
    uint32_t wait_ms = 0;

    while(true) {
        uint32_t flags = furi_thread_flags_wait(BF_ALL, FuriFlagWaitAny, wait_ms);
        if(flags & FuriFlagError) flags = 0;
        if(flags & BF_STOP) break;

        if(flags & BF_SETTINGS) {
            furi_mutex_acquire(b->mutex, FuriWaitForever);
            b->active = b->settings;
            furi_mutex_release(b->mutex);
            mfrc522_set_version(b->chip, b->active.version);
        }

        cp210x_usb_get_line(&b->line);

        if(b->line.configured != b->was_configured) {
            /* Bus power applied / removed: the MFRC522 module is powered by the adapter */
            b->was_configured = b->line.configured;
            bridge_power_cycle_chip(b);
        }
        if(flags & BF_POWER) bridge_power_cycle_chip(b);

        uint32_t purge = cp210x_usb_take_purge();
        if(purge & CP210X_PURGE_TX) wire_clear(&b->to_chip);
        if(purge & CP210X_PURGE_RX) wire_clear(&b->to_host);
        if(!b->line.enabled) {
            wire_clear(&b->to_chip);
            wire_clear(&b->to_host);
        }

        RfResult res;
        while(furi_message_queue_get(b->rf_results, &res, 0) == FuriStatusOk) {
            mfrc522_rf_complete(b->chip, &res);
        }

        bridge_pump_from_usb(b, bridge_clock_us(b) * 1000U);
        bridge_deliver_to_chip(b);
        mfrc522_tick(b->chip);
        bridge_pump_to_usb(b);

        cp210x_usb_set_queue_levels(b->to_host.count, b->to_chip.count);
        bridge_publish_status(b);
        wait_ms = bridge_wait_ms(b);
    }

    rf_set_field(b->rf, false);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

Bridge* bridge_alloc(const BridgeSettings* settings) {
    Bridge* b = malloc(sizeof(Bridge));
    memset(b, 0, sizeof(Bridge));
    b->settings = *settings;
    b->active = *settings;
    b->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    b->rf_results = furi_message_queue_alloc(8, sizeof(RfResult));
    b->last_cyc = furi_hal_cortex_timer_get(0).start;

    b->rf = rf_alloc(rf_done_callback, b);

    Mfrc522Hooks hooks = {
        .uart_tx = hook_uart_tx,
        .set_field = hook_set_field,
        .rf_submit = hook_rf_submit,
        .now_us = hook_now_us,
        .ctx = b,
    };
    b->chip = mfrc522_alloc(&hooks);
    mfrc522_set_version(b->chip, settings->version);

    b->thread = furi_thread_alloc_ex("Mfrc522Bridge", 4 * 1024, bridge_worker, b);
    return b;
}

bool bridge_start(Bridge* b) {
    furi_thread_start(b->thread);
    b->usb_ok = cp210x_usb_start(furi_thread_get_id(b->thread));
    furi_thread_flags_set(furi_thread_get_id(b->thread), BF_SETTINGS);
    return b->usb_ok;
}

void bridge_free(Bridge* b) {
    furi_thread_flags_set(furi_thread_get_id(b->thread), BF_STOP);
    furi_thread_join(b->thread);
    furi_thread_free(b->thread);
    if(b->usb_ok) cp210x_usb_stop();
    rf_free(b->rf);
    mfrc522_free(b->chip);
    furi_message_queue_free(b->rf_results);
    furi_mutex_free(b->mutex);
    free(b);
}

void bridge_get_status(Bridge* b, BridgeStatus* status) {
    furi_mutex_acquire(b->mutex, FuriWaitForever);
    *status = b->status;
    status->usb_ok = b->usb_ok;
    furi_mutex_release(b->mutex);
}

void bridge_set_settings(Bridge* b, const BridgeSettings* settings) {
    furi_mutex_acquire(b->mutex, FuriWaitForever);
    b->settings = *settings;
    furi_mutex_release(b->mutex);
    furi_thread_flags_set(furi_thread_get_id(b->thread), BF_SETTINGS);
}

void bridge_power_cycle(Bridge* b) {
    furi_thread_flags_set(furi_thread_get_id(b->thread), BF_POWER);
}

void bridge_format_line(uint16_t line_ctl, char* out, uint32_t out_size) {
    static const char parity_chars[] = "NOEMS";
    uint8_t p = line_parity(line_ctl);
    uint8_t s = line_stop(line_ctl);
    snprintf(
        out,
        out_size,
        "%u%c%s",
        (unsigned)line_data_bits(line_ctl),
        p < 5 ? parity_chars[p] : '?',
        s == 0 ? "1" : (s == 1 ? "1.5" : "2"));
}
