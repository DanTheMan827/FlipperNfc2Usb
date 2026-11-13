#pragma once

#include "reader.h"
#include "picc.h"
#include "rf.h"
#include "ccid_usb.h"

#include <furi.h>
#include <notification/notification.h>

#define READER_RESP_MAX (CCID_MAX_DATA)

#define READER_IFACE_PICC 0
#define READER_IFACE_SAM  1

/* PN532 state behind the ACR122U */
typedef struct {
    uint8_t ciu[64]; /* CIU registers 0x6300..0x633F */
    uint8_t params; /* SetParameters flags */
    bool target_active;
    uint8_t target_br; /* 00 106, 01 212, 02 424 */
    uint8_t target_type; /* 00 A/B, 10 FeliCa, 02 Jewel */
    uint8_t rf_retries; /* MxRtyPassiveActivation */
    uint8_t retry_timeout; /* fRetryTimeout code (RFConfiguration item 2) */
    uint8_t last_error;
} Pn532;

/* ACR1552U PC/SC 2.0 Part 3 transparent session */
typedef struct {
    bool active;
    uint8_t flags; /* Transceive flag data object */
    uint8_t tx_bits; /* Transmit bit framing */
    uint32_t timeout_us; /* 0 = use FWTI */
    uint8_t fwti;
    uint8_t retry;
    uint8_t layer; /* 2, 3 or 4 */
    uint8_t baud;
} TransparentSession;

struct Reader {
    ReaderModel model;
    FuriThread* thread;
    FuriMutex* mutex;
    FuriMessageQueue* rf_results;
    Rf* rf;
    Picc picc;
    NotificationApp* notification;

    bool usb_ok;
    bool configured;
    bool field_wanted; /* host antenna control */
    bool field_on;

    bool card_present;
    bool card_powered;
    uint16_t card_name; /* PC/SC Part 3 card name for storage cards */
    uint8_t atr[40];
    uint8_t atr_len;

    uint8_t keys[2][22];
    uint8_t key_len[2];

    uint32_t direct_mode_until_tick; /* host drives the PN532 directly */
    uint32_t next_poll_tick;

    /* UI / indicators */
    bool buzzer_enabled; /* user setting on the Flipper */
    uint8_t led_state; /* ACR122U: bit0 red, bit1 green. ACR1552U: bit0 blue, bit1 green */
    bool beeping;
    uint32_t beep_off_tick;

    /* ACR122U settings */
    uint8_t acr122_picc_param; /* FF 00 51 */
    uint8_t acr122_timeout;
    bool acr122_beep_on_detect;

    /* ACR1552U settings */
    uint8_t rf_mode; /* E0 00 00 25: 00 off, 01 on + polling, 02 on without polling */
    uint8_t polling_option; /* E0 00 00 23 */
    uint16_t polling_type; /* E0 00 01 20: byte1 << 8 | byte0 */
    uint8_t auto_pps;
    uint8_t ui_behaviour;
    uint8_t rf_power;
    uint8_t selective_suspend;
    uint8_t sn_in_descriptor;
    TransparentSession session;

    Pn532 pn532;

    uint8_t msg[CCID_MAX_MSG];
    uint8_t out[CCID_MAX_MSG];

    ReaderStatus status;
    char last[40];
    uint32_t commands;
    uint32_t losses; /* cards dropped by presence check / power-on */
};

/* reader.c helpers shared with pn532.c */
void reader_set_field(Reader* r, bool on);
void reader_card_activated(Reader* r);
void reader_card_lost(Reader* r);

/* pn532.c */
void pn532_reset(Pn532* pn);
/** Handle a PN532 frame body (D4 cmd ...). Returns response length (D5 ...), 0 on error. */
size_t pn532_command(Reader* r, const uint8_t* cmd, size_t len, uint8_t* resp, size_t resp_max);
