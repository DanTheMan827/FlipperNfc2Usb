/**
 * PC/SC contactless reader personalities (USB CCID) backed by the Flipper's
 * NFC front-end: ACS ACR122U and ACS ACR1552U.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ReaderModelAcr122u,
    ReaderModelAcr1552u,
} ReaderModel;

typedef struct {
    bool usb_ok;
    bool usb_configured;
    bool field_on;
    bool card_present;
    bool card_powered;
    bool crypto_on;
    bool buzzer_enabled;
    char card[24];
    char uid[24];
    char last[40];
    uint32_t commands;
} ReaderStatus;

typedef struct Reader Reader;

Reader* reader_alloc(ReaderModel model);
bool reader_start(Reader* reader);
void reader_free(Reader* reader);

void reader_get_status(Reader* reader, ReaderStatus* status);
/** Allow or mute the Flipper speaker used as the reader's buzzer. */
void reader_set_buzzer(Reader* reader, bool enabled);

const char* reader_model_name(ReaderModel model);
