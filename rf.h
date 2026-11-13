/**
 * RF front-end: executes MFRC522 RF operations on the Flipper's ST25R3916
 * through the firmware NFC library (ISO14443-A poller, 106 kbit/s).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Large enough for an ISO-DEP frame at FSD = 256 plus CRC */
#define RF_MAX_FRAME (260)

/** RF technology the front-end is configured for (Flipper NFC HAL modes). */
typedef enum {
    RfTechA, /* ISO14443-A, 106 kbit/s */
    RfTechB, /* ISO14443-B, 106 kbit/s (also ST25TB) */
    RfTechF, /* FeliCa, 212 kbit/s */
    RfTechV, /* ISO15693 (also Picopass) */
} RfTech;

typedef enum {
    RfReqTransceive, /**< Send FIFO frame, optionally listen for answer */
    RfReqAuth, /**< MIFARE Classic three-pass authentication (MFAuthent) */
} RfReqType;

typedef struct {
    uint32_t seq;
    RfReqType type;
    uint8_t tx[RF_MAX_FRAME];
    uint16_t tx_bits;
    bool listen; /**< receiver active (Transceive) or not (Transmit/RcvOff) */
    bool crypto; /**< Crypto1 is active: encrypt tx / decrypt rx */
    uint8_t rx_align;
    uint32_t fwt_fc; /**< frame wait time in carrier cycles (1/13.56 MHz) */
    /* MFAuthent parameters */
    uint8_t auth_cmd;
    uint8_t auth_block;
    uint8_t key[6];
    uint8_t uid[4];
} RfRequest;

typedef enum {
    RfStatusOk,
    RfStatusTimeout,
    RfStatusCollision,
    RfStatusParityError,
    RfStatusAuthFailed,
} RfStatus;

typedef struct {
    uint32_t seq;
    RfReqType type;
    RfStatus status;
    uint8_t rx[RF_MAX_FRAME];
    uint16_t rx_bits;
} RfResult;

typedef void (*RfCompleteCallback)(const RfResult* result, void* context);

typedef struct Rf Rf;

Rf* rf_alloc(RfCompleteCallback callback, void* context);
void rf_free(Rf* rf);

/** Switch the 13.56 MHz field. Turning it off completes queued requests with a timeout. */
void rf_set_field(Rf* rf, bool on);
bool rf_is_field_on(Rf* rf);

/** Select the RF technology. If the field is on it is cycled (cards lose state). */
void rf_set_tech(Rf* rf, RfTech tech);
RfTech rf_get_tech(Rf* rf);

/** Queue a request. Completion is always reported through the callback. */
void rf_submit(Rf* rf, const RfRequest* request);
