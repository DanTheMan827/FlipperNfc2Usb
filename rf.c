#include "rf.h"
#include "crypto1.h"
#include "crc.h"

#include <furi.h>
#include <furi_hal_random.h>
#include <nfc/nfc.h>
#include <toolbox/bit_buffer.h>
#include <string.h>

#define TAG "Mfrc522Rf"

#define RF_QUEUE_LEN (4)

/* Per-technology timing, as used by the stock firmware pollers */
typedef struct {
    uint32_t guard_time_us;
    uint32_t fdt_poll_fc;
    uint32_t poll_poll_us;
    NfcTech nfc_tech;
} RfTechTiming;

static const RfTechTiming rf_timing[] = {
    [RfTechA] = {5000, 1620, 1100, NfcTechIso14443a},
    [RfTechB] = {5000, 9000, 1280, NfcTechIso14443b},
    [RfTechF] = {20000, 10000, 1280, NfcTechFelica},
    [RfTechV] = {5000, 4202, 1500, NfcTechIso15693},
};
/* When only transmitting (Transmit command / RcvOff) we still have to give the
 * HAL a wait time; keep it short, the answer is discarded anyway. */
#define RF_TX_ONLY_FWT_FC    (2000)

struct Rf {
    Nfc* nfc;
    FuriMessageQueue* queue;
    RfCompleteCallback callback;
    void* context;
    volatile bool running;
    volatile bool stop_requested;
    RfTech tech;
    BitBuffer* tx;
    BitBuffer* rx;
    Mfc1State crypto;
};

static uint32_t rf_be32(const uint8_t* d) {
    return ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
}

static bool rf_is_anticollision(const uint8_t* tx, uint16_t bits) {
    if(bits < 16) return false;
    if(tx[0] != 0x93 && tx[0] != 0x95 && tx[0] != 0x97) return false;
    return tx[1] != 0x70; /* NVB 0x70 is a full SELECT: standard frame */
}

static NfcError rf_trx_plain(Rf* rf, const uint8_t* tx, uint16_t bits, uint32_t fwt) {
    bit_buffer_reset(rf->rx);

    if(rf->tech != RfTechA) {
        /* B / FeliCa / 15693: SOF/EOF and coding by the HAL, CRC by the caller */
        bit_buffer_copy_bits(rf->tx, tx, bits);
        return nfc_poller_trx(rf->nfc, rf->tx, rf->rx, fwt);
    }

    if(bits == 7 && (tx[0] == 0x26 || tx[0] == 0x52)) {
        return nfc_iso14443a_poller_trx_short_frame(
            rf->nfc,
            tx[0] == 0x26 ? NfcIso14443aShortFrameSensReq : NfcIso14443aShortFrameAllReqa,
            rf->rx,
            fwt);
    }

    bit_buffer_copy_bits(rf->tx, tx, bits);
    if(rf_is_anticollision(tx, bits)) {
        return nfc_iso14443a_poller_trx_sdd_frame(rf->nfc, rf->tx, rf->rx, fwt);
    }
    return nfc_poller_trx(rf->nfc, rf->tx, rf->rx, fwt);
}

/* Encrypts `bits` of plaintext with the running Crypto1 state (including the
 * encrypted parity bits) and exchanges it. */
static NfcError rf_trx_crypto(Rf* rf, const uint8_t* tx, uint16_t bits, uint32_t fwt) {
    Mfc1State* s = &rf->crypto;
    bit_buffer_reset(rf->rx);

    if(bits < 8) {
        /* No parity bit exists in a frame shorter than one byte; the custom
         * parity path would always append one, so use the plain path. */
        uint8_t enc = 0;
        for(uint16_t i = 0; i < bits; i++) {
            enc |= (uint8_t)((mfc1_bit(s, 0, 0) ^ ((tx[0] >> i) & 1U)) << i);
        }
        bit_buffer_copy_bits(rf->tx, &enc, bits);
        return nfc_poller_trx(rf->nfc, rf->tx, rf->rx, fwt);
    } else {
        size_t n = bits / 8U;
        bit_buffer_reset(rf->tx);
        bit_buffer_set_size_bytes(rf->tx, n);
        for(size_t i = 0; i < n; i++) {
            uint8_t plain = tx[i];
            uint8_t enc = mfc1_byte(s, 0, 0) ^ plain;
            bool par = ((mfc1_peek(s) ^ mfc1_odd_parity8(plain)) & 1U) != 0;
            bit_buffer_set_byte_with_parity(rf->tx, i, enc, par);
        }
    }
    return nfc_iso14443a_poller_trx_custom_parity(rf->nfc, rf->tx, rf->rx, fwt);
}

static void rf_copy_rx(Rf* rf, RfResult* res, bool decrypt) {
    size_t bits = bit_buffer_get_size(rf->rx);
    if(bits > RF_MAX_FRAME * 8U) bits = RF_MAX_FRAME * 8U;
    size_t bytes = (bits + 7U) / 8U;
    const uint8_t* d = bit_buffer_get_data(rf->rx);
    memcpy(res->rx, d, bytes);
    res->rx_bits = (uint16_t)bits;

    if(!decrypt) return;
    Mfc1State* s = &rf->crypto;
    if(bits < 8) {
        uint8_t v = 0;
        for(size_t i = 0; i < bits; i++) {
            v |= (uint8_t)((mfc1_bit(s, 0, 0) ^ ((res->rx[0] >> i) & 1U)) << i);
        }
        res->rx[0] = v;
    } else {
        /* Encrypted frames are always byte oriented */
        size_t n = bits / 8U;
        for(size_t i = 0; i < n; i++) {
            res->rx[i] ^= mfc1_byte(s, 0, 0);
        }
        res->rx_bits = (uint16_t)(n * 8U);
    }
}

static bool rf_auth(Rf* rf, const RfRequest* req) {
    uint8_t cmd[4] = {req->auth_cmd, req->auth_block, 0, 0};
    crc_append(CrcTypeA, cmd, 2);

    uint32_t uid = rf_be32(req->uid);
    uint64_t key = 0;
    for(size_t i = 0; i < 6; i++) key = (key << 8) | req->key[i];

    const bool nested = req->crypto;
    NfcError err;
    if(nested) {
        err = rf_trx_crypto(rf, cmd, 32, req->fwt_fc);
    } else {
        bit_buffer_copy_bytes(rf->tx, cmd, sizeof(cmd));
        bit_buffer_reset(rf->rx);
        err = nfc_poller_trx(rf->nfc, rf->tx, rf->rx, req->fwt_fc);
    }
    if(err != NfcErrorNone || bit_buffer_get_size(rf->rx) != 32) return false;

    uint32_t nt_raw = rf_be32(bit_buffer_get_data(rf->rx));
    Mfc1State s;
    mfc1_init(&s, key);
    uint32_t nt;
    if(nested) {
        nt = mfc1_word(&s, nt_raw ^ uid, 1) ^ nt_raw;
    } else {
        nt = nt_raw;
        mfc1_word(&s, uid ^ nt, 0);
    }

    uint8_t nr[4];
    furi_hal_random_fill_buf(nr, sizeof(nr));

    bit_buffer_reset(rf->tx);
    bit_buffer_set_size_bytes(rf->tx, 8);
    for(size_t pos = 0; pos < 4; pos++) {
        uint8_t enc = mfc1_byte(&s, nr[pos], 0) ^ nr[pos];
        bool par = ((mfc1_peek(&s) ^ mfc1_odd_parity8(nr[pos])) & 1U) != 0;
        bit_buffer_set_byte_with_parity(rf->tx, pos, enc, par);
    }
    uint32_t suc = mfc1_prng_successor(nt, 32);
    for(size_t pos = 4; pos < 8; pos++) {
        suc = mfc1_prng_successor(suc, 8);
        uint8_t plain = (uint8_t)(suc & 0xFF);
        uint8_t enc = mfc1_byte(&s, 0, 0) ^ plain;
        bool par = ((mfc1_peek(&s) ^ mfc1_odd_parity8(plain)) & 1U) != 0;
        bit_buffer_set_byte_with_parity(rf->tx, pos, enc, par);
    }

    bit_buffer_reset(rf->rx);
    err = nfc_iso14443a_poller_trx_custom_parity(rf->nfc, rf->tx, rf->rx, req->fwt_fc);
    if(err != NfcErrorNone || bit_buffer_get_size(rf->rx) != 32) return false;

    suc = mfc1_prng_successor(suc, 32);
    uint32_t at = mfc1_word(&s, 0, 0) ^ rf_be32(bit_buffer_get_data(rf->rx));
    if(at != suc) return false;

    rf->crypto = s;
    return true;
}

static void rf_execute(Rf* rf, const RfRequest* req, RfResult* res) {
    memset(res, 0, sizeof(*res));
    res->seq = req->seq;
    res->type = req->type;

    if(req->type == RfReqAuth) {
        res->status = rf_auth(rf, req) ? RfStatusOk : RfStatusAuthFailed;
        return;
    }

    uint32_t fwt = req->listen ? req->fwt_fc : RF_TX_ONLY_FWT_FC;
    bool crypto = req->crypto && rf->tech == RfTechA;
    NfcError err = crypto ? rf_trx_crypto(rf, req->tx, req->tx_bits, fwt) :
                                 rf_trx_plain(rf, req->tx, req->tx_bits, fwt);

    if(!req->listen) {
        res->status = RfStatusTimeout;
        return;
    }

    switch(err) {
    case NfcErrorNone:
        res->status = RfStatusOk;
        break;
    case NfcErrorIncompleteFrame:
        res->status = RfStatusCollision;
        break;
    case NfcErrorDataFormat:
        res->status = RfStatusParityError;
        break;
    default:
        res->status = RfStatusTimeout;
        return;
    }
    rf_copy_rx(rf, res, crypto);
    if(res->rx_bits == 0) res->status = RfStatusTimeout;
}

static NfcCommand rf_nfc_callback(NfcEvent event, void* context) {
    Rf* rf = context;
    if(event.type != NfcEventTypePollerReady) return NfcCommandContinue;

    for(size_t i = 0; i < 10; i++) {
        if(rf->stop_requested) return NfcCommandStop;
        RfRequest req;
        if(furi_message_queue_get(rf->queue, &req, 5) == FuriStatusOk) {
            RfResult res;
            rf_execute(rf, &req, &res);
            rf->callback(&res, rf->context);
            return NfcCommandContinue;
        }
    }
    return rf->stop_requested ? NfcCommandStop : NfcCommandContinue;
}

static void rf_complete_timeout(Rf* rf, const RfRequest* req) {
    RfResult res;
    memset(&res, 0, sizeof(res));
    res.seq = req->seq;
    res.type = req->type;
    res.status = (req->type == RfReqAuth) ? RfStatusAuthFailed : RfStatusTimeout;
    rf->callback(&res, rf->context);
}

Rf* rf_alloc(RfCompleteCallback callback, void* context) {
    Rf* rf = malloc(sizeof(Rf));
    memset(rf, 0, sizeof(Rf));
    rf->callback = callback;
    rf->context = context;
    rf->queue = furi_message_queue_alloc(RF_QUEUE_LEN, sizeof(RfRequest));
    rf->tx = bit_buffer_alloc(RF_MAX_FRAME + 4);
    rf->rx = bit_buffer_alloc(RF_MAX_FRAME + 4);
    rf->nfc = nfc_alloc();
    rf->tech = RfTechA;
    return rf;
}

void rf_free(Rf* rf) {
    rf_set_field(rf, false);
    nfc_free(rf->nfc);
    bit_buffer_free(rf->tx);
    bit_buffer_free(rf->rx);
    furi_message_queue_free(rf->queue);
    free(rf);
}

void rf_set_field(Rf* rf, bool on) {
    if(on == rf->running) return;
    if(on) {
        rf->stop_requested = false;
        /* The worker resets the configuration when it stops, so the library
         * must be configured again before every start. */
        const RfTechTiming* t = &rf_timing[rf->tech];
        nfc_config(rf->nfc, NfcModePoller, t->nfc_tech);
        nfc_set_guard_time_us(rf->nfc, t->guard_time_us);
        nfc_set_fdt_poll_fc(rf->nfc, t->fdt_poll_fc);
        nfc_set_fdt_poll_poll_us(rf->nfc, t->poll_poll_us);
        nfc_start(rf->nfc, rf_nfc_callback, rf);
        rf->running = true;
        FURI_LOG_D(TAG, "Field on");
    } else {
        rf->stop_requested = true;
        nfc_stop(rf->nfc);
        rf->running = false;
        rf->stop_requested = false;
        RfRequest req;
        while(furi_message_queue_get(rf->queue, &req, 0) == FuriStatusOk) {
            rf_complete_timeout(rf, &req);
        }
        FURI_LOG_D(TAG, "Field off");
    }
}

bool rf_is_field_on(Rf* rf) {
    return rf->running;
}

void rf_set_tech(Rf* rf, RfTech tech) {
    if(tech == rf->tech) return;
    bool was_on = rf->running;
    if(was_on) rf_set_field(rf, false);
    rf->tech = tech;
    if(was_on) rf_set_field(rf, true);
}

RfTech rf_get_tech(Rf* rf) {
    return rf->tech;
}

void rf_submit(Rf* rf, const RfRequest* request) {
    if(!rf->running || furi_message_queue_put(rf->queue, request, 0) != FuriStatusOk) {
        /* No field: the transmission goes nowhere, nothing answers. */
        rf_complete_timeout(rf, request);
    }
}
