/**
 * Synchronous contactless card handling on top of the asynchronous RF
 * front-end, for the PC/SC reader personalities:
 *  - ISO14443-A: activation, MIFARE Classic / Ultralight operations, ISO-DEP
 *  - ISO14443-B: REQB/WUPB + ATTRIB, ISO-DEP
 *  - FeliCa (212 kbit/s): polling, native frames
 *  - ISO15693: inventory, system info, addressed block access, raw commands
 *  - ST25TB / SRI / SRIX (over ISO14443-B)
 *  - Picopass / iClass (over ISO15693 RF), identification and block reads
 */
#pragma once

#include "rf.h"
#include <furi.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PICC_FDT_FC        (5000U) /* activation / MIFARE answers (~370 us) */
#define PICC_MFC_FWT_FC    (135600U) /* 10 ms: MIFARE write / value ops */
#define PICC_V_FWT_FC      (4202U) /* ISO15693 response (firmware poller value) */
#define PICC_V_WRITE_FWT_FC (300000U) /* ISO15693 write / lock (~22 ms) */
#define PICC_F_FWT_FC      (200000U) /* FeliCa (firmware poller value) */
#define PICC_B_FWT_FC      (9000U) /* ATQB */
#define PICC_ST_FWT_FC     (8494U) /* ST25TB */
#define PICC_ST_WRITE_FWT_FC (90000U) /* ST25TB write, no answer expected */

typedef enum {
    PiccTypeNone,
    PiccTypeA,
    PiccTypeB,
    PiccTypeFelica,
    PiccType15693,
    PiccTypeSt25tb,
    PiccTypePicopass,
} PiccType;

#define PICC_MAX_WTX 100 /* S(WTX) requests accepted for one block */

typedef struct {
    Rf* rf;
    /* Called about every 100 ms while waiting for a long RF exchange */
    void (*wait_hook)(void* ctx);
    void* wait_ctx;
    FuriMessageQueue* results; /* RfResult, filled by the RF completion callback */
    uint32_t seq;

    PiccType type;
    bool present;
    const char* fail; /* step at which the last activation / presence check failed */
    uint32_t last_answer_tick; /* furi tick of the last frame the card answered */
    bool silent; /* the last exchange got no answer */
    bool crypto; /* MIFARE Crypto1 session active */

    /* A: UID; B: PUPI; FeliCa: IDm; 15693: UID (transmission order);
     * ST25TB: UID (transmission order); Picopass: CSN */
    uint8_t uid[10];
    uint8_t uid_len;

    /* ISO14443-A */
    uint8_t atqa[2]; /* as received over the air (LSB first) */
    uint8_t sak;
    uint8_t sak_all[3];
    uint8_t sak_count;

    /* ISO-DEP (A or B) */
    bool iso_dep;
    uint8_t ats[64];
    uint8_t ats_len;
    uint8_t hist_off;
    uint8_t hist_len;
    uint16_t fsc;
    uint32_t fwt_fc;
    uint8_t block_num;

    /* ISO14443-B */
    uint8_t atqb[12];
    uint8_t attrib_res[16];
    uint8_t attrib_res_len;

    /* FeliCa */
    uint8_t pol_req[5]; /* polling payload without length byte */
    uint8_t pol_res[20]; /* polling response including length byte */
    uint8_t pol_res_len;

    /* ISO15693 */
    uint8_t dsfid;
    uint8_t v_block_size;
    uint16_t v_block_count;
    bool v_sysinfo;

    /* ST25TB */
    uint8_t chip_id;

    /* Picopass */
    uint8_t pp_config[8];
    bool pp_config_ok;
} Picc;

void picc_init(Picc* p, Rf* rf, FuriMessageQueue* results);

/** Select the RF technology (cycles the field if it changes). */
void picc_use_tech(Picc* p, RfTech tech);

/** Raw exchange: exactly the bits given, encrypted when p->crypto is set (A only). */
RfStatus picc_raw(
    Picc* p,
    const uint8_t* tx,
    uint16_t tx_bits,
    uint8_t* rx,
    uint16_t* rx_bits,
    uint32_t fwt_fc);

/** Frame exchange with the link-layer CRC of the current card type added and checked/stripped. */
bool picc_frame(Picc* p, const uint8_t* tx, size_t len, uint8_t* rx, size_t* rx_len, uint32_t fwt);

/** ISO14443-A frame whose answer must be a 4-bit MIFARE ACK. */
bool picc_ack_frame(Picc* p, const uint8_t* tx, size_t len, uint32_t fwt);

/* Activation per technology (the RF technology is switched as needed) */
bool picc_activate_a(Picc* p, bool wakeup, bool rats);
/** RATS on a selected ISO14443-A card (ISO14443-4 activation). */
bool picc_rats(Picc* p);
bool picc_activate_b(Picc* p, uint8_t afi, bool wakeup);
bool picc_activate_felica(Picc* p, const uint8_t pol_req[5]);
bool picc_activate_15693(Picc* p);
bool picc_activate_st25tb(Picc* p);
bool picc_activate_picopass(Picc* p);

/** Cold re-activation of the current card (CCID power-on). */
bool picc_reactivate(Picc* p, bool rats);

void picc_halt(Picc* p);
/** ISO-DEP DESELECT. */
void picc_deselect(Picc* p);

bool picc_mfc_auth(Picc* p, uint8_t key_cmd, uint8_t block, const uint8_t key[6]);

/** Exchange one APDU over ISO-DEP (A or B; chaining, WTX, error recovery). */
bool picc_isodep_apdu(
    Picc* p,
    const uint8_t* apdu,
    size_t len,
    uint8_t* resp,
    size_t* resp_len,
    size_t resp_max);

/** Presence check (non-destructive for ISO-DEP, FeliCa, 15693; re-selects MIFARE). */
bool picc_presence(Picc* p);

/* ISO15693 */
/** Raw ISO15693 request (flags, cmd, ...) without CRC. Response includes the flags byte. */
bool picc_15693_request(
    Picc* p,
    const uint8_t* req,
    size_t len,
    uint8_t* resp,
    size_t* resp_len,
    uint32_t fwt);
bool picc_15693_read_block(Picc* p, uint16_t block, uint8_t* out, size_t* out_len);
bool picc_15693_write_block(Picc* p, uint16_t block, const uint8_t* data, size_t len);

/* ST25TB */
bool picc_st25tb_read_block(Picc* p, uint8_t block, uint8_t out[4]);
bool picc_st25tb_write_block(Picc* p, uint8_t block, const uint8_t data[4]);

/* Picopass */
bool picc_picopass_read_block(Picc* p, uint8_t block, uint8_t out[8]);

/* FeliCa: frame starts with its length byte; CRC handled here */
bool picc_felica_exchange(
    Picc* p,
    const uint8_t* frame,
    size_t len,
    uint8_t* resp,
    size_t* resp_len,
    uint32_t fwt);

bool picc_is_ultralight(const Picc* p);
bool picc_is_mifare_classic(const Picc* p);
const char* picc_type_name(const Picc* p);
