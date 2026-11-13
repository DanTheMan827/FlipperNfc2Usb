#include "picc.h"
#include "crc.h"

#include <string.h>

#define PICC_FSDI     8 /* FSD = 256 */
#define PICC_RATS_FWT (65536U) /* FWI 4 default, ~4.8 ms */
#define PICC_MAX_FWT  (67800000U) /* ~5 s, FWI 14 */
#define PICC_RETRIES  2

/* ISO15693 request flags */
#define V_FLAG_HIGH_RATE 0x02
#define V_FLAG_INVENTORY 0x04
#define V_FLAG_ADDRESSED 0x20
#define V_FLAG_SLOTS_1   0x20
#define V_FLAG_OPTION    0x40

/* Picopass commands */
#define PP_ACTALL   0x0A
#define PP_IDENTIFY 0x0C
#define PP_SELECT   0x81
#define PP_READ     0x0C

static const uint16_t fsc_table[] = {16, 24, 32, 40, 48, 64, 96, 128, 256};

static CrcType picc_crc_type(const Picc* p) {
    switch(p->type) {
    case PiccTypeB:
    case PiccTypeSt25tb:
        return CrcTypeB;
    case PiccTypeFelica:
        return CrcTypeFelica;
    case PiccType15693:
        return CrcType15693;
    case PiccTypePicopass:
        return CrcTypePicopass;
    default:
        return CrcTypeA;
    }
}

void picc_init(Picc* p, Rf* rf, FuriMessageQueue* results) {
    memset(p, 0, sizeof(Picc));
    p->rf = rf;
    p->results = results;
}

void picc_use_tech(Picc* p, RfTech tech) {
    if(rf_get_tech(p->rf) != tech) {
        p->present = false;
        p->crypto = false;
    }
    rf_set_tech(p->rf, tech);
}

static RfStatus picc_wait(Picc* p, uint32_t seq, RfResult* res, uint32_t fwt_fc) {
    uint32_t start = furi_get_tick();
    uint32_t timeout = furi_ms_to_ticks(fwt_fc / 13560U + 1000U);
    while(furi_get_tick() - start < timeout) {
        /* Wait in short slices so the owner can keep the host informed */
        if(furi_message_queue_get(p->results, res, furi_ms_to_ticks(100)) != FuriStatusOk) {
            if(p->wait_hook) p->wait_hook(p->wait_ctx);
            continue;
        }
        if(res->seq == seq) {
            /* Any answer, even a corrupted one, means a card is still there */
            p->silent = res->status == RfStatusTimeout;
            if(!p->silent) p->last_answer_tick = furi_get_tick();
            return res->status;
        }
    }
    p->silent = true;
    return RfStatusTimeout;
}

RfStatus picc_raw(
    Picc* p,
    const uint8_t* tx,
    uint16_t tx_bits,
    uint8_t* rx,
    uint16_t* rx_bits,
    uint32_t fwt_fc) {
    RfRequest req;
    memset(&req, 0, sizeof(req));
    size_t bytes = (tx_bits + 7U) / 8U;
    if(bytes > RF_MAX_FRAME) return RfStatusTimeout;
    req.type = RfReqTransceive;
    memcpy(req.tx, tx, bytes);
    req.tx_bits = tx_bits;
    req.listen = true;
    req.crypto = p->crypto;
    req.fwt_fc = fwt_fc > PICC_MAX_FWT ? PICC_MAX_FWT : fwt_fc;
    req.seq = ++p->seq;
    rf_submit(p->rf, &req);

    RfResult res;
    RfStatus st = picc_wait(p, req.seq, &res, req.fwt_fc);
    if(st == RfStatusOk || st == RfStatusCollision || st == RfStatusParityError) {
        if(rx) memcpy(rx, res.rx, (res.rx_bits + 7U) / 8U);
        if(rx_bits) *rx_bits = res.rx_bits;
    } else if(rx_bits) {
        *rx_bits = 0;
    }
    return st;
}

static bool picc_frame_crc(
    Picc* p,
    CrcType crc,
    const uint8_t* tx,
    size_t len,
    uint8_t* rx,
    size_t* rx_len,
    uint32_t fwt) {
    uint8_t buf[RF_MAX_FRAME];
    if(len + 2 > sizeof(buf)) return false;
    memcpy(buf, tx, len);
    crc_append(crc, buf, len);

    uint8_t in[RF_MAX_FRAME];
    uint16_t bits = 0;
    if(picc_raw(p, buf, (uint16_t)((len + 2) * 8U), in, &bits, fwt) != RfStatusOk) return false;
    if(bits % 8U || bits < 24) return false;
    size_t n = bits / 8U;
    if(!crc_check(crc, in, n)) return false;
    if(rx) memcpy(rx, in, n - 2);
    if(rx_len) *rx_len = n - 2;
    return true;
}

bool picc_frame(Picc* p, const uint8_t* tx, size_t len, uint8_t* rx, size_t* rx_len, uint32_t fwt) {
    return picc_frame_crc(p, picc_crc_type(p), tx, len, rx, rx_len, fwt);
}

bool picc_ack_frame(Picc* p, const uint8_t* tx, size_t len, uint32_t fwt) {
    uint8_t buf[RF_MAX_FRAME];
    if(len + 2 > sizeof(buf)) return false;
    memcpy(buf, tx, len);
    crc_append(CrcTypeA, buf, len);
    uint8_t in[RF_MAX_FRAME];
    uint16_t bits = 0;
    if(picc_raw(p, buf, (uint16_t)((len + 2) * 8U), in, &bits, fwt) != RfStatusOk) return false;
    return bits == 4 && (in[0] & 0x0F) == 0x0A;
}

static void picc_reset_card(Picc* p, PiccType type) {
    p->type = type;
    p->present = false;
    p->crypto = false;
    p->iso_dep = false;
    p->ats_len = 0;
    p->hist_off = 0;
    p->hist_len = 0;
    p->uid_len = 0;
    p->sak_count = 0;
    p->attrib_res_len = 0;
    p->pol_res_len = 0;
    p->v_sysinfo = false;
    p->pp_config_ok = false;
}

/* ------------------------------------------------------------------------ */
/* ISO14443-A                                                                */
/* ------------------------------------------------------------------------ */

static void picc_parse_ats(Picc* p) {
    p->fsc = 32;
    p->fwt_fc = PICC_RATS_FWT;
    p->hist_off = 0;
    p->hist_len = 0;
    if(p->ats_len < 1) return;
    uint8_t tl = p->ats[0];
    if(tl > p->ats_len) tl = p->ats_len;
    if(tl < 2) return;
    uint8_t t0 = p->ats[1];
    uint8_t fsci = t0 & 0x0F;
    p->fsc = fsci < 9 ? fsc_table[fsci] : 256;
    uint8_t idx = 2;
    if(t0 & 0x10) idx++; /* TA */
    if(t0 & 0x20) { /* TB: FWI / SFGI */
        if(idx < tl) {
            uint8_t fwi = p->ats[idx] >> 4;
            if(fwi > 14) fwi = 4;
            p->fwt_fc = (256U * 16U) << fwi;
        }
        idx++;
    }
    if(t0 & 0x40) idx++; /* TC */
    if(idx < tl) {
        p->hist_off = idx;
        p->hist_len = (uint8_t)(tl - idx);
    }
}

bool picc_activate_a(Picc* p, bool wakeup, bool rats) {
    picc_use_tech(p, RfTechA);
    picc_reset_card(p, PiccTypeA);

    uint8_t cmd = wakeup ? 0x52 : 0x26;
    uint8_t rx[RF_MAX_FRAME];
    uint16_t bits = 0;
    p->fail = wakeup ? "WUPA" : "REQA";
    if(picc_raw(p, &cmd, 7, rx, &bits, PICC_FDT_FC) != RfStatusOk || bits != 16) return false;
    p->atqa[0] = rx[0];
    p->atqa[1] = rx[1];

    uint8_t sak = 0;
    for(uint8_t level = 0; level < 3; level++) {
        uint8_t sel = (uint8_t)(0x93 + 2 * level);
        uint8_t ac[2] = {sel, 0x20};
        p->fail = level == 0 ? "ANTICOLL1" : level == 1 ? "ANTICOLL2" : "ANTICOLL3";
        if(picc_raw(p, ac, 16, rx, &bits, PICC_FDT_FC) != RfStatusOk || bits != 40) return false;
        if((rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) return false;
        p->fail = level == 0 ? "SELECT1" : level == 1 ? "SELECT2" : "SELECT3";

        uint8_t select[7] = {sel, 0x70, rx[0], rx[1], rx[2], rx[3], rx[4]};
        uint8_t sak_buf[4];
        size_t n = 0;
        if(!picc_frame_crc(p, CrcTypeA, select, sizeof(select), sak_buf, &n, PICC_FDT_FC) ||
           n != 1) {
            return false;
        }
        sak = sak_buf[0];
        p->sak_all[p->sak_count++] = sak;
        if(sak & 0x04) {
            if(rx[0] != 0x88) return false; /* cascade tag expected */
            memcpy(&p->uid[p->uid_len], &rx[1], 3);
            p->uid_len += 3;
        } else {
            memcpy(&p->uid[p->uid_len], &rx[0], 4);
            p->uid_len += 4;
            break;
        }
    }
    if(sak & 0x04) return false;
    p->sak = sak;
    p->present = true;
    p->fail = NULL;

    if(rats && (sak & 0x20)) picc_rats(p);
    return true;
}

bool picc_rats(Picc* p) {
    if(p->type != PiccTypeA || !p->present || !(p->sak & 0x20) || p->iso_dep) return p->iso_dep;
    uint8_t r[2] = {0xE0, (uint8_t)(PICC_FSDI << 4)};
    size_t n = 0;
    if(picc_frame_crc(p, CrcTypeA, r, 2, p->ats, &n, PICC_RATS_FWT) && n >= 1) {
        p->ats_len = (uint8_t)(n > sizeof(p->ats) ? sizeof(p->ats) : n);
        picc_parse_ats(p);
        p->block_num = 0;
        p->iso_dep = true;
    }
    return p->iso_dep;
}

bool picc_mfc_auth(Picc* p, uint8_t key_cmd, uint8_t block, const uint8_t key[6]) {
    if(p->type != PiccTypeA) return false;
    RfRequest req;
    memset(&req, 0, sizeof(req));
    req.type = RfReqAuth;
    req.auth_cmd = key_cmd;
    req.auth_block = block;
    memcpy(req.key, key, 6);
    /* Crypto1 uses the last 4 bytes of the UID (NUID for 7-byte UIDs) */
    memcpy(req.uid, &p->uid[p->uid_len >= 4 ? p->uid_len - 4 : 0], 4);
    req.crypto = p->crypto;
    req.listen = true;
    req.fwt_fc = PICC_MFC_FWT_FC;
    req.seq = ++p->seq;
    rf_submit(p->rf, &req);
    RfResult res;
    bool ok = picc_wait(p, req.seq, &res, req.fwt_fc) == RfStatusOk;
    p->crypto = ok;
    return ok;
}

/* ------------------------------------------------------------------------ */
/* ISO14443-B                                                                */
/* ------------------------------------------------------------------------ */

bool picc_activate_b(Picc* p, uint8_t afi, bool wakeup) {
    picc_use_tech(p, RfTechB);
    picc_reset_card(p, PiccTypeB);

    uint8_t req[3] = {0x05, afi, wakeup ? 0x08 : 0x00};
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(!picc_frame(p, req, sizeof(req), rx, &n, PICC_B_FWT_FC) || n != 12 || rx[0] != 0x50) {
        return false;
    }
    memcpy(p->atqb, rx, 12);
    memcpy(p->uid, &rx[1], 4);
    p->uid_len = 4;

    const uint8_t* proto = &p->atqb[9];
    uint8_t fsci = proto[1] >> 4;
    p->fsc = fsci < 9 ? fsc_table[fsci] : 256;
    uint8_t fwi = proto[2] >> 4;
    if(fwi > 14) fwi = 4;
    p->fwt_fc = (256U * 16U) << fwi;

    /* ATTRIB: PUPI, Param1 (default TR0/TR1, SOF/EOF), Param2 (106 kbit/s,
     * FSDI 256), Param3 (protocol type from ATQB), Param4 (CID 0) */
    uint8_t attrib[9] = {
        0x1D, p->uid[0], p->uid[1], p->uid[2], p->uid[3], 0x00, 0x08, proto[1] & 0x0F, 0x00};
    if(!picc_frame(p, attrib, sizeof(attrib), rx, &n, p->fwt_fc) || n < 1) return false;
    p->attrib_res_len = (uint8_t)(n > sizeof(p->attrib_res) ? sizeof(p->attrib_res) : n);
    memcpy(p->attrib_res, rx, p->attrib_res_len);

    p->iso_dep = (proto[1] & 0x01) != 0;
    p->block_num = 0;
    p->present = true;
    return true;
}

/* ------------------------------------------------------------------------ */
/* FeliCa                                                                    */
/* ------------------------------------------------------------------------ */

bool picc_activate_felica(Picc* p, const uint8_t pol_req[5]) {
    picc_use_tech(p, RfTechF);
    picc_reset_card(p, PiccTypeFelica);
    memcpy(p->pol_req, pol_req, 5);

    uint8_t req[6] = {0x06, pol_req[0], pol_req[1], pol_req[2], pol_req[3], pol_req[4]};
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(!picc_frame(p, req, sizeof(req), rx, &n, PICC_F_FWT_FC)) return false;
    if((n != 18 && n != 20) || rx[0] != n || rx[1] != 0x01) return false;
    memcpy(p->pol_res, rx, n);
    p->pol_res_len = (uint8_t)n;
    memcpy(p->uid, &rx[2], 8); /* IDm */
    p->uid_len = 8;
    p->present = true;
    return true;
}

bool picc_felica_exchange(
    Picc* p,
    const uint8_t* frame,
    size_t len,
    uint8_t* resp,
    size_t* resp_len,
    uint32_t fwt) {
    if(p->type != PiccTypeFelica || len < 2) return false;
    return picc_frame(p, frame, len, resp, resp_len, fwt);
}

/* ------------------------------------------------------------------------ */
/* ISO15693                                                                  */
/* ------------------------------------------------------------------------ */

bool picc_15693_request(
    Picc* p,
    const uint8_t* req,
    size_t len,
    uint8_t* resp,
    size_t* resp_len,
    uint32_t fwt) {
    return picc_frame_crc(p, CrcType15693, req, len, resp, resp_len, fwt);
}

static size_t v_addressed(const Picc* p, uint8_t* buf, uint8_t flags, uint8_t cmd) {
    buf[0] = flags;
    buf[1] = cmd;
    memcpy(&buf[2], p->uid, 8);
    return 10;
}

static void picc_15693_sysinfo(Picc* p) {
    uint8_t req[10];
    size_t n = v_addressed(p, req, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x2B);
    uint8_t rx[RF_MAX_FRAME];
    size_t rn = 0;
    p->v_block_size = 4;
    p->v_block_count = 0;
    if(!picc_15693_request(p, req, n, rx, &rn, PICC_V_FWT_FC) || rn < 10 || (rx[0] & 0x01)) {
        return;
    }
    uint8_t info = rx[1];
    size_t idx = 10; /* flags, info flags, UID */
    if(info & 0x01) idx++; /* DSFID */
    if(info & 0x02) idx++; /* AFI */
    if((info & 0x04) && idx + 1 < rn) {
        p->v_block_count = (uint16_t)(rx[idx] + 1U);
        p->v_block_size = (uint8_t)((rx[idx + 1] & 0x1F) + 1U);
    }
    p->v_sysinfo = true;
}

bool picc_activate_15693(Picc* p) {
    picc_use_tech(p, RfTechV);
    picc_reset_card(p, PiccType15693);
    uint8_t req[3] = {V_FLAG_HIGH_RATE | V_FLAG_INVENTORY | V_FLAG_SLOTS_1, 0x01, 0x00};
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(!picc_15693_request(p, req, sizeof(req), rx, &n, PICC_V_FWT_FC) || n != 10 ||
       (rx[0] & 0x01)) {
        return false;
    }
    p->dsfid = rx[1];
    memcpy(p->uid, &rx[2], 8);
    p->uid_len = 8;
    p->present = true;
    picc_15693_sysinfo(p);
    return true;
}

bool picc_15693_read_block(Picc* p, uint16_t block, uint8_t* out, size_t* out_len) {
    uint8_t req[13];
    size_t n;
    if(block > 0xFF) {
        n = v_addressed(p, req, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x30); /* extended */
        req[n++] = (uint8_t)(block & 0xFF);
        req[n++] = (uint8_t)(block >> 8);
    } else {
        n = v_addressed(p, req, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x20);
        req[n++] = (uint8_t)block;
    }
    uint8_t rx[RF_MAX_FRAME];
    size_t rn = 0;
    if(!picc_15693_request(p, req, n, rx, &rn, PICC_V_FWT_FC) || rn < 2 || (rx[0] & 0x01)) {
        return false;
    }
    memcpy(out, &rx[1], rn - 1);
    *out_len = rn - 1;
    return true;
}

bool picc_15693_write_block(Picc* p, uint16_t block, const uint8_t* data, size_t len) {
    uint8_t req[16 + 13];
    if(len > 32) return false;
    size_t n;
    if(block > 0xFF) {
        n = v_addressed(p, req, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x31);
        req[n++] = (uint8_t)(block & 0xFF);
        req[n++] = (uint8_t)(block >> 8);
    } else {
        n = v_addressed(p, req, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x21);
        req[n++] = (uint8_t)block;
    }
    if(n + len > sizeof(req)) return false;
    memcpy(&req[n], data, len);
    n += len;
    uint8_t rx[RF_MAX_FRAME];
    size_t rn = 0;
    return picc_15693_request(p, req, n, rx, &rn, PICC_V_WRITE_FWT_FC) && rn >= 1 &&
           !(rx[0] & 0x01);
}

/* ------------------------------------------------------------------------ */
/* ST25TB (SRI / SRIX)                                                       */
/* ------------------------------------------------------------------------ */

bool picc_activate_st25tb(Picc* p) {
    picc_use_tech(p, RfTechB);
    picc_reset_card(p, PiccTypeSt25tb);
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    uint8_t initiate[2] = {0x06, 0x00};
    if(!picc_frame(p, initiate, 2, rx, &n, PICC_ST_FWT_FC) || n != 1) return false;
    p->chip_id = rx[0];
    uint8_t select[2] = {0x0E, p->chip_id};
    if(!picc_frame(p, select, 2, rx, &n, PICC_ST_FWT_FC) || n != 1 || rx[0] != p->chip_id) {
        return false;
    }
    uint8_t get_uid = 0x0B;
    if(!picc_frame(p, &get_uid, 1, rx, &n, PICC_ST_FWT_FC) || n != 8) return false;
    memcpy(p->uid, rx, 8);
    p->uid_len = 8;
    p->present = true;
    return true;
}

bool picc_st25tb_read_block(Picc* p, uint8_t block, uint8_t out[4]) {
    uint8_t cmd[2] = {0x08, block};
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(!picc_frame(p, cmd, 2, rx, &n, PICC_ST_FWT_FC) || n != 4) return false;
    memcpy(out, rx, 4);
    return true;
}

bool picc_st25tb_write_block(Picc* p, uint8_t block, const uint8_t data[4]) {
    uint8_t cmd[8] = {0x09, block, data[0], data[1], data[2], data[3]};
    crc_append(CrcTypeB, cmd, 6);
    /* WRITE_BLOCK has no answer; verify by reading back */
    picc_raw(p, cmd, 64, NULL, NULL, PICC_ST_WRITE_FWT_FC);
    uint8_t check[4];
    return picc_st25tb_read_block(p, block, check) && memcmp(check, data, 4) == 0;
}

/* ------------------------------------------------------------------------ */
/* Picopass (iClass) over ISO15693 RF                                        */
/* ------------------------------------------------------------------------ */

static bool pp_xchg(Picc* p, const uint8_t* tx, size_t len, uint8_t* rx, size_t* rn) {
    uint8_t in[RF_MAX_FRAME];
    uint16_t bits = 0;
    if(picc_raw(p, tx, (uint16_t)(len * 8U), in, &bits, PICC_V_FWT_FC) != RfStatusOk) {
        return false;
    }
    if(bits % 8U || bits < 24) return false;
    size_t n = bits / 8U;
    if(!crc_check(CrcTypePicopass, in, n)) return false;
    memcpy(rx, in, n - 2);
    *rn = n - 2;
    return true;
}

bool picc_activate_picopass(Picc* p) {
    picc_use_tech(p, RfTechV);
    picc_reset_card(p, PiccTypePicopass);
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    uint8_t actall = PP_ACTALL;
    picc_raw(p, &actall, 8, NULL, NULL, PICC_V_FWT_FC); /* answered by an SOF only */
    uint8_t identify = PP_IDENTIFY;
    if(!pp_xchg(p, &identify, 1, rx, &n) || n != 8) return false;
    uint8_t select[9] = {PP_SELECT};
    memcpy(&select[1], rx, 8);
    if(!pp_xchg(p, select, sizeof(select), rx, &n) || n != 8) return false;
    memcpy(p->uid, rx, 8); /* CSN */
    p->uid_len = 8;
    p->present = true;
    p->pp_config_ok = picc_picopass_read_block(p, 1, p->pp_config);
    return true;
}

bool picc_picopass_read_block(Picc* p, uint8_t block, uint8_t out[8]) {
    /* READ: command, block, CRC over the block number only */
    uint8_t cmd[4] = {PP_READ, block};
    crc_append(CrcTypePicopass, &cmd[1], 1);
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(!pp_xchg(p, cmd, sizeof(cmd), rx, &n) || n != 8) return false;
    memcpy(out, rx, 8);
    return true;
}

/* ------------------------------------------------------------------------ */
/* Common                                                                    */
/* ------------------------------------------------------------------------ */

void picc_halt(Picc* p) {
    if(p->type == PiccTypeA) {
        uint8_t hlta[4] = {0x50, 0x00};
        crc_append(CrcTypeA, hlta, 2);
        picc_raw(p, hlta, 32, NULL, NULL, PICC_FDT_FC); /* no answer expected */
    } else if(p->type == PiccTypeB && !p->iso_dep) {
        uint8_t hltb[5] = {0x50, p->uid[0], p->uid[1], p->uid[2], p->uid[3]};
        uint8_t rx[8];
        size_t n;
        picc_frame(p, hltb, sizeof(hltb), rx, &n, PICC_B_FWT_FC);
    }
    p->crypto = false;
}

void picc_deselect(Picc* p) {
    if(!p->iso_dep) return;
    uint8_t s = 0xC2;
    uint8_t rx[4];
    size_t n;
    picc_frame(p, &s, 1, rx, &n, p->fwt_fc);
}

bool picc_reactivate(Picc* p, bool rats) {
    uint8_t uid[10];
    uint8_t uid_len = p->uid_len;
    memcpy(uid, p->uid, sizeof(uid));
    bool ok;
    switch(p->type) {
    case PiccTypeA:
        if(p->iso_dep) picc_deselect(p);
        picc_halt(p);
        ok = picc_activate_a(p, true, rats);
        if(!ok) {
            /* The card may have missed the HLTA or be in an odd state: power
             * cycle it (field off/on) and start again from IDLE */
            const char* first = p->fail;
            rf_set_field(p->rf, false);
            furi_delay_ms(10);
            rf_set_field(p->rf, true);
            ok = picc_activate_a(p, false, rats);
            if(!ok && first) p->fail = first;
        }
        break;
    case PiccTypeB:
        if(p->iso_dep) picc_deselect(p);
        picc_halt(p);
        ok = picc_activate_b(p, 0x00, true);
        break;
    case PiccTypeFelica: {
        uint8_t req[5];
        memcpy(req, p->pol_req, 5);
        ok = picc_activate_felica(p, req);
        break;
    }
    case PiccType15693: {
        /* Reset to ready (addressed) so a card the host quieted answers again */
        uint8_t r[10];
        size_t n = v_addressed(p, r, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x26);
        uint8_t rx[16];
        size_t rn;
        picc_15693_request(p, r, n, rx, &rn, PICC_V_FWT_FC);
        ok = picc_activate_15693(p);
        break;
    }
    case PiccTypeSt25tb: {
        uint8_t reset = 0x0C; /* RESET_TO_INVENTORY */
        uint8_t rx[16];
        size_t rn;
        picc_frame(p, &reset, 1, rx, &rn, PICC_ST_FWT_FC);
        ok = picc_activate_st25tb(p);
        break;
    }
    case PiccTypePicopass:
        ok = picc_activate_picopass(p);
        break;
    default:
        ok = false;
        break;
    }
    if(ok && (p->uid_len != uid_len || memcmp(p->uid, uid, uid_len) != 0)) {
        p->fail = "other card";
        return false;
    }
    return ok;
}

/* One ISO-DEP block exchange with WTX handling and R(NAK) recovery. */
static bool isodep_xchg(Picc* p, const uint8_t* tx, size_t n, uint8_t* rx, size_t* rn) {
    for(uint8_t attempt = 0; attempt <= PICC_RETRIES; attempt++) {
        bool ok;
        if(attempt == 0) {
            ok = picc_frame(p, tx, n, rx, rn, p->fwt_fc);
        } else {
            uint8_t nak = (uint8_t)(0xB2 | p->block_num);
            ok = picc_frame(p, &nak, 1, rx, rn, p->fwt_fc);
            /* R(ACK) with our block number: the card missed our block, resend */
            if(ok && *rn >= 1 && (rx[0] & 0xF6) == 0xA2 && (rx[0] & 0x01) == p->block_num &&
               (tx[0] & 0xC0) == 0x00) {
                ok = picc_frame(p, tx, n, rx, rn, p->fwt_fc);
            }
        }
        for(uint8_t wtx = 0; ok && *rn >= 2 && (rx[0] & 0xF7) == 0xF2; wtx++) { /* S(WTX) */
            if(wtx >= PICC_MAX_WTX) return false; /* card stuck asking for time */
            uint8_t wtxm = rx[1] & 0x3F;
            if(wtxm == 0) wtxm = 1;
            uint8_t s[2] = {0xF2, wtxm};
            uint64_t fwt = (uint64_t)p->fwt_fc * wtxm;
            ok = picc_frame(
                p, s, 2, rx, rn, (uint32_t)(fwt > PICC_MAX_FWT ? PICC_MAX_FWT : fwt));
        }
        if(ok && *rn >= 1) return true;
    }
    return false;
}

bool picc_isodep_apdu(
    Picc* p,
    const uint8_t* apdu,
    size_t len,
    uint8_t* resp,
    size_t* resp_len,
    size_t resp_max) {
    *resp_len = 0;
    if(!p->iso_dep) return false;

    size_t frame_max = p->fsc < RF_MAX_FRAME ? p->fsc : RF_MAX_FRAME;
    size_t max_inf = frame_max - 3; /* PCB + CRC */
    uint8_t frame[RF_MAX_FRAME];
    uint8_t rx[RF_MAX_FRAME];
    size_t rn = 0;
    size_t off = 0;

    /* Send (chained if needed) */
    while(true) {
        size_t chunk = len - off;
        if(chunk > max_inf) chunk = max_inf;
        bool more = off + chunk < len;
        frame[0] = (uint8_t)(0x02 | p->block_num | (more ? 0x10 : 0x00));
        memcpy(&frame[1], &apdu[off], chunk);
        if(!isodep_xchg(p, frame, chunk + 1, rx, &rn)) return false;
        if(!more) break;
        if((rx[0] & 0xF6) != 0xA2) return false; /* expected R(ACK) */
        p->block_num ^= 1;
        off += chunk;
    }

    /* Receive (chained if needed) */
    while(true) {
        if((rx[0] & 0xE2) != 0x02) return false; /* expected I-block */
        p->block_num ^= 1;
        size_t inf = rn - 1;
        if(*resp_len + inf > resp_max) return false;
        memcpy(&resp[*resp_len], &rx[1], inf);
        *resp_len += inf;
        if(!(rx[0] & 0x10)) break;
        uint8_t ack = (uint8_t)(0xA2 | p->block_num);
        if(!isodep_xchg(p, &ack, 1, rx, &rn)) return false;
    }
    return true;
}

bool picc_presence(Picc* p) {
    if(!p->present) return false;
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(p->iso_dep) {
        uint8_t nak = (uint8_t)(0xB2 | p->block_num);
        return picc_frame(p, &nak, 1, rx, &n, p->fwt_fc);
    }
    switch(p->type) {
    case PiccTypeFelica: {
        uint8_t req[6] = {0x06, p->pol_req[0], p->pol_req[1], p->pol_req[2], p->pol_req[3], 0x00};
        return picc_frame(p, req, sizeof(req), rx, &n, PICC_F_FWT_FC) && n >= 10 &&
               memcmp(&rx[2], p->uid, 8) == 0;
    }
    case PiccType15693: {
        uint8_t req[11];
        size_t rn = v_addressed(p, req, V_FLAG_HIGH_RATE | V_FLAG_ADDRESSED, 0x20);
        req[rn++] = 0x00;
        return picc_15693_request(p, req, rn, rx, &n, PICC_V_FWT_FC) && n >= 1;
    }
    case PiccTypeSt25tb: {
        uint8_t get_uid = 0x0B;
        return picc_frame(p, &get_uid, 1, rx, &n, PICC_ST_FWT_FC) && n == 8 &&
               memcmp(rx, p->uid, 8) == 0;
    }
    case PiccTypePicopass: {
        uint8_t blk[8];
        return picc_picopass_read_block(p, 0, blk) && memcmp(blk, p->uid, 8) == 0;
    }
    default:
        if(picc_is_ultralight(p) && !p->crypto) {
            /* Ultralight / NTAG: READ page 0 leaves the card selected and its
             * state untouched. Page 0 starts with UID0..2. */
            uint8_t rd[2] = {0x30, 0x00};
            if(picc_frame(p, rd, 2, rx, &n, PICC_FDT_FC * 4) && n == 16 &&
               memcmp(rx, p->uid, 3) == 0) {
                return true;
            }
        }
        /* MIFARE and other part-3 cards: halt and re-select (drops Crypto1) */
        return picc_reactivate(p, false);
    }
}

bool picc_is_ultralight(const Picc* p) {
    return p->present && p->type == PiccTypeA && !p->iso_dep && (p->sak & 0x7F) == 0x00;
}

bool picc_is_mifare_classic(const Picc* p) {
    if(!p->present || p->type != PiccTypeA || p->iso_dep) return false;
    uint8_t s = p->sak & 0x7F;
    return s == 0x08 || s == 0x18 || s == 0x09 || s == 0x10 || s == 0x11 || s == 0x01 ||
           s == 0x28 || s == 0x38 || s == 0x88;
}

const char* picc_type_name(const Picc* p) {
    if(!p->present) return "none";
    switch(p->type) {
    case PiccTypeB:
        return p->iso_dep ? "ISO14443-4B" : "ISO14443-3B";
    case PiccTypeFelica:
        return "FeliCa";
    case PiccType15693:
        return "ISO15693";
    case PiccTypeSt25tb:
        return "SRI/SRIX";
    case PiccTypePicopass:
        return "Picopass";
    default:
        break;
    }
    if(p->iso_dep) return "ISO14443-4A";
    switch(p->sak & 0x7F) {
    case 0x00:
        return "Ultralight/NTAG";
    case 0x08:
        return "Classic 1K";
    case 0x09:
        return "Classic Mini";
    case 0x18:
        return "Classic 4K";
    case 0x10:
    case 0x11:
        return "Plus SL2";
    default:
        return "ISO14443-3A";
    }
}
