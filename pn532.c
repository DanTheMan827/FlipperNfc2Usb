/**
 * PN532 command set as reached through the ACR122U "Direct Transmit"
 * pseudo-APDU (FF 00 00 00 Lc D4 ...), following the PN532 User Manual
 * (UM0701-02). Initiator functions for 106 kbit/s type A and B and FeliCa
 * 212 kbit/s are backed by RF. FeliCa 424, Jewel and DEP find no target.
 */
#include "reader_i.h"
#include "crc.h"

#include <string.h>

#define CIU_BASE        0x6300
#define CIU_TX_MODE     0x02
#define CIU_RX_MODE     0x03
#define CIU_MANUAL_RCV  0x0D
#define CIU_STATUS2     0x38
#define CIU_CONTROL     0x3C
#define CIU_BIT_FRAMING 0x3D

#define PN_PARAM_AUTO_RATS 0x10

/* Error codes (UM0701 §7.1) */
#define PN_ST_OK        0x00
#define PN_ST_TIMEOUT   0x01
#define PN_ST_CRC       0x02
#define PN_ST_PARITY    0x03
#define PN_ST_COLLISION 0x06
#define PN_ST_INVALID   0x10
#define PN_ST_FORMAT    0x13
#define PN_ST_MFC_AUTH  0x14
#define PN_ST_WRONG_CTX 0x27

#define PN_TYPE_A_B    0x00
#define PN_TYPE_FELICA 0x10

void pn532_reset(Pn532* pn) {
    memset(pn, 0, sizeof(*pn));
    pn->params = 0x14; /* fAutomaticATR_RES | fAutomaticRATS */
    pn->ciu[CIU_TX_MODE] = 0x80; /* TxCRCEn */
    pn->ciu[CIU_RX_MODE] = 0x80; /* RxCRCEn */
    pn->rf_retries = 0xFF;
    pn->retry_timeout = 0x0A; /* 51.2 ms */
}

/* fRetryTimeout coding: 0 = none, n = 100 us * 2^(n-1) */
static uint32_t pn_timeout_fc(const Pn532* pn) {
    uint8_t n = pn->retry_timeout;
    if(n == 0 || n > 16) return 13560000U; /* "no timeout": bounded to 1 s */
    uint64_t us = 100ULL << (n - 1);
    return (uint32_t)(us * 1356U / 100U);
}

static uint8_t pn_read_reg(Reader* r, uint16_t addr) {
    if(addr < CIU_BASE || addr > CIU_BASE + 0x3F) return 0x00;
    uint8_t i = (uint8_t)(addr - CIU_BASE);
    if(i == CIU_STATUS2) {
        return (uint8_t)((r->pn532.ciu[i] & ~0x08) | (r->picc.crypto ? 0x08 : 0x00));
    }
    return r->pn532.ciu[i];
}

static void pn_write_reg(Reader* r, uint16_t addr, uint8_t v) {
    if(addr < CIU_BASE || addr > CIU_BASE + 0x3F) return;
    uint8_t i = (uint8_t)(addr - CIU_BASE);
    if(i == CIU_STATUS2 && !(v & 0x08)) r->picc.crypto = false;
    r->pn532.ciu[i] = v;
}

/* TargetData as returned by InListPassiveTarget, per modulation */
static size_t pn_target_data(Reader* r, uint8_t* o) {
    Picc* p = &r->picc;
    size_t n = 0;
    o[n++] = 0x01; /* Tg */
    switch(p->type) {
    case PiccTypeB:
        memcpy(&o[n], p->atqb, 12);
        n += 12;
        o[n++] = p->attrib_res_len;
        memcpy(&o[n], p->attrib_res, p->attrib_res_len);
        n += p->attrib_res_len;
        break;
    case PiccTypeFelica:
        memcpy(&o[n], p->pol_res, p->pol_res_len); /* POL_RES length, 01, IDm, PMm, [SC] */
        n += p->pol_res_len;
        break;
    default:
        o[n++] = p->atqa[1]; /* SENS_RES, MSB first as the PN532 reports it */
        o[n++] = p->atqa[0];
        o[n++] = p->sak;
        o[n++] = p->uid_len;
        memcpy(&o[n], p->uid, p->uid_len);
        n += p->uid_len;
        if(p->iso_dep) {
            memcpy(&o[n], p->ats, p->ats_len);
            n += p->ats_len;
        }
        break;
    }
    return n;
}

static uint8_t pn_retries(const Pn532* pn) {
    uint8_t tries = pn->rf_retries == 0xFF ? 3 : (uint8_t)(pn->rf_retries + 1);
    return tries > 5 ? 5 : tries;
}

static void pn_target_set(Reader* r, uint8_t br, uint8_t type) {
    r->pn532.target_active = true;
    r->pn532.target_br = br;
    r->pn532.target_type = type;
    reader_card_activated(r);
}

static void pn_release_current(Reader* r) {
    Picc* p = &r->picc;
    if(!p->present) return;
    if(p->iso_dep) picc_deselect(p);
    picc_halt(p);
}

static bool pn_activate_a(Reader* r, const uint8_t* want_uid, size_t want_len) {
    if(!r->field_on) return false;
    bool rats = (r->pn532.params & PN_PARAM_AUTO_RATS) != 0;
    for(uint8_t t = 0; t < pn_retries(&r->pn532); t++) {
        /* HLTA + WUPA reaches cards in any state (idle, halted or still selected) */
        if(r->picc.type == PiccTypeA) pn_release_current(r);
        if(picc_activate_a(&r->picc, true, rats)) {
            if(want_len) {
                /* InitiatorData may carry cascade tags: compare the UID bytes only */
                uint8_t uid[10];
                size_t ul = 0;
                for(size_t i = 0; i < want_len && ul < sizeof(uid); i++) {
                    bool ct = want_uid[i] == 0x88 && (i == 0 || i == 4) && want_len > 4;
                    if(!ct) uid[ul++] = want_uid[i];
                }
                if(ul != r->picc.uid_len || memcmp(uid, r->picc.uid, ul) != 0) return false;
            }
            pn_target_set(r, 0x00, PN_TYPE_A_B);
            return true;
        }
        furi_delay_ms(20);
    }
    return false;
}

static bool pn_activate_b(Reader* r, uint8_t afi) {
    if(!r->field_on) return false;
    for(uint8_t t = 0; t < pn_retries(&r->pn532); t++) {
        if(r->picc.type == PiccTypeB) pn_release_current(r);
        if(picc_activate_b(&r->picc, afi, true)) {
            pn_target_set(r, 0x00, PN_TYPE_A_B);
            return true;
        }
        furi_delay_ms(20);
    }
    return false;
}

static bool pn_activate_felica(Reader* r, const uint8_t req[5]) {
    if(!r->field_on) return false;
    for(uint8_t t = 0; t < pn_retries(&r->pn532); t++) {
        if(picc_activate_felica(&r->picc, req)) {
            pn_target_set(r, 0x01, PN_TYPE_FELICA);
            return true;
        }
        furi_delay_ms(20);
    }
    return false;
}

static bool pn_value_operand(Reader* r, const uint8_t* v4) {
    uint8_t operand[6];
    memcpy(operand, v4, 4);
    crc_append(CrcTypeA, operand, 4);
    uint8_t rx[RF_MAX_FRAME];
    uint16_t bits = 0;
    RfStatus st = picc_raw(&r->picc, operand, 48, rx, &bits, PICC_MFC_FWT_FC / 2);
    /* No answer is the success case; a 4-bit NAK is a failure */
    return !(st == RfStatusOk && bits == 4 && (rx[0] & 0x0F) != 0x0A);
}

static uint8_t pn_data_exchange(
    Reader* r,
    const uint8_t* d,
    size_t dl,
    uint8_t* out,
    size_t* ol,
    size_t max) {
    Picc* p = &r->picc;
    *ol = 0;
    if(dl == 0) return PN_ST_INVALID;

    if(p->iso_dep) {
        return picc_isodep_apdu(p, d, dl, out, ol, max) ? PN_ST_OK : PN_ST_TIMEOUT;
    }
    if(p->type == PiccTypeFelica) {
        /* Frame built by the host (Len, Cmd, Data); CRC by the PN532 */
        if(d[0] != dl) return PN_ST_INVALID;
        return picc_felica_exchange(p, d, dl, out, ol, pn_timeout_fc(&r->pn532)) ? PN_ST_OK :
                                                                                     PN_ST_TIMEOUT;
    }
    if(p->type != PiccTypeA) return PN_ST_WRONG_CTX;

    switch(d[0]) {
    case 0x60:
    case 0x61: /* MIFARE authentication: cmd block key[6] uid[4] */
        if(dl != 12) return PN_ST_INVALID;
        return picc_mfc_auth(p, d[0], d[1], &d[2]) ? PN_ST_OK : PN_ST_MFC_AUTH;
    case 0xA0: /* MIFARE 16-byte write: both phases handled by the PN532 */
        if(dl == 18) {
            bool ok = picc_ack_frame(p, d, 2, PICC_MFC_FWT_FC) &&
                      picc_ack_frame(p, &d[2], 16, PICC_MFC_FWT_FC);
            if(!ok) p->crypto = false;
            return ok ? PN_ST_OK : PN_ST_MFC_AUTH;
        }
        return PN_ST_INVALID;
    case 0xC0:
    case 0xC1:
    case 0xC2: /* decrement / increment / restore with operand */
        if(dl == 6) {
            bool ok = picc_ack_frame(p, d, 2, PICC_MFC_FWT_FC) && pn_value_operand(r, &d[2]);
            if(!ok) p->crypto = false;
            return ok ? PN_ST_OK : PN_ST_MFC_AUTH;
        }
        return PN_ST_INVALID;
    case 0xB0: /* transfer */
    case 0xA2: /* 4-byte write */
        if((d[0] == 0xB0 && dl == 2) || (d[0] == 0xA2 && dl == 6)) {
            bool ok = picc_ack_frame(p, d, dl, PICC_MFC_FWT_FC);
            if(!ok) p->crypto = false;
            return ok ? PN_ST_OK : PN_ST_MFC_AUTH;
        }
        return PN_ST_INVALID;
    default:
        break;
    }

    size_t n = 0;
    uint8_t rx[RF_MAX_FRAME];
    if(!picc_frame(p, d, dl, rx, &n, pn_timeout_fc(&r->pn532))) {
        p->crypto = false;
        return PN_ST_TIMEOUT;
    }
    if(n > max) n = max;
    memcpy(out, rx, n);
    *ol = n;
    return PN_ST_OK;
}

static uint8_t pn_communicate_thru(
    Reader* r,
    const uint8_t* d,
    size_t dl,
    uint8_t* out,
    size_t* ol,
    size_t max) {
    Pn532* pn = &r->pn532;
    Picc* p = &r->picc;
    *ol = 0;
    if(dl == 0 || dl + 2 > RF_MAX_FRAME) return PN_ST_INVALID;

    if(p->type == PiccTypeFelica || p->type == PiccTypeB) {
        /* CRC added and checked by the PN532 at these rates */
        size_t n = 0;
        uint8_t rx[RF_MAX_FRAME];
        if(!picc_frame(p, d, dl, rx, &n, pn_timeout_fc(pn))) return PN_ST_TIMEOUT;
        if(n > max) n = max;
        memcpy(out, rx, n);
        *ol = n;
        return PN_ST_OK;
    }

    if(pn->ciu[CIU_MANUAL_RCV] & 0x10) {
        /* Host-supplied parity bits (ParityDisable) are not supported */
        return PN_ST_INVALID;
    }

    uint8_t tx[RF_MAX_FRAME];
    memcpy(tx, d, dl);
    uint8_t last = pn->ciu[CIU_BIT_FRAMING] & 0x07;
    uint16_t bits = last ? (uint16_t)((dl - 1) * 8U + last) : (uint16_t)(dl * 8U);
    if((pn->ciu[CIU_TX_MODE] & 0x80) && last == 0) {
        crc_append(CrcTypeA, tx, dl);
        bits += 16;
    }

    uint8_t rx[RF_MAX_FRAME];
    uint16_t rbits = 0;
    RfStatus st = picc_raw(p, tx, bits, rx, &rbits, pn_timeout_fc(pn));
    if(st == RfStatusTimeout || st == RfStatusAuthFailed) return PN_ST_TIMEOUT;

    uint8_t status = PN_ST_OK;
    if(st == RfStatusParityError) status = PN_ST_PARITY;
    if(st == RfStatusCollision) status = PN_ST_COLLISION;
    if((pn->ciu[CIU_RX_MODE] & 0x80) && rbits >= 8) {
        if(rbits % 8U == 0 && crc_check(CrcTypeA, rx, rbits / 8U)) {
            rbits -= 16;
        } else {
            status = PN_ST_CRC;
        }
    }
    pn->ciu[CIU_CONTROL] = (uint8_t)((pn->ciu[CIU_CONTROL] & 0xF8) | (rbits % 8U));
    size_t n = (rbits + 7U) / 8U;
    if(n > max) n = max;
    memcpy(out, rx, n);
    *ol = n;
    return status;
}

/* InAutoPoll: try one requested type, return the output type code or 0xFF */
static uint8_t pn_autopoll_type(Reader* r, uint8_t type) {
    static const uint8_t felica_req[5] = {0x00, 0xFF, 0xFF, 0x00, 0x00};
    switch(type) {
    case 0x00: /* generic 106 kbit/s type A */
    case 0x10: /* MIFARE */
    case 0x20: /* ISO14443-4A */
        if(!pn_activate_a(r, NULL, 0)) return 0xFF;
        if(type == 0x20 && !r->picc.iso_dep) return 0xFF;
        if(type == 0x10 && r->picc.iso_dep) return 0xFF;
        return r->picc.iso_dep ? 0x20 : 0x10;
    case 0x01: /* generic 212 kbit/s */
    case 0x11: /* FeliCa 212 */
        return pn_activate_felica(r, felica_req) ? 0x11 : 0xFF;
    case 0x03: /* ISO14443-4B */
    case 0x23:
        return pn_activate_b(r, 0x00) && r->picc.iso_dep ? 0x23 : 0xFF;
    default: /* 424 kbit/s, Jewel and DEP are not reachable */
        return 0xFF;
    }
}

size_t pn532_command(Reader* r, const uint8_t* c, size_t len, uint8_t* o, size_t max) {
    if(len < 2 || c[0] != 0xD4 || max < 64) return 0;
    Pn532* pn = &r->pn532;
    uint8_t cmd = c[1];
    const uint8_t* p = &c[2];
    size_t pl = len - 2;
    size_t n = 0;
    o[n++] = 0xD5;
    o[n++] = (uint8_t)(cmd + 1);

    switch(cmd) {
    case 0x02: /* GetFirmwareVersion: PN532 v1.6, ISO18092 + 14443 A/B */
        o[n++] = 0x32;
        o[n++] = 0x01;
        o[n++] = 0x06;
        o[n++] = 0x07;
        break;

    case 0x04: /* GetGeneralStatus */
        o[n++] = pn->last_error;
        pn->last_error = 0;
        o[n++] = 0x00; /* no external RF field detected */
        o[n++] = pn->target_active ? 0x01 : 0x00;
        if(pn->target_active) {
            o[n++] = 0x01; /* Tg */
            o[n++] = pn->target_br; /* BrRx */
            o[n++] = pn->target_br; /* BrTx */
            o[n++] = pn->target_type;
        }
        o[n++] = 0x80; /* SAM status: CLAD line high */
        break;

    case 0x06: /* ReadRegister */
        for(size_t i = 0; i + 1 < pl && n < max; i += 2) {
            o[n++] = pn_read_reg(r, (uint16_t)((p[i] << 8) | p[i + 1]));
        }
        break;

    case 0x08: /* WriteRegister */
        for(size_t i = 0; i + 2 < pl; i += 3) {
            pn_write_reg(r, (uint16_t)((p[i] << 8) | p[i + 1]), p[i + 2]);
        }
        break;

    case 0x12: /* SetParameters */
        if(pl < 1) return 0;
        pn->params = p[0];
        break;

    case 0x14: /* SAMConfiguration */
        break;

    case 0x16: /* PowerDown */
        o[n++] = 0x00;
        break;

    case 0x32: /* RFConfiguration */
        if(pl >= 2 && p[0] == 0x01) {
            reader_set_field(r, (p[1] & 0x01) != 0);
            if(!(p[1] & 0x01)) pn->target_active = false;
        } else if(pl >= 4 && p[0] == 0x02) {
            pn->retry_timeout = p[3]; /* fRetryTimeout */
        } else if(pl >= 4 && p[0] == 0x05) {
            pn->rf_retries = p[3]; /* MxRtyPassiveActivation */
        }
        break;

    case 0x4A: { /* InListPassiveTarget */
        if(pl < 2 || p[0] == 0 || p[0] > 2) return 0;
        uint8_t brty = p[1];
        bool found = false;
        if(brty == 0x00) {
            found = pn_activate_a(r, &p[2], pl - 2);
        } else if(brty == 0x01) {
            if(pl < 7) return 0; /* POL_REQ payload is mandatory */
            found = pn_activate_felica(r, &p[2]);
        } else if(brty == 0x03) {
            if(pl < 3) return 0; /* AFI is mandatory */
            found = pn_activate_b(r, p[2]);
        } else if(brty > 0x04) {
            return 0;
        }
        /* 0x02 (FeliCa 424) and 0x04 (Jewel) find nothing at reachable rates */
        if(!found) {
            o[n++] = 0x00;
            pn->last_error = PN_ST_TIMEOUT;
        } else {
            o[n++] = 0x01;
            n += pn_target_data(r, &o[n]);
        }
        break;
    }

    case 0x60: { /* InAutoPoll */
        if(pl < 3) return 0;
        uint8_t polls = (p[0] == 0xFF || p[0] == 0) ? 3 : (p[0] > 3 ? 3 : p[0]);
        uint8_t found_type = 0xFF;
        for(uint8_t i = 0; i < polls && found_type == 0xFF; i++) {
            for(size_t t = 2; t < pl && found_type == 0xFF; t++) {
                found_type = pn_autopoll_type(r, p[t]);
            }
        }
        if(found_type == 0xFF) {
            o[n++] = 0x00;
        } else {
            o[n++] = 0x01;
            o[n++] = found_type;
            size_t len_pos = n++;
            size_t tl = pn_target_data(r, &o[n]);
            o[len_pos] = (uint8_t)tl;
            n += tl;
        }
        break;
    }

    case 0x40: { /* InDataExchange */
        if(pl < 1 || (p[0] & 0x0F) != 0x01 || !pn->target_active) {
            o[n++] = PN_ST_WRONG_CTX;
            pn->last_error = PN_ST_WRONG_CTX;
            break;
        }
        size_t ol = 0;
        uint8_t st = pn_data_exchange(r, &p[1], pl - 1, &o[n + 1], &ol, max - n - 1);
        o[n] = st;
        if(st) pn->last_error = st;
        n += 1 + ol;
        break;
    }

    case 0x42: { /* InCommunicateThru */
        size_t ol = 0;
        uint8_t st = pn_communicate_thru(r, p, pl, &o[n + 1], &ol, max - n - 1);
        o[n] = st;
        if(st) pn->last_error = st;
        n += 1 + ol;
        break;
    }

    case 0x44: /* InDeselect */
    case 0x52: /* InRelease */
        if(pn->target_active) {
            if(r->picc.type != PiccTypeFelica) pn_release_current(r);
            pn->target_active = false;
        }
        o[n++] = 0x00;
        break;

    case 0x54: { /* InSelect */
        bool ok = false;
        if(r->picc.uid_len) {
            switch(r->picc.type) {
            case PiccTypeB:
                ok = pn_activate_b(r, 0x00);
                break;
            case PiccTypeFelica:
                ok = true; /* FeliCa needs no selection */
                pn->target_active = true;
                break;
            default: {
                uint8_t uid[10];
                uint8_t ul = r->picc.uid_len;
                memcpy(uid, r->picc.uid, ul);
                ok = pn_activate_a(r, uid, ul);
                break;
            }
            }
        }
        o[n++] = ok ? PN_ST_OK : PN_ST_WRONG_CTX;
        break;
    }

    case 0x4E: /* InPSL: only the current rate is available; accept */
        o[n++] = 0x00;
        break;

    default:
        return 0; /* unsupported: reader answers 63 00 */
    }
    return n;
}
