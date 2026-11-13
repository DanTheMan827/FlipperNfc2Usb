#include "reader_i.h"
#include "crc.h"

#include <furi_hal_speaker.h>
#include <furi_hal_version.h>
#include <notification/notification_messages.h>
#include <usb_std.h>
#include <string.h>
#include <stdio.h>

#define TAG "Reader"

#define RD_STOP      (1U << 7)
#define RD_ALL_FLAGS (CCID_FLAG_MSG | CCID_FLAG_TX | CCID_FLAG_CONFIG | RD_STOP)

#define PRESENCE_MS     250 /* idle time before a presence check */
#define PRESENCE_MFC_MS 1000 /* MIFARE Classic: the check re-selects and ends Crypto1 */
#define DIRECT_MODE_HOLD_MS 3000
#define BEEP_SHORT_MS       100
#define BEEP_MAX_MS         3000
#define LED_SEQUENCE_MAX_MS 5000

/* CCID message types */
#define PC_TO_RDR_SET_PARAMETERS   0x61
#define PC_TO_RDR_ICC_POWER_ON     0x62
#define PC_TO_RDR_ICC_POWER_OFF    0x63
#define PC_TO_RDR_GET_SLOT_STATUS  0x65
#define PC_TO_RDR_ESCAPE           0x6B
#define PC_TO_RDR_GET_PARAMETERS   0x6C
#define PC_TO_RDR_RESET_PARAMETERS 0x6D
#define PC_TO_RDR_ICC_CLOCK        0x6E
#define PC_TO_RDR_XFR_BLOCK        0x6F

#define RDR_TO_PC_DATA_BLOCK  0x80
#define RDR_TO_PC_SLOT_STATUS 0x81
#define RDR_TO_PC_PARAMETERS  0x82
#define RDR_TO_PC_ESCAPE      0x83

#define CCID_ERR_CMD_NOT_SUPPORTED 0x00
#define CCID_ERR_BAD_SLOT          0x05
#define CCID_ERR_ICC_MUTE          0xFE
#define CCID_STATUS_FAILED         0x40

#define SW_OK            0x9000
#define SW_FAILED        0x6300
#define SW_WRONG_LENGTH  0x6700
#define SW_NOT_SUPPORTED 0x6A81
#define SW_WRONG_P1P2    0x6B00
#define SW_END_OF_DATA   0x6282
#define SW_SECURITY      0x6982

#define ACR122U_FIRMWARE  "ACR122U207"
#define ACR1552U_FIRMWARE "ACR1552 R FW 1.07.00"

/* FeliCa polling used during auto-polling: wildcard system code, request system code */
static const uint8_t felica_poll_req[5] = {0x00, 0xFF, 0xFF, 0x01, 0x00};

/* ------------------------------------------------------------------------ */
/* USB identities                                                            */
/* ------------------------------------------------------------------------ */

static const struct usb_string_descriptor str_acs = USB_STRING_DESC("ACS");
static const struct usb_string_descriptor str_acr122u = USB_STRING_DESC("ACR122U PICC Interface");
static const struct usb_string_descriptor str_acr1552u = USB_STRING_DESC("ACR1552 1S CL Reader");
static const struct usb_string_descriptor str_acr1552u_picc =
    USB_STRING_DESC("ACR1552 1S CL Reader PICC");
static const struct usb_string_descriptor str_acr1552u_sam =
    USB_STRING_DESC("ACR1552 1S CL Reader SAM");

static struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wString[16];
} __attribute__((packed, aligned(2))) str_serial;

static char serial_ascii[17];

static void reader_build_serial(void) {
    static const char hex[] = "0123456789ABCDEF";
    const uint8_t* uid = furi_hal_version_uid();
    size_t len = furi_hal_version_uid_size();
    if(len > 8) len = 8;
    str_serial.bLength = (uint8_t)(2 + len * 4);
    str_serial.bDescriptorType = USB_DTYPE_STRING;
    for(size_t i = 0; i < len; i++) {
        serial_ascii[i * 2] = hex[uid[i] >> 4];
        serial_ascii[i * 2 + 1] = hex[uid[i] & 0x0F];
    }
    serial_ascii[len * 2] = 0;
    for(size_t i = 0; i < len * 2; i++) str_serial.wString[i] = (uint16_t)serial_ascii[i];
}

static void reader_usb_config(Reader* r, CcidUsbConfig* c) {
    memset(c, 0, sizeof(*c));
    c->vid = 0x072F;
    c->bcd_usb = 0x0200;
    c->str_manufacturer = &str_acs;
    c->voltage_support = 0x07;
    c->protocols = 0x00000003; /* T=0, T=1 */
    c->default_clock = 4000;
    c->max_clock = 4000;
    c->data_rate = 9600;
    c->max_data_rate = 344086;
    c->max_ifsd = 254;
    /* Automatic parameter negotiation + short APDU level exchange */
    c->features = 0x00020040;
    c->max_message_length = 271;
    if(r->model == ReaderModelAcr122u) {
        c->pid = 0x2200;
        c->bcd_device = 0x0207;
        c->max_power_ma = 200;
        c->str_product = &str_acr122u;
        c->str_serial = NULL;
        c->interfaces = 1;
    } else {
        c->pid = 0x2303;
        c->bcd_device = 0x0107;
        c->max_power_ma = 300;
        c->str_product = &str_acr1552u;
        reader_build_serial();
        c->str_serial = r->sn_in_descriptor ? &str_serial : NULL;
        c->interfaces = 2;
        c->str_interface[READER_IFACE_PICC] = &str_acr1552u_picc;
        c->str_interface[READER_IFACE_SAM] = &str_acr1552u_sam;
    }
}

const char* reader_model_name(ReaderModel model) {
    return model == ReaderModelAcr122u ? "ACR122U" : "ACR1552U";
}

/* ------------------------------------------------------------------------ */
/* Indicators                                                                */
/* ------------------------------------------------------------------------ */

static void reader_led(Reader* r, uint8_t state) {
    r->led_state = state & 0x03;
    if(r->model == ReaderModelAcr122u) {
        /* Bi-colour LED: bit0 red, bit1 green */
        notification_message(
            r->notification, (state & 0x01) ? &sequence_set_red_255 : &sequence_reset_red);
    } else {
        /* ACR1552U: bit0 blue, bit1 green */
        notification_message(
            r->notification, (state & 0x01) ? &sequence_set_blue_255 : &sequence_reset_blue);
    }
    notification_message(
        r->notification, (state & 0x02) ? &sequence_set_green_255 : &sequence_reset_green);
}

static void reader_beep_stop(Reader* r) {
    if(!r->beeping) return;
    furi_hal_speaker_stop();
    furi_hal_speaker_release();
    r->beeping = false;
}

static void reader_beep(Reader* r, uint32_t ms) {
    if(!r->buzzer_enabled || ms == 0) return;
    if(ms > BEEP_MAX_MS) ms = BEEP_MAX_MS;
    if(!r->beeping) {
        if(!furi_hal_speaker_acquire(30)) return;
        furi_hal_speaker_start(2700.0f, 0.6f);
        r->beeping = true;
    }
    r->beep_off_tick = furi_get_tick() + furi_ms_to_ticks(ms);
}

/* LED showing card state, per model defaults */
static void reader_status_led(Reader* r, bool card) {
    if(r->model == ReaderModelAcr122u) {
        reader_led(r, card ? 0x02 : 0x01);
        return;
    }
    uint8_t ui = r->ui_behaviour;
    uint8_t sel = (uint8_t)(((ui & 0x40) ? 0x01 : 0) | ((ui & 0x80) ? 0x02 : 0));
    bool on = card ? (ui & 0x04) != 0 : false;
    reader_led(r, on ? sel : 0x00);
}

/* ------------------------------------------------------------------------ */
/* ATR and card identification                                               */
/* ------------------------------------------------------------------------ */

static size_t atr_storage(uint8_t* a, uint8_t standard, uint16_t name) {
    static const uint8_t head[] = {
        0x3B, 0x8F, 0x80, 0x01, 0x80, 0x4F, 0x0C, 0xA0, 0x00, 0x00, 0x03, 0x06};
    memcpy(a, head, sizeof(head));
    size_t n = sizeof(head);
    a[n++] = standard;
    a[n++] = (uint8_t)(name >> 8);
    a[n++] = (uint8_t)(name & 0xFF);
    a[n++] = 0x00;
    a[n++] = 0x00;
    a[n++] = 0x00;
    a[n++] = 0x00;
    return n;
}

static size_t atr_hist(uint8_t* a, const uint8_t* hist, uint8_t hl) {
    if(hl > 15) hl = 15;
    size_t n = 0;
    a[n++] = 0x3B;
    a[n++] = (uint8_t)(0x80 | hl);
    a[n++] = 0x80;
    a[n++] = 0x01;
    memcpy(&a[n], hist, hl);
    return n + hl;
}

static bool reader_part4_atr_for_a(Reader* r) {
    const Picc* p = &r->picc;
    if(!p->iso_dep) return false;
    uint8_t sak = p->sak & 0x7F;
    bool mifare_emulation = (sak == 0x28 || sak == 0x38);
    if(r->model == ReaderModelAcr1552u && mifare_emulation) {
        return (r->polling_option & 0x80) != 0; /* "Enable Part 4 ATR for SmartMX" */
    }
    return true;
}

static uint16_t reader_name_from_sak(Reader* r) {
    uint8_t sak = r->picc.sak & 0x7F;
    switch(sak) {
    case 0x08:
    case 0x28:
        return 0x0001;
    case 0x18:
    case 0x38:
        return 0x0002;
    case 0x00:
        return 0x0003;
    case 0x09:
        return 0x0026;
    case 0x10:
        return r->model == ReaderModelAcr1552u ? 0x0038 : (uint16_t)(0xFF00 | sak);
    case 0x11:
        return r->model == ReaderModelAcr1552u ? 0x0039 : (uint16_t)(0xFF00 | sak);
    default:
        return (uint16_t)(0xFF00 | sak);
    }
}

/* ACR1552U "extra MIFARE type identification" for Ultralight family */
static void reader_identify_ultralight(Reader* r) {
    Picc* p = &r->picc;
    uint8_t get_version = 0x60;
    uint8_t rx[RF_MAX_FRAME];
    size_t n = 0;
    if(picc_frame(p, &get_version, 1, rx, &n, PICC_FDT_FC * 4) && n == 8 && rx[0] == 0x00 &&
       rx[1] == 0x04) {
        r->card_name = rx[2] == 0x03 ? 0x003D : 0x0003; /* Ultralight EV1 / NTAG */
        return;
    }
    /* Unsupported command left the card idle: wake it up again */
    if(!picc_reactivate(p, false)) return;
    uint8_t auth[2] = {0x1A, 0x00};
    if(picc_frame(p, auth, 2, rx, &n, PICC_FDT_FC * 4) && n == 9 && rx[0] == 0xAF) {
        r->card_name = 0x003A; /* Ultralight C */
    }
    picc_reactivate(p, false);
}

static uint16_t reader_15693_name(const Picc* p) {
    uint8_t mfg = p->uid[6];
    if(mfg == 0x04) { /* NXP */
        uint8_t kind = (uint8_t)((p->uid[4] >> 3) & 0x03);
        return kind == 0 ? 0x0014 : 0x0035; /* ICODE SLI / SLIX(2) */
    }
    if(mfg == 0x02) return 0x0013; /* ST LRI */
    if(mfg == 0x05) return 0x000E; /* Infineon my-d vicinity */
    return 0x0000;
}

static void reader_build_atr(Reader* r) {
    Picc* p = &r->picc;
    uint8_t* a = r->atr;
    size_t n;
    bool acr122 = r->model == ReaderModelAcr122u;

    switch(p->type) {
    case PiccTypeA:
        if(reader_part4_atr_for_a(r)) {
            n = atr_hist(a, &p->ats[p->hist_off], p->hist_len);
        } else {
            n = atr_storage(a, 0x03, r->card_name);
        }
        break;
    case PiccTypeB:
        if(acr122) {
            /* ACR122U: the whole ATQB (without CRC) as historical bytes */
            n = atr_hist(a, p->atqb, 12);
        } else {
            uint8_t h[8];
            memcpy(h, &p->atqb[5], 4); /* application data */
            memcpy(&h[4], &p->atqb[9], 3); /* protocol info */
            h[7] = (uint8_t)(p->attrib_res_len ? (p->attrib_res[0] & 0xF0) : 0x00); /* MBLI */
            n = atr_hist(a, h, 8);
        }
        break;
    case PiccTypeFelica:
        n = acr122 ? atr_storage(a, 0x03, 0xF011) : atr_storage(a, 0x11, 0x003B);
        break;
    case PiccType15693:
        n = atr_storage(a, 0x0B, reader_15693_name(p));
        break;
    case PiccTypeSt25tb:
        n = atr_storage(a, 0x06, 0x0007);
        break;
    case PiccTypePicopass:
        n = atr_storage(a, 0x0A, 0x0018);
        break;
    default:
        n = atr_storage(a, 0x03, 0x0000);
        break;
    }
    uint8_t tck = 0;
    for(size_t i = 1; i < n; i++) tck ^= a[i];
    a[n++] = tck;
    r->atr_len = (uint8_t)n;
}

static bool reader_wants_rats(Reader* r) {
    if(r->model == ReaderModelAcr122u) return (r->acr122_picc_param & 0x40) != 0;
    return true;
}

static bool reader_detect_a(Reader* r) {
    Picc* p = &r->picc;
    if(!picc_activate_a(p, false, false)) return false;
    bool rats = reader_wants_rats(r);
    uint8_t sak = p->sak & 0x7F;
    if(r->model == ReaderModelAcr1552u && (sak == 0x28 || sak == 0x38) &&
       !(r->polling_option & 0x80)) {
        rats = false; /* SmartMX with MIFARE emulation stays a MIFARE card */
    }
    if(rats && (p->sak & 0x20)) picc_rats(p); /* card is still selected: RATS directly */
    r->card_name = reader_name_from_sak(r);
    if(r->model == ReaderModelAcr1552u && (r->polling_option & 0x08) && picc_is_ultralight(p)) {
        reader_identify_ultralight(r);
    }
    return true;
}

static bool reader_detect(Reader* r) {
    Picc* p = &r->picc;
    if(r->model == ReaderModelAcr122u) {
        uint8_t prm = r->acr122_picc_param;
        if((prm & 0x01) && reader_detect_a(r)) return true;
        if((prm & 0x02) && picc_activate_b(p, 0x00, false)) return true;
        /* FeliCa 424K polling (bit 4) needs 424 kbit/s, not available here */
        if((prm & 0x08) && picc_activate_felica(p, felica_poll_req)) return true;
        return false;
    }
    uint8_t b1 = (uint8_t)(r->polling_type >> 8);
    uint8_t b0 = (uint8_t)(r->polling_type & 0xFF);
    if((b1 & 0x01) && reader_detect_a(r)) return true;
    if((b1 & 0x02) && picc_activate_b(p, 0x00, false)) return true;
    if((b1 & 0x40) && picc_activate_st25tb(p)) return true;
    if((b1 & 0x04) && picc_activate_felica(p, felica_poll_req)) return true;
    if((b0 & 0x04) && picc_activate_15693(p)) return true;
    if((b0 & 0x02) && picc_activate_picopass(p)) return true;
    return false;
}

void reader_card_activated(Reader* r) {
    bool was_present = r->card_present;
    if(r->picc.type != PiccTypeA) r->card_name = 0;
    reader_build_atr(r);
    r->card_present = true;
    if(!was_present) {
        ccid_usb_notify_slot(READER_IFACE_PICC, true, true);
        bool beep = r->model == ReaderModelAcr122u ? r->acr122_beep_on_detect :
                                                     (r->ui_behaviour & 0x08) != 0;
        if(beep) reader_beep(r, BEEP_SHORT_MS);
        reader_status_led(r, true);
    }
}

void reader_card_lost(Reader* r) {
    bool was_present = r->card_present;
    r->card_present = false;
    r->card_powered = false;
    r->picc.present = false;
    r->picc.crypto = false;
    r->pn532.target_active = false;
    if(was_present) {
        ccid_usb_notify_slot(READER_IFACE_PICC, false, true);
        if(r->model == ReaderModelAcr1552u && (r->ui_behaviour & 0x10)) {
            reader_beep(r, BEEP_SHORT_MS);
        }
        reader_status_led(r, false);
    }
}

void reader_set_field(Reader* r, bool on) {
    r->field_wanted = on;
    bool effective = on && r->configured;
    if(effective == r->field_on) return;
    rf_set_field(r->rf, effective);
    r->field_on = effective;
    if(!effective) reader_card_lost(r);
}

static bool reader_polling_enabled(Reader* r) {
    if(r->model == ReaderModelAcr122u) return (r->acr122_picc_param & 0x80) != 0;
    return r->rf_mode == 0x01 && (r->polling_option & 0x01) && !r->session.active;
}

static uint32_t reader_poll_interval_ms(Reader* r) {
    if(r->model == ReaderModelAcr122u) return (r->acr122_picc_param & 0x20) ? 250 : 500;
    static const uint16_t interval[4] = {250, 500, 1000, 2500};
    return (r->polling_option & 0x02) ? interval[(r->polling_option >> 4) & 0x03] : 250;
}

static void reader_set_last(Reader* r, const char* what, const uint8_t* d, size_t n);

/* Record why a card was dropped on the status screen's last line */
static void reader_note_loss(Reader* r, const char* what) {
    char msg[40];
    r->losses++;
    snprintf(
        msg,
        sizeof(msg),
        "%s@%s x%lu",
        what,
        r->picc.fail ? r->picc.fail : "?",
        (unsigned long)r->losses);
    reader_set_last(r, msg, NULL, 0);
}

static void reader_poll(Reader* r) {
    uint32_t now = furi_get_tick();
    if(!r->field_on || r->session.active) return;
    if((int32_t)(now - r->direct_mode_until_tick) < 0) return;
    /* A card that just stopped answering is checked at once */
    bool urgent = r->card_present && r->picc.silent;
    if(!urgent && (int32_t)(now - r->next_poll_tick) < 0) return;

    if(!r->card_present) {
        r->next_poll_tick = now + furi_ms_to_ticks(reader_poll_interval_ms(r));
        if(!reader_polling_enabled(r)) return;
        if(reader_detect(r)) reader_card_activated(r);
        return;
    }

    /* Removal is detected whatever the host is doing, like on the real
     * readers. Only answers from the card itself postpone the check; a card
     * that stopped answering is checked straight away. */
    uint32_t idle_ms = picc_is_mifare_classic(&r->picc) ? PRESENCE_MFC_MS : PRESENCE_MS;
    uint32_t idle = furi_ms_to_ticks(idle_ms);
    if(!r->picc.silent && now - r->picc.last_answer_tick < idle) {
        r->next_poll_tick = r->picc.last_answer_tick + idle;
        return;
    }
    if(!picc_presence(&r->picc)) {
        reader_card_lost(r);
        reader_note_loss(r, "Lost");
    }
    r->next_poll_tick = now + idle;
}

/* ------------------------------------------------------------------------ */
/* APDU helpers                                                              */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t cla;
    uint8_t ins;
    uint8_t p1;
    uint8_t p2;
    const uint8_t* data;
    size_t lc;
    size_t le; /* 256 when Le = 00 */
    bool has_le;
} Apdu;

static bool apdu_parse(const uint8_t* a, size_t n, Apdu* x) {
    if(n < 4) return false;
    memset(x, 0, sizeof(*x));
    x->cla = a[0];
    x->ins = a[1];
    x->p1 = a[2];
    x->p2 = a[3];
    if(n == 4) return true;
    if(n == 5) {
        x->has_le = true;
        x->le = a[4] ? a[4] : 256;
        return true;
    }
    size_t lc = a[4];
    if(lc == 0) return false; /* extended APDUs are not supported */
    x->lc = lc;
    x->data = &a[5];
    if(n == 5 + lc) return true;
    if(n == 6 + lc) {
        x->has_le = true;
        x->le = a[n - 1] ? a[n - 1] : 256;
        return true;
    }
    return false;
}

static size_t sw(uint8_t* out, size_t n, uint16_t status) {
    out[n] = (uint8_t)(status >> 8);
    out[n + 1] = (uint8_t)(status & 0xFF);
    return n + 2;
}

static bool mfc_read_block(Reader* r, uint8_t block, uint8_t* out16) {
    uint8_t cmd[2] = {0x30, block};
    size_t n = 0;
    uint8_t rx[RF_MAX_FRAME];
    if(!picc_frame(&r->picc, cmd, 2, rx, &n, PICC_FDT_FC * 4) || n != 16) {
        r->picc.crypto = false;
        return false;
    }
    memcpy(out16, rx, 16);
    return true;
}

static bool mfc_write_block(Reader* r, uint8_t block, const uint8_t* data16) {
    uint8_t cmd[2] = {0xA0, block};
    if(!picc_ack_frame(&r->picc, cmd, 2, PICC_MFC_FWT_FC) ||
       !picc_ack_frame(&r->picc, data16, 16, PICC_MFC_FWT_FC)) {
        r->picc.crypto = false;
        return false;
    }
    return true;
}

static bool ul_write_page(Reader* r, uint8_t page, const uint8_t* data4) {
    uint8_t cmd[6] = {0xA2, page, data4[0], data4[1], data4[2], data4[3]};
    return picc_ack_frame(&r->picc, cmd, 6, PICC_MFC_FWT_FC);
}

/* INC/DEC/RESTORE: command (ACK), operand (no answer expected) */
static bool mfc_value_op(Reader* r, uint8_t op, uint8_t block, uint32_t value) {
    uint8_t cmd[2] = {op, block};
    if(!picc_ack_frame(&r->picc, cmd, 2, PICC_MFC_FWT_FC)) {
        r->picc.crypto = false;
        return false;
    }
    uint8_t operand[6] = {
        (uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    crc_append(CrcTypeA, operand, 4);
    uint16_t bits = 0;
    uint8_t rx[RF_MAX_FRAME];
    RfStatus st = picc_raw(&r->picc, operand, 48, rx, &bits, PICC_MFC_FWT_FC / 2);
    if(st == RfStatusOk && bits == 4 && (rx[0] & 0x0F) != 0x0A) {
        r->picc.crypto = false;
        return false; /* NAK */
    }
    return true;
}

static bool mfc_transfer(Reader* r, uint8_t block) {
    uint8_t cmd[2] = {0xB0, block};
    if(!picc_ack_frame(&r->picc, cmd, 2, PICC_MFC_FWT_FC)) {
        r->picc.crypto = false;
        return false;
    }
    return true;
}

static bool mfc_is_trailer(uint16_t block) {
    return block < 128 ? (block % 4) == 3 : (block % 16) == 15;
}

/* ------------------------------------------------------------------------ */
/* PC/SC Part 3 pseudo-APDUs                                                 */
/* ------------------------------------------------------------------------ */

static size_t data_with_le(const Apdu* a, uint8_t* out, const uint8_t* d, size_t len) {
    size_t le = a->has_le ? a->le : 256;
    if(le != 256 && le < len) return sw(out, 0, (uint16_t)(0x6C00 | len));
    memmove(out, d, len);
    return sw(out, len, (le != 256 && le > len) ? SW_END_OF_DATA : SW_OK);
}

static size_t cmd_get_data(Reader* r, const Apdu* a, uint8_t* out) {
    Picc* p = &r->picc;
    if(!r->card_present) return sw(out, 0, SW_FAILED);
    uint8_t buf[64];
    size_t n = 0;
    switch(a->p1) {
    case 0x00: /* UID / PUPI / IDm / SN */
        memcpy(buf, p->uid, p->uid_len);
        n = p->uid_len;
        break;
    case 0x01: /* full ATS of an ISO14443-4A card */
        if(p->type != PiccTypeA || !p->iso_dep) return sw(out, 0, SW_NOT_SUPPORTED);
        memcpy(buf, p->ats, p->ats_len);
        n = p->ats_len;
        break;
    case 0x02:
    case 0x80:
        if(r->model != ReaderModelAcr1552u) return sw(out, 0, SW_WRONG_P1P2);
        switch(p->type) {
        case PiccTypeA:
            buf[n++] = p->atqa[0];
            buf[n++] = p->atqa[1];
            memcpy(&buf[n], p->uid, p->uid_len);
            n += p->uid_len;
            if(a->p1 == 0x02) {
                buf[n++] = p->sak;
            } else {
                memcpy(&buf[n], p->sak_all, p->sak_count);
                n += p->sak_count;
            }
            break;
        case PiccTypeB:
            memcpy(buf, p->atqb, 12);
            n = 12;
            break;
        case PiccTypeFelica:
            if(a->p1 != 0x80) return sw(out, 0, SW_WRONG_P1P2);
            memcpy(buf, &p->pol_res[1], 17); /* response code + IDm + PMm */
            n = 17;
            break;
        case PiccTypeSt25tb:
            if(a->p1 != 0x80) return sw(out, 0, SW_WRONG_P1P2);
            memcpy(buf, p->uid, 8);
            buf[8] = p->chip_id;
            n = 9;
            break;
        case PiccType15693:
            if(a->p1 != 0x80) return sw(out, 0, SW_WRONG_P1P2);
            buf[0] = p->dsfid;
            memcpy(&buf[1], p->uid, 8);
            n = 9;
            break;
        default:
            return sw(out, 0, SW_NOT_SUPPORTED);
        }
        break;
    default:
        return sw(out, 0, SW_WRONG_P1P2);
    }
    return data_with_le(a, out, buf, n);
}

static size_t cmd_load_key(Reader* r, const Apdu* a, uint8_t* out) {
    if(a->p1 != 0x00 || a->p2 > 0x01) return sw(out, 0, SW_FAILED);
    bool ok_len = a->lc == 6 ||
                  (r->model == ReaderModelAcr1552u && (a->lc == 0x10 || a->lc == 0x16));
    if(!ok_len) return sw(out, 0, r->model == ReaderModelAcr122u ? SW_FAILED : SW_WRONG_LENGTH);
    memcpy(r->keys[a->p2], a->data, a->lc);
    r->key_len[a->p2] = (uint8_t)a->lc;
    return sw(out, 0, SW_OK);
}

static size_t do_auth(Reader* r, uint16_t block, uint8_t key_type, uint8_t key_no, uint8_t* out) {
    if(!r->card_present || !picc_is_mifare_classic(&r->picc)) {
        /* Ultralight C (80h) / DESFire (0Ah) 3DES authentication is not implemented */
        return sw(out, 0, (key_type == 0x80 || key_type == 0x0A) ? SW_NOT_SUPPORTED : SW_FAILED);
    }
    if(block > 0xFF || key_no > 1 || r->key_len[key_no] < 6) return sw(out, 0, SW_FAILED);
    if(key_type != 0x60 && key_type != 0x61) return sw(out, 0, SW_FAILED);
    bool ok = picc_mfc_auth(&r->picc, key_type, (uint8_t)block, r->keys[key_no]);
    return sw(out, 0, ok ? SW_OK : SW_FAILED);
}

static size_t cmd_general_auth(Reader* r, const Apdu* a, uint8_t* out) {
    if(a->lc != 5 || a->data[0] != 0x01) return sw(out, 0, SW_FAILED);
    uint16_t block = (uint16_t)((a->data[1] << 8) | a->data[2]);
    return do_auth(r, block, a->data[3], a->data[4], out);
}

static size_t cmd_read_binary(Reader* r, const Apdu* a, uint8_t* out) {
    Picc* p = &r->picc;
    if(!r->card_present) return sw(out, 0, SW_FAILED);
    bool acr122 = r->model == ReaderModelAcr122u;
    uint8_t mode = acr122 ? 0x08 : (uint8_t)(a->p1 >> 4);
    uint16_t addr = acr122 ? (uint16_t)((a->p1 << 8) | a->p2) :
                             (uint16_t)(((a->p1 & 0x0F) << 8) | a->p2);
    size_t le = a->has_le ? a->le : 16;
    if(acr122 && le > 16) return sw(out, 0, SW_FAILED);
    size_t got = 0;
    uint8_t blk[32];

    if(picc_is_ultralight(p)) {
        while(got < le) {
            if(!mfc_read_block(r, (uint8_t)(addr + got / 4), blk)) return sw(out, 0, SW_FAILED);
            size_t take = le - got > 16 ? 16 : le - got;
            memcpy(&out[got], blk, take);
            got += take;
        }
    } else if(picc_is_mifare_classic(p)) {
        uint16_t b = addr;
        while(got < le) {
            if(b > 0xFF) return sw(out, 0, SW_FAILED);
            if(mode == 0x00 && mfc_is_trailer(b)) {
                b++;
                continue;
            }
            if(!mfc_read_block(r, (uint8_t)b, blk)) return sw(out, 0, SW_FAILED);
            size_t take = le - got > 16 ? 16 : le - got;
            memcpy(&out[got], blk, take);
            got += take;
            b++;
        }
    } else if(!acr122 && p->type == PiccType15693) {
        uint16_t b = addr;
        while(got < le) {
            size_t bl = 0;
            if(!picc_15693_read_block(p, b++, blk, &bl) || bl == 0) return sw(out, 0, SW_FAILED);
            size_t take = le - got > bl ? bl : le - got;
            memcpy(&out[got], blk, take);
            got += take;
        }
    } else if(!acr122 && p->type == PiccTypeSt25tb) {
        uint16_t b = addr;
        while(got < le) {
            if(b > 0xFF || !picc_st25tb_read_block(p, (uint8_t)b++, blk)) {
                return sw(out, 0, SW_FAILED);
            }
            size_t take = le - got > 4 ? 4 : le - got;
            memcpy(&out[got], blk, take);
            got += take;
        }
    } else if(!acr122 && p->type == PiccTypePicopass) {
        uint16_t b = addr;
        while(got < le) {
            if(b > 0xFF || !picc_picopass_read_block(p, (uint8_t)b++, blk)) {
                return sw(out, 0, SW_FAILED);
            }
            size_t take = le - got > 8 ? 8 : le - got;
            memcpy(&out[got], blk, take);
            got += take;
        }
    } else {
        return sw(out, 0, acr122 ? SW_FAILED : SW_NOT_SUPPORTED);
    }
    return sw(out, got, SW_OK);
}

static size_t cmd_update_binary(Reader* r, const Apdu* a, uint8_t* out) {
    Picc* p = &r->picc;
    if(!r->card_present || a->lc == 0) return sw(out, 0, SW_FAILED);
    bool acr122 = r->model == ReaderModelAcr122u;
    uint8_t mode = acr122 ? 0x08 : (uint8_t)(a->p1 >> 4);
    uint16_t addr = acr122 ? (uint16_t)((a->p1 << 8) | a->p2) :
                             (uint16_t)(((a->p1 & 0x0F) << 8) | a->p2);

    if(picc_is_ultralight(p)) {
        if(a->lc % 4) return sw(out, 0, SW_FAILED);
        for(size_t i = 0; i < a->lc / 4; i++) {
            if(!ul_write_page(r, (uint8_t)(addr + i), &a->data[i * 4])) {
                return sw(out, 0, SW_FAILED);
            }
        }
    } else if(picc_is_mifare_classic(p)) {
        if(a->lc % 16) return sw(out, 0, SW_FAILED);
        uint16_t b = addr;
        for(size_t i = 0; i < a->lc / 16;) {
            if(b > 0xFF) return sw(out, 0, SW_FAILED);
            if(mode == 0x00 && mfc_is_trailer(b)) {
                b++;
                continue;
            }
            if(!mfc_write_block(r, (uint8_t)b, &a->data[i * 16])) return sw(out, 0, SW_FAILED);
            b++;
            i++;
        }
    } else if(!acr122 && p->type == PiccType15693) {
        size_t bs = p->v_block_size ? p->v_block_size : 4;
        if(a->lc % bs) return sw(out, 0, SW_WRONG_LENGTH);
        for(size_t i = 0; i < a->lc / bs; i++) {
            if(!picc_15693_write_block(p, (uint16_t)(addr + i), &a->data[i * bs], bs)) {
                return sw(out, 0, SW_FAILED);
            }
        }
    } else if(!acr122 && p->type == PiccTypeSt25tb) {
        if(a->lc % 4) return sw(out, 0, SW_WRONG_LENGTH);
        for(size_t i = 0; i < a->lc / 4; i++) {
            if(addr + i > 0xFF ||
               !picc_st25tb_write_block(p, (uint8_t)(addr + i), &a->data[i * 4])) {
                return sw(out, 0, SW_FAILED);
            }
        }
    } else if(!acr122 && p->type == PiccTypePicopass) {
        /* Picopass writes need a key-derived MAC (authentication); not available */
        return sw(out, 0, SW_SECURITY);
    } else {
        return sw(out, 0, acr122 ? SW_FAILED : SW_NOT_SUPPORTED);
    }
    return sw(out, 0, SW_OK);
}

static size_t cmd_value_block(Reader* r, const Apdu* a, uint8_t* out) {
    if(!r->card_present || !picc_is_mifare_classic(&r->picc) || a->lc < 2) {
        return sw(out, 0, SW_FAILED);
    }
    uint8_t op = a->data[0];
    if(op == 0x03) { /* copy / restore: FF D7 00 src 02 03 dst */
        if(a->lc != 2 || a->p1 != 0x00) return sw(out, 0, SW_FAILED);
        bool ok = mfc_value_op(r, 0xC2, a->p2, 0) && mfc_transfer(r, a->data[1]);
        return sw(out, 0, ok ? SW_OK : SW_FAILED);
    }
    if(a->lc != 5) return sw(out, 0, SW_FAILED);
    uint32_t value = ((uint32_t)a->data[1] << 24) | ((uint32_t)a->data[2] << 16) |
                     ((uint32_t)a->data[3] << 8) | a->data[4];
    uint8_t source = a->p2;
    /* ACR122U: P1 = 00. ACR1552U: P1 = target block (00 = same as source) */
    uint8_t target = a->p1 ? a->p1 : a->p2;
    bool ok = false;
    if(op == 0x00) {
        if(a->p1 != 0x00) return sw(out, 0, SW_FAILED);
        uint8_t vb[16];
        uint32_t inv = ~value;
        for(size_t i = 0; i < 4; i++) {
            vb[i] = (uint8_t)(value >> (8 * i));
            vb[4 + i] = (uint8_t)(inv >> (8 * i));
            vb[8 + i] = (uint8_t)(value >> (8 * i));
        }
        vb[12] = source;
        vb[13] = (uint8_t)~source;
        vb[14] = source;
        vb[15] = (uint8_t)~source;
        ok = mfc_write_block(r, source, vb);
    } else if(op == 0x01) {
        ok = mfc_value_op(r, 0xC1, source, value) && mfc_transfer(r, target);
    } else if(op == 0x02) {
        ok = mfc_value_op(r, 0xC0, source, value) && mfc_transfer(r, target);
    }
    return sw(out, 0, ok ? SW_OK : SW_FAILED);
}

static size_t cmd_read_value(Reader* r, const Apdu* a, uint8_t* out) {
    if(!r->card_present || !picc_is_mifare_classic(&r->picc) || a->p1 != 0x00) {
        return sw(out, 0, SW_FAILED);
    }
    uint8_t vb[16];
    if(!mfc_read_block(r, a->p2, vb)) return sw(out, 0, SW_FAILED);
    uint32_t v = (uint32_t)vb[0] | ((uint32_t)vb[1] << 8) | ((uint32_t)vb[2] << 16) |
                 ((uint32_t)vb[3] << 24);
    uint32_t vi = (uint32_t)vb[4] | ((uint32_t)vb[5] << 8) | ((uint32_t)vb[6] << 16) |
                  ((uint32_t)vb[7] << 24);
    uint32_t v2 = (uint32_t)vb[8] | ((uint32_t)vb[9] << 8) | ((uint32_t)vb[10] << 16) |
                  ((uint32_t)vb[11] << 24);
    if(v != v2 || v != ~vi) return sw(out, 0, SW_FAILED); /* not a value block */
    out[0] = (uint8_t)(v >> 24);
    out[1] = (uint8_t)(v >> 16);
    out[2] = (uint8_t)(v >> 8);
    out[3] = (uint8_t)v;
    return sw(out, 4, SW_OK);
}

/* Native command to the current card; the reader adds and checks the link CRC */
static bool reader_native(Reader* r, const uint8_t* d, size_t len, uint8_t* out, size_t* n) {
    Picc* p = &r->picc;
    switch(p->type) {
    case PiccTypeFelica:
        return picc_felica_exchange(p, d, len, out, n, PICC_F_FWT_FC);
    case PiccType15693:
        return picc_15693_request(p, d, len, out, n, PICC_V_WRITE_FWT_FC);
    default:
        return picc_frame(p, d, len, out, n, PICC_MFC_FWT_FC);
    }
}

/* ACR1552U pass-through "FF 00 00 00": ISO14443-3 native commands and FeliCa */
static size_t cmd_pass_through(Reader* r, const Apdu* a, uint8_t* out) {
    if(!r->card_present || a->lc == 0) return sw(out, 0, SW_FAILED);
    size_t n = 0;
    if(!reader_native(r, a->data, a->lc, out, &n)) {
        r->picc.crypto = false;
        return sw(out, 0, SW_FAILED);
    }
    return sw(out, n, SW_OK);
}

/* ACR1552U "FF FB 00 Flags Lc Cmd Data": ISO15693 pass-through */
static size_t cmd_15693_pass_through(Reader* r, const Apdu* a, uint8_t* out) {
    Picc* p = &r->picc;
    if(!r->card_present || p->type != PiccType15693 || a->lc < 1 || a->p1 != 0x00) {
        return sw(out, 0, SW_FAILED);
    }
    uint8_t req[RF_MAX_FRAME];
    size_t n = 0;
    uint8_t cmd = a->data[0];
    if(a->p2 == 0x00) {
        if(cmd >= 0xA0) return sw(out, 0, SW_NOT_SUPPORTED); /* custom needs explicit flags */
        req[n++] = 0x22; /* high data rate, addressed */
        req[n++] = cmd;
        memcpy(&req[n], p->uid, 8);
        n += 8;
    } else {
        req[n++] = a->p2;
        req[n++] = cmd;
    }
    if(n + a->lc - 1 > sizeof(req) - 2) return sw(out, 0, SW_WRONG_LENGTH);
    memcpy(&req[n], &a->data[1], a->lc - 1);
    n += a->lc - 1;
    bool write = (cmd >= 0x21 && cmd <= 0x2A && cmd != 0x23 && cmd != 0x25 && cmd != 0x26) ||
                 cmd == 0x31 || cmd == 0x32 || cmd == 0x34;
    size_t rn = 0;
    if(!picc_15693_request(
           p, req, n, out, &rn, write ? PICC_V_WRITE_FWT_FC : PICC_V_FWT_FC)) {
        /* Stay Quiet never answers */
        return sw(out, 0, cmd == 0x02 ? SW_OK : SW_FAILED);
    }
    return sw(out, rn, SW_OK);
}

/* ------------------------------------------------------------------------ */
/* ACR1552U PC/SC 2.0 Part 3 transparent session: FF C2 00 xx                */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t* buf;
    size_t len;
} Tlv;

static void tlv_put(Tlv* t, const uint8_t* tag, size_t tag_len, const uint8_t* v, size_t vl) {
    memcpy(&t->buf[t->len], tag, tag_len);
    t->len += tag_len;
    if(vl < 0x80) {
        t->buf[t->len++] = (uint8_t)vl;
    } else if(vl < 0x100) {
        t->buf[t->len++] = 0x81;
        t->buf[t->len++] = (uint8_t)vl;
    } else {
        t->buf[t->len++] = 0x82;
        t->buf[t->len++] = (uint8_t)(vl >> 8);
        t->buf[t->len++] = (uint8_t)vl;
    }
    if(vl) memmove(&t->buf[t->len], v, vl);
    t->len += vl;
}

static void tlv_status(Tlv* t, uint8_t obj, uint16_t status) {
    const uint8_t tag = 0xC0;
    uint8_t v[3] = {obj, (uint8_t)(status >> 8), (uint8_t)status};
    tlv_put(t, &tag, 1, v, 3);
}

/* Parse one BER-TLV data object. Returns false at the end or on a format error. */
static bool tlv_next(
    const uint8_t* d,
    size_t len,
    size_t* off,
    uint16_t* tag,
    const uint8_t** val,
    size_t* vlen) {
    if(*off >= len) return false;
    size_t i = *off;
    uint16_t t = d[i++];
    if((t & 0x1F) == 0x1F) { /* two-byte tag */
        if(i >= len) return false;
        t = (uint16_t)((t << 8) | d[i++]);
    }
    if(i >= len) return false;
    size_t l = d[i++];
    if(l == 0x81) {
        if(i >= len) return false;
        l = d[i++];
    } else if(l == 0x82) {
        if(i + 1 >= len) return false;
        l = ((size_t)d[i] << 8) | d[i + 1];
        i += 2;
    }
    if(i + l > len) return false;
    *tag = t;
    *val = &d[i];
    *vlen = l;
    *off = i + l;
    return true;
}

static uint32_t session_fwt_fc(Reader* r) {
    uint64_t us = r->session.timeout_us;
    if(us == 0) us = (uint64_t)302 << (r->session.fwti > 15 ? 15 : r->session.fwti);
    uint64_t fc = us * 1356U / 100U;
    if(fc < 1000) fc = 1000;
    if(fc > 67800000U) fc = 67800000U;
    return (uint32_t)fc;
}

static CrcType session_crc(const Picc* p) {
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

static bool session_transceive(Reader* r, Tlv* rsp, uint8_t obj, const uint8_t* d, size_t len) {
    Picc* p = &r->picc;
    TransparentSession* s = &r->session;
    uint8_t rx[RF_MAX_FRAME];
    size_t rn = 0;
    uint8_t status = 0;
    uint8_t rx_last = 0;

    if(s->flags & 0x0C) { /* host-handled parity is not available */
        tlv_status(rsp, obj, SW_NOT_SUPPORTED);
        return false;
    }
    bool ok = false;
    uint8_t tries = (uint8_t)(s->retry + 1);
    for(uint8_t t = 0; t < tries && !ok; t++) {
        if(s->layer == 4 && !(s->flags & 0x10) && p->iso_dep) {
            ok = picc_isodep_apdu(p, d, len, rx, &rn, sizeof(rx));
            continue;
        }
        uint8_t tx[RF_MAX_FRAME];
        if(len == 0 || len + 2 > sizeof(tx)) break;
        memcpy(tx, d, len);
        uint16_t bits = (uint16_t)(len * 8U);
        if(s->tx_bits & 0x07) bits = (uint16_t)((len - 1) * 8U + (s->tx_bits & 0x07));
        if(!(s->flags & 0x01) && (s->tx_bits & 0x07) == 0) {
            crc_append(session_crc(p), tx, len);
            bits += 16;
        }
        uint16_t rbits = 0;
        RfStatus st = picc_raw(p, tx, bits, rx, &rbits, session_fwt_fc(r));
        if(st == RfStatusTimeout || st == RfStatusAuthFailed) continue;
        ok = true;
        if(st == RfStatusCollision) status |= 0x02;
        if(st == RfStatusParityError) status |= 0x04;
        rn = (rbits + 7U) / 8U;
        rx_last = (uint8_t)(rbits % 8U);
        if(!(s->flags & 0x02) && rbits >= 24 && rx_last == 0) {
            if(crc_check(session_crc(p), rx, rn)) {
                rn -= 2;
            } else {
                status |= 0x01;
            }
        }
    }
    if(!ok) {
        tlv_status(rsp, obj, 0x6401); /* no response from ICC */
        return false;
    }
    uint8_t t92 = 0x92;
    tlv_put(rsp, &t92, 1, &rx_last, 1);
    uint8_t t96 = 0x96;
    uint8_t st2[2] = {status, 0x00};
    tlv_put(rsp, &t96, 1, st2, 2);
    uint8_t t97 = 0x97;
    tlv_put(rsp, &t97, 1, rx, rn);
    return true;
}

static bool session_switch(Reader* r, Tlv* rsp, uint8_t rf, uint8_t layer) {
    Picc* p = &r->picc;
    PiccType target;
    switch(rf == 0xFF ? (uint8_t)(p->type == PiccTypeB ? 1 : p->type == PiccType15693 ? 2 :
                                                       p->type == PiccTypeFelica ? 3 :
                                                                                   0) :
                        rf) {
    case 0x00:
        target = PiccTypeA;
        break;
    case 0x01:
        target = PiccTypeB;
        break;
    case 0x02:
        target = PiccType15693;
        break;
    case 0x03:
        target = PiccTypeFelica;
        break;
    default:
        return false;
    }
    r->session.layer = layer;
    if(layer == 2) {
        if(!r->session.active) return false;
        static const RfTech tech[] = {
            [PiccTypeA] = RfTechA,
            [PiccTypeB] = RfTechB,
            [PiccTypeFelica] = RfTechF,
            [PiccType15693] = RfTechV};
        picc_use_tech(p, tech[target]);
        p->type = target;
        return true;
    }
    /* Release the current card so that it answers the wake-up request */
    if(p->present) {
        if(p->iso_dep) picc_deselect(p);
        picc_halt(p);
    }
    bool ok;
    switch(target) {
    case PiccTypeA:
        ok = picc_activate_a(p, true, layer == 4);
        if(ok && layer == 4 && !p->iso_dep) ok = false;
        break;
    case PiccTypeB:
        ok = picc_activate_b(p, 0x00, true);
        if(ok && layer == 4 && !p->iso_dep) ok = false;
        break;
    case PiccTypeFelica:
        ok = layer == 3 && picc_activate_felica(p, felica_poll_req);
        break;
    default:
        ok = layer == 3 && picc_activate_15693(p);
        break;
    }
    if(!ok) return false;
    reader_card_activated(r);
    if(layer == 4) {
        const uint8_t t5f51[2] = {0x5F, 0x51};
        tlv_put(rsp, t5f51, 2, r->atr, r->atr_len);
    } else if(target == PiccTypeA) {
        uint8_t t8f = 0x8F;
        tlv_put(rsp, &t8f, 1, &p->sak, 1);
    } else if(target == PiccTypeB) {
        uint8_t t8f = 0x8F;
        tlv_put(rsp, &t8f, 1, &p->atqb[9], 3);
    }
    return true;
}

static size_t cmd_transparent(Reader* r, const Apdu* a, uint8_t* out) {
    if(a->p1 != 0x00 || a->p2 > 0x02) return sw(out, 0, SW_WRONG_P1P2);
    if(a->lc == 0) return sw(out, 0, SW_WRONG_LENGTH);
    /* Response objects are collected after the C0 status object */
    uint8_t obj_buf[RF_MAX_FRAME + 32];
    Tlv objs = {obj_buf, 0};
    uint8_t err_buf[8];
    Tlv err = {err_buf, 0};
    uint8_t obj = 0;
    size_t off = 0;
    uint16_t tag;
    const uint8_t* v;
    size_t vl;
    TransparentSession* s = &r->session;

    while(err.len == 0 && tlv_next(a->data, a->lc, &off, &tag, &v, &vl)) {
        obj++;
        if(a->p2 == 0x00) { /* Manage session */
            if(tag == 0x81 && vl == 0) {
                s->active = true;
                s->layer = r->picc.iso_dep ? 4 : 3;
            } else if(tag == 0x82 && vl == 0) {
                s->active = false;
                r->next_poll_tick = furi_get_tick();
            } else if(tag == 0x83 && vl == 0) {
                rf_set_field(r->rf, false);
                r->field_on = false;
                r->picc.present = false;
            } else if(tag == 0x84 && vl == 0) {
                if(r->configured) {
                    rf_set_field(r->rf, true);
                    r->field_on = true;
                }
            } else if(tag == 0x5F46 && vl == 4) {
                uint32_t us = (uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) |
                              ((uint32_t)v[3] << 24);
                furi_delay_ms((us + 999U) / 1000U);
            } else {
                tlv_status(&err, obj, SW_NOT_SUPPORTED);
            }
        } else if(a->p2 == 0x01) { /* Transparent exchange */
            if(tag == 0x90 && vl == 2) {
                s->flags = v[0];
            } else if(tag == 0x91 && vl == 1) {
                s->tx_bits = v[0] & 0x07;
            } else if(tag == 0x5F46 && vl == 4) {
                s->timeout_us = (uint32_t)v[0] | ((uint32_t)v[1] << 8) |
                                ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
            } else if(tag == 0xFF6E && vl >= 3 && v[0] == 0xF0 && v[1] == 0x01) {
                s->retry = v[2];
            } else if(tag == 0xFF6E && vl >= 3 && v[0] == 0x03 && v[1] == 0x01) {
                s->fwti = v[2];
                s->timeout_us = 0;
            } else if(tag == 0xFF6E && vl >= 3) {
                /* other parameters (e.g. PCB) are accepted */
            } else if(tag == 0x95) {
                if(!r->field_on) {
                    tlv_status(&err, obj, 0x6400); /* no response from IFD */
                } else {
                    session_transceive(r, &objs, obj, v, vl);
                    s->tx_bits = 0;
                    /* a failed transceive leaves a C0 error object at the end */
                    if(objs.len >= 5 && objs.buf[objs.len - 5] == 0xC0) {
                        memcpy(err.buf, &objs.buf[objs.len - 5], 5);
                        err.len = 5;
                        objs.len -= 5;
                    }
                }
            } else {
                tlv_status(&err, obj, SW_NOT_SUPPORTED);
            }
        } else { /* Switch protocol */
            if(tag == 0xFF6E && vl >= 3 && v[0] == 0x05 && v[1] == 0x01) {
                s->baud = v[2];
                if(v[2] != 0x98 && v[2] != 0x80) {
                    tlv_status(&err, obj, 0x6A80); /* only 106 kbit/s / 26 kbit/s */
                }
            } else if(tag == 0x8F && vl == 2) {
                if(!session_switch(r, &objs, v[0], v[1])) tlv_status(&err, obj, 0x6401);
            } else {
                tlv_status(&err, obj, SW_NOT_SUPPORTED);
            }
        }
    }

    if(!a->has_le) {
        if(err.len) return sw(out, 0, (uint16_t)((err.buf[3] << 8) | err.buf[4]));
        return sw(out, 0, SW_OK);
    }
    size_t n = 0;
    if(err.len) {
        memcpy(out, err.buf, err.len);
        n = err.len;
    } else {
        Tlv head = {out, 0};
        tlv_status(&head, 0, SW_OK);
        n = head.len;
    }
    memcpy(&out[n], objs.buf, objs.len);
    n += objs.len;
    return sw(out, n, SW_OK);
}

/* ------------------------------------------------------------------------ */
/* ACR122U-specific: FF 00 xx                                                */
/* ------------------------------------------------------------------------ */

static size_t acr122u_ff00(Reader* r, const Apdu* a, uint8_t* out) {
    switch(a->p1) {
    case 0x00: { /* Direct Transmit to the PN532 */
        if(a->p2 != 0x00 || a->lc < 2 || a->data[0] != 0xD4) return sw(out, 0, SW_FAILED);
        r->direct_mode_until_tick = furi_get_tick() + furi_ms_to_ticks(DIRECT_MODE_HOLD_MS);
        size_t n = pn532_command(r, a->data, a->lc, out, READER_RESP_MAX - 2);
        if(n == 0) return sw(out, 0, SW_FAILED);
        return sw(out, n, SW_OK);
    }
    case 0x40: { /* Bi-color LED and buzzer control */
        if(a->lc != 4) return sw(out, 0, SW_FAILED);
        uint8_t p2 = a->p2;
        uint32_t t1 = a->data[0] * 100U;
        uint32_t t2 = a->data[1] * 100U;
        uint32_t reps = a->data[2];
        uint8_t link = a->data[3];
        if(reps) {
            uint32_t beep = ((link & 0x01) ? t1 : 0) + ((link & 0x02) ? t2 : 0);
            if(beep) reader_beep(r, beep * reps);
            /* The response comes after the blinking sequence, as on the device */
            uint8_t initial = (uint8_t)(((p2 >> 4) & 0x01) | ((p2 >> 4) & 0x02));
            uint8_t blink_mask = (uint8_t)(((p2 >> 6) & 0x01) | ((p2 >> 6) & 0x02));
            uint32_t total = 0;
            for(uint32_t i = 0; i < reps && total < LED_SEQUENCE_MAX_MS; i++) {
                if(blink_mask) reader_led(r, (uint8_t)((r->led_state & ~blink_mask) | (initial & blink_mask)));
                furi_delay_ms(t1);
                if(blink_mask) reader_led(r, (uint8_t)((r->led_state & ~blink_mask) | (~initial & blink_mask)));
                furi_delay_ms(t2);
                total += t1 + t2;
            }
        }
        uint8_t led = r->led_state;
        if(p2 & 0x04) led = (uint8_t)((led & ~0x01) | (p2 & 0x01));
        if(p2 & 0x08) led = (uint8_t)((led & ~0x02) | (p2 & 0x02));
        reader_led(r, led);
        out[0] = 0x90;
        out[1] = r->led_state;
        return 2;
    }
    case 0x41: /* Set timeout parameter */
        r->acr122_timeout = a->p2;
        return sw(out, 0, SW_OK);
    case 0x48: /* Get firmware version: ASCII, no status word */
        memcpy(out, ACR122U_FIRMWARE, strlen(ACR122U_FIRMWARE));
        return strlen(ACR122U_FIRMWARE);
    case 0x50: /* Get PICC operating parameter */
        out[0] = 0x90;
        out[1] = r->acr122_picc_param;
        return 2;
    case 0x51: /* Set PICC operating parameter */
        r->acr122_picc_param = a->p2;
        out[0] = 0x90;
        out[1] = r->acr122_picc_param;
        return 2;
    case 0x52: /* Buzzer output during card detection */
        r->acr122_beep_on_detect = a->p2 != 0x00;
        return sw(out, 0, SW_OK);
    default:
        return sw(out, 0, SW_NOT_SUPPORTED);
    }
}

/* ------------------------------------------------------------------------ */
/* ACR1552U-specific: escape commands E0 00 xx yy                            */
/* ------------------------------------------------------------------------ */

static size_t esc_reply(uint8_t* out, const uint8_t* d, uint8_t len) {
    out[0] = 0xE1;
    out[1] = 0x00;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = len;
    if(len) memmove(&out[5], d, len);
    return 5U + len;
}

static uint8_t acr1552u_picc_status(Reader* r) {
    if(!r->field_on) return 0x00;
    if(!r->card_present) return 0x01;
    return r->card_powered ? 0x03 : 0x02;
}

static uint8_t acr1552u_picc_type(Reader* r) {
    const Picc* p = &r->picc;
    if(!r->card_present) return 0xCC;
    switch(p->type) {
    case PiccTypeA:
        return p->iso_dep ? 0x20 : 0x10;
    case PiccTypeB:
        return 0x23;
    case PiccTypeFelica:
        return 0x11;
    case PiccTypeSt25tb:
        return 0x28;
    case PiccTypePicopass:
        return 0x30;
    default:
        return 0xFF;
    }
}

static void acr1552u_rf_mode(Reader* r, uint8_t mode) {
    r->rf_mode = mode;
    reader_set_field(r, mode != 0x00);
    if(mode == 0x01) r->next_poll_tick = furi_get_tick();
}

static size_t acr1552u_escape(Reader* r, const uint8_t* c, size_t n, uint8_t* out) {
    if(n < 5 || c[0] != 0xE0 || c[1] != 0x00) return sw(out, 0, SW_NOT_SUPPORTED);
    uint8_t p1 = c[2];
    uint8_t cmd = c[3];
    uint8_t len = c[4];
    if(n < 5U + len) return sw(out, 0, SW_WRONG_LENGTH);
    const uint8_t* d = &c[5];
    uint8_t v[3];

    if(p1 == 0x01 && cmd == 0x20) { /* PICC polling type (2 bytes, byte 1 first) */
        if(len == 2) r->polling_type = (uint16_t)((d[0] << 8) | d[1]);
        v[0] = (uint8_t)(r->polling_type >> 8);
        v[1] = (uint8_t)(r->polling_type & 0xFF);
        return esc_reply(out, v, 2);
    }
    if(p1 != 0x00) return sw(out, 0, SW_NOT_SUPPORTED);

    switch(cmd) {
    case 0x25: /* RF control / PCD-PICC status */
        if(len == 1) {
            acr1552u_rf_mode(r, d[0] > 0x02 ? 0x01 : d[0]);
            return esc_reply(out, &r->rf_mode, 1);
        }
        v[0] = acr1552u_picc_status(r);
        return esc_reply(out, v, 1);
    case 0x23: /* Polling / ATR option */
        if(len == 1) {
            r->polling_option = d[0];
            r->next_poll_tick = furi_get_tick();
        }
        return esc_reply(out, &r->polling_option, 1);
    case 0x24: /* Auto PPS: max speed; current speed is always 106 kbit/s here */
        if(len == 1) r->auto_pps = d[0];
        v[0] = r->auto_pps;
        v[1] = 0x00;
        return esc_reply(out, v, 2);
    case 0x35: /* Read PICC type */
        v[0] = acr1552u_picc_type(r);
        v[1] = acr1552u_picc_status(r);
        return esc_reply(out, v, 2);
    case 0x50: /* RF power setting */
        if(len == 1) r->rf_power = d[0];
        return esc_reply(out, &r->rf_power, 1);
    case 0xE5: /* Selective suspend */
        if(len == 1) r->selective_suspend = d[0];
        return esc_reply(out, &r->selective_suspend, 1);
    case 0x18: /* Firmware version */
        return esc_reply(
            out, (const uint8_t*)ACR1552U_FIRMWARE, (uint8_t)strlen(ACR1552U_FIRMWARE));
    case 0x33: /* Serial number */
        return esc_reply(out, (const uint8_t*)serial_ascii, (uint8_t)strlen(serial_ascii));
    case 0xF0: /* S/N in USB descriptor (applies at next enumeration) */
        if(len >= 1) r->sn_in_descriptor = d[0];
        v[0] = r->sn_in_descriptor;
        v[1] = 0x90;
        v[2] = 0x00;
        return esc_reply(out, v, 3);
    case 0x28: /* Buzzer: single (1 byte) or repeatable (on, off, repeat) */
        if(len == 1) {
            reader_beep(r, d[0] * 10U);
            return esc_reply(out, d, 1);
        }
        if(len == 3) {
            uint32_t on = d[0] * 10U;
            uint32_t off = d[1] * 10U;
            for(uint8_t i = 0; i < d[2]; i++) {
                reader_beep(r, on);
                furi_delay_ms(on);
                reader_beep_stop(r);
                furi_delay_ms(off);
            }
            return esc_reply(out, d, 3);
        }
        return sw(out, 0, SW_WRONG_LENGTH);
    case 0x29: /* LED control: bit0 blue, bit1 green */
        if(len == 1) reader_led(r, d[0]);
        return esc_reply(out, &r->led_state, 1);
    case 0x21: /* UI behaviour */
        if(len == 1) r->ui_behaviour = d[0];
        return esc_reply(out, &r->ui_behaviour, 1);
    default:
        /* HID keyboard, card emulation and discovery modes are not emulated */
        return sw(out, 0, SW_NOT_SUPPORTED);
    }
}

/* ------------------------------------------------------------------------ */
/* APDU dispatch                                                             */
/* ------------------------------------------------------------------------ */

static size_t reader_pseudo_apdu(Reader* r, const uint8_t* raw, size_t n, uint8_t* out) {
    /* Obsolete ACR122U authentication: FF 88 00 block keytype keyno (no Lc) */
    if(n == 6 && raw[1] == 0x88) return do_auth(r, raw[3], raw[4], raw[5], out);

    Apdu a;
    if(!apdu_parse(raw, n, &a)) return sw(out, 0, SW_WRONG_LENGTH);
    bool acr1552 = r->model == ReaderModelAcr1552u;

    switch(a.ins) {
    case 0xCA:
        return cmd_get_data(r, &a, out);
    case 0x82:
        return cmd_load_key(r, &a, out);
    case 0x86:
        return cmd_general_auth(r, &a, out);
    case 0xB0:
        return cmd_read_binary(r, &a, out);
    case 0xD6:
        return cmd_update_binary(r, &a, out);
    case 0xD7:
        return cmd_value_block(r, &a, out);
    case 0xB1:
        return cmd_read_value(r, &a, out);
    case 0x00:
        if(!acr1552) return acr122u_ff00(r, &a, out);
        if(a.p1 == 0x00 && a.p2 == 0x00) return cmd_pass_through(r, &a, out);
        return sw(out, 0, SW_NOT_SUPPORTED);
    case 0xFB:
        if(acr1552) return cmd_15693_pass_through(r, &a, out);
        return sw(out, 0, SW_NOT_SUPPORTED);
    case 0xC2:
        if(acr1552) return cmd_transparent(r, &a, out);
        return sw(out, 0, SW_NOT_SUPPORTED);
    default:
        return sw(out, 0, SW_NOT_SUPPORTED);
    }
}

static size_t reader_apdu(Reader* r, const uint8_t* apdu, size_t n, uint8_t* out) {
    if(n >= 4 && apdu[0] == 0xFF) return reader_pseudo_apdu(r, apdu, n, out);
    Picc* p = &r->picc;
    if(r->card_present && p->iso_dep) {
        size_t rl = 0;
        if(!picc_isodep_apdu(p, apdu, n, out, &rl, READER_RESP_MAX - 2)) {
            return sw(out, 0, SW_FAILED);
        }
        /* DESFire native mode: short answers get a status word to satisfy PC/SC */
        if(rl < 2) rl = sw(out, rl, SW_OK);
        return rl;
    }
    if(r->card_present && r->model == ReaderModelAcr122u && p->type == PiccTypeFelica) {
        /* FeliCa native command (length byte first), response + 90 00 */
        size_t rl = 0;
        if(!picc_felica_exchange(p, apdu, n, out, &rl, PICC_F_FWT_FC)) {
            return sw(out, 0, SW_FAILED);
        }
        return sw(out, rl, SW_OK);
    }
    return sw(out, 0, SW_NOT_SUPPORTED);
}

/* ------------------------------------------------------------------------ */
/* CCID                                                                      */
/* ------------------------------------------------------------------------ */

static uint8_t reader_icc_status(Reader* r) {
    if(!r->card_present) return 0x02;
    return r->card_powered ? 0x00 : 0x01;
}

static void reader_send(
    Reader* r,
    uint8_t iface,
    uint8_t type,
    uint8_t seq,
    uint8_t status,
    uint8_t error,
    uint8_t b9,
    size_t data_len) {
    uint8_t* o = r->out;
    o[0] = type;
    o[1] = (uint8_t)data_len;
    o[2] = (uint8_t)(data_len >> 8);
    o[3] = 0;
    o[4] = 0;
    o[5] = 0; /* slot */
    o[6] = seq;
    o[7] = status;
    o[8] = error;
    o[9] = b9;
    ccid_usb_send(iface, o, CCID_HEADER_SIZE + data_len);
}

static void reader_set_last(Reader* r, const char* what, const uint8_t* d, size_t n) {
    int w = snprintf(r->last, sizeof(r->last), "%s", what);
    if(w < 0) return;
    size_t used = (size_t)w;
    for(size_t i = 0; i < n && i < 6 && used + 3 < sizeof(r->last); i++) {
        w = snprintf(&r->last[used], sizeof(r->last) - used, " %02X", d[i]);
        if(w < 0) break;
        used += (size_t)w;
    }
}

static bool reader_power_on(Reader* r) {
    if(!r->card_present) return false;
    if(r->session.active) {
        r->card_powered = true;
        return true;
    }
    /* Cold reset of the contactless card: deselect / halt and activate again */
    if(!picc_reactivate(&r->picc, r->picc.iso_dep || reader_wants_rats(r))) {
        reader_card_lost(r);
        reader_note_loss(r, "PwrOn lost");
        return false;
    }
    reader_build_atr(r);
    r->card_powered = true;
    return true;
}

static void reader_handle_unsupported(Reader* r, uint8_t iface, uint8_t type, uint8_t seq, uint8_t icc) {
    uint8_t rtype = (type == PC_TO_RDR_ICC_POWER_ON || type == PC_TO_RDR_XFR_BLOCK) ?
                        RDR_TO_PC_DATA_BLOCK :
                    type == PC_TO_RDR_ESCAPE ? RDR_TO_PC_ESCAPE :
                                               RDR_TO_PC_SLOT_STATUS;
    reader_send(r, iface, rtype, seq, CCID_STATUS_FAILED | icc, CCID_ERR_CMD_NOT_SUPPORTED, 0, 0);
}

/* The SAM interface has an empty contact slot: no SAM is fitted */
static void reader_handle_sam(Reader* r, size_t len) {
    UNUSED(len);
    const uint8_t* m = r->msg;
    uint8_t type = m[0];
    uint8_t seq = m[6];
    uint8_t* d = &r->out[CCID_HEADER_SIZE];
    switch(type) {
    case PC_TO_RDR_ICC_POWER_ON:
    case PC_TO_RDR_XFR_BLOCK:
        reader_send(
            r, READER_IFACE_SAM, RDR_TO_PC_DATA_BLOCK, seq, CCID_STATUS_FAILED | 0x02,
            CCID_ERR_ICC_MUTE, 0, 0);
        break;
    case PC_TO_RDR_ICC_POWER_OFF:
    case PC_TO_RDR_GET_SLOT_STATUS:
    case PC_TO_RDR_ICC_CLOCK:
        reader_send(r, READER_IFACE_SAM, RDR_TO_PC_SLOT_STATUS, seq, 0x02, 0, 0x00, 0);
        break;
    case PC_TO_RDR_GET_PARAMETERS:
    case PC_TO_RDR_RESET_PARAMETERS:
    case PC_TO_RDR_SET_PARAMETERS:
        reader_send(
            r, READER_IFACE_SAM, RDR_TO_PC_PARAMETERS, seq, CCID_STATUS_FAILED | 0x02,
            CCID_ERR_ICC_MUTE, 0x00, 0);
        break;
    case PC_TO_RDR_ESCAPE: {
        size_t n = sw(d, 0, SW_NOT_SUPPORTED);
        reader_send(r, READER_IFACE_SAM, RDR_TO_PC_ESCAPE, seq, 0x02, 0, 0, n);
        break;
    }
    default:
        reader_handle_unsupported(r, READER_IFACE_SAM, type, seq, 0x02);
        break;
    }
}

static void reader_handle_message(Reader* r, size_t len) {
    const uint8_t* m = r->msg;
    uint8_t type = m[0];
    size_t dlen = len - CCID_HEADER_SIZE;
    uint8_t slot = m[5];
    uint8_t seq = m[6];
    const uint8_t* data = &m[CCID_HEADER_SIZE];
    uint8_t* d = &r->out[CCID_HEADER_SIZE];
    const uint8_t iface = READER_IFACE_PICC;
    r->commands++;

    if(slot != 0) {
        uint8_t rtype = (type == PC_TO_RDR_ICC_POWER_ON || type == PC_TO_RDR_XFR_BLOCK) ?
                            RDR_TO_PC_DATA_BLOCK :
                            RDR_TO_PC_SLOT_STATUS;
        reader_send(r, iface, rtype, seq, CCID_STATUS_FAILED | 0x02, CCID_ERR_BAD_SLOT, 0, 0);
        return;
    }

    switch(type) {
    case PC_TO_RDR_ICC_POWER_ON:
        reader_set_last(r, "PowerOn", NULL, 0);
        if(reader_power_on(r)) {
            memcpy(d, r->atr, r->atr_len);
            reader_send(r, iface, RDR_TO_PC_DATA_BLOCK, seq, reader_icc_status(r), 0, 0, r->atr_len);
        } else {
            reader_send(
                r,
                iface,
                RDR_TO_PC_DATA_BLOCK,
                seq,
                CCID_STATUS_FAILED | reader_icc_status(r),
                CCID_ERR_ICC_MUTE,
                0,
                0);
        }
        break;

    case PC_TO_RDR_ICC_POWER_OFF:
        reader_set_last(r, "PowerOff", NULL, 0);
        r->card_powered = false;
        reader_send(r, iface, RDR_TO_PC_SLOT_STATUS, seq, reader_icc_status(r), 0, 0x00, 0);
        break;

    case PC_TO_RDR_GET_SLOT_STATUS:
    case PC_TO_RDR_ICC_CLOCK:
        reader_send(r, iface, RDR_TO_PC_SLOT_STATUS, seq, reader_icc_status(r), 0, 0x00, 0);
        break;

    case PC_TO_RDR_XFR_BLOCK:
        reader_set_last(r, "APDU", data, dlen);
        if(!r->card_powered) {
            reader_send(
                r,
                iface,
                RDR_TO_PC_DATA_BLOCK,
                seq,
                CCID_STATUS_FAILED | reader_icc_status(r),
                CCID_ERR_ICC_MUTE,
                0,
                0);
        } else {
            size_t n = reader_apdu(r, data, dlen, d);
            reader_send(r, iface, RDR_TO_PC_DATA_BLOCK, seq, reader_icc_status(r), 0, 0, n);
        }
        break;

    case PC_TO_RDR_GET_PARAMETERS:
    case PC_TO_RDR_RESET_PARAMETERS:
    case PC_TO_RDR_SET_PARAMETERS: {
        if(!r->card_present) {
            reader_send(
                r,
                iface,
                RDR_TO_PC_PARAMETERS,
                seq,
                CCID_STATUS_FAILED | reader_icc_status(r),
                CCID_ERR_ICC_MUTE,
                0x01,
                0);
            break;
        }
        /* T=1 protocol data structure */
        static const uint8_t t1[7] = {0x11, 0x10, 0x00, 0x4D, 0x00, 0xFE, 0x00};
        memcpy(d, t1, sizeof(t1));
        reader_send(r, iface, RDR_TO_PC_PARAMETERS, seq, reader_icc_status(r), 0, 0x01, sizeof(t1));
        break;
    }

    case PC_TO_RDR_ESCAPE: {
        reader_set_last(r, "Escape", data, dlen);
        size_t n;
        if(r->model == ReaderModelAcr1552u && dlen >= 1 && data[0] == 0xE0) {
            n = acr1552u_escape(r, data, dlen, d);
        } else if(dlen >= 4 && data[0] == 0xFF) {
            n = reader_pseudo_apdu(r, data, dlen, d);
        } else {
            n = sw(d, 0, SW_NOT_SUPPORTED);
        }
        reader_send(r, iface, RDR_TO_PC_ESCAPE, seq, reader_icc_status(r), 0, 0, n);
        break;
    }

    default:
        reader_handle_unsupported(r, iface, type, seq, reader_icc_status(r));
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Worker                                                                    */
/* ------------------------------------------------------------------------ */

static void reader_rf_done(const RfResult* result, void* context) {
    Reader* r = context;
    if(furi_message_queue_put(r->rf_results, result, 100) != FuriStatusOk) {
        FURI_LOG_E(TAG, "RF result dropped");
    }
}

static void reader_publish(Reader* r) {
    furi_mutex_acquire(r->mutex, FuriWaitForever);
    ReaderStatus* s = &r->status;
    s->usb_ok = r->usb_ok;
    s->usb_configured = r->configured;
    s->field_on = r->field_on;
    s->card_present = r->card_present;
    s->card_powered = r->card_powered;
    s->crypto_on = r->picc.crypto;
    s->buzzer_enabled = r->buzzer_enabled;
    snprintf(s->card, sizeof(s->card), "%s", r->card_present ? picc_type_name(&r->picc) : "-");
    s->uid[0] = 0;
    if(r->card_present) {
        size_t used = 0;
        for(size_t i = 0; i < r->picc.uid_len && used + 3 < sizeof(s->uid); i++) {
            int w = snprintf(&s->uid[used], sizeof(s->uid) - used, "%02X", r->picc.uid[i]);
            if(w < 0) break;
            used += (size_t)w;
        }
    }
    memcpy(s->last, r->last, sizeof(s->last));
    s->commands = r->commands;
    furi_mutex_release(r->mutex);
}

static void reader_on_configured(Reader* r) {
    r->next_poll_tick = furi_get_tick();
    ccid_usb_notify_slot(READER_IFACE_PICC, r->card_present, true);
    if(r->model == ReaderModelAcr1552u) {
        ccid_usb_notify_slot(READER_IFACE_SAM, false, true);
        if(r->ui_behaviour & 0x20) reader_beep(r, BEEP_SHORT_MS); /* power-up beep */
    } else {
        reader_beep(r, BEEP_SHORT_MS);
    }
}

static int32_t reader_worker(void* context) {
    Reader* r = context;
    reader_status_led(r, false);

    while(true) {
        uint32_t now = furi_get_tick();
        uint32_t wait = furi_ms_to_ticks(250);
        if(r->beeping) {
            uint32_t left = (int32_t)(r->beep_off_tick - now) > 0 ? r->beep_off_tick - now : 0;
            if(left < wait) wait = left;
        }
        if(r->field_on && (int32_t)(r->next_poll_tick - now) > 0 &&
           r->next_poll_tick - now < wait) {
            wait = r->next_poll_tick - now;
        }

        uint32_t flags = furi_thread_flags_wait(RD_ALL_FLAGS, FuriFlagWaitAny, wait);
        if(flags & FuriFlagError) flags = 0;
        if(flags & RD_STOP) break;

        bool cfg = ccid_usb_is_configured();
        if(cfg != r->configured) {
            r->configured = cfg;
            reader_set_field(r, r->field_wanted);
            if(cfg) reader_on_configured(r);
        }

        if(!ccid_usb_tx_busy(READER_IFACE_PICC)) {
            size_t n = ccid_usb_take_message(READER_IFACE_PICC, r->msg, sizeof(r->msg));
            if(n >= CCID_HEADER_SIZE) {
                reader_handle_message(r, n);
            }
        }
        if(r->model == ReaderModelAcr1552u && !ccid_usb_tx_busy(READER_IFACE_SAM)) {
            size_t n = ccid_usb_take_message(READER_IFACE_SAM, r->msg, sizeof(r->msg));
            if(n >= CCID_HEADER_SIZE) reader_handle_sam(r, n);
        }

        reader_poll(r);

        if(r->beeping && (int32_t)(furi_get_tick() - r->beep_off_tick) >= 0) {
            reader_beep_stop(r);
        }
        reader_publish(r);
    }

    reader_beep_stop(r);
    rf_set_field(r->rf, false);
    r->field_on = false;
    notification_message(r->notification, &sequence_reset_rgb);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

Reader* reader_alloc(ReaderModel model) {
    Reader* r = malloc(sizeof(Reader));
    memset(r, 0, sizeof(Reader));
    r->model = model;
    r->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    r->rf_results = furi_message_queue_alloc(4, sizeof(RfResult));
    r->rf = rf_alloc(reader_rf_done, r);
    picc_init(&r->picc, r->rf, r->rf_results);
    pn532_reset(&r->pn532);
    r->notification = furi_record_open(RECORD_NOTIFICATION);

    r->field_wanted = true;
    r->buzzer_enabled = true;
    /* ACR122U defaults (API 2.04) */
    r->acr122_picc_param = 0xFF;
    r->acr122_beep_on_detect = true;
    /* ACR1552U defaults (Reference Manual 1.09) */
    r->rf_mode = 0x01;
    r->polling_option = 0x8B;
    r->polling_type = 0x0705;
    r->auto_pps = 0x02;
    r->ui_behaviour = 0x6F;
    r->rf_power = 0x00;
    r->selective_suspend = 0x00;
    r->sn_in_descriptor = 0x01;

    r->thread = furi_thread_alloc_ex("ReaderWorker", 6 * 1024, reader_worker, r);
    return r;
}

bool reader_start(Reader* r) {
    furi_thread_start(r->thread);
    CcidUsbConfig cfg;
    reader_usb_config(r, &cfg);
    r->usb_ok = ccid_usb_start(&cfg, furi_thread_get_id(r->thread));
    furi_thread_flags_set(furi_thread_get_id(r->thread), CCID_FLAG_CONFIG);
    return r->usb_ok;
}

void reader_free(Reader* r) {
    furi_thread_flags_set(furi_thread_get_id(r->thread), RD_STOP);
    furi_thread_join(r->thread);
    furi_thread_free(r->thread);
    if(r->usb_ok) ccid_usb_stop();
    rf_free(r->rf);
    furi_record_close(RECORD_NOTIFICATION);
    furi_message_queue_free(r->rf_results);
    furi_mutex_free(r->mutex);
    free(r);
}

void reader_get_status(Reader* r, ReaderStatus* status) {
    furi_mutex_acquire(r->mutex, FuriWaitForever);
    *status = r->status;
    status->usb_ok = r->usb_ok;
    furi_mutex_release(r->mutex);
}

void reader_set_buzzer(Reader* r, bool enabled) {
    /* Read by the worker; a stale value for one loop is harmless */
    r->buzzer_enabled = enabled;
}
