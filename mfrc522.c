#include "mfrc522.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <furi_hal_random.h>

/* ------------------------------------------------------------------------ */
/* Register map                                                             */
/* ------------------------------------------------------------------------ */

enum {
    RegCommand = 0x01,
    RegComIEn = 0x02,
    RegDivIEn = 0x03,
    RegComIrq = 0x04,
    RegDivIrq = 0x05,
    RegError = 0x06,
    RegStatus1 = 0x07,
    RegStatus2 = 0x08,
    RegFIFOData = 0x09,
    RegFIFOLevel = 0x0A,
    RegWaterLevel = 0x0B,
    RegControl = 0x0C,
    RegBitFraming = 0x0D,
    RegColl = 0x0E,
    RegMode = 0x11,
    RegTxMode = 0x12,
    RegRxMode = 0x13,
    RegTxControl = 0x14,
    RegTxASK = 0x15,
    RegTxSel = 0x16,
    RegRxSel = 0x17,
    RegRxThreshold = 0x18,
    RegDemod = 0x19,
    RegMfTx = 0x1C,
    RegMfRx = 0x1D,
    RegSerialSpeed = 0x1F,
    RegCRCResultH = 0x21,
    RegCRCResultL = 0x22,
    RegModWidth = 0x24,
    RegRFCfg = 0x26,
    RegGsN = 0x27,
    RegCWGsP = 0x28,
    RegModGsP = 0x29,
    RegTMode = 0x2A,
    RegTPrescaler = 0x2B,
    RegTReloadH = 0x2C,
    RegTReloadL = 0x2D,
    RegTCounterValH = 0x2E,
    RegTCounterValL = 0x2F,
    RegTestSel1 = 0x31,
    RegTestSel2 = 0x32,
    RegTestPinEn = 0x33,
    RegTestPinValue = 0x34,
    RegTestBus = 0x35,
    RegAutoTest = 0x36,
    RegVersion = 0x37,
    RegAnalogTest = 0x38,
    RegTestDAC1 = 0x39,
    RegTestDAC2 = 0x3A,
    RegTestADC = 0x3B,
};

/* Commands (CommandReg[3:0]) */
enum {
    CmdIdle = 0x0,
    CmdMem = 0x1,
    CmdGenerateRandomId = 0x2,
    CmdCalcCrc = 0x3,
    CmdTransmit = 0x4,
    CmdNoCmdChange = 0x7,
    CmdReceive = 0x8,
    CmdTransceive = 0xC,
    CmdMfAuthent = 0xE,
    CmdSoftReset = 0xF,
};

/* CommandReg */
#define CMD_RCV_OFF    0x20
#define CMD_POWER_DOWN 0x10

/* ComIrqReg */
#define IRQ_SET1    0x80
#define IRQ_TX      0x40
#define IRQ_RX      0x20
#define IRQ_IDLE    0x10
#define IRQ_HIALERT 0x08
#define IRQ_LOALERT 0x04
#define IRQ_ERR     0x02
#define IRQ_TIMER   0x01

/* DivIrqReg */
#define DIVIRQ_SET2    0x80
#define DIVIRQ_MFINACT 0x10
#define DIVIRQ_CRC     0x04
#define DIVIRQ_MASK    (DIVIRQ_MFINACT | DIVIRQ_CRC)

/* ErrorReg */
#define ERR_WR       0x80
#define ERR_TEMP     0x40
#define ERR_BUFOVFL  0x10
#define ERR_COLL     0x08
#define ERR_CRC      0x04
#define ERR_PARITY   0x02
#define ERR_PROTOCOL 0x01
#define ERR_RX_MASK  (ERR_COLL | ERR_CRC | ERR_PARITY | ERR_PROTOCOL)

/* Status2Reg */
#define STATUS2_CRYPTO1ON 0x08

/* BitFramingReg */
#define BITFRAMING_STARTSEND 0x80

/* TModeReg */
#define TMODE_TAUTO        0x80
#define TMODE_TAUTORESTART 0x10

/* RxModeReg / TxModeReg */
#define MODE_CRC_EN      0x80
#define RXMODE_MULTIPLE  0x04
#define RXMODE_NOERR     0x08

/* ModemState (Status2Reg[2:0]) */
enum {
    ModemIdle = 0,
    ModemWaitStartSend = 1,
    ModemTxWait = 2,
    ModemTransmitting = 3,
    ModemRxWait = 4,
    ModemWaitData = 5,
    ModemReceiving = 6,
};

#define FIFO_SIZE 64
#define INTERNAL_BUFFER_SIZE 25

/* HAL frame-wait limits (carrier cycles) */
#define FWT_MIN_FC     (2000U)
#define FWT_DEFAULT_FC (1356000U) /* 100 ms when the timer is not in auto mode */
#define FWT_MAX_FC     (6780000U) /* 500 ms */

/* Soft power-down exit: 1024 clocks of 27.12 MHz */
#define WAKEUP_TIME_US (38U)

static const uint8_t reg_reset_values[64] = {
    [RegCommand] = 0x20,
    [RegComIEn] = 0x80,
    [RegComIrq] = 0x14,
    [RegStatus1] = 0x21,
    [RegWaterLevel] = 0x08,
    [RegControl] = 0x10,
    [RegColl] = 0xA0,
    [RegMode] = 0x3F,
    [RegTxControl] = 0x80,
    [RegTxSel] = 0x10,
    [RegRxSel] = 0x84,
    [RegRxThreshold] = 0x84,
    [RegDemod] = 0x4D,
    [RegMfTx] = 0x62,
    [RegSerialSpeed] = 0xEB,
    [RegCRCResultH] = 0xFF,
    [RegCRCResultL] = 0xFF,
    [RegModWidth] = 0x26,
    [RegRFCfg] = 0x48,
    [RegGsN] = 0x88,
    [RegCWGsP] = 0x20,
    [RegModGsP] = 0x20,
    [RegTestPinEn] = 0x80,
    [RegAutoTest] = 0x40,
    /* Reserved registers with non-zero reset values (datasheet Rev 3.9) */
    [0x23] = 0x88,
    [0x25] = 0x87,
    [0x3C] = 0xFF,
    [0x3E] = 0x03,
};

/* Addresses that are "reserved for future use": read their reset value, writes ignored. */
static bool reg_is_reserved(uint8_t a) {
    switch(a) {
    case 0x00:
    case 0x0F:
    case 0x10:
    case 0x1A:
    case 0x1B:
    case 0x1E:
    case 0x20:
    case 0x23:
    case 0x25:
    case 0x30:
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
        return true;
    default:
        return false;
    }
}

/* Self-test results (AutoTestReg=09h, CalcCRC, internal buffer cleared). */
static const uint8_t selftest_v0_0[64] = {
    0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19, 0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
    0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50, 0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
    0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78, 0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
    0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF, 0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D};
static const uint8_t selftest_v1_0[64] = {
    0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
    0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
    0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
    0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79};
static const uint8_t selftest_v2_0[64] = {
    0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
    0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
    0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
    0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F};
static const uint8_t selftest_fm17522[64] = {
    0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18, 0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
    0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7, 0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
    0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7, 0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
    0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB, 0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62};

/* ------------------------------------------------------------------------ */

struct Mfrc522 {
    Mfrc522Hooks hooks;
    uint8_t version;
    uint8_t reg[64];

    uint8_t fifo[FIFO_SIZE];
    uint8_t fifo_len;
    uint8_t internal[INTERNAL_BUFFER_SIZE];

    /* UART framing */
    bool have_addr;
    uint8_t pending_addr;

    /* Command engine */
    uint8_t cmd;
    uint8_t modem;
    uint8_t rx_last_bits;

    /* CRC coprocessor */
    bool crc_active;
    bool crc_ready;
    uint16_t crc;

    /* RF */
    bool rf_pending;
    uint32_t rf_seq;
    uint8_t rf_cmd;
    bool field_on;

    /* Timer unit */
    bool t_running;
    uint64_t t_start_us;
    uint32_t t_tick_fc;
    uint16_t t_reload;
    uint16_t t_value;

    /* Power */
    bool soft_pd;
    bool waking;
    uint64_t wake_at_us;

    Mfrc522Status stats;
};

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

static uint64_t now_us(Mfrc522* m) {
    return m->hooks.now_us(m->hooks.ctx);
}

static uint8_t rev8(uint8_t b) {
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

static uint16_t rev16(uint16_t v) {
    return (uint16_t)((rev8((uint8_t)(v & 0xFF)) << 8) | rev8((uint8_t)(v >> 8)));
}

static uint16_t crc_a_byte(uint16_t crc, uint8_t b) {
    b ^= (uint8_t)(crc & 0xFF);
    b ^= (uint8_t)(b << 4);
    return (uint16_t)((crc >> 8) ^ ((uint16_t)b << 8) ^ ((uint16_t)b << 3) ^ (b >> 4));
}

static uint16_t crc_preset(Mfrc522* m) {
    switch(m->reg[RegMode] & 0x03) {
    case 0:
        return 0x0000;
    case 1:
        return 0x6363;
    case 2:
        return 0xA671;
    default:
        return 0xFFFF;
    }
}

static uint16_t crc_calc(uint16_t preset, const uint8_t* d, size_t n) {
    uint16_t crc = preset;
    for(size_t i = 0; i < n; i++) crc = crc_a_byte(crc, d[i]);
    return crc;
}

static uint16_t crc_result(Mfrc522* m) {
    return (m->reg[RegMode] & 0x80) ? rev16(m->crc) : m->crc;
}

static void irq_set(Mfrc522* m, uint8_t bits) {
    m->reg[RegComIrq] |= bits;
}

static void error_set(Mfrc522* m, uint8_t bits) {
    m->reg[RegError] |= bits;
    irq_set(m, IRQ_ERR);
}

static bool status_lo_alert(Mfrc522* m) {
    return m->fifo_len <= (m->reg[RegWaterLevel] & 0x3F);
}

static bool status_hi_alert(Mfrc522* m) {
    return (FIFO_SIZE - m->fifo_len) <= (m->reg[RegWaterLevel] & 0x3F);
}

static void fifo_update_alerts(Mfrc522* m) {
    if(status_lo_alert(m)) irq_set(m, IRQ_LOALERT);
    if(status_hi_alert(m)) irq_set(m, IRQ_HIALERT);
}

static void fifo_clear(Mfrc522* m) {
    m->fifo_len = 0;
    fifo_update_alerts(m);
}

static void fifo_push(Mfrc522* m, uint8_t b) {
    if(m->fifo_len >= FIFO_SIZE) {
        error_set(m, ERR_BUFOVFL);
        return;
    }
    m->fifo[m->fifo_len++] = b;
    fifo_update_alerts(m);
}

static uint8_t fifo_pop(Mfrc522* m) {
    if(m->fifo_len == 0) return 0x00;
    uint8_t b = m->fifo[0];
    m->fifo_len--;
    memmove(m->fifo, m->fifo + 1, m->fifo_len);
    fifo_update_alerts(m);
    return b;
}

/* ------------------------------------------------------------------------ */
/* Timer unit                                                                */
/* ------------------------------------------------------------------------ */

static uint64_t us_to_fc(uint64_t us) {
    return us * 1356U / 100U;
}

static uint64_t fc_to_us(uint64_t fc) {
    return (fc * 100U + 1355U) / 1356U;
}

static uint32_t timer_tick_fc(Mfrc522* m) {
    uint32_t p = ((uint32_t)(m->reg[RegTMode] & 0x0F) << 8) | m->reg[RegTPrescaler];
    /* DemodReg.TPrescalEven (bit 4) only changes the formula on v2.0 silicon */
    bool even = (m->reg[RegDemod] & 0x10) && m->version == 0x92;
    return even ? (2U * p + 2U) : (2U * p + 1U);
}

static uint16_t timer_reload_value(Mfrc522* m) {
    return (uint16_t)((m->reg[RegTReloadH] << 8) | m->reg[RegTReloadL]);
}

static void timer_start(Mfrc522* m) {
    m->t_running = true;
    m->t_start_us = now_us(m);
    m->t_tick_fc = timer_tick_fc(m);
    m->t_reload = timer_reload_value(m);
}

static uint16_t timer_value(Mfrc522* m) {
    if(!m->t_running) return m->t_value;
    uint64_t elapsed_fc = us_to_fc(now_us(m) - m->t_start_us);
    uint64_t ticks = elapsed_fc / m->t_tick_fc;
    if(ticks >= m->t_reload) return 0;
    return (uint16_t)(m->t_reload - ticks);
}

static void timer_stop(Mfrc522* m) {
    if(!m->t_running) return;
    m->t_value = timer_value(m);
    m->t_running = false;
}

static uint64_t timer_period_fc(Mfrc522* m) {
    return (uint64_t)m->t_tick_fc * ((uint64_t)m->t_reload + 1U);
}

static void timer_poll(Mfrc522* m) {
    if(!m->t_running) return;
    /* While an RF exchange is in flight the real silicon would already know
     * whether a reply started; hold the timeout until the outcome is known. */
    if(m->rf_pending) return;
    uint64_t now = now_us(m);
    uint64_t period = timer_period_fc(m);
    uint64_t elapsed = us_to_fc(now - m->t_start_us);
    if(elapsed < period) return;

    irq_set(m, IRQ_TIMER);
    if(m->reg[RegTMode] & TMODE_TAUTORESTART) {
        /* Reload from TReloadReg and keep counting */
        uint64_t periods = elapsed / period;
        m->t_start_us += fc_to_us(periods * period);
        m->t_tick_fc = timer_tick_fc(m);
        m->t_reload = timer_reload_value(m);
    } else {
        m->t_running = false;
        m->t_value = 0;
    }
}

/* ------------------------------------------------------------------------ */
/* Power / field                                                             */
/* ------------------------------------------------------------------------ */

static void update_field(Mfrc522* m) {
    bool on = !m->soft_pd && !m->waking && (m->reg[RegTxControl] & 0x03);
    if(on != m->field_on) {
        m->field_on = on;
        m->hooks.set_field(on, m->hooks.ctx);
    }
}

/* ------------------------------------------------------------------------ */
/* Command engine                                                            */
/* ------------------------------------------------------------------------ */

static void abort_rf(Mfrc522* m) {
    if(m->rf_pending) {
        m->rf_pending = false;
        m->rf_seq++; /* any late result is now stale */
    }
}

static void terminate_command(Mfrc522* m, bool idle_irq) {
    m->cmd = CmdIdle;
    m->modem = ModemIdle;
    m->crc_active = false;
    if(idle_irq) irq_set(m, IRQ_IDLE);
}

static void reset_registers(Mfrc522* m) {
    abort_rf(m);
    memcpy(m->reg, reg_reset_values, sizeof(m->reg));
    m->fifo_len = 0;
    m->cmd = CmdIdle;
    m->modem = ModemIdle;
    m->rx_last_bits = 0;
    m->crc_active = false;
    m->crc_ready = true;
    m->crc = 0xFFFF;
    m->t_running = false;
    m->t_value = 0;
    m->soft_pd = false;
    m->waking = false;
    update_field(m);
}

static void set_last_rf(Mfrc522* m, const char* prefix, const uint8_t* d, uint16_t bits) {
    char* p = m->stats.last_rf;
    size_t left = sizeof(m->stats.last_rf);
    int w = snprintf(p, left, "%s", prefix);
    if(w < 0) return;
    size_t used = (size_t)w;
    size_t bytes = (bits + 7U) / 8U;
    for(size_t i = 0; i < bytes && used + 3 < left; i++) {
        w = snprintf(p + used, left - used, "%02X", d[i]);
        if(w < 0) break;
        used += (size_t)w;
    }
    if(bits % 8 && used + 4 < left) snprintf(p + used, left - used, "/%u", (unsigned)(bits % 8));
}

static uint32_t compute_fwt(Mfrc522* m) {
    uint64_t fc;
    if(m->reg[RegTMode] & TMODE_TAUTO) {
        fc = (uint64_t)timer_tick_fc(m) * ((uint64_t)timer_reload_value(m) + 1U);
    } else {
        fc = FWT_DEFAULT_FC;
    }
    if(fc < FWT_MIN_FC) fc = FWT_MIN_FC;
    if(fc > FWT_MAX_FC) fc = FWT_MAX_FC;
    return (uint32_t)fc;
}

static void clear_rx_errors(Mfrc522* m) {
    m->reg[RegError] &= (uint8_t)~ERR_RX_MASK;
}

static void submit(Mfrc522* m, RfRequest* req) {
    req->seq = ++m->rf_seq;
    m->rf_pending = true;
    m->rf_cmd = m->cmd;
    m->stats.rf_frames++;
    m->hooks.rf_submit(req, m->hooks.ctx);
}

static void complete_locally(Mfrc522* m, RfReqType type, RfStatus status) {
    RfResult res;
    memset(&res, 0, sizeof(res));
    res.seq = m->rf_seq;
    res.type = type;
    res.status = status;
    mfrc522_rf_complete(m, &res);
}

static void start_transmit(Mfrc522* m, bool transceive) {
    RfRequest req;
    memset(&req, 0, sizeof(req));
    req.type = RfReqTransceive;

    uint8_t n = m->fifo_len;
    memcpy(req.tx, m->fifo, n);
    fifo_clear(m);

    uint8_t last = m->reg[RegBitFraming] & 0x07;
    uint16_t bits = 0;
    if(n) bits = last ? (uint16_t)((n - 1U) * 8U + last) : (uint16_t)(n * 8U);
    if((m->reg[RegTxMode] & MODE_CRC_EN) && bits && (bits % 8U) == 0 && n + 2U <= RF_MAX_FRAME) {
        uint16_t crc = crc_calc(crc_preset(m), req.tx, n);
        req.tx[n] = (uint8_t)(crc & 0xFF);
        req.tx[n + 1] = (uint8_t)(crc >> 8);
        bits += 16;
    }
    req.tx_bits = bits;
    req.listen = transceive && !(m->reg[RegCommand] & CMD_RCV_OFF);
    req.crypto = (m->reg[RegStatus2] & STATUS2_CRYPTO1ON) != 0;
    req.rx_align = (m->reg[RegBitFraming] >> 4) & 0x07;
    req.fwt_fc = compute_fwt(m);

    if(transceive) clear_rx_errors(m);
    m->rx_last_bits = 0;
    m->modem = ModemTransmitting;
    if(m->reg[RegTMode] & TMODE_TAUTO) timer_start(m);

    set_last_rf(m, "> ", req.tx, req.tx_bits);

    /* Only 106 kBd is reachable through the Flipper ISO14443-A poller. A frame
     * sent at another rate is never understood by the card. */
    bool unsupported_rate = ((m->reg[RegTxMode] >> 4) & 0x07) || ((m->reg[RegRxMode] >> 4) & 0x07);
    if(bits == 0 || unsupported_rate) {
        m->rf_seq++;
        m->rf_pending = true;
        m->rf_cmd = m->cmd;
        complete_locally(m, RfReqTransceive, RfStatusTimeout);
        return;
    }
    submit(m, &req);
}

static void start_auth(Mfrc522* m) {
    clear_rx_errors(m);
    if(m->fifo_len < 12) {
        /* "Each command that needs a certain number of arguments starts processing
         * only when it has received the correct number of arguments". FIFO access
         * is blocked during MFAuthent, so it waits until the host cancels it. */
        m->modem = ModemIdle;
        return;
    }
    RfRequest req;
    memset(&req, 0, sizeof(req));
    req.type = RfReqAuth;
    req.auth_cmd = m->fifo[0];
    req.auth_block = m->fifo[1];
    memcpy(req.key, &m->fifo[2], 6);
    memcpy(req.uid, &m->fifo[8], 4);
    fifo_clear(m);
    req.crypto = (m->reg[RegStatus2] & STATUS2_CRYPTO1ON) != 0;
    req.listen = true;
    req.fwt_fc = compute_fwt(m);

    m->modem = ModemTransmitting;
    if(m->reg[RegTMode] & TMODE_TAUTO) timer_start(m);
    uint8_t shown[2] = {req.auth_cmd, req.auth_block};
    set_last_rf(m, "AUTH ", shown, 16);
    submit(m, &req);
}

static void self_test(Mfrc522* m) {
    fifo_clear(m);
    bool zero = true;
    for(size_t i = 0; i < INTERNAL_BUFFER_SIZE; i++) {
        if(m->internal[i]) zero = false;
    }
    const uint8_t* ref = NULL;
    switch(m->version) {
    case 0x90:
        ref = selftest_v0_0;
        break;
    case 0x91:
        ref = selftest_v1_0;
        break;
    case 0x92:
        ref = selftest_v2_0;
        break;
    case 0x88:
        ref = selftest_fm17522;
        break;
    default:
        break;
    }
    if(zero && ref) {
        for(size_t i = 0; i < 64; i++) fifo_push(m, ref[i]);
    } else {
        /* Deterministic, configuration dependent output */
        uint32_t x = 0x9E3779B9U ^ m->version;
        for(size_t i = 0; i < INTERNAL_BUFFER_SIZE; i++) x = (x ^ m->internal[i]) * 16777619U;
        fifo_push(m, 0x00);
        for(size_t i = 1; i < 64; i++) {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            fifo_push(m, (uint8_t)x);
        }
    }
    m->crc_ready = true;
    m->reg[RegDivIrq] |= DIVIRQ_CRC;
}

static void crc_consume_fifo(Mfrc522* m) {
    while(m->fifo_len) {
        uint8_t b = fifo_pop(m);
        if(m->reg[RegMode] & 0x80) b = rev8(b);
        m->crc = crc_a_byte(m->crc, b);
    }
    uint16_t r = crc_result(m);
    m->reg[RegCRCResultH] = (uint8_t)(r >> 8);
    m->reg[RegCRCResultL] = (uint8_t)(r & 0xFF);
    m->crc_ready = true;
    m->reg[RegDivIrq] |= DIVIRQ_CRC;
}

static void mem_try_store(Mfrc522* m) {
    if(m->fifo_len < INTERNAL_BUFFER_SIZE) return;
    for(size_t i = 0; i < INTERNAL_BUFFER_SIZE; i++) m->internal[i] = fifo_pop(m);
    terminate_command(m, true);
}

static void start_command(Mfrc522* m, uint8_t cmd) {
    abort_rf(m);
    m->crc_active = false;

    switch(cmd) {
    case CmdIdle:
        /* Host-initiated Idle does not raise IdleIRq */
        m->cmd = CmdIdle;
        m->modem = ModemIdle;
        break;

    case CmdMem:
        m->cmd = CmdMem;
        m->modem = ModemIdle;
        if(m->fifo_len == 0) {
            for(size_t i = 0; i < INTERNAL_BUFFER_SIZE; i++) fifo_push(m, m->internal[i]);
            terminate_command(m, true);
        } else {
            mem_try_store(m); /* waits for 25 bytes if fewer are present */
        }
        break;

    case CmdGenerateRandomId:
        furi_hal_random_fill_buf(m->internal, 10);
        terminate_command(m, true);
        break;

    case CmdCalcCrc:
        m->cmd = CmdCalcCrc;
        m->modem = ModemIdle;
        if((m->reg[RegAutoTest] & 0x0F) == 0x09) {
            self_test(m);
        } else {
            m->crc = crc_preset(m);
            m->crc_ready = false;
            m->crc_active = true;
            crc_consume_fifo(m);
        }
        break;

    case CmdTransmit:
        m->cmd = CmdTransmit;
        start_transmit(m, false);
        break;

    case CmdReceive:
        m->cmd = CmdReceive;
        clear_rx_errors(m);
        m->modem = ModemWaitData;
        break;

    case CmdTransceive:
        m->cmd = CmdTransceive;
        m->modem = ModemWaitStartSend;
        if(m->reg[RegBitFraming] & BITFRAMING_STARTSEND) start_transmit(m, true);
        break;

    case CmdMfAuthent:
        m->cmd = CmdMfAuthent;
        start_auth(m);
        break;

    case CmdSoftReset:
        reset_registers(m);
        break;

    default:
        /* Reserved / unknown command: terminates immediately with IdleIRq */
        terminate_command(m, true);
        break;
    }
}

static void write_rx_to_fifo(Mfrc522* m, const RfResult* res) {
    uint8_t align = (m->reg[RegBitFraming] >> 4) & 0x07;
    uint16_t bits = res->rx_bits;
    uint8_t err = 0;

    if((m->reg[RegRxMode] & MODE_CRC_EN) && bits >= 8) {
        if((bits % 8U) == 0 && bits >= 24 &&
           crc_calc(crc_preset(m), res->rx, bits / 8U) == 0) {
            bits -= 16; /* CRC verified and removed */
        } else {
            err |= ERR_CRC;
        }
    }

    uint8_t buf[RF_MAX_FRAME + 1];
    memset(buf, 0, sizeof(buf));
    for(uint16_t i = 0; i < bits; i++) {
        uint8_t bit = (res->rx[i / 8U] >> (i % 8U)) & 1U;
        uint16_t pos = (uint16_t)(align + i);
        buf[pos / 8U] |= (uint8_t)(bit << (pos % 8U));
    }
    uint16_t total = (uint16_t)(align + bits);
    uint16_t nbytes = (uint16_t)((total + 7U) / 8U);
    for(uint16_t i = 0; i < nbytes; i++) fifo_push(m, buf[i]);
    m->rx_last_bits = (uint8_t)(total % 8U);

    /* No collision position information is available from the front-end */
    m->reg[RegColl] = (uint8_t)((m->reg[RegColl] & 0x80) | 0x20);
    if(res->status == RfStatusCollision) err |= ERR_COLL;
    if(res->status == RfStatusParityError) err |= ERR_PARITY;
    if(err) error_set(m, err);

    char prefix[6] = "< ";
    set_last_rf(m, prefix, res->rx, res->rx_bits);
}

/* ------------------------------------------------------------------------ */
/* Register access                                                           */
/* ------------------------------------------------------------------------ */

static uint8_t reg_read(Mfrc522* m, uint8_t a) {
    if(reg_is_reserved(a)) return m->reg[a];
    switch(a) {
    case RegCommand:
        return (uint8_t)((m->reg[RegCommand] & CMD_RCV_OFF) |
                         ((m->soft_pd || m->waking) ? CMD_POWER_DOWN : 0) | m->cmd);
    case RegStatus1: {
        uint8_t s = 0;
        if(crc_result(m) == 0) s |= 0x40;
        if(m->crc_ready) s |= 0x20;
        bool irq = ((m->reg[RegComIrq] & m->reg[RegComIEn] & 0x7F) != 0) ||
                   ((m->reg[RegDivIrq] & m->reg[RegDivIEn] & DIVIRQ_MASK) != 0);
        if(irq) s |= 0x10;
        if(m->t_running) s |= 0x08;
        if(status_hi_alert(m)) s |= 0x02;
        if(status_lo_alert(m)) s |= 0x01;
        return s;
    }
    case RegStatus2:
        return (uint8_t)((m->reg[RegStatus2] & 0xC8) | m->modem);
    case RegFIFOData:
        return fifo_pop(m);
    case RegFIFOLevel:
        return m->fifo_len;
    case RegControl:
        return (uint8_t)((m->reg[RegControl] & 0x38) | m->rx_last_bits);
    case RegTCounterValH:
        return (uint8_t)(timer_value(m) >> 8);
    case RegTCounterValL:
        return (uint8_t)(timer_value(m) & 0xFF);
    case RegVersion:
        return m->version;
    case RegTestBus:
    case RegTestADC:
        return 0x00;
    default:
        return m->reg[a];
    }
}

static void reg_write(Mfrc522* m, uint8_t a, uint8_t v) {
    if(reg_is_reserved(a)) return;
    switch(a) {
    case RegCommand: {
        m->reg[RegCommand] = (uint8_t)(v & CMD_RCV_OFF);
        uint8_t cmd = v & 0x0F;
        if(v & CMD_POWER_DOWN) {
            if(cmd != CmdSoftReset && !m->soft_pd) {
                m->soft_pd = true;
                m->waking = false;
                abort_rf(m);
                update_field(m);
            }
        } else if(m->soft_pd) {
            m->soft_pd = false;
            m->waking = true;
            m->wake_at_us = now_us(m) + WAKEUP_TIME_US;
        }
        if(cmd != CmdNoCmdChange) start_command(m, cmd);
        break;
    }
    case RegComIrq:
        if(v & IRQ_SET1) {
            m->reg[RegComIrq] |= (uint8_t)(v & 0x7F);
        } else {
            m->reg[RegComIrq] &= (uint8_t)~(v & 0x7F);
        }
        break;
    case RegDivIrq:
        if(v & DIVIRQ_SET2) {
            m->reg[RegDivIrq] |= (uint8_t)(v & DIVIRQ_MASK);
        } else {
            m->reg[RegDivIrq] &= (uint8_t)~(v & DIVIRQ_MASK);
        }
        break;
    case RegStatus2:
        /* TempSensClear / I2CForceHS are R/W; MFCrypto1On can only be cleared */
        m->reg[RegStatus2] =
            (uint8_t)((v & 0xC0) | (m->reg[RegStatus2] & v & STATUS2_CRYPTO1ON));
        break;
    case RegFIFOData:
        if(m->cmd == CmdMfAuthent) {
            error_set(m, ERR_WR); /* FIFO access is blocked during MFAuthent */
            break;
        }
        if(m->rf_pending) error_set(m, ERR_WR);
        fifo_push(m, v);
        if(m->crc_active) crc_consume_fifo(m);
        if(m->cmd == CmdMem) mem_try_store(m);
        break;
    case RegFIFOLevel:
        if(v & 0x80) {
            m->fifo_len = 0;
            m->reg[RegError] &= (uint8_t)~ERR_BUFOVFL;
            fifo_update_alerts(m);
        }
        break;
    case RegWaterLevel:
        m->reg[a] = v & 0x3F;
        fifo_update_alerts(m);
        break;
    case RegControl:
        if(v & 0x80) timer_stop(m); /* TStopNow */
        if(v & 0x40) timer_start(m); /* TStartNow */
        break;
    case RegBitFraming:
        m->reg[a] = v & 0xF7;
        if((v & BITFRAMING_STARTSEND) && m->cmd == CmdTransceive && !m->rf_pending &&
           (m->modem == ModemWaitStartSend || m->modem == ModemWaitData)) {
            start_transmit(m, true);
        }
        break;
    case RegColl:
        m->reg[a] = (uint8_t)((m->reg[a] & 0x7F) | (v & 0x80));
        break;
    case RegTxControl:
        m->reg[a] = v;
        update_field(m);
        break;
    case RegError:
    case RegStatus1:
    case RegTCounterValH:
    case RegTCounterValL:
    case RegVersion:
    case RegTestBus:
    case RegTestADC:
        break; /* read-only */
    default:
        m->reg[a] = v;
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

Mfrc522* mfrc522_alloc(const Mfrc522Hooks* hooks) {
    Mfrc522* m = malloc(sizeof(Mfrc522));
    memset(m, 0, sizeof(Mfrc522));
    m->hooks = *hooks;
    m->version = 0x92;
    mfrc522_hard_reset(m);
    return m;
}

void mfrc522_free(Mfrc522* m) {
    free(m);
}

void mfrc522_set_version(Mfrc522* m, uint8_t version) {
    m->version = version;
}

void mfrc522_hard_reset(Mfrc522* m) {
    memset(m->internal, 0, sizeof(m->internal));
    m->have_addr = false;
    reset_registers(m);
}

void mfrc522_uart_rx(Mfrc522* m, uint8_t byte) {
    if(!m->have_addr) {
        if(byte & 0x80) {
            /* Read: address byte with MSB set, chip answers with register data */
            uint8_t v = reg_read(m, byte & 0x3F);
            m->stats.reg_reads++;
            m->hooks.uart_tx(v, m->hooks.ctx);
        } else {
            /* Write: the chip echoes the address byte as soon as it has been
             * received (datasheet Fig. 10: TX starts after the address byte's
             * stop bit). Hosts may wait for this echo before sending the data
             * byte, or send the data byte straight away. */
            m->have_addr = true;
            m->pending_addr = byte;
            m->hooks.uart_tx(byte, m->hooks.ctx);
        }
        return;
    }

    /* Second byte of a write: the data. The echo has already gone out, so a
     * SerialSpeedReg update or SoftReset never affects it. */
    m->have_addr = false;
    m->stats.reg_writes++;
    reg_write(m, m->pending_addr & 0x3F, byte);
}

void mfrc522_rf_complete(Mfrc522* m, const RfResult* res) {
    if(!m->rf_pending || res->seq != m->rf_seq) return; /* stale */
    m->rf_pending = false;

    if(res->type == RfReqAuth) {
        /* RxIRq and TxIRq are blocked during authentication processing */
        if(res->status == RfStatusOk) {
            m->reg[RegStatus2] |= STATUS2_CRYPTO1ON;
            if(m->reg[RegTMode] & TMODE_TAUTO) timer_stop(m);
            terminate_command(m, true);
            snprintf(m->stats.last_rf, sizeof(m->stats.last_rf), "AUTH ok");
        } else {
            /* Card stays silent: the command keeps waiting, only the timer ends it */
            m->reg[RegStatus2] &= (uint8_t)~STATUS2_CRYPTO1ON;
            m->modem = ModemWaitData;
            m->stats.rf_timeouts++;
            snprintf(m->stats.last_rf, sizeof(m->stats.last_rf), "AUTH failed");
        }
        return;
    }

    irq_set(m, IRQ_TX);

    if(m->rf_cmd == CmdTransmit) {
        terminate_command(m, true);
        return;
    }

    if(res->status == RfStatusTimeout) {
        /* Receiver keeps listening; TimerIRq (if armed) reports the timeout */
        m->modem = ModemWaitData;
        m->stats.rf_timeouts++;
        snprintf(m->stats.last_rf, sizeof(m->stats.last_rf), "< (no answer)");
        return;
    }

    if((m->reg[RegRxMode] & RXMODE_NOERR) && res->rx_bits < 4) {
        /* RxNoErr: an invalid data stream (< 4 bits) is ignored, receiver stays active */
        m->modem = ModemWaitData;
        return;
    }
    if((m->reg[RegTMode] & TMODE_TAUTO) && !(m->reg[RegRxMode] & RXMODE_MULTIPLE)) {
        timer_stop(m);
    }
    write_rx_to_fifo(m, res);
    m->modem = ModemWaitStartSend; /* Transceive stays active */
    irq_set(m, IRQ_RX);
}

void mfrc522_tick(Mfrc522* m) {
    if(m->waking && now_us(m) >= m->wake_at_us) {
        m->waking = false;
        update_field(m);
    }
    timer_poll(m);
}

uint64_t mfrc522_next_event_us(Mfrc522* m) {
    uint64_t next = UINT64_MAX;
    if(m->waking) next = m->wake_at_us;
    if(m->t_running && !m->rf_pending) {
        uint64_t t = m->t_start_us + fc_to_us(timer_period_fc(m));
        if(t < next) next = t;
    }
    return next;
}

uint32_t mfrc522_uart_baud(Mfrc522* m) {
    uint8_t v = m->reg[RegSerialSpeed];
    uint32_t t0 = (v >> 5) & 0x07;
    uint32_t t1 = v & 0x1F;
    uint64_t div = (t0 == 0) ? (t1 + 1U) : ((t1 + 33U) << (t0 - 1U));
    return (uint32_t)((27120000ULL + div / 2U) / div);
}

void mfrc522_get_status(Mfrc522* m, Mfrc522Status* status) {
    m->stats.command = m->cmd;
    m->stats.serial_speed = m->reg[RegSerialSpeed];
    m->stats.uart_baud = mfrc522_uart_baud(m);
    m->stats.field_on = m->field_on;
    m->stats.crypto_on = (m->reg[RegStatus2] & STATUS2_CRYPTO1ON) != 0;
    m->stats.soft_power_down = m->soft_pd;
    *status = m->stats;
}

const char* mfrc522_command_name(uint8_t command) {
    switch(command) {
    case CmdIdle:
        return "Idle";
    case CmdMem:
        return "Mem";
    case CmdGenerateRandomId:
        return "RandomID";
    case CmdCalcCrc:
        return "CalcCRC";
    case CmdTransmit:
        return "Transmit";
    case CmdReceive:
        return "Receive";
    case CmdTransceive:
        return "Transceive";
    case CmdMfAuthent:
        return "MFAuthent";
    case CmdSoftReset:
        return "SoftReset";
    default:
        return "?";
    }
}
