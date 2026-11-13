#include "crc.h"

uint16_t crc_a_update(uint16_t crc, uint8_t byte) {
    byte ^= (uint8_t)(crc & 0xFF);
    byte ^= (uint8_t)(byte << 4);
    return (uint16_t)((crc >> 8) ^ ((uint16_t)byte << 8) ^ ((uint16_t)byte << 3) ^ (byte >> 4));
}

static uint16_t crc_reflected(uint16_t crc, const uint8_t* d, size_t n) {
    for(size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for(int b = 0; b < 8; b++) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static uint16_t crc_felica(const uint8_t* d, size_t n) {
    uint16_t crc = 0x0000;
    for(size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)(d[i] << 8);
        for(int b = 0; b < 8; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Returns the two CRC bytes in transmission order */
static void crc_bytes(CrcType type, const uint8_t* d, size_t n, uint8_t out[2]) {
    uint16_t crc;
    switch(type) {
    case CrcTypeA:
        crc = 0x6363;
        for(size_t i = 0; i < n; i++) crc = crc_a_update(crc, d[i]);
        break;
    case CrcTypeB:
        crc = 0xFFFF;
        for(size_t i = 0; i < n; i++) crc = crc_a_update(crc, d[i]);
        crc = (uint16_t)~crc;
        break;
    case CrcTypeFelica:
        crc = crc_felica(d, n);
        out[0] = (uint8_t)(crc >> 8);
        out[1] = (uint8_t)(crc & 0xFF);
        return;
    case CrcType15693:
        crc = (uint16_t)~crc_reflected(0xFFFF, d, n);
        break;
    case CrcTypePicopass:
    default:
        crc = crc_reflected(0xE012, d, n);
        break;
    }
    out[0] = (uint8_t)(crc & 0xFF);
    out[1] = (uint8_t)(crc >> 8);
}

void crc_append(CrcType type, uint8_t* data, size_t len) {
    crc_bytes(type, data, len, &data[len]);
}

bool crc_check(CrcType type, const uint8_t* frame, size_t len_with_crc) {
    if(len_with_crc < 3) return false;
    uint8_t c[2];
    crc_bytes(type, frame, len_with_crc - 2, c);
    return c[0] == frame[len_with_crc - 2] && c[1] == frame[len_with_crc - 1];
}
