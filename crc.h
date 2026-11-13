/**
 * Link-layer CRCs of the supported contactless technologies.
 * Byte order on the air matches the Flipper firmware helpers.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum {
    CrcTypeA, /* ISO14443-A: init 6363, LSB first */
    CrcTypeB, /* ISO14443-B / ST25TB: init FFFF, complemented, LSB first */
    CrcTypeFelica, /* JIS X 6319-4: CCITT init 0000, MSB first */
    CrcType15693, /* ISO13239: init FFFF, complemented, LSB first */
    CrcTypePicopass, /* ISO13239 variant: init E012, not complemented */
} CrcType;

/** Append the 2 CRC bytes to data[0..len). */
void crc_append(CrcType type, uint8_t* data, size_t len);

/** Check a frame whose last 2 bytes are the CRC. */
bool crc_check(CrcType type, const uint8_t* frame, size_t len_with_crc);

uint16_t crc_a_update(uint16_t crc, uint8_t byte);
