/**
 * Minimal MIFARE Classic Crypto1 implementation (reader side).
 * Based on the publicly documented cipher (crapto1 reference).
 * Prefixed mfc1_ to avoid clashing with firmware-exported symbols.
 */
#pragma once

#include <stdint.h>

typedef struct {
    uint32_t odd;
    uint32_t even;
} Mfc1State;

void mfc1_init(Mfc1State* s, uint64_t key);
uint8_t mfc1_bit(Mfc1State* s, uint8_t in, int is_encrypted);
uint8_t mfc1_byte(Mfc1State* s, uint8_t in, int is_encrypted);
uint32_t mfc1_word(Mfc1State* s, uint32_t in, int is_encrypted);
/** Current keystream bit (used for encrypted parity). */
uint8_t mfc1_peek(const Mfc1State* s);
uint32_t mfc1_prng_successor(uint32_t x, uint32_t n);
/** Odd parity bit of a byte (1 when the byte has an even number of ones). */
uint8_t mfc1_odd_parity8(uint8_t b);
