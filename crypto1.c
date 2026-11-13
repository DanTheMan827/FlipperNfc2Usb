#include "crypto1.h"

#define LF_POLY_ODD  (0x29CE5CU)
#define LF_POLY_EVEN (0x870804U)
#define BIT(x, n)    (((x) >> (n)) & 1U)
#define BEBIT(x, n)  BIT(x, (n) ^ 24U)

static inline uint32_t mfc1_filter(uint32_t x) {
    uint32_t f;
    f = (0xf22c0U >> (x & 0xfU)) & 16U;
    f |= (0x6c9c0U >> ((x >> 4) & 0xfU)) & 8U;
    f |= (0x3c8b0U >> ((x >> 8) & 0xfU)) & 4U;
    f |= (0x1e458U >> ((x >> 12) & 0xfU)) & 2U;
    f |= (0x0d938U >> ((x >> 16) & 0xfU)) & 1U;
    return BIT(0xEC57E80AU, f);
}

static inline uint32_t mfc1_parity32(uint32_t x) {
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    return BIT(0x6996U, x & 0xfU);
}

void mfc1_init(Mfc1State* s, uint64_t key) {
    s->odd = 0;
    s->even = 0;
    for(int i = 47; i > 0; i -= 2) {
        s->odd = (s->odd << 1) | (uint32_t)((key >> ((unsigned)(i - 1) ^ 7U)) & 1U);
        s->even = (s->even << 1) | (uint32_t)((key >> ((unsigned)i ^ 7U)) & 1U);
    }
}

uint8_t mfc1_bit(Mfc1State* s, uint8_t in, int is_encrypted) {
    uint32_t ret = mfc1_filter(s->odd);
    uint32_t feedin = ret & (is_encrypted ? 1U : 0U);
    feedin ^= in ? 1U : 0U;
    feedin ^= LF_POLY_ODD & s->odd;
    feedin ^= LF_POLY_EVEN & s->even;
    s->even = (s->even << 1) | mfc1_parity32(feedin);
    uint32_t t = s->odd;
    s->odd = s->even;
    s->even = t;
    return (uint8_t)ret;
}

uint8_t mfc1_byte(Mfc1State* s, uint8_t in, int is_encrypted) {
    uint8_t ret = 0;
    for(unsigned i = 0; i < 8; i++) {
        ret |= (uint8_t)(mfc1_bit(s, (uint8_t)BIT(in, i), is_encrypted) << i);
    }
    return ret;
}

uint32_t mfc1_word(Mfc1State* s, uint32_t in, int is_encrypted) {
    uint32_t ret = 0;
    for(unsigned i = 0; i < 32; i++) {
        ret |= (uint32_t)mfc1_bit(s, (uint8_t)BEBIT(in, i), is_encrypted) << (i ^ 24U);
    }
    return ret;
}

uint8_t mfc1_peek(const Mfc1State* s) {
    return (uint8_t)mfc1_filter(s->odd);
}

static inline uint32_t swap_endian(uint32_t x) {
    x = ((x >> 8) & 0xff00ffU) | ((x & 0xff00ffU) << 8);
    return (x >> 16) | (x << 16);
}

uint32_t mfc1_prng_successor(uint32_t x, uint32_t n) {
    x = swap_endian(x);
    while(n--) {
        x = (x >> 1) | (((x >> 16) ^ (x >> 18) ^ (x >> 19) ^ (x >> 21)) << 31);
    }
    return swap_endian(x);
}

uint8_t mfc1_odd_parity8(uint8_t b) {
    b ^= b >> 4;
    b ^= b >> 2;
    b ^= b >> 1;
    return (uint8_t)(~b & 1U);
}
