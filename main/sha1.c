#include "sha1.h"

#include <string.h>

static uint32_t rol(uint32_t value, int bits) {
    return (value << bits) | (value >> (32 - bits));
}

bool sha1_oneshot(const uint8_t *msg, size_t len, uint8_t out[20]) {
    if (msg == NULL || out == NULL || len > 55) {
        return false;
    }

    uint8_t block[64];
    memset(block, 0, sizeof(block));
    if (len > 0) {
        memcpy(block, msg, len);
    }
    block[len] = 0x80;
    uint64_t bit_len = (uint64_t)len * 8u;
    block[60] = (uint8_t)(bit_len >> 24);
    block[61] = (uint8_t)(bit_len >> 16);
    block[62] = (uint8_t)(bit_len >> 8);
    block[63] = (uint8_t)bit_len;

    uint32_t w[80];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[i * 4] << 24) | ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] << 8) | (uint32_t)block[i * 4 + 3];
    }
    for (int i = 16; i < 80; i++) {
        w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    uint32_t a = 0x67452301u;
    uint32_t b = 0xEFCDAB89u;
    uint32_t c = 0x98BADCFEu;
    uint32_t d = 0x10325476u;
    uint32_t e = 0xC3D2E1F0u;

    for (int i = 0; i < 80; i++) {
        uint32_t f;
        uint32_t k;
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999u;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1u;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDCu;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6u;
        }
        uint32_t temp = rol(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol(b, 30);
        b = a;
        a = temp;
    }

    a += 0x67452301u;
    b += 0xEFCDAB89u;
    c += 0x98BADCFEu;
    d += 0x10325476u;
    e += 0xC3D2E1F0u;

    uint32_t h[5] = {a, b, c, d, e};
    for (int i = 0; i < 5; i++) {
        out[i * 4] = (uint8_t)(h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(h[i] >> 8);
        out[i * 4 + 3] = (uint8_t)h[i];
    }
    return true;
}

bool ti_unseal_response(const uint8_t key[16],
                        const uint8_t challenge_wire[20],
                        uint8_t response_wire[20]) {
    if (key == NULL || challenge_wire == NULL || response_wire == NULL) {
        return false;
    }

    // TRM: wire bytes are LSB first. The SHA-1 message appends M MSB first.
    uint8_t block[36];
    memcpy(block, key, 16);
    for (int i = 0; i < 20; i++) {
        block[16 + i] = challenge_wire[19 - i];
    }

    uint8_t hash1[20];
    if (!sha1_oneshot(block, sizeof(block), hash1)) {
        return false;
    }
    memcpy(block + 16, hash1, 20);

    uint8_t hash2[20];
    if (!sha1_oneshot(block, sizeof(block), hash2)) {
        return false;
    }
    for (int i = 0; i < 20; i++) {
        response_wire[i] = hash2[19 - i];
    }
    return true;
}

bool sha1_self_test(void) {
    static const uint8_t abc_expect[20] = {
        0xA9, 0x99, 0x3E, 0x36, 0x47, 0x06, 0x81, 0x6A, 0xBA, 0x3E,
        0x25, 0x71, 0x78, 0x50, 0xC2, 0x6C, 0x9C, 0xD0, 0xD8, 0x9D,
    };
    uint8_t digest[20];
    if (!sha1_oneshot((const uint8_t *)"abc", 3, digest) ||
        memcmp(digest, abc_expect, 20) != 0) {
        return false;
    }

    static const uint8_t key[16] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
        0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10,
    };
    uint8_t challenge[20];
    for (int i = 0; i < 20; i++) {
        challenge[i] = (uint8_t)i;
    }
    static const uint8_t response_expect[20] = {
        0x67, 0xD8, 0xD9, 0x05, 0x16, 0xEC, 0xD5, 0x5A, 0x3C, 0x75,
        0x1F, 0xC0, 0xBE, 0x7A, 0x36, 0x1D, 0xB3, 0x66, 0xA2, 0x46,
    };
    uint8_t response[20];
    if (!ti_unseal_response(key, challenge, response) ||
        memcmp(response, response_expect, 20) != 0) {
        return false;
    }
    return true;
}
