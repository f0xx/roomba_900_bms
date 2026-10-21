#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// One-shot SHA-1 for messages of 0..55 bytes (a single 512-bit block).
bool sha1_oneshot(const uint8_t *msg, size_t len, uint8_t out[20]);

// TI gauge unseal response.
// challenge_wire and response_wire are SMBus order, least-significant byte first.
// key is the 16-byte key in the order given by the TRM (MSB first).
bool ti_unseal_response(const uint8_t key[16],
                        const uint8_t challenge_wire[20],
                        uint8_t response_wire[20]);

bool sha1_self_test(void);
