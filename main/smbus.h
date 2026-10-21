#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t smbus_init(void);
esp_err_t smbus_probe(uint8_t addr, bool *found);
esp_err_t smbus_find_mux(uint8_t *addr_out);
esp_err_t smbus_mux_select(uint8_t mux_addr, int channel);
void smbus_log_scan(void);

esp_err_t smbus_read_word(uint8_t addr, uint8_t cmd, uint16_t *value);
esp_err_t smbus_write_word(uint8_t addr, uint8_t cmd, uint16_t value);
esp_err_t smbus_read_block(uint8_t addr, uint8_t cmd, uint8_t *data, size_t *len, size_t max_len);
esp_err_t smbus_write_block(uint8_t addr, uint8_t cmd, const uint8_t *data, size_t len);
esp_err_t smbus_write_raw_pec(uint8_t addr, uint8_t cmd, const uint8_t *data, size_t len);

// -1 until the first successful gauge read, then 0 (off) or 1 (on).
int smbus_pec_mode(void);

// Used by the command scan. A clock-stretch past the ESP32 I2C limit leaves SCL low;
// the next probe has to reset the peripheral or every later command looks like a NACK.
void smbus_set_timeout_ms(int timeout_ms);
esp_err_t smbus_bus_recover(void);
