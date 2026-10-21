#include "smbus.h"

#include "config.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "smbus";

static i2c_master_bus_handle_t s_bus;
static int s_pec = -1;
static int s_timeout_ms = 1000;

typedef struct {
    uint8_t addr;
    bool used;
    i2c_master_dev_handle_t handle;
} dev_slot_t;

static dev_slot_t s_devs[4];

int smbus_pec_mode(void) {
    return s_pec;
}

void smbus_set_timeout_ms(int timeout_ms) {
    s_timeout_ms = timeout_ms < 20 ? 20 : timeout_ms;
}

esp_err_t smbus_bus_recover(void) {
    if (s_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2c_master_bus_reset(s_bus);
    vTaskDelay(pdMS_TO_TICKS(20));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus reset: %s", esp_err_to_name(err));
    }
    return err;
}

static uint8_t crc8(uint8_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t pec_write(uint8_t addr, const uint8_t *body, size_t body_len) {
    uint8_t addr_w = (uint8_t)(addr << 1);
    uint8_t crc = crc8(0, &addr_w, 1);
    return crc8(crc, body, body_len);
}

static uint8_t pec_read(uint8_t addr, uint8_t cmd, const uint8_t *payload, size_t payload_len) {
    uint8_t addr_w = (uint8_t)(addr << 1);
    uint8_t addr_r = (uint8_t)(addr_w | 1u);
    uint8_t crc = crc8(0, &addr_w, 1);
    crc = crc8(crc, &cmd, 1);
    crc = crc8(crc, &addr_r, 1);
    return crc8(crc, payload, payload_len);
}

esp_err_t smbus_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t device_for(uint8_t addr, i2c_master_dev_handle_t *out) {
    for (int i = 0; i < 4; i++) {
        if (s_devs[i].used && s_devs[i].addr == addr) {
            *out = s_devs[i].handle;
            return ESP_OK;
        }
    }
    for (int i = 0; i < 4; i++) {
        if (s_devs[i].used) {
            continue;
        }
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_FREQ_HZ,
            .scl_wait_us = 100000,
        };
        esp_err_t err = i2c_master_bus_add_device(s_bus, &cfg, &s_devs[i].handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "add device 0x%02x failed: %s", addr, esp_err_to_name(err));
            return err;
        }
        s_devs[i].addr = addr;
        s_devs[i].used = true;
        *out = s_devs[i].handle;
        return ESP_OK;
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t smbus_probe(uint8_t addr, bool *found) {
    if (found == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_master_probe(s_bus, addr, 100);
    *found = (err == ESP_OK);
    return ESP_OK;
}

esp_err_t smbus_find_mux(uint8_t *addr_out) {
    if (addr_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t chosen = 0;
    int count = 0;
    for (uint8_t addr = 0x70; addr <= 0x77; addr++) {
        bool found = false;
        smbus_probe(addr, &found);
        if (!found) {
            continue;
        }
        count++;
        ESP_LOGI(TAG, "TCA9548A candidate at 0x%02x", addr);
        if (chosen == 0 || addr == MUX_ADDR_DEFAULT) {
            chosen = addr;
        }
    }
    if (chosen == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    if (count > 1) {
        ESP_LOGW(TAG, "more than one mux address answered; using 0x%02x", chosen);
    }
    *addr_out = chosen;
    return ESP_OK;
}

esp_err_t smbus_mux_select(uint8_t mux_addr, int channel) {
    if (channel < 0 || channel > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mux_addr,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(s_bus, &cfg, &dev);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t mask = (uint8_t)(1u << channel);
    err = i2c_master_transmit(dev, &mask, 1, 1000);
    uint8_t readback = 0;
    if (err == ESP_OK) {
        err = i2c_master_receive(dev, &readback, 1, 1000);
    }
    i2c_master_bus_rm_device(dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mux 0x%02x channel write failed: %s", mux_addr, esp_err_to_name(err));
        return err;
    }
    if (readback != mask) {
        ESP_LOGE(TAG, "mux readback 0x%02x, channel %d needs mask 0x%02x. Tie RESET to 3.3V.",
                 readback, channel, mask);
        return ESP_ERR_INVALID_RESPONSE;
    }
    ESP_LOGI(TAG, "mux 0x%02x channel %d selected (mask 0x%02x)", mux_addr, channel, mask);
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

void smbus_log_scan(void) {
    char list[192];
    size_t pos = 0;
    int count = 0;
    list[0] = '\0';
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        bool found = false;
        smbus_probe(addr, &found);
        if (!found) {
            continue;
        }
        int wrote = snprintf(list + pos, sizeof(list) - pos, " 0x%02x", addr);
        if (wrote < 0 || (size_t)wrote >= sizeof(list) - pos) {
            break;
        }
        pos += (size_t)wrote;
        count++;
    }
    if (count == 0) {
        ESP_LOGW(TAG, "no device ACKed on the selected mux channel");
    } else {
        ESP_LOGI(TAG, "ACK on selected channel:%s", list);
    }
}

static esp_err_t transmit(uint8_t addr, const uint8_t *data, size_t len) {
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = device_for(addr, &dev);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_master_transmit(dev, data, len, s_timeout_ms);
}

static esp_err_t transceive(uint8_t addr, const uint8_t *wr, size_t wr_len, uint8_t *rd, size_t rd_len) {
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = device_for(addr, &dev);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_master_transmit_receive(dev, wr, wr_len, rd, rd_len, s_timeout_ms);
}

static esp_err_t write_body(uint8_t addr, const uint8_t *body, size_t body_len) {
    bool try_pec = s_pec != 0;
    bool try_plain = s_pec != 1;
    if (try_pec) {
        uint8_t frame[36];
        if (body_len + 1 > sizeof(frame)) {
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(frame, body, body_len);
        frame[body_len] = pec_write(addr, body, body_len);
        esp_err_t err = transmit(addr, frame, body_len + 1);
        if (err == ESP_OK || !try_plain) {
            return err;
        }
        ESP_LOGW(TAG, "write with PEC was not accepted (%s); retrying without PEC",
                 esp_err_to_name(err));
    }
    return transmit(addr, body, body_len);
}

esp_err_t smbus_write_word(uint8_t addr, uint8_t cmd, uint16_t value) {
    uint8_t body[3] = {cmd, (uint8_t)value, (uint8_t)(value >> 8)};
    return write_body(addr, body, sizeof(body));
}

esp_err_t smbus_write_block(uint8_t addr, uint8_t cmd, const uint8_t *data, size_t len) {
    if (data == NULL || len > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t body[34];
    body[0] = cmd;
    body[1] = (uint8_t)len;
    memcpy(body + 2, data, len);
    return write_body(addr, body, 2 + len);
}

esp_err_t smbus_write_raw_pec(uint8_t addr, uint8_t cmd, const uint8_t *data, size_t len) {
    if (data == NULL || len > 32) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t body[34];
    body[0] = cmd;
    memcpy(body + 1, data, len);
    return write_body(addr, body, 1 + len);
}

static esp_err_t read_word_pec(uint8_t addr, uint8_t cmd, uint16_t *value) {
    uint8_t raw[3] = {0};
    esp_err_t err = transceive(addr, &cmd, 1, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    uint8_t payload[2] = {raw[0], raw[1]};
    if (pec_read(addr, cmd, payload, 2) != raw[2]) {
        ESP_LOGW(TAG, "word 0x%02x bytes %02x %02x pec %02x", cmd, raw[0], raw[1], raw[2]);
        return ESP_ERR_INVALID_CRC;
    }
    *value = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    return ESP_OK;
}

esp_err_t smbus_read_word(uint8_t addr, uint8_t cmd, uint16_t *value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    bool try_pec = s_pec != 0;
    bool try_plain = s_pec != 1;
    if (try_pec) {
        esp_err_t err = read_word_pec(addr, cmd, value);
        if (err == ESP_OK && s_pec < 0) {
            uint16_t confirm = 0;
            esp_err_t confirm_err = read_word_pec(addr, cmd, &confirm);
            if (confirm_err == ESP_OK) {
                s_pec = 1;
                *value = confirm;
                return ESP_OK;
            }
            if (confirm_err != ESP_ERR_INVALID_CRC) {
                s_pec = 1;
                return ESP_OK;
            }
            err = confirm_err;
        } else if (err == ESP_OK) {
            return ESP_OK;
        }
        if (!try_plain) {
            return err;
        }
    }

    uint8_t raw[2] = {0};
    esp_err_t err = transceive(addr, &cmd, 1, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    if (s_pec < 0) {
        s_pec = 0;
    }
    *value = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    return ESP_OK;
}

esp_err_t smbus_read_block(uint8_t addr, uint8_t cmd, uint8_t *data, size_t *len, size_t max_len) {
    if (data == NULL || len == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    bool try_pec = s_pec != 0;
    bool try_plain = s_pec != 1;

    if (try_pec) {
        uint8_t raw[34] = {0};
        esp_err_t err = transceive(addr, &cmd, 1, raw, sizeof(raw));
        if (err == ESP_OK) {
            uint8_t count = raw[0];
            if (count <= 32 && pec_read(addr, cmd, raw, (size_t)count + 1) == raw[1 + count]) {
                s_pec = 1;
                size_t copy = count;
                if (copy > max_len) {
                    copy = max_len;
                }
                memcpy(data, raw + 1, copy);
                *len = copy;
                return count <= max_len ? ESP_OK : ESP_ERR_INVALID_SIZE;
            }
            // Dump the first 16 raw bytes so we can see what the gauge sent even if PEC failed.
            char hex[64];
            size_t pos = 0;
            for (int i = 0; i < 16 && pos + 4 < sizeof(hex); i++) {
                pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, " %02x", raw[i]);
            }
            ESP_LOGW(TAG, "block 0x%02x count %u raw:%s", cmd, count, hex);
            err = ESP_ERR_INVALID_CRC;
        }
        if (!try_plain) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint8_t raw[33] = {0};
    esp_err_t err = transceive(addr, &cmd, 1, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    uint8_t count = raw[0];
    if (count > 32) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (s_pec < 0) {
        s_pec = 0;
    }
    size_t copy = count;
    if (copy > max_len) {
        copy = max_len;
    }
    memcpy(data, raw + 1, copy);
    *len = copy;
    return count <= max_len ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
