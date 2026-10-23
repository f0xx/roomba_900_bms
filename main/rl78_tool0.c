#include "rl78_tool0.h"
#include "config.h"
#include "smbus.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rl78";

enum {
    RL78_SOH = 0x01,
    RL78_STX = 0x02,
    RL78_ETX = 0x03,
    RL78_ETB = 0x17,

    RL78_CMD_RESET       = 0x00,
    RL78_CMD_BLOCK_ERASE = 0x22,
    RL78_CMD_BLANK_CHECK = 0x32,
    RL78_CMD_PROGRAM     = 0x40,
    RL78_CMD_BAUD_SET    = 0x9A,
    RL78_CMD_SEC_GET     = 0xA1,
    RL78_CMD_SILICON_SIG = 0xC0,

    RL78_ST_ACK          = 0x06,
    RL78_MODE_1WIRE      = 0x3A,
};

static uint8_t rl78_checksum(const uint8_t *bytes, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum - bytes[i]);
    }
    return sum;
}

static void log_bytes(const char *label, const uint8_t *data, size_t len) {
    char buf[120];
    size_t pos = 0;
    buf[0] = '\0';
    size_t n = len > 32 ? 32 : len;
    for (size_t i = 0; i < n; i++) {
        int wrote = snprintf(buf + pos, sizeof(buf) - pos, "%02x%s", data[i], (i + 1 == n) ? "" : " ");
        if (wrote < 0 || (size_t)wrote >= sizeof(buf) - pos) {
            break;
        }
        pos += (size_t)wrote;
    }
    ESP_LOGI(TAG, "%s: %s", label, buf);
}

void rl78_reset_to_normal(void) {
    gpio_reset_pin(PIN_RL78_TOOL0);
    gpio_reset_pin(PIN_RL78_RESET);
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_TOOL0, 1);
    gpio_set_direction(PIN_RL78_RESET, GPIO_MODE_OUTPUT_OD);
    gpio_set_pull_mode(PIN_RL78_RESET, GPIO_PULLUP_ONLY);
    gpio_set_level(PIN_RL78_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(PIN_RL78_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_RL78_TOOL0, GPIO_PULLUP_ONLY);
    gpio_set_direction(PIN_RL78_RESET, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_RL78_RESET, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(250));
    smbus_reset_pec();
}

static void rl78_enter_smbus_boot(void) {
    gpio_reset_pin(PIN_RL78_TOOL0);
    gpio_reset_pin(PIN_RL78_RESET);
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_TOOL0, 0);
    gpio_set_direction(PIN_RL78_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(15));
    gpio_set_level(PIN_RL78_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RL78_TOOL0, 1);
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_RL78_TOOL0, GPIO_PULLUP_ONLY);
    gpio_set_direction(PIN_RL78_RESET, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_RL78_RESET, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(100));
    smbus_reset_pec();
}

static esp_err_t boot_read_ram(uint32_t addr, uint16_t size, uint8_t *out) {
    uint8_t wr[6] = {
        0xFC,
        (uint8_t)(addr & 0xffu),
        (uint8_t)((addr >> 8) & 0xffu),
        (uint8_t)((addr >> 16) & 0xffu),
        (uint8_t)(size & 0xffu),
        (uint8_t)((size >> 8) & 0xffu),
    };
    return smbus_raw_writeread(BMS_ADDR, wr, sizeof(wr), out, size);
}

static esp_err_t boot_erase_block(uint32_t addr) {
    uint8_t wr[5] = {
        0x20,
        0x03,
        (uint8_t)(addr & 0xffu),
        (uint8_t)((addr >> 8) & 0xffu),
        (uint8_t)((addr >> 16) & 0xffu),
    };
    esp_err_t err = smbus_raw_write(BMS_ADDR, wr, sizeof(wr));
    vTaskDelay(pdMS_TO_TICKS(500));
    uint8_t st[2] = {0};
    uint8_t cmd50 = 0x50;
    smbus_raw_writeread(BMS_ADDR, &cmd50, 1, st, 2);
    ESP_LOGI(TAG, "boot erase 0x%06x: write=%s status=%02x %02x",
             (unsigned)addr, esp_err_to_name(err), st[0], st[1]);
    return err;
}

esp_err_t rl78_smbus_boot_service(void) {
    ESP_LOGI(TAG, "=== Entering Renesas SMBus Boot Mode (TOOL0=0, RESET pulse) ===");
    rl78_enter_smbus_boot();

    uint8_t probe[2] = {0};
    uint8_t cmd09 = 0x09;
    esp_err_t err = smbus_raw_writeread(BMS_ADDR, &cmd09, 1, probe, 2);
    ESP_LOGI(TAG, "boot probe 0x09: %s (%02x %02x)", esp_err_to_name(err), probe[0], probe[1]);

    if (err == ESP_OK && probe[0] == 0x01 && probe[1] == 0xFF) {
        ESP_LOGI(TAG, "Renesas SMBus Boot Mode active (0xFF01)!");
        uint8_t buf[32] = {0};
        static const uint32_t addrs[] = {0x000000u, 0x004000u, 0x0F1400u, 0x0F1800u, 0x0FF800u};
        for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
            memset(buf, 0, sizeof(buf));
            esp_err_t er = boot_read_ram(addrs[i], 32, buf);
            char lbl[32];
            snprintf(lbl, sizeof(lbl), "boot 0xFC @ %06x (%s)", (unsigned)addrs[i], esp_err_to_name(er));
            log_bytes(lbl, buf, 32);
        }

        // Try erasing the RMA record block at 0x0F1400 in SMBus Boot Mode:
        boot_erase_block(0x0F1400u);

        memset(buf, 0, sizeof(buf));
        if (boot_read_ram(0x0F1400u, 32, buf) == ESP_OK) {
            log_bytes("boot 0xFC @ 0f1400 after", buf, 32);
        }

        // Send 0xD001 to 0x00 (Execute Flash) and also hardware-reset with TOOL0=1:
        uint8_t exec_cmd[3] = {0x00, 0x01, 0xD0};
        smbus_raw_write(BMS_ADDR, exec_cmd, sizeof(exec_cmd));
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "=== Resetting RAJ240045 back to Normal SMBus Mode (TOOL0=1, RESET pulse) ===");
    rl78_reset_to_normal();
    return ESP_OK;
}

static void tool0_tx_bytes(const uint8_t *buf, size_t len) {
    uart_flush_input(UART_NUM_1);
    uart_write_bytes(UART_NUM_1, buf, len);
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100));
    uint8_t echo[32];
    size_t rem = len;
    while (rem > 0) {
        size_t chunk = rem > sizeof(echo) ? sizeof(echo) : rem;
        int got = uart_read_bytes(UART_NUM_1, echo, chunk, pdMS_TO_TICKS(30));
        if (got <= 0) {
            break;
        }
        rem -= (size_t)got;
    }
}

static esp_err_t rl78_send_cmd(uint8_t cmd, const uint8_t *info, size_t info_len) {
    if (info_len > 24) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t frame[32];
    uint8_t len_byte = (uint8_t)(1 + info_len);
    frame[0] = RL78_SOH;
    frame[1] = len_byte;
    frame[2] = cmd;
    if (info_len > 0 && info != NULL) {
        memcpy(frame + 3, info, info_len);
    }
    frame[3 + info_len] = rl78_checksum(frame + 1, 1 + len_byte);
    frame[4 + info_len] = RL78_ETX;
    tool0_tx_bytes(frame, 5 + info_len);
    return ESP_OK;
}

static esp_err_t rl78_recv_data_frame(uint8_t *out, size_t *out_len, size_t max_len, int timeout_ms) {
    uint8_t hdr[2] = {0};
    int got = uart_read_bytes(UART_NUM_1, hdr, 2, pdMS_TO_TICKS(timeout_ms));
    if (got < 2 || hdr[0] != RL78_STX) {
        return ESP_ERR_TIMEOUT;
    }
    size_t len = (hdr[1] == 0) ? 256 : hdr[1];
    if (len > max_len || len > 64) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t tail[66] = {0};
    got = uart_read_bytes(UART_NUM_1, tail, len + 2, pdMS_TO_TICKS(timeout_ms));
    if (got < (int)(len + 2)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t sum = (uint8_t)(0u - hdr[1]);
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum - tail[i]);
    }
    if (sum != tail[len] || (tail[len + 1] != RL78_ETX && tail[len + 1] != RL78_ETB)) {
        return ESP_ERR_INVALID_CRC;
    }
    memcpy(out, tail, len);
    *out_len = len;
    return ESP_OK;
}

esp_err_t rl78_tool0_try_unlock(void) {
    ESP_LOGI(TAG, "RL78 TOOL0=GPIO%d (pin 7), RESET=GPIO%d (pin 8)", PIN_RL78_TOOL0, PIN_RL78_RESET);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_RL78_TOOL0) | (1ULL << PIN_RL78_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    gpio_set_level(PIN_RL78_TOOL0, 0);
    gpio_set_level(PIN_RL78_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RL78_RESET, 1);
    esp_rom_delay_us(2000);
    gpio_set_level(PIN_RL78_TOOL0, 1);

    uart_config_t ucfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (!uart_is_driver_installed(UART_NUM_1)) {
        uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);
    }
    uart_param_config(UART_NUM_1, &ucfg);
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(PIN_RL78_TOOL0, GPIO_PULLUP_ONLY);
    uart_set_pin(UART_NUM_1, PIN_RL78_TOOL0, PIN_RL78_TOOL0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(PIN_RL78_TOOL0, GPIO_PULLUP_ONLY);

    esp_rom_delay_us(1000);

    uint8_t mode_byte = RL78_MODE_1WIRE;
    tool0_tx_bytes(&mode_byte, 1);
    esp_rom_delay_us(2000);

    uint8_t baud_info[2] = {0x00, 0x21};
    rl78_send_cmd(RL78_CMD_BAUD_SET, baud_info, sizeof(baud_info));

    uint8_t rx[32] = {0};
    size_t rx_len = 0;
    esp_err_t err = rl78_recv_data_frame(rx, &rx_len, sizeof(rx), 60);
    uart_driver_delete(UART_NUM_1);
    if (err != ESP_OK || rx_len < 3) {
        ESP_LOGI(TAG, "TOOL0 UART did not answer (P137=0 selects SMBus Boot Mode instead of UART)");
        rl78_reset_to_normal();
        return err;
    }

    ESP_LOGI(TAG, "RL78 UART status=0x%02x freq=%u MHz mode=0x%02x", rx[0], rx[1], rx[2]);
    rl78_reset_to_normal();
    return ESP_OK;
}
