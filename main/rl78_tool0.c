#include "rl78_tool0.h"
#include "config.h"
#include "smbus.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rl78";

enum {
    RL78_SOH = 0x01,
    RL78_STX = 0x02,
    RL78_ETX = 0x03,
    RL78_ETB = 0x17,

    RL78_CMD_RESET       = 0x00,
    RL78_CMD_BAUD_SET    = 0x9A,
    RL78_CMD_CHECKSUM    = 0xB0,
    RL78_CMD_SEC_GET     = 0xA1,
    RL78_CMD_SILICON_SIG = 0xC0,

    RL78_ST_ACK          = 0x06,
    RL78_MODE_1WIRE      = 0x3A,

    // CREG2 is 3.3 V. Protocol A wants tenths of a volt, as a hex byte: 33 -> 0x21.
    RL78_VDD_3V3         = 0x21,
};

static uint8_t rl78_checksum(const uint8_t *bytes, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum - bytes[i]);
    }
    return sum;
}

static void log_bytes(const char *label, const uint8_t *data, size_t len) {
    char buf[180];
    size_t pos = 0;
    buf[0] = '\0';
    size_t n = len > 40 ? 40 : len;
    for (size_t i = 0; i < n; i++) {
        int wrote = snprintf(buf + pos, sizeof(buf) - pos, "%02x%s", data[i], (i + 1 == n) ? "" : " ");
        if (wrote < 0 || (size_t)wrote >= sizeof(buf) - pos) {
            break;
        }
        pos += (size_t)wrote;
    }
    ESP_LOGI(TAG, "%s (%u): %s", label, (unsigned)len, buf);
}

static const char *status_name(uint8_t st) {
    switch (st) {
    case 0x04: return "command number error";
    case 0x05: return "parameter error";
    case 0x06: return "ACK";
    case 0x07: return "checksum error";
    case 0x0F: return "verify error";
    case 0x10: return "protect error";
    case 0x15: return "NACK";
    case 0x1A: return "erase error";
    case 0x1B: return "blank error";
    case 0x1C: return "write error";
    default: return "unknown";
    }
}

typedef struct {
    const uint8_t *data;
    int len;
    int next;
} frame_t;

static bool next_frame(const uint8_t *buf, int n, int start, frame_t *out) {
    for (int i = start; i + 4 < n; i++) {
        if (buf[i] != RL78_STX) {
            continue;
        }
        int len = buf[i + 1];
        if (len <= 0 || len > 48) {
            continue;
        }
        int end = i + 2 + len + 2;
        if (end > n) {
            continue;
        }
        uint8_t term = buf[i + 2 + len + 1];
        if (term != RL78_ETX && term != RL78_ETB) {
            continue;
        }
        if (rl78_checksum(buf + i + 1, (size_t)(1 + len)) != buf[i + 2 + len]) {
            continue;
        }
        out->data = buf + i + 2;
        out->len = len;
        out->next = end;
        return true;
    }
    return false;
}

static int count_frames(const uint8_t *buf, int n) {
    int count = 0;
    int pos = 0;
    frame_t fr;
    while (next_frame(buf, n, pos, &fr)) {
        count++;
        pos = fr.next;
    }
    return count;
}

static uint32_t s_bit_cycles;

static uint32_t ccount(void) {
    uint32_t c;
    __asm__ __volatile__("rsr.ccount %0" : "=a"(c));
    return c;
}

static void wait_until(uint32_t deadline) {
    while ((int32_t)(deadline - ccount()) > 0) {
    }
}

static int s_tx_pin = PIN_RL78_RXD0;
static int s_rx_pin = PIN_RL78_TXD0;

static inline void pin_low(int pin) {
    GPIO.out_w1tc = (1u << pin);
}

static inline void pin_high(int pin) {
    GPIO.out_w1ts = (1u << pin);
}

static inline int pin_read(int pin) {
    return (int)((GPIO.in >> pin) & 1u);
}

static void tool0_od_high(void) {
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(PIN_RL78_TOOL0, GPIO_PULLUP_ONLY);
    pin_high(PIN_RL78_TOOL0);
}

// 115200 8N2. Edges are register writes on a cycle deadline.
static void bb_tx(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        portDISABLE_INTERRUPTS();
        uint32_t t = ccount();
        pin_low(s_tx_pin);
        t += s_bit_cycles;
        wait_until(t);
        for (int bit = 0; bit < 8; bit++) {
            if ((b >> bit) & 1) {
                pin_high(s_tx_pin);
            } else {
                pin_low(s_tx_pin);
            }
            t += s_bit_cycles;
            wait_until(t);
        }
        pin_high(s_tx_pin);
        t += s_bit_cycles * 2;
        wait_until(t);
        portENABLE_INTERRUPTS();
    }
}

// Interrupts stay off. Returns the number of bytes captured inside window_us.
static int bb_rx_burst(uint8_t *buf, size_t max, int window_us) {
    uint32_t window = (uint32_t)(((uint64_t)s_bit_cycles * 115200ull * (uint32_t)window_us) / 1000000ull);
    uint32_t start = ccount();
    int n = 0;
    portDISABLE_INTERRUPTS();
    while (n < (int)max && (uint32_t)(ccount() - start) < window) {
        if (pin_read(s_rx_pin)) {
            continue;
        }
        uint32_t t = ccount() + s_bit_cycles + s_bit_cycles / 2;
        uint8_t v = 0;
        for (int bit = 0; bit < 8; bit++) {
            wait_until(t);
            if (pin_read(s_rx_pin)) {
                v |= (uint8_t)(1u << bit);
            }
            t += s_bit_cycles;
        }
        wait_until(t);
        buf[n++] = v;
    }
    portENABLE_INTERRUPTS();
    return n;
}

static int read_until(uint8_t *buf, size_t max, int timeout_ms, int need_frames) {
    int window_us = timeout_ms * 1000;
    if (window_us > 80000) {
        window_us = 80000;
    }
    int n = bb_rx_burst(buf, max, window_us);
    if (count_frames(buf, n) >= need_frames) {
        return n;
    }
    return n;
}

static size_t build_cmd(uint8_t *frame, uint8_t cmd, const uint8_t *info, size_t info_len) {
    frame[0] = RL78_SOH;
    frame[1] = (uint8_t)(1 + info_len);
    frame[2] = cmd;
    if (info_len > 0 && info != NULL) {
        memcpy(frame + 3, info, info_len);
    }
    frame[3 + info_len] = rl78_checksum(frame + 1, 2 + info_len);
    frame[4 + info_len] = RL78_ETX;
    return 5 + info_len;
}

// Protocol A single-wire entry. TOOL0 must be low for >= 1 ms after RESET rises,
// then 0x3A and the Baud Rate Set frame must finish within 100 ms of that edge.
static void enter_programming(int hold_us) {
    gpio_reset_pin(PIN_RL78_TOOL0);
    gpio_reset_pin(PIN_RL78_RESET);
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_RL78_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_TOOL0, 0);
    gpio_set_level(PIN_RL78_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RL78_RESET, 1);
    esp_rom_delay_us(hold_us);
    tool0_od_high();
    esp_rom_delay_us(200);
}

static esp_err_t rl78_command(uint8_t cmd, const uint8_t *info, size_t info_len,
                              uint8_t *status, size_t *status_len,
                              uint8_t *data, size_t *data_len,
                              int timeout_ms) {
    uint8_t frame[16];
    size_t flen = build_cmd(frame, cmd, info, info_len);
    log_bytes("tx", frame, flen);
    bb_tx(frame, flen);

    uint8_t rx[96] = {0};
    int need = (data != NULL) ? 2 : 1;
    int n = read_until(rx, sizeof(rx), timeout_ms, need);
    if (n <= 0) {
        ESP_LOGW(TAG, "cmd 0x%02x: no reply", cmd);
        return ESP_ERR_TIMEOUT;
    }
    log_bytes("rx", rx, (size_t)n);

    frame_t st;
    if (!next_frame(rx, n, 0, &st) || st.len < 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (status != NULL && status_len != NULL && *status_len > 0) {
        size_t copy = (size_t)st.len;
        if (copy > *status_len) {
            copy = *status_len;
        }
        memcpy(status, st.data, copy);
        *status_len = copy;
    }
    ESP_LOGI(TAG, "cmd 0x%02x status 0x%02x (%s)", cmd, st.data[0], status_name(st.data[0]));
    if (st.data[0] != RL78_ST_ACK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (data == NULL || data_len == NULL) {
        return ESP_OK;
    }
    frame_t payload;
    if (!next_frame(rx, n, st.next, &payload)) {
        ESP_LOGW(TAG, "cmd 0x%02x: ACK without a data frame", cmd);
        return ESP_ERR_INVALID_RESPONSE;
    }
    size_t copy = (size_t)payload.len;
    if (copy > *data_len) {
        copy = *data_len;
    }
    memcpy(data, payload.data, copy);
    *data_len = copy;
    return ESP_OK;
}

static void log_signature(const uint8_t *sig, size_t len) {
    if (len < 22) {
        log_bytes("silicon", sig, len);
        return;
    }
    char name[11];
    memcpy(name, sig + 3, 10);
    name[10] = '\0';
    for (int i = 0; i < 10; i++) {
        if (name[i] < 0x20 || name[i] > 0x7e) {
            name[i] = ' ';
        }
    }
    uint32_t code_end = (uint32_t)sig[13] | ((uint32_t)sig[14] << 8) | ((uint32_t)sig[15] << 16);
    uint32_t data_end = (uint32_t)sig[16] | ((uint32_t)sig[17] << 8) | ((uint32_t)sig[18] << 16);
    ESP_LOGI(TAG, "silicon dev %02x %02x %02x name '%.10s' code_end=0x%06x data_end=0x%06x ver %u.%u.%u",
             sig[0], sig[1], sig[2], name, (unsigned)code_end, (unsigned)data_end,
             sig[19], sig[20], sig[21]);
}

static void log_security(const uint8_t *sec, size_t len) {
    if (len < 8) {
        log_bytes("security", sec, len);
        return;
    }
    uint8_t flg = sec[0];
    ESP_LOGI(TAG, "security FLG=0x%02x program=%s erase=%s boot_rewrite=%s BOT=%u FSW=%02x%02x-%02x%02x",
             flg,
             (flg & 0x10) ? "enabled" : "disabled",
             (flg & 0x04) ? "enabled" : "disabled",
             (flg & 0x02) ? "enabled" : "disabled",
             sec[1], sec[3], sec[2], sec[5], sec[4]);
}

static esp_err_t read_checksum(uint32_t start, uint32_t end) {
    uint8_t info[6] = {
        (uint8_t)start, (uint8_t)(start >> 8), (uint8_t)(start >> 16),
        (uint8_t)end, (uint8_t)(end >> 8), (uint8_t)(end >> 16),
    };
    uint8_t st[4] = {0};
    size_t st_len = sizeof(st);
    uint8_t data[4] = {0};
    size_t data_len = sizeof(data);
    esp_err_t err = rl78_command(RL78_CMD_CHECKSUM, info, sizeof(info), st, &st_len, data, &data_len, 1000);
    if (err == ESP_OK && data_len >= 2) {
        unsigned sum = (unsigned)data[0] | ((unsigned)data[1] << 8);
        ESP_LOGI(TAG, "checksum 0x%06x..0x%06x = 0x%04x", (unsigned)start, (unsigned)end, sum);
    }
    return err;
}

static void twowire_pins_idle(void) {
    gpio_reset_pin(PIN_RL78_RXD0);
    gpio_set_direction(PIN_RL78_RXD0, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_RXD0, 1);
    gpio_reset_pin(PIN_RL78_TXD0);
    gpio_set_direction(PIN_RL78_TXD0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_RL78_TXD0, GPIO_PULLUP_ONLY);
}

static esp_err_t program_session(int hold_us) {
    ESP_LOGI(TAG, "two-wire entry, hold %d us. GPIO%d -> pin 5 RxD0, GPIO%d <- pin 6 TxD0",
             hold_us, PIN_RL78_RXD0, PIN_RL78_TXD0);
    s_tx_pin = PIN_RL78_RXD0;
    s_rx_pin = PIN_RL78_TXD0;
    twowire_pins_idle();
    enter_programming(hold_us);

    uint8_t mode = 0x00;
    bb_tx(&mode, 1);
    ESP_LOGI(TAG, "sent mode byte 0x00 on RxD0");
    esp_rom_delay_us(1000);

    uint8_t baud_info[2] = {0x00, RL78_VDD_3V3};
    uint8_t st[8] = {0};
    size_t st_len = sizeof(st);
    esp_err_t err = rl78_command(RL78_CMD_BAUD_SET, baud_info, sizeof(baud_info), st, &st_len, NULL, NULL, 150);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "pin 6 sent no frame. GPIO%d goes to pin 5 RxD0, GPIO%d comes from pin 6 TxD0, 3.3k on pin 6.",
                 PIN_RL78_RXD0, PIN_RL78_TXD0);
        return err;
    }
    if (st_len >= 3) {
        ESP_LOGI(TAG, "baud sync freq=%u MHz mode=%s", st[1], st[2] == 0x01 ? "wide-voltage" : "full-speed");
    }
    esp_rom_delay_us(2000);

    st_len = sizeof(st);
    err = rl78_command(RL78_CMD_RESET, NULL, 0, st, &st_len, NULL, NULL, 150);
    if (err != ESP_OK) {
        return err;
    }
    esp_rom_delay_us(2000);

    uint8_t sig[24] = {0};
    size_t sig_len = sizeof(sig);
    st_len = sizeof(st);
    err = rl78_command(RL78_CMD_SILICON_SIG, NULL, 0, st, &st_len, sig, &sig_len, 300);
    if (err != ESP_OK) {
        return err;
    }
    log_signature(sig, sig_len);
    esp_rom_delay_us(2000);

    uint8_t sec[12] = {0};
    size_t sec_len = sizeof(sec);
    st_len = sizeof(st);
    err = rl78_command(RL78_CMD_SEC_GET, NULL, 0, st, &st_len, sec, &sec_len, 300);
    if (err == ESP_OK) {
        log_security(sec, sec_len);
    }
    esp_rom_delay_us(2000);

    // Read-only. Block size is 1 KB. 0x0F1400 is the permanent-failure page.
    read_checksum(0x0F1000u, 0x0F13FFu);
    esp_rom_delay_us(2000);
    read_checksum(0x0F1400u, 0x0F17FFu);
    return ESP_OK;
}

void rl78_reset_to_normal(void) {
    gpio_reset_pin(PIN_RL78_TOOL0);
    gpio_reset_pin(PIN_RL78_RESET);
    // Push-pull. RESET VIH is 0.8 * CREG2 (about 2.6 V). Open-drain release
    // sat near 0.8 V on this board and left the chip in reset.
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_TOOL0, 1);
    gpio_set_direction(PIN_RL78_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_RL78_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RL78_RESET, 1);
    ESP_LOGI(TAG, "normal mode: TOOL0 driven high, RESET driven high");
    vTaskDelay(pdMS_TO_TICKS(800));
    // TOOL0 has the 3.3k pull-up, so it can be released. RESET is a direct
    // wire with no pull-up; leave GPIO16 driving it high or the pin sags.
    gpio_set_direction(PIN_RL78_TOOL0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_RL78_TOOL0, GPIO_PULLUP_ONLY);
    gpio_set_level(PIN_RL78_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    smbus_reset_pec();
}

esp_err_t rl78_tool0_try_unlock(void) {
    ESP_LOGI(TAG, "GPIO%d is TOOL0 pin 7, GPIO%d is RESET pin 8",
             PIN_RL78_TOOL0, PIN_RL78_RESET);
    ESP_LOGI(TAG, "GPIO%d drives pin 5 RxD0, GPIO%d reads pin 6 TxD0",
             PIN_RL78_RXD0, PIN_RL78_TXD0);
    uint32_t ticks_us = esp_rom_get_cpu_ticks_per_us();
    uint32_t hz = ticks_us * 1000000u;
    s_bit_cycles = hz / 115200u;
    if (s_bit_cycles < 100) {
        s_bit_cycles = 1388;
    }
    ESP_LOGI(TAG, "CPU %u Hz, bit %u cycles", hz, s_bit_cycles);
    tool0_od_high();
    esp_rom_delay_us(100);
    int idle = gpio_get_level(PIN_RL78_TOOL0);
    gpio_set_level(PIN_RL78_TOOL0, 0);
    esp_rom_delay_us(100);
    int driven = gpio_get_level(PIN_RL78_TOOL0);
    gpio_set_level(PIN_RL78_TOOL0, 1);
    ESP_LOGI(TAG, "TOOL0 idle=%d driven_low=%d (expect 1 then 0)", idle, driven);

    static const int holds_us[] = {3000, 8000};
    esp_err_t err = ESP_FAIL;
    for (size_t i = 0; i < sizeof(holds_us) / sizeof(holds_us[0]); i++) {
        err = program_session(holds_us[i]);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "programming link is up. No erase was sent.");
            break;
        }
        ESP_LOGW(TAG, "programming attempt %u failed: %s", (unsigned)(i + 1), esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "leaving programming mode");
    rl78_reset_to_normal();
    return err;
}

esp_err_t rl78_smbus_boot_service(void) {
    // 0x09 -> 01 ff only means the application is not running. It is not a
    // memory-read protocol. The 0xFC write clock-stretched past the ESP32
    // I2C limit and the driver's bus clear dropped GPIO21/22.
    ESP_LOGI(TAG, "SMBus boot poke skipped. Programming is single-wire UART on TOOL0.");
    return ESP_OK;
}
