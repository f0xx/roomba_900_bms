#include "bms.h"

#include "config.h"
#include "sha1.h"
#include "smbus.h"
#include "tft.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bms";

// OperationStatus bits shared by bq30z554 (SLUUA79) and bq40z50-R2 (SLUUBK0B).
// SEC1:SEC0 in bits 9:8 is 11 sealed, 10 unsealed, 01 full access.
enum {
    BIT_DSG = 1u << 1,
    BIT_CHG = 1u << 2,
    BIT_PCHG = 1u << 3,
    BIT_FUSE = 1u << 5,
    BIT_SEC_SHIFT = 8,
    BIT_SS = 1u << 11,
    BIT_PF = 1u << 12,
    BIT_XDSG = 1u << 13,
    BIT_XCHG = 1u << 14,
    BIT_PF_FUSE = 1u << 19,
    BIT_PF_2LVL = 1u << 22,
    BIT_FET_EN = 1u << 4,
};

static int s_family;
static uint16_t s_type;
static uint32_t s_op;
static unsigned s_sec;
static uint16_t s_cell[4];
static int s_pack_mv = -1;
static int s_bat_mv = -1;
static bms_display_t s_disp;

static void log_hex(const char *label, const uint8_t *data, size_t len) {
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

static const char *sec_name(unsigned sec) {
    switch (sec) {
    case 1:
        return "full access";
    case 2:
        return "unsealed";
    case 3:
        return "sealed";
    default:
        return "reserved";
    }
}

static const char *type_name(uint16_t type) {
    switch (type) {
    case 0x0700:
        return "bq20z70";
    case 0x0750:
        return "bq20z75";
    case 0x0500:
        return "bq30z50";
    case 0x0550:
        return "bq30z55";
    case 0x0554:
        return "bq30z554";
    case 0x4307:
        return "bq40z307";
    case 0x4500:
        return "bq40z50";
    case 0x4600:
        return "bq40z60";
    case 0x4800:
        return "bq40z80";
    default:
        if ((type >> 8) == 0x07) {
            return "bq20z70/bq20z75";
        }
        return "unrecognized";
    }
}

static int family_of(uint16_t type) {
    if (type == 0x4307) {
        return 40;
    }
    uint8_t hi = (uint8_t)(type >> 8);
    if (hi == 0x07) {
        return 20;
    }
    if (type == 0x0500 || type == 0x0550 || type == 0x0554) {
        return 30;
    }
    if (hi == 0x45 || hi == 0x46 || hi == 0x48) {
        return 40;
    }
    return 0;
}

static uint32_t load_le(const uint8_t *data, size_t len) {
    uint32_t value = 0;
    for (size_t i = 0; i < len && i < 4; i++) {
        value |= (uint32_t)data[i] << (8 * i);
    }
    return value;
}

static esp_err_t read_mac(uint16_t sub, uint8_t *data, size_t *len, size_t max_len) {
    uint8_t req[2] = {(uint8_t)sub, (uint8_t)(sub >> 8)};
    uint8_t raw[32];
    size_t n = 0;

    if (smbus_write_block(BMS_ADDR, 0x44, req, sizeof(req)) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (smbus_read_block(BMS_ADDR, 0x44, raw, &n, sizeof(raw)) == ESP_OK &&
            n >= 2 && raw[0] == req[0] && raw[1] == req[1] && (n - 2) <= max_len && n > 2) {
            memcpy(data, raw + 2, n - 2);
            *len = n - 2;
            return ESP_OK;
        }
    }

    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, sub);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    n = 0;
    // 0x23 is accepted only when it echoes the subcommand. On bq20z70/bq20z75 it is a
    // static ASCII field (often "00010302") and is not the ManufacturerAccess result.
    if (smbus_read_block(BMS_ADDR, 0x23, raw, &n, sizeof(raw)) == ESP_OK &&
        n >= 4 && raw[0] == req[0] && raw[1] == req[1] && (n - 2) <= max_len) {
        memcpy(data, raw + 2, n - 2);
        *len = n - 2;
        return ESP_OK;
    }

    uint16_t word = 0;
    err = smbus_read_word(BMS_ADDR, 0x00, &word);
    if (err == ESP_OK && word != sub && max_len >= 2) {
        data[0] = (uint8_t)word;
        data[1] = (uint8_t)(word >> 8);
        *len = 2;
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t read_status_sub(uint16_t sub, uint32_t *value) {
    uint8_t raw[8];
    size_t n = 0;
    esp_err_t err = read_mac(sub, raw, &n, sizeof(raw));
    if (err != ESP_OK || n < 2) {
        return ESP_FAIL;
    }
    uint32_t word = load_le(raw, n);
    // A stuck ManufacturerAccess register repeats the device-type word for every subcommand.
    if (s_type != 0 && (uint16_t)word == s_type && sub != s_type) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *value = word;
    return ESP_OK;
}

static void log_direct_word(uint8_t cmd, const char *name) {
    uint16_t word = 0;
    if (smbus_read_word(BMS_ADDR, cmd, &word) == ESP_OK) {
        ESP_LOGI(TAG, "%s command 0x%02x = 0x%04x", name, cmd, word);
    } else {
        ESP_LOGW(TAG, "%s command 0x%02x unread", name, cmd);
    }
}

static void log_block_raw(uint8_t cmd, const char *name) {
    uint8_t raw[33];
    size_t n = 0;
    esp_err_t err = smbus_read_block(BMS_ADDR, cmd, raw, &n, sizeof(raw));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s command 0x%02x unread (%s)", name, cmd, esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "%s command 0x%02x length %u", name, cmd, (unsigned)n);
    log_hex(name, raw, n);
}

static bool i2c_stuck(esp_err_t err) {
    // IDF 6 reports a held SCL as INVALID_RESPONSE ("bus is still busy"), not TIMEOUT.
    return err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE || err == ESP_ERR_INVALID_RESPONSE;
}

// Try block read first (Panasonic extended commands 0x51/0x53/0x54/0x5a are blocks),
// then fall back to a word read. Only reset the I2C peripheral if the bus actually wedged,
// because ESP-IDF 6 logs false "GPIO 21/22 is not usable" warnings on every bus reset.
static void probe_extended(uint8_t cmd) {
    uint8_t blk[33];
    size_t n = 0;
    esp_err_t eb_blk = smbus_read_block(BMS_ADDR, cmd, blk, &n, sizeof(blk));
    if (eb_blk == ESP_OK) {
        char label[24];
        snprintf(label, sizeof(label), "cmd 0x%02x block", cmd);
        log_hex(label, blk, n);
        if (cmd == 0x51 && n >= 10) {
            s_disp.pack_status_51 = (uint16_t)((blk[8] << 8) | blk[9]);
            ESP_LOGI(TAG, "Panasonic 0x51: cells %u/%u/%u/%u mV, PACK_STATUS=0x%04x",
                     (unsigned)((blk[0] << 8) | blk[1]),
                     (unsigned)((blk[2] << 8) | blk[3]),
                     (unsigned)((blk[4] << 8) | blk[5]),
                     (unsigned)((blk[6] << 8) | blk[7]),
                     (unsigned)s_disp.pack_status_51);
        } else if (cmd == 0x53 && n >= 7) {
            memcpy(s_disp.blk53, blk, 7);
        } else if (cmd == 0x54 && n >= 13) {
            memcpy(s_disp.blk54, blk, 13);
        } else if (cmd == 0x5a && n >= 4) {
            memcpy(s_disp.blk5a, blk, 4);
        }
        return;
    }
    if (i2c_stuck(eb_blk)) {
        smbus_bus_recover();
        vTaskDelay(pdMS_TO_TICKS(20));
        ESP_LOGW(TAG, "cmd 0x%02x %s", cmd, esp_err_to_name(eb_blk));
        return;
    }
    uint16_t a = 0;
    esp_err_t ea = smbus_read_word(BMS_ADDR, cmd, &a);
    if (ea == ESP_OK) {
        ESP_LOGI(TAG, "cmd 0x%02x = 0x%04x", cmd, a);
        return;
    }
    if (i2c_stuck(ea)) {
        smbus_bus_recover();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGW(TAG, "cmd 0x%02x %s", cmd, esp_err_to_name(ea));
}

// Non-destructive probe of Panasonic / bq80xx / M37512 extended vendor commands.
// Step 1: read-only check of 0x40..0x43, 0x70..0x74.
// Step 2: send the documented bq8030/bq8050 challenge trigger (write 0x0214 to 0x71, read 0x73/0x74)
//         WITHOUT sending the BootROM jump (0x0517 to 0x70), so flash cannot be erased.
static esp_err_t read_mem_block(uint32_t addr24, uint8_t out[32]) {
    uint8_t req[3] = {
        (uint8_t)(addr24 & 0xffu),
        (uint8_t)((addr24 >> 8) & 0xffu),
        (uint8_t)((addr24 >> 16) & 0xffu),
    };
    esp_err_t e = smbus_write_block(BMS_ADDR, 0x40, req, 3);
    if (e != ESP_OK) {
        if (i2c_stuck(e)) smbus_bus_recover();
        return e;
    }
    size_t n = 0;
    uint8_t tmp[33] = {0};
    e = smbus_read_block(BMS_ADDR, 0x43, tmp, &n, sizeof(tmp));
    if (e != ESP_OK || n < 32) {
        if (i2c_stuck(e)) smbus_bus_recover();
        return e != ESP_OK ? e : ESP_ERR_INVALID_SIZE;
    }
    memcpy(out, tmp, 32);
    return ESP_OK;
}

static uint8_t s_ram_a[4096];
static uint8_t s_ram_b[4096];

static void snapshot_ram(uint8_t dst[4096]) {
    for (uint32_t off = 0; off < 4096u; off += 32u) {
        if (read_mem_block(0x0FF000u + off, dst + off) != ESP_OK) {
            memset(dst + off, 0, 32);
        }
    }
}

static void diff_ram(const char *tag_name) {
    for (uint32_t off = 0x0300u; off < 4096u; off += 32u) {
        if (off >= 0x0d80u && off < 0x0e00u) continue; // skip CPU call stack churn
        if (memcmp(s_ram_a + off, s_ram_b + off, 32) != 0) {
            char lb[32], la[32];
            snprintf(lb, sizeof(lb), "%s OLD %06" PRIx32, tag_name, 0x0FF000u + off);
            snprintf(la, sizeof(la), "%s NEW %06" PRIx32, tag_name, 0x0FF000u + off);
            log_hex(lb, s_ram_a + off, 32);
            log_hex(la, s_ram_b + off, 32);
        }
    }
    memcpy(s_ram_a, s_ram_b, 4096);
}

static void panasonic_vendor_probe(void) {
#if BMS_DEBUG_DIAG
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_log_level_set("i2c.common", ESP_LOG_NONE);
    smbus_set_timeout_ms(35);

    snapshot_ram(s_ram_a);

    // 1. What does WW 0x3b <= 0x1234 change in RAM?
    ESP_LOGI(TAG, "=== 1. RAM diff after WW 0x3b <= 0x1234 ===");
    smbus_write_word(BMS_ADDR, 0x3b, 0x1234);
    snapshot_ram(s_ram_b);
    diff_ram("3b");
    smbus_write_word(BMS_ADDR, 0x3b, 0x0000);
    snapshot_ram(s_ram_a);

    // 2. What does WBL 0x40 len=1 [0x55] and len=2 [0x34, 0x12] change in RAM?
    ESP_LOGI(TAG, "=== 2. RAM diff after WBL 0x40 len=1 [55] & len=2 [34 12] ===");
    uint8_t b1 = 0x55;
    smbus_write_block(BMS_ADDR, 0x40, &b1, 1);
    snapshot_ram(s_ram_b);
    diff_ram("40_1");
    uint8_t b2[2] = {0x34, 0x12};
    smbus_write_block(BMS_ADDR, 0x40, b2, 2);
    snapshot_ram(s_ram_b);
    diff_ram("40_2");

    // 3. Arm 0x71 <= 0x0214, read 0x73(8), and see what WBL 0x73 [11 22 33 44 55 66 77 88] changes in RAM:
    ESP_LOGI(TAG, "=== 3. RAM diff after 0x71<=0x0214 + RBL 0x73 + WBL 0x73 ===");
    uint8_t ch[33] = {0};
    size_t n = 0;
    smbus_write_word(BMS_ADDR, 0x71, 0x0214);
    vTaskDelay(pdMS_TO_TICKS(5));
    smbus_read_block(BMS_ADDR, 0x73, ch, &n, sizeof(ch));
    log_hex("0x73 ch(8)", ch, n);
    if (n == 8) {
        s_disp.blk73_len = 8;
        memcpy(s_disp.blk73, ch, 8);
    }
    snapshot_ram(s_ram_a);

    uint8_t pat8[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    esp_err_t ew73 = smbus_write_block(BMS_ADDR, 0x73, pat8, 8);
    ESP_LOGI(TAG, "WBL 0x73 [11..88]: %s", esp_err_to_name(ew73));
    snapshot_ram(s_ram_b);
    diff_ram("w73");

    // 4. Arm 0x71 <= 0x0214, read 0x73(8), then write WW 0x71 <= 0x5678 and see what changes in RAM:
    ESP_LOGI(TAG, "=== 4. RAM diff after 0x71<=0x0214 + RBL 0x73 + WW 0x71<=0x5678 ===");
    smbus_write_word(BMS_ADDR, 0x71, 0x0214);
    vTaskDelay(pdMS_TO_TICKS(5));
    smbus_read_block(BMS_ADDR, 0x73, ch, &n, sizeof(ch));
    log_hex("0x73 ch_b(8)", ch, n);
    snapshot_ram(s_ram_a);

    esp_err_t ew71 = smbus_write_word(BMS_ADDR, 0x71, 0x5678);
    ESP_LOGI(TAG, "WW 0x71 <= 0x5678: %s", esp_err_to_name(ew71));
    snapshot_ram(s_ram_b);
    diff_ram("w71");

    // 5. Fine-grained 256-byte page scan across 0x0F0000..0x0FFFFF to see EVERY readable 256-byte range:
    ESP_LOGI(TAG, "=== 5. Scanning all readable 256-byte pages in 0x0F0000..0x0FFFFF ===");
    uint8_t page_blk[32];
    for (uint32_t addr = 0x0F0000u; addr < 0x100000u; addr += 0x0100u) {
        if (addr >= 0x0F1000u && addr < 0x0F2000u) continue; // already dumped DF
        if (addr >= 0x0FF000u) continue;                     // already dumped RAM
        if (read_mem_block(addr, page_blk) == ESP_OK) {
            char lbl[28];
            snprintf(lbl, sizeof(lbl), "PAGE %06" PRIx32, addr);
            log_hex(lbl, page_blk, 32);
        }
    }

    // 6. Scan which commands in 0x24..0xFF accept a 1-byte or 2-byte write (in normal mode):
    ESP_LOGI(TAG, "=== 6. Scanning writable commands 0x24..0xFF (1-byte & 2-byte) ===");
    for (int c = 0x24; c <= 0xFF; c++) {
        uint8_t z2[2] = {0, 0};
        esp_err_t e1 = smbus_write_raw_pec(BMS_ADDR, (uint8_t)c, z2, 1);
        if (i2c_stuck(e1)) {
            smbus_bus_recover();
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        esp_err_t e2 = smbus_write_raw_pec(BMS_ADDR, (uint8_t)c, z2, 2);
        if (i2c_stuck(e2)) {
            smbus_bus_recover();
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        if (e1 == ESP_OK || e2 == ESP_OK) {
            ESP_LOGI(TAG, "  writable cmd 0x%02x: 1B=%s 2B=%s",
                     c, (e1 == ESP_OK) ? "OK" : "no", (e2 == ESP_OK) ? "OK" : "no");
        }
    }

    smbus_set_timeout_ms(1000);
    esp_log_level_set("i2c.master", ESP_LOG_WARN);
    esp_log_level_set("i2c.common", ESP_LOG_WARN);
#endif
}

static void log_word_map(void) {
    smbus_set_timeout_ms(80);
    // Cell voltages answer cleanly; read them first as a bus sanity check.
    static const uint8_t cells[] = {0x3c, 0x3d, 0x3e, 0x3f};
    for (size_t i = 0; i < sizeof(cells); i++) {
        uint16_t w = 0;
        if (smbus_read_word(BMS_ADDR, cells[i], &w) == ESP_OK) {
            ESP_LOGI(TAG, "cmd 0x%02x = 0x%04x", cells[i], w);
        } else {
            ESP_LOGW(TAG, "cmd 0x%02x unread", cells[i]);
        }
    }
    // Only read the 4 Panasonic status blocks that actually exist on this gauge.
    // Skipping 0x55/0x5d/0x46/0x4f avoids the 4 I2C bus lockups and GPIO 21/22 reset warnings.
    static const uint8_t status[] = {0x51, 0x53, 0x54, 0x5a};
    for (size_t i = 0; i < sizeof(status); i++) {
        probe_extended(status[i]);
    }
    smbus_set_timeout_ms(1000);
    panasonic_vendor_probe();
}

static esp_err_t read_cells_sbs(uint16_t cell[4]);

// Reads, plus the Panasonic status/vendor blocks. No unseal and no PF key.
static void gauge_fingerprint(uint16_t stack_mv) {
    uint16_t ma = 0;
    if (smbus_read_word(BMS_ADDR, 0x00, &ma) == ESP_OK) {
        ESP_LOGI(TAG, "ManufacturerAccess read 0x%04x", ma);
    }
    log_block_raw(0x23, "ManufacturerData");
    log_word_map();

    uint16_t cell[4];
    if (read_cells_sbs(cell) == ESP_OK) {
        unsigned sum = 0;
        for (int i = 0; i < 4; i++) {
            s_disp.cell_mv[i] = cell[i];
            sum += cell[i];
            ESP_LOGI(TAG, "cell %d = %u mV", i + 1, cell[i]);
        }
        ESP_LOGI(TAG, "cell sum %u mV, Voltage() %u mV", sum, stack_mv);
    }
}

static void log_op(uint32_t op) {
    unsigned sec = (unsigned)((op >> BIT_SEC_SHIFT) & 3u);
    ESP_LOGI(TAG,
             "OperationStatus 0x%08" PRIx32 " SEC=%u (%s) DSG=%lu CHG=%lu PCHG=%lu FUSE=%lu SS=%lu PF=%lu XDSG=%lu XCHG=%lu",
             op,
             sec,
             sec_name(sec),
             (op & BIT_DSG) ? 1ul : 0ul,
             (op & BIT_CHG) ? 1ul : 0ul,
             (op & BIT_PCHG) ? 1ul : 0ul,
             (op & BIT_FUSE) ? 1ul : 0ul,
             (op & BIT_SS) ? 1ul : 0ul,
             (op & BIT_PF) ? 1ul : 0ul,
             (op & BIT_XDSG) ? 1ul : 0ul,
             (op & BIT_XCHG) ? 1ul : 0ul);
}

static esp_err_t refresh_security(void) {
    esp_err_t err = read_status_sub(0x0054, &s_op);
    if (err != ESP_OK) {
        return err;
    }
    s_sec = (unsigned)((s_op >> BIT_SEC_SHIFT) & 3u);
    return ESP_OK;
}

static void log_string(uint8_t cmd, const char *name) {
    uint8_t raw[33];
    size_t n = 0;
    if (smbus_read_block(BMS_ADDR, cmd, raw, &n, sizeof(raw)) != ESP_OK) {
        ESP_LOGW(TAG, "%s unread", name);
        return;
    }
    char text[34];
    size_t used = 0;
    for (size_t i = 0; i < n && used + 1 < sizeof(text); i++) {
        text[used++] = (raw[i] >= 32 && raw[i] < 127) ? (char)raw[i] : '.';
    }
    text[used] = '\0';
    ESP_LOGI(TAG, "%s: \"%s\"", name, text);
}

static void log_identity_words(void) {
    uint16_t raw = 0;
    if (smbus_read_word(BMS_ADDR, 0x0A, &raw) == ESP_OK) {
        s_disp.current_ma = (int16_t)raw;
        ESP_LOGI(TAG, "Current() = %d mA", (int)(int16_t)raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x0D, &raw) == ESP_OK) {
        s_disp.rsoc_pct = raw;
        ESP_LOGI(TAG, "RelativeStateOfCharge() = %u %%", raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x14, &raw) == ESP_OK) {
        s_disp.chg_current_ma = raw;
        ESP_LOGI(TAG, "ChargingCurrent() = %u mA", raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x15, &raw) == ESP_OK) {
        s_disp.chg_voltage_mv = raw;
        ESP_LOGI(TAG, "ChargingVoltage() = %u mV", raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x16, &raw) == ESP_OK) {
        s_disp.battery_status = raw;
        ESP_LOGI(TAG, "BatteryStatus 0x%04x INIT=%u DSG=%u FC=%u FD=%u TCA=%u TDA=%u",
                 raw,
                 (raw >> 7) & 1u,
                 (raw >> 6) & 1u,
                 (raw >> 5) & 1u,
                 (raw >> 4) & 1u,
                 (raw >> 14) & 1u,
                 (raw >> 11) & 1u);
    }
    if (smbus_read_word(BMS_ADDR, 0x17, &raw) == ESP_OK) {
        s_disp.cycle_count = raw;
        ESP_LOGI(TAG, "CycleCount() = %u", raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x18, &raw) == ESP_OK) {
        s_disp.design_cap_mah = raw;
        ESP_LOGI(TAG, "DesignCapacity() = %u", raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x19, &raw) == ESP_OK) {
        s_disp.design_volt_mv = raw;
        ESP_LOGI(TAG, "DesignVoltage() = %u mV", raw);
    }
    if (smbus_read_word(BMS_ADDR, 0x1C, &raw) == ESP_OK) {
        s_disp.serial_num = raw;
        ESP_LOGI(TAG, "SerialNumber() = %u", raw);
    }
}

static esp_err_t read_cells_da(uint16_t cell[4], int *bat_mv, int *pack_mv) {
    uint8_t raw[32];
    size_t n = 0;
    esp_err_t err = read_mac(0x0071, raw, &n, sizeof(raw));
    if (err != ESP_OK || n < 12) {
        return ESP_FAIL;
    }
    for (int i = 0; i < 4; i++) {
        cell[i] = (uint16_t)raw[i * 2] | ((uint16_t)raw[i * 2 + 1] << 8);
    }
    *bat_mv = (int)((uint16_t)raw[8] | ((uint16_t)raw[9] << 8));
    *pack_mv = (int)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));
    return ESP_OK;
}

static esp_err_t read_cells_sbs(uint16_t cell[4]) {
    static const uint8_t cmd[4] = {0x3F, 0x3E, 0x3D, 0x3C};
    for (int i = 0; i < 4; i++) {
        uint16_t value = 0;
        esp_err_t err = smbus_read_word(BMS_ADDR, cmd[i], &value);
        if (err != ESP_OK) {
            return err;
        }
        cell[i] = value;
    }
    return ESP_OK;
}

static bool read_cells(void) {
    s_pack_mv = -1;
    s_bat_mv = -1;
    esp_err_t err = ESP_FAIL;
    if (s_family == 30 || s_family == 20) {
        err = read_cells_sbs(s_cell);
        if (err != ESP_OK && s_family == 30) {
            err = read_cells_da(s_cell, &s_bat_mv, &s_pack_mv);
        }
    } else {
        err = read_cells_da(s_cell, &s_bat_mv, &s_pack_mv);
        if (err != ESP_OK) {
            s_pack_mv = -1;
            s_bat_mv = -1;
            err = read_cells_sbs(s_cell);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cell voltage read failed");
        return false;
    }
    return true;
}

static bool cells_in_window(void) {
    uint16_t lo = 65535;
    uint16_t hi = 0;
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "cell %d = %u mV", i + 1, s_cell[i]);
        if (s_cell[i] < lo) {
            lo = s_cell[i];
        }
        if (s_cell[i] > hi) {
            hi = s_cell[i];
        }
    }
    unsigned spread = (unsigned)(hi - lo);
    ESP_LOGI(TAG, "cells %u..%u mV, spread %u mV; allowed %d..%d mV, spread <= %d",
             lo, hi, spread, CELL_MV_MIN, CELL_MV_MAX, CELL_SPREAD_MV_MAX);
    return lo >= CELL_MV_MIN && hi <= CELL_MV_MAX && spread <= CELL_SPREAD_MV_MAX;
}

static bool temperature_allows(void) {
    uint16_t raw = 0;
    if (smbus_read_word(BMS_ADDR, 0x08, &raw) != ESP_OK || raw < 2000) {
        ESP_LOGW(TAG, "temperature unread; the cell-voltage window is the gate");
        return true;
    }
    int deci_c = (int)raw - 2731;
    int frac = deci_c % 10;
    if (frac < 0) {
        frac = -frac;
    }
    ESP_LOGI(TAG, "Temperature() = %d.%d C", deci_c / 10, frac);
    if (deci_c > TEMP_C_MAX * 10 || deci_c < TEMP_C_MIN * 10) {
        ESP_LOGE(TAG, "ERROR: temperature is outside %d..%d C. PF reset was not sent.",
                 TEMP_C_MIN, TEMP_C_MAX);
        return false;
    }
    return true;
}

static bool key_word_safe(uint16_t word) {
    if ((word >> 8) == 0) {
        ESP_LOGE(TAG, "ERROR: refusing key word 0x%04x. A 0x00 high byte can brick a TI gauge.", word);
        return false;
    }
    return true;
}

static void bus_quiet(void) {
    ESP_LOGI(TAG, "waiting 4.1 s with no bus traffic before the key");
    vTaskDelay(pdMS_TO_TICKS(4100));
}

static bool security_reached(unsigned target) {
    return s_sec == target || (target == 2 && s_sec == 1);
}

static esp_err_t bq40_enter(uint16_t w0, uint16_t w1, unsigned target, const char *what) {
    if (!key_word_safe(w0) || !key_word_safe(w1)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (refresh_security() != ESP_OK) {
        return ESP_FAIL;
    }
    if (security_reached(target)) {
        ESP_LOGI(TAG, "gauge is already %s", sec_name(s_sec));
        return ESP_OK;
    }
    unsigned required = (target == 1) ? 2u : 3u;
    if (s_sec != required) {
        ESP_LOGE(TAG, "ERROR: %s starts from SEC %u, gauge is SEC %u (%s)",
                 what, required, s_sec, sec_name(s_sec));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "%s key words 0x%04x then 0x%04x on ManufacturerAccess 0x00", what, w0, w1);
    bus_quiet();
    esp_err_t first = smbus_write_word(BMS_ADDR, 0x00, w0);
    vTaskDelay(pdMS_TO_TICKS(15));
    esp_err_t second = smbus_write_word(BMS_ADDR, 0x00, w1);
    vTaskDelay(pdMS_TO_TICKS(400));
    ESP_LOGI(TAG, "%s word writes: %s, %s", what, esp_err_to_name(first), esp_err_to_name(second));

    if (refresh_security() != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: OperationStatus unread after %s. No second key was sent.", what);
        return ESP_FAIL;
    }
    log_op(s_op);
    if (security_reached(target)) {
        return ESP_OK;
    }
    if (s_sec != required) {
        ESP_LOGE(TAG, "ERROR: %s left the gauge in SEC %u (%s)", what, s_sec, sec_name(s_sec));
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "%s word key was not accepted; trying ManufacturerBlockAccess 0x44", what);
    bus_quiet();
    uint8_t first_block[2] = {(uint8_t)w0, (uint8_t)(w0 >> 8)};
    uint8_t second_block[2] = {(uint8_t)w1, (uint8_t)(w1 >> 8)};
    first = smbus_write_block(BMS_ADDR, 0x44, first_block, sizeof(first_block));
    vTaskDelay(pdMS_TO_TICKS(15));
    second = smbus_write_block(BMS_ADDR, 0x44, second_block, sizeof(second_block));
    vTaskDelay(pdMS_TO_TICKS(400));
    ESP_LOGI(TAG, "%s block writes: %s, %s", what, esp_err_to_name(first), esp_err_to_name(second));
    if (refresh_security() != ESP_OK) {
        return ESP_FAIL;
    }
    log_op(s_op);
    if (security_reached(target)) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "ERROR: %s key rejected. This pack is not using the TI default key.", what);
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t bq30_enter(uint16_t sub, unsigned target, const char *what) {
    if (!sha1_self_test()) {
        ESP_LOGE(TAG, "ERROR: SHA-1 self-test failed. No authentication block was sent.");
        return ESP_FAIL;
    }
    if (refresh_security() != ESP_OK) {
        return ESP_FAIL;
    }
    if (security_reached(target)) {
        ESP_LOGI(TAG, "gauge is already %s", sec_name(s_sec));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "%s: ManufacturerAccess 0x%04x, SHA-1 response to ManufacturerInput 0x2F", what, sub);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t wrote = smbus_write_word(BMS_ADDR, 0x00, sub);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t challenge[32];
    size_t n = 0;
    esp_err_t read = smbus_read_block(BMS_ADDR, 0x2F, challenge, &n, sizeof(challenge));
    if (wrote != ESP_OK || read != ESP_OK || n != 20) {
        ESP_LOGE(TAG, "ERROR: %s challenge write %s, read %s, length %u (need 20)",
                 what, esp_err_to_name(wrote), esp_err_to_name(read), (unsigned)n);
        if (n > 0) {
            log_hex("challenge", challenge, n);
        }
        return ESP_FAIL;
    }
    log_hex("challenge", challenge, 20);

    static const uint8_t key[16] = {BQ30_DEFAULT_KEY_INIT};
    uint8_t response[20];
    if (!ti_unseal_response(key, challenge, response)) {
        return ESP_FAIL;
    }
    esp_err_t response_write = smbus_write_block(BMS_ADDR, 0x2F, response, sizeof(response));
    vTaskDelay(pdMS_TO_TICKS(400));
    ESP_LOGI(TAG, "%s response write: %s", what, esp_err_to_name(response_write));
    if (refresh_security() != ESP_OK) {
        return ESP_FAIL;
    }
    log_op(s_op);
    if (security_reached(target)) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "ERROR: %s rejected. This pack is not using the TI default SHA-1 key.", what);
    return ESP_ERR_INVALID_RESPONSE;
}

static void seal_device(void) {
    if (refresh_security() != ESP_OK) {
        ESP_LOGW(TAG, "security state unread; Seal Device was not sent");
        return;
    }
    if (s_sec == 3) {
        ESP_LOGI(TAG, "gauge is sealed");
        return;
    }
    if (s_sec != 1 && s_sec != 2) {
        ESP_LOGW(TAG, "SEC %u (%s); Seal Device was not sent", s_sec, sec_name(s_sec));
        return;
    }
    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, 0x0030);
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(TAG, "Seal Device 0x0030: %s", esp_err_to_name(err));
    if (refresh_security() == ESP_OK) {
        log_op(s_op);
    }
}

static void ensure_fet_control(void) {
    uint32_t mfg = 0;
    if (read_status_sub(0x0057, &mfg) != ESP_OK) {
        ESP_LOGW(TAG, "ManufacturingStatus unread; FET Control was not sent");
        return;
    }
    ESP_LOGI(TAG, "ManufacturingStatus 0x%08" PRIx32 " FET_EN=%lu", mfg, (mfg & BIT_FET_EN) ? 1ul : 0ul);
    if (mfg == 0 || (mfg & BIT_FET_EN) != 0) {
        if (mfg == 0) {
            ESP_LOGW(TAG, "ManufacturingStatus is 0; FET Control was not sent");
        }
        return;
    }
    ESP_LOGI(TAG, "firmware FET control is off; sending FET Control 0x0022");
    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, 0x0022);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "FET Control write: %s", esp_err_to_name(err));
    if (read_status_sub(0x0057, &mfg) == ESP_OK) {
        ESP_LOGI(TAG, "ManufacturingStatus now 0x%08" PRIx32 " FET_EN=%lu",
                 mfg, (mfg & BIT_FET_EN) ? 1ul : 0ul);
    }
}

static void reset_gauge(void) {
    uint16_t cmd = (s_family == 30) ? 0x0012 : 0x0041;
    ESP_LOGI(TAG, "device reset, ManufacturerAccess 0x%04x", cmd);
    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, cmd);
    ESP_LOGI(TAG, "reset write: %s", esp_err_to_name(err));
    for (int attempt = 0; attempt < 10; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(300));
        uint16_t voltage = 0;
        if (smbus_read_word(BMS_ADDR, 0x09, &voltage) == ESP_OK) {
            ESP_LOGI(TAG, "gauge answered after reset, Voltage() = %u mV", voltage);
            return;
        }
    }
    ESP_LOGE(TAG, "ERROR: gauge did not answer for 3 s after reset");
}

static esp_err_t report_output(void) {
    bool have_cells = read_cells();
    bool dsg = false;
    if (refresh_security() == ESP_OK) {
        log_op(s_op);
        dsg = (s_op & BIT_DSG) != 0;
    }
    int sum = 0;
    if (have_cells) {
        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "cell %d = %u mV", i + 1, s_cell[i]);
            sum += s_cell[i];
        }
        ESP_LOGI(TAG, "cell sum %d mV (four cells at 4100 mV sum to 16400 mV)", sum);
    }
    if (s_pack_mv >= 0) {
        ESP_LOGI(TAG, "DAStatus1 BAT pin %d mV, PACK pin %d mV", s_bat_mv, s_pack_mv);
    }

    bool pack_live = have_cells && s_pack_mv >= 0 && s_pack_mv > 8000 && (s_pack_mv + 2000) >= sum;
    if (dsg && s_pack_mv < 0) {
        ESP_LOGI(TAG, "RESULT: discharge FET is on. Measure the battery output tabs; they should sit near the cell sum.");
        return ESP_OK;
    }
    if (dsg && pack_live) {
        ESP_LOGI(TAG, "RESULT: discharge FET is on and the pack pin reads %d mV.", s_pack_mv);
        return ESP_OK;
    }
    if (dsg && s_pack_mv >= 0) {
        ESP_LOGE(TAG, "RESULT: discharge FET reads on, but the pack pin is %d mV. The series path is still open.",
                 s_pack_mv);
        return ESP_FAIL;
    }
    ESP_LOGE(TAG, "RESULT: discharge FET is off. The output tabs stay near 0 V.");
    return ESP_FAIL;
}

// bq20z70/bq20z75 OperationStatus (SLUU310 Figure B-4). FAS 0 means full access. SS 1 means sealed.
// DSG in this word copies BatteryStatus discharging; it is not the discharge FET.
enum {
    BQ20_XDSG = 1u << 5,
    BQ20_SS = 1u << 13,
    BQ20_FAS = 1u << 14,
    BQ20_SAFETY_PF = 1u << 5,
};

static esp_err_t read_ma_word(uint16_t sub, uint16_t *value) {
    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, sub);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    uint16_t word = 0;
    err = smbus_read_word(BMS_ADDR, 0x00, &word);
    if (err != ESP_OK) {
        return err;
    }
    if (word == sub || word == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *value = word;
    return ESP_OK;
}

static esp_err_t bq20_read_status(uint8_t cmd, uint16_t *value) {
    if (smbus_read_word(BMS_ADDR, cmd, value) == ESP_OK) {
        return ESP_OK;
    }
    return read_ma_word(cmd, value);
}

static void log_bq20_op(uint16_t op) {
    ESP_LOGI(TAG, "OperationStatus 0x%04x FAS=%u (0=full access) SS=%u (1=sealed) XDSG=%u",
             op,
             (op & BQ20_FAS) ? 1 : 0,
             (op & BQ20_SS) ? 1 : 0,
             (op & BQ20_XDSG) ? 1 : 0);
}

static void log_bq20_safety(uint16_t safety) {
    ESP_LOGI(TAG,
             "SafetyStatus 0x%04x PF=%u CUV=%u COV=%u OCD=%u OCC=%u OTD=%u OTC=%u SCD=%u SCC=%u",
             safety,
             (safety & BQ20_SAFETY_PF) ? 1 : 0,
             (safety & (1u << 7)) ? 1 : 0,
             (safety & (1u << 6)) ? 1 : 0,
             (safety & (1u << 13)) ? 1 : 0,
             (safety & (1u << 12)) ? 1 : 0,
             (safety & (1u << 15)) ? 1 : 0,
             (safety & (1u << 14)) ? 1 : 0,
             (safety & (1u << 0)) ? 1 : 0,
             (safety & (1u << 1)) ? 1 : 0);
}

static bool bq20_pf_set(bool safety_ok, uint16_t safety, bool pf_ok, uint16_t pf) {
    return (safety_ok && (safety & BQ20_SAFETY_PF) != 0) || (pf_ok && pf != 0);
}

static esp_err_t bq20_write_pair(uint16_t w0, uint16_t w1, const char *what) {
    if (!key_word_safe(w0) || !key_word_safe(w1)) {
        ESP_LOGE(TAG, "ERROR: %s refused. No word was sent.", what);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "%s: 0x%04x then 0x%04x", what, w0, w1);
    bus_quiet();
    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, w0);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
    err = smbus_write_word(BMS_ADDR, 0x00, w1);
    vTaskDelay(pdMS_TO_TICKS(400));
    return err;
}

static void bq20_seal(void) {
    uint16_t op = 0;
    if (bq20_read_status(0x54, &op) != ESP_OK) {
        ESP_LOGW(TAG, "seal skipped; OperationStatus unread");
        return;
    }
    if ((op & BQ20_SS) != 0) {
        ESP_LOGI(TAG, "gauge is sealed");
        return;
    }
    ESP_LOGI(TAG, "Seal Device 0x0020");
    esp_err_t err = smbus_write_word(BMS_ADDR, 0x00, 0x0020);
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_LOGI(TAG, "seal write: %s", esp_err_to_name(err));
    if (bq20_read_status(0x54, &op) == ESP_OK) {
        log_bq20_op(op);
    }
}

static bool bq20_refresh_pf(uint16_t *safety, uint16_t *pf, bool *safety_ok, bool *pf_ok) {
    *safety_ok = bq20_read_status(0x51, safety) == ESP_OK;
    *pf_ok = bq20_read_status(0x53, pf) == ESP_OK;
    if (*safety_ok) {
        log_bq20_safety(*safety);
    }
    if (*pf_ok) {
        ESP_LOGI(TAG, "PFStatus 0x%04x", *pf);
    }
    return *safety_ok || *pf_ok;
}

static esp_err_t bq20_service(void) {
    uint16_t op = 0;
    uint16_t safety = 0;
    uint16_t pf = 0;
    uint16_t pack = 0;
    uint16_t stack = 0;
    if (bq20_read_status(0x54, &op) != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: OperationStatus unread. No unlock attempted.");
        return ESP_FAIL;
    }
    log_bq20_op(op);
    bool safety_ok = false;
    bool pf_ok = false;
    bq20_refresh_pf(&safety, &pf, &safety_ok, &pf_ok);
    if (!safety_ok) {
        ESP_LOGW(TAG, "SafetyStatus unread");
    }
    if (!pf_ok) {
        ESP_LOGW(TAG, "PFStatus unread");
    }
    bool pack_ok = bq20_read_status(0x5a, &pack) == ESP_OK;
    if (pack_ok) {
        s_pack_mv = (int)pack;
        ESP_LOGI(TAG, "PackVoltage() = %u mV", pack);
    }
    smbus_read_word(BMS_ADDR, 0x09, &stack);

    bool pf_on = bq20_pf_set(safety_ok, safety, pf_ok, pf);
    if (!safety_ok && !pf_ok) {
        ESP_LOGE(TAG, "ERROR: neither SafetyStatus nor PFStatus could be read. No key was sent.");
        return ESP_FAIL;
    }
    if (!pf_on) {
        if (pack_ok && pack > 8000 && stack > 0 && (unsigned)pack + 2000u >= stack) {
            ESP_LOGI(TAG, "RESULT: permanent fail is clear and the pack pin reads %u mV. No security command was sent.",
                     pack);
            return ESP_OK;
        }
        ESP_LOGE(TAG, "ERROR: permanent fail is clear and XDSG=%u. FETs were not forced on.",
                 (op & BQ20_XDSG) ? 1 : 0);
        return ESP_ERR_INVALID_STATE;
    }

    if (!temperature_allows()) {
        return ESP_ERR_INVALID_STATE;
    }
    bool cells_ready = read_cells();
    if (cells_ready && !cells_in_window()) {
        ESP_LOGE(TAG, "ERROR: cell voltages are outside the safe window. PF key was not sent.");
        return ESP_ERR_INVALID_STATE;
    }
    if (!cells_ready) {
        if (stack < 12000 || stack > 17000) {
            ESP_LOGE(TAG, "ERROR: cell voltages unread and stack %u mV is outside 12000..17000 mV. PF key was not sent.",
                     stack);
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGW(TAG, "cell voltages unread; stack %u mV is inside 12000..17000 mV", stack);
    }
    if (smbus_pec_mode() < 0) {
        ESP_LOGE(TAG, "ERROR: PEC mode unknown. No key was sent.");
        return ESP_FAIL;
    }

    bool sent_key = false;
    if ((op & BQ20_SS) != 0) {
        esp_err_t err = bq20_write_pair(BQ40_UNSEAL_W0, BQ40_UNSEAL_W1, "unseal");
        if (err != ESP_OK) {
            return err;
        }
        sent_key = true;
        if (bq20_read_status(0x54, &op) != ESP_OK) {
            ESP_LOGE(TAG, "ERROR: OperationStatus unread after unseal.");
            return ESP_FAIL;
        }
        log_bq20_op(op);
        if ((op & BQ20_SS) != 0) {
            err = bq20_write_pair(0x1404, 0x7236, "unseal byte-swapped");
            if (err != ESP_OK) {
                return err;
            }
            if (bq20_read_status(0x54, &op) != ESP_OK || (op & BQ20_SS) != 0) {
                if (bq20_read_status(0x54, &op) == ESP_OK) {
                    log_bq20_op(op);
                }
                ESP_LOGE(TAG, "ERROR: unseal key rejected. No PF key was sent.");
                return ESP_ERR_INVALID_RESPONSE;
            }
            log_bq20_op(op);
        }
    } else {
        ESP_LOGI(TAG, "already unsealed");
    }

    if (!cells_ready) {
        cells_ready = read_cells() && cells_in_window();
        if (!cells_ready) {
            ESP_LOGE(TAG, "ERROR: cell voltages are missing or outside the safe window. PF key was not sent.");
            if (sent_key) {
                bq20_seal();
            }
            return ESP_ERR_INVALID_STATE;
        }
    }

    esp_err_t err = bq20_write_pair(BQ20_PF_W0, BQ20_PF_W1, "PF clear");
    if (err != ESP_OK) {
        bq20_seal();
        return err;
    }
    bool readable = bq20_refresh_pf(&safety, &pf, &safety_ok, &pf_ok);
    bool cleared = readable && !bq20_pf_set(safety_ok, safety, pf_ok, pf);
    if (!cleared) {
        ESP_LOGW(TAG, "PF still set; trying the byte-swapped key once");
        err = bq20_write_pair(BQ20_PF_W0_ALT, BQ20_PF_W1_ALT, "PF clear byte-swapped");
        if (err != ESP_OK) {
            bq20_seal();
            return err;
        }
        readable = bq20_refresh_pf(&safety, &pf, &safety_ok, &pf_ok);
        cleared = readable && !bq20_pf_set(safety_ok, safety, pf_ok, pf);
    }
    bq20_seal();
    if (!cleared) {
        ESP_LOGE(TAG, "ERROR: permanent fail is still set. The pack key is not the TI default.");
        return ESP_FAIL;
    }

    pack_ok = bq20_read_status(0x5a, &pack) == ESP_OK;
    smbus_read_word(BMS_ADDR, 0x09, &stack);
    if (bq20_read_status(0x54, &op) == ESP_OK) {
        log_bq20_op(op);
    }
    if (pack_ok) {
        ESP_LOGI(TAG, "PackVoltage() = %u mV, Voltage() = %u mV", pack, stack);
    }
    if (pack_ok && pack > 8000 && stack > 0 && (unsigned)pack + 2000u >= stack) {
        ESP_LOGI(TAG, "RESULT: permanent fail cleared and the pack pin reads %u mV.", pack);
        return ESP_OK;
    }
    if (pack_ok) {
        ESP_LOGE(TAG, "RESULT: permanent fail reads clear. Pack pin is %u mV. Output was not forced on.", pack);
    } else {
        ESP_LOGE(TAG, "RESULT: permanent fail reads clear. Pack pin was unread. Output was not forced on.");
    }
    return ESP_FAIL;
}

esp_err_t bms_unlock(bool *talking) {
    memset(&s_disp, 0, sizeof(s_disp));
    snprintf(s_disp.state_line, sizeof(s_disp.state_line), "PROBING BMS...");
    if (talking != NULL) {
        *talking = false;
    }

    uint16_t voltage = 0;
    esp_err_t err = smbus_read_word(BMS_ADDR, 0x09, &voltage);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: BMS 0x%02x did not return Voltage() (%s). No unlock attempted.",
                 BMS_ADDR, esp_err_to_name(err));
        tft_update(&s_disp);
        return err;
    }
    s_disp.talking = true;
    s_disp.voltage_mv = voltage;
    if (talking != NULL) {
        *talking = true;
    }
    ESP_LOGI(TAG, "Voltage() = %u mV, PEC %s", voltage, smbus_pec_mode() == 1 ? "on" : "off");
    log_string(0x20, "ManufacturerName");
    log_string(0x21, "DeviceName");
    log_string(0x22, "DeviceChemistry");
    log_identity_words();

    log_string(0x23, "ManufacturerData");

    s_family = 0;
    s_type = 0;
    uint16_t idle_ma = 0;
    if (smbus_read_word(BMS_ADDR, 0x00, &idle_ma) == ESP_OK) {
        s_disp.ma_word = idle_ma;
        ESP_LOGI(TAG, "ManufacturerAccess before any command 0x%04x", idle_ma);
    }
    uint16_t type_word = 0;
    bool have_type_word = read_ma_word(0x0001, &type_word) == ESP_OK;
    if (have_type_word) {
        ESP_LOGI(TAG, "ManufacturerAccess device type word 0x%04x", type_word);
        s_type = type_word;
        s_family = family_of(s_type);
    } else {
        uint8_t ident[16];
        size_t n = 0;
        if (read_mac(0x0001, ident, &n, sizeof(ident)) == ESP_OK && n >= 2) {
            log_hex("DeviceType MAC", ident, n);
            s_type = (uint16_t)ident[0] | ((uint16_t)ident[1] << 8);
            s_family = family_of(s_type);
        }
    }
    ESP_LOGI(TAG, "device type 0x%04x (%s)", s_type, type_name(s_type));
    if (!have_type_word && s_family == 0) {
        ESP_LOGE(TAG, "ERROR: device type unread. No unlock attempted.");
        gauge_fingerprint(voltage);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint16_t fw = 0;
    bool have_fw = read_ma_word(0x0002, &fw) == ESP_OK;
    if (have_fw) {
        ESP_LOGI(TAG, "firmware version word 0x%04x (%u.%02u)", fw, (unsigned)(fw >> 8), (unsigned)(fw & 0xff));
    } else if (s_family != 20) {
        uint8_t ident[16];
        size_t n = 0;
        if (read_mac(0x0002, ident, &n, sizeof(ident)) == ESP_OK) {
            log_hex("FirmwareVersion", ident, n);
        }
    }

    if (s_family == 0 || (s_type != 0 && have_fw && fw == s_type)) {
        ESP_LOGE(TAG,
                 "ERROR: no known gauge family with a changing status word (ManufacturerAccess 0x%04x). "
                 "No security command was sent.",
                 s_type);
        gauge_fingerprint(voltage);
        snprintf(s_disp.state_line, sizeof(s_disp.state_line), "LOCKED PF (FUSE)");
        tft_update(&s_disp);
        s_family = 0;
        s_type = 0;
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (s_family == 20) {
        return bq20_service();
    }

    uint32_t pf = 0;
    bool pf_ok = read_status_sub(0x0053, &pf) == ESP_OK;
    if (refresh_security() != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: OperationStatus did not return a new word (last device type was 0x%04x). No security command was sent.",
                 s_type);
        log_direct_word(0x51, "SafetyStatus");
        log_direct_word(0x53, "PFStatus");
        log_direct_word(0x54, "OperationStatus");
        log_direct_word(0x5a, "PackVoltage");
        s_family = 0;
        return ESP_ERR_INVALID_RESPONSE;
    }
    log_op(s_op);
    if (pf_ok) {
        ESP_LOGI(TAG, "PFStatus 0x%08" PRIx32, pf);
    } else {
        ESP_LOGW(TAG, "PFStatus unread");
    }

    bool fuse = ((s_op & BIT_FUSE) != 0) || (pf_ok && ((pf & BIT_PF_FUSE) != 0 || (pf & BIT_PF_2LVL) != 0));
    if (fuse) {
        ESP_LOGE(TAG, "ERROR: fuse or second-level protector is indicated. No unseal was sent.");
        return ESP_ERR_INVALID_STATE;
    }

    bool pf_on = (pf_ok && pf != 0) || ((s_op & BIT_PF) != 0);
    bool dsg = (s_op & BIT_DSG) != 0;
    if (!pf_on && dsg) {
        ESP_LOGI(TAG, "RESULT: discharge FET is already on and permanent fail is clear. No security command was sent.");
        read_cells();
        return ESP_OK;
    }
    if (!pf_on) {
        uint32_t safety = 0;
        if (read_status_sub(0x0051, &safety) == ESP_OK) {
            ESP_LOGI(TAG, "SafetyStatus 0x%08" PRIx32, safety);
        }
        ESP_LOGE(TAG, "ERROR: discharge FET is off and permanent fail is clear. FETs were not forced on.");
        return ESP_ERR_INVALID_STATE;
    }

    if (!temperature_allows()) {
        return ESP_ERR_INVALID_STATE;
    }

    bool cells_ready = read_cells();
    if (cells_ready && !cells_in_window()) {
        ESP_LOGE(TAG, "ERROR: cell voltages are outside the safe window. PF reset was not sent.");
        return ESP_ERR_INVALID_STATE;
    }
    if (!cells_ready) {
        ESP_LOGW(TAG, "cell voltages unread while sealed; unseal will be used only to read them again");
    }
    if (smbus_pec_mode() < 0) {
        ESP_LOGE(TAG, "ERROR: PEC mode unknown. No key was sent.");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "permanent fail is set; unsealing the %s", type_name(s_type));
    err = (s_family == 30) ? bq30_enter(0x0031, 2, "unseal")
                           : bq40_enter(BQ40_UNSEAL_W0, BQ40_UNSEAL_W1, 2, "unseal");
    if (err != ESP_OK) {
        seal_device();
        return err;
    }
    if (!cells_ready) {
        cells_ready = read_cells() && cells_in_window();
        if (!cells_ready) {
            ESP_LOGE(TAG, "ERROR: cell voltages are missing or outside the safe window. PF reset was not sent.");
            seal_device();
            return ESP_ERR_INVALID_STATE;
        }
    }

    err = (s_family == 30) ? bq30_enter(0x0032, 1, "full access")
                           : bq40_enter(BQ40_FULL_W0, BQ40_FULL_W1, 1, "full access");
    if (s_family == 30 && err != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: bq30 PF reset needs full access. PF reset was not sent.");
        seal_device();
        return err;
    }
    if (s_family == 40 && err != ESP_OK) {
        ESP_LOGW(TAG, "full access was not granted; PF reset will be tried while unsealed");
        if (refresh_security() != ESP_OK || (s_sec != 1 && s_sec != 2)) {
            seal_device();
            return err;
        }
    }

    ensure_fet_control();
    ESP_LOGI(TAG, "Permanent Fail Data Reset 0x0029");
    esp_err_t pf_write = smbus_write_word(BMS_ADDR, 0x00, 0x0029);
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "PF reset write: %s", esp_err_to_name(pf_write));

    pf_ok = read_status_sub(0x0053, &pf) == ESP_OK;
    bool op_ok = refresh_security() == ESP_OK;
    if (op_ok) {
        log_op(s_op);
    }
    if (pf_ok) {
        ESP_LOGI(TAG, "PFStatus after reset 0x%08" PRIx32, pf);
    }
    if (!pf_ok || pf != 0 || (op_ok && (s_op & BIT_PF) != 0)) {
        ESP_LOGE(TAG, "ERROR: permanent-fail data is still set.");
        seal_device();
        return ESP_FAIL;
    }

    if (!op_ok || (s_op & BIT_DSG) == 0) {
        reset_gauge();
    }
    seal_device();
    return report_output();
}

void bms_log_snapshot(void) {
    uint16_t voltage = 0;
    uint16_t current_raw = 0;
    uint16_t status = 0;
    if (smbus_read_word(BMS_ADDR, 0x09, &voltage) == ESP_OK &&
        smbus_read_word(BMS_ADDR, 0x0A, &current_raw) == ESP_OK) {
        s_disp.voltage_mv = voltage;
        s_disp.current_ma = (int16_t)current_raw;
        ESP_LOGI(TAG, "snapshot Voltage()=%u mV Current()=%d mA", voltage, (int)(int16_t)current_raw);
    } else {
        ESP_LOGW(TAG, "snapshot voltage read failed");
        return;
    }
    if (smbus_read_word(BMS_ADDR, 0x16, &status) == ESP_OK) {
        s_disp.battery_status = status;
        ESP_LOGI(TAG, "snapshot BatteryStatus 0x%04x", status);
    }
    read_cells_sbs(s_disp.cell_mv);
    tft_update(&s_disp);
    if (s_family == 20) {
        uint16_t op = 0;
        uint16_t pack = 0;
        if (bq20_read_status(0x54, &op) == ESP_OK) {
            log_bq20_op(op);
        }
        if (bq20_read_status(0x5a, &pack) == ESP_OK) {
            ESP_LOGI(TAG, "snapshot PackVoltage()=%u mV", pack);
        }
    } else if (s_family != 0 && refresh_security() == ESP_OK) {
        log_op(s_op);
    }
    if (s_family == 40 && read_cells()) {
        ESP_LOGI(TAG, "snapshot BAT pin %d mV, PACK pin %d mV", s_bat_mv, s_pack_mv);
    }
}
