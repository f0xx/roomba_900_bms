#include "tft.h"
#include "config.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tft";
static spi_device_handle_t s_spi = NULL;
static uint16_t s_line_buf[240 * 12];

// Standard 5x7 ASCII font (0x20..0x7E)
static const uint8_t s_font5x7[95][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x00, 0x00, 0x5f, 0x00, 0x00}, // '!'
    {0x00, 0x07, 0x00, 0x07, 0x00}, // '"'
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, // '#'
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // '$'
    {0x23, 0x13, 0x08, 0x64, 0x62}, // '%'
    {0x36, 0x49, 0x55, 0x22, 0x50}, // '&'
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '''
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // '('
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // ')'
    {0x14, 0x08, 0x3e, 0x08, 0x14}, // '*'
    {0x08, 0x08, 0x3e, 0x08, 0x08}, // '+'
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ','
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-'
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.'
    {0x20, 0x10, 0x08, 0x04, 0x02}, // '/'
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // '0'
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ';'
    {0x08, 0x14, 0x22, 0x41, 0x00}, // '<'
    {0x14, 0x14, 0x14, 0x14, 0x14}, // '='
    {0x00, 0x41, 0x22, 0x14, 0x08}, // '>'
    {0x02, 0x01, 0x51, 0x09, 0x06}, // '?'
    {0x32, 0x49, 0x79, 0x41, 0x3e}, // '@'
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // 'A'
    {0x7f, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, // 'D'
    {0x7f, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, // 'G'
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // 'H'
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3f, 0x01}, // 'J'
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // 'M'
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // 'N'
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // 'O'
    {0x7f, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // 'Q'
    {0x7f, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // 'T'
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, // 'U'
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, // 'V'
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
    {0x00, 0x7f, 0x41, 0x41, 0x00}, // '['
    {0x02, 0x04, 0x08, 0x10, 0x20}, // '\'
    {0x00, 0x41, 0x41, 0x7f, 0x00}, // ']'
    {0x04, 0x02, 0x01, 0x02, 0x04}, // '^'
    {0x40, 0x40, 0x40, 0x40, 0x40}, // '_'
    {0x00, 0x01, 0x02, 0x04, 0x00}, // '`'
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 'a'
    {0x7f, 0x48, 0x44, 0x44, 0x38}, // 'b'
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 'c'
    {0x38, 0x44, 0x44, 0x48, 0x7f}, // 'd'
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 'e'
    {0x08, 0x7e, 0x09, 0x01, 0x02}, // 'f'
    {0x0c, 0x52, 0x52, 0x52, 0x3e}, // 'g'
    {0x7f, 0x08, 0x04, 0x04, 0x78}, // 'h'
    {0x00, 0x44, 0x7d, 0x40, 0x00}, // 'i'
    {0x20, 0x40, 0x44, 0x3d, 0x00}, // 'j'
    {0x7f, 0x10, 0x28, 0x44, 0x00}, // 'k'
    {0x00, 0x41, 0x7f, 0x40, 0x00}, // 'l'
    {0x7c, 0x04, 0x18, 0x04, 0x78}, // 'm'
    {0x7c, 0x08, 0x04, 0x04, 0x78}, // 'n'
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 'o'
    {0x7c, 0x14, 0x14, 0x14, 0x08}, // 'p'
    {0x08, 0x14, 0x14, 0x18, 0x7c}, // 'q'
    {0x7c, 0x08, 0x04, 0x04, 0x08}, // 'r'
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 's'
    {0x04, 0x3f, 0x44, 0x40, 0x20}, // 't'
    {0x3c, 0x40, 0x40, 0x20, 0x7c}, // 'u'
    {0x1c, 0x20, 0x40, 0x20, 0x1c}, // 'v'
    {0x3c, 0x40, 0x30, 0x40, 0x3c}, // 'w'
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 'x'
    {0x0c, 0x50, 0x50, 0x50, 0x3c}, // 'y'
    {0x44, 0x64, 0x54, 0x4c, 0x44}, // 'z'
    {0x00, 0x08, 0x36, 0x41, 0x00}, // '{'
    {0x00, 0x00, 0x7f, 0x00, 0x00}, // '|'
    {0x00, 0x41, 0x36, 0x08, 0x00}, // '}'
    {0x08, 0x04, 0x08, 0x10, 0x08}, // '~'
};

static inline uint16_t swap16(uint16_t v) {
    return (uint16_t)((v >> 8) | (v << 8));
}

static void lcd_cmd(uint8_t cmd) {
    gpio_set_level(PIN_TFT_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_data(const void *data, size_t len_bytes) {
    if (len_bytes == 0) {
        return;
    }
    gpio_set_level(PIN_TFT_DC, 1);
    spi_transaction_t t = {
        .length = len_bytes * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_u8(uint8_t v) {
    lcd_data(&v, 1);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t caset[4] = {(uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1};
    uint8_t raset[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint16_t)y1};
    lcd_cmd(0x2A);
    lcd_data(caset, 4);
    lcd_cmd(0x2B);
    lcd_data(raset, 4);
    lcd_cmd(0x2C);
}

static void lcd_fill_screen(uint16_t rgb565) {
    uint16_t be = swap16(rgb565);
    for (int i = 0; i < 240 * 12; i++) {
        s_line_buf[i] = be;
    }
    lcd_set_window(0, 0, 239, 239);
    for (int y = 0; y < 240; y += 12) {
        lcd_data(s_line_buf, sizeof(s_line_buf));
    }
}

// Compact 1x font: 6px cell width (38 chars/line), 11px row height (up to 21 rows on 240x240).
static void lcd_draw_row(uint16_t y0, const char *text, uint16_t fg565, uint16_t bg565) {
    if (y0 + 11 > 240) {
        return;
    }
    uint16_t fg = swap16(fg565);
    uint16_t bg = swap16(bg565);
    for (int i = 0; i < 240 * 11; i++) {
        s_line_buf[i] = bg;
    }
    for (int c = 0; text[c] != '\0' && c < 38; c++) {
        uint8_t ch = (uint8_t)text[c];
        if (ch < 0x20 || ch > 0x7E) {
            ch = '?';
        }
        const uint8_t *glyph = s_font5x7[ch - 0x20];
        int x_base = 4 + c * 6;
        for (int col = 0; col < 5; col++) {
            uint8_t bits = glyph[col];
            for (int row = 0; row < 7; row++) {
                if ((bits >> row) & 1u) {
                    int py = 2 + row;
                    int px = x_base + col;
                    s_line_buf[py * 240 + px] = fg;
                }
            }
        }
    }
    lcd_set_window(0, y0, 239, (uint16_t)(y0 + 10));
    lcd_data(s_line_buf, 240 * 11 * sizeof(uint16_t));
}

void tft_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_TFT_DC) | (1ULL << PIN_TFT_RST) | (1ULL << PIN_TFT_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PIN_TFT_BL, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_TFT_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(s_line_buf),
    };
    if (spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGW(TAG, "SPI bus init failed");
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    if (spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi) != ESP_OK) {
        ESP_LOGW(TAG, "SPI device add failed");
        return;
    }

    gpio_set_level(PIN_TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x01); // SWRESET
    vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0x11); // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x3A); // COLMOD 16-bit RGB565
    lcd_u8(0x55);
    lcd_cmd(0x36); // MADCTL
    lcd_u8(0x00);
    lcd_cmd(0x21); // INVON (IPS panel)
    lcd_cmd(0x13); // NORON
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_cmd(0x29); // DISPON
    vTaskDelay(pdMS_TO_TICKS(20));

    lcd_fill_screen(0x0000);
    gpio_set_level(PIN_TFT_BL, 1);
    ESP_LOGI(TAG, "ST7789 240x240 initialized (compact 38x20 layout)");

    lcd_draw_row(2, "ROOMBA 4SU0016 BMS (PanasonicEnergy)", 0x07FF, 0x0010);
    lcd_draw_row(16, "Scanning SMBus ch3 @ 0x0B...", 0xFFFF, 0x0000);
}

void tft_update(const bms_display_t *d) {
    if (s_spi == NULL || d == NULL) {
        return;
    }
    char line[44];
    uint16_t y = 2;
    lcd_draw_row(y, "ROOMBA 4SU0016  Panasonic F12432809", 0x07FF, 0x0010);
    y += 13;

    if (!d->talking) {
        lcd_draw_row(y, "NO BMS ACK AT 0x0B (check mux ch3)", 0xF800, 0x0000);
        return;
    }

    snprintf(line, sizeof(line), "Stack:%5umV Cur:%4dmA RSOC:%3u%%",
             d->voltage_mv, (int)d->current_ma, d->rsoc_pct);
    lcd_draw_row(y, line, 0xFFFF, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "C1:%4umV C2:%4umV C3:%4umV C4:%4u",
             d->cell_mv[0], d->cell_mv[1], d->cell_mv[2], d->cell_mv[3]);
    lcd_draw_row(y, line, 0x07E0, 0x0000);
    y += 12;

    uint16_t cmin = d->cell_mv[0], cmax = d->cell_mv[0];
    unsigned sum = d->cell_mv[0];
    for (int i = 1; i < 4; i++) {
        if (d->cell_mv[i] < cmin) cmin = d->cell_mv[i];
        if (d->cell_mv[i] > cmax) cmax = d->cell_mv[i];
        sum += d->cell_mv[i];
    }
    snprintf(line, sizeof(line), "Sum:%5umV  Spread dV:%3umV", sum, (unsigned)(cmax - cmin));
    lcd_draw_row(y, line, 0xFFE0, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "Cyc:%4u Cap:%4umAh DesV:%5umV",
             d->cycle_count, d->design_cap_mah, d->design_volt_mv);
    lcd_draw_row(y, line, 0xBDF7, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "ChgReq:%4umA/%5umV SN:%5u",
             d->chg_current_ma, d->chg_voltage_mv, d->serial_num);
    lcd_draw_row(y, line, 0xBDF7, 0x0000);
    y += 12;

    unsigned init = (d->battery_status >> 7) & 1u;
    unsigned dsg  = (d->battery_status >> 6) & 1u;
    unsigned fd   = (d->battery_status >> 4) & 1u;
    unsigned tca  = (d->battery_status >> 14) & 1u;
    snprintf(line, sizeof(line), "Stat:%04X INIT:%u DSG:%u FD:%u TCA:%u",
             d->battery_status, init, dsg, fd, tca);
    lcd_draw_row(y, line, (fd || tca) ? 0xFD20 : 0x07E0, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "MA(00):%04X  51_ST:%04X  72:%04X",
             d->ma_word, d->pack_status_51, d->cmd72);
    lcd_draw_row(y, line, 0x07FF, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "53:%02X %02X %02X %02X %02X %02X %02X",
             d->blk53[0], d->blk53[1], d->blk53[2], d->blk53[3],
             d->blk53[4], d->blk53[5], d->blk53[6]);
    lcd_draw_row(y, line, 0x07FF, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "54:%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X",
             d->blk54[0], d->blk54[1], d->blk54[2], d->blk54[3],
             d->blk54[4], d->blk54[5], d->blk54[6], d->blk54[7],
             d->blk54[8], d->blk54[9], d->blk54[10], d->blk54[11], d->blk54[12]);
    lcd_draw_row(y, line, 0x07FF, 0x0000);
    y += 12;

    snprintf(line, sizeof(line), "5A:%02X %02X %02X %02X  MfgData:00010302",
             d->blk5a[0], d->blk5a[1], d->blk5a[2], d->blk5a[3]);
    lcd_draw_row(y, line, 0x07FF, 0x0000);
    y += 12;

    if (d->blk73_len > 0) {
        char h73[36];
        size_t p = (size_t)snprintf(h73, sizeof(h73), "73(%u):", d->blk73_len);
        for (uint8_t i = 0; i < d->blk73_len && i < 8 && p + 4 < sizeof(h73); i++) {
            p += (size_t)snprintf(h73 + p, sizeof(h73) - p, "%02X ", d->blk73[i]);
        }
        lcd_draw_row(y, h73, 0xFFE0, 0x0000);
    } else {
        lcd_draw_row(y, "73: unread", 0x7BEF, 0x0000);
    }
    y += 14;

    lcd_draw_row(y, d->state_line, 0xF800, 0x0000);
}
