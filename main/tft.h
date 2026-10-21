#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool talking;
    uint16_t voltage_mv;
    int16_t current_ma;
    uint16_t rsoc_pct;
    uint16_t chg_current_ma;
    uint16_t chg_voltage_mv;
    uint16_t battery_status;
    uint16_t cycle_count;
    uint16_t design_cap_mah;
    uint16_t design_volt_mv;
    uint16_t serial_num;
    uint16_t ma_word;
    uint16_t cell_mv[4];
    uint16_t pack_status_51;
    uint8_t blk53[7];
    uint8_t blk54[13];
    uint8_t blk5a[4];
    uint16_t cmd72;
    uint8_t blk73[16];
    uint8_t blk73_len;
    char state_line[32];
} bms_display_t;

void tft_init(void);
void tft_update(const bms_display_t *d);
