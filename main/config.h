#pragma once

// Enable comprehensive one-shot Renesas RAJ240045 (045A20) diagnostics by default.
#ifndef BMS_DEBUG_DIAG
#define BMS_DEBUG_DIAG 1
#endif

// ESP32-WROOM-32 pins wired straight to the TCA9548A upstream bus.
// GPIO21 and GPIO22 are not strapping pins.
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// GMT130-V0 240x240 ST7789 IPS SPI display (pins from esp-zhiga-mozgi).
// CS-less module, uses SPI Mode 3.
#define PIN_TFT_MOSI 23
#define PIN_TFT_SCLK 19
#define PIN_TFT_RST  32
#define PIN_TFT_DC   33
#define PIN_TFT_BL   27

// SMBus is specified to 100 kHz. 50 kHz leaves margin for the mux,
// 3k pull-ups, and a gauge that clock-stretches during flash writes.
#define I2C_FREQ_HZ 50000

// TCA9548A with A2:A1:A0 = GND answers at 0x70.
// Channel 3 is the SD3/SC3 pair (mask bit 3).
#define MUX_ADDR_DEFAULT 0x70
#define MUX_CHANNEL 3

// 7-bit SMBus address of a TI SBS gauge.
#define BMS_ADDR 0x0B

// Cells must already be in this window before any permanent-fail latch is cleared.
// A Roomba 900 4S pack at the top of charge is about 4100 mV per cell.
#define CELL_MV_MIN 3000
#define CELL_MV_MAX 4250
#define CELL_SPREAD_MV_MAX 250

// SBS Temperature() is in 0.1 K. These are degrees Celsius.
#define TEMP_C_MIN (-10)
#define TEMP_C_MAX 55

// bq40z50-R2 default keys (TRM SLUUBK0B, section 11.5). High byte must not be 0x00.
// bq20z70/bq20z75 use the same default unseal pair (TRM SLUU310).
#define BQ40_UNSEAL_W0 0x0414
#define BQ40_UNSEAL_W1 0x3672
#define BQ40_FULL_W0 0xFFFF
#define BQ40_FULL_W1 0xFFFF

// bq20z70/bq20z75 permanent-fail clear key (TRM SLUU310, PFKey 0x62).
// The second pair is the byte-swapped form from the same section, tried only if the first leaves PF set.
#define BQ20_PF_W0 0x2673
#define BQ20_PF_W1 0x1712
#define BQ20_PF_W0_ALT 0x7326
#define BQ20_PF_W1_ALT 0x1217

// bq30z55 / bq30z554 default 128-bit SHA-1 key, MSB first (TRM SLUUA79, section 9.5).
#define BQ30_DEFAULT_KEY_INIT \
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, \
    0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10
