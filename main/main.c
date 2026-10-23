#include "bms.h"
#include "config.h"
#include "rl78_tool0.h"
#include "smbus.h"
#include "tft.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "roomba";

static void hang(void) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Roomba 900 BMS unlock, ESP32-WROOM-32");
    ESP_LOGI(TAG, "I2C SDA GPIO%d SCL GPIO%d %u Hz, TCA9548A 0x%02X channel %d (SD3/SC3), gauge 0x%02X",
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ, MUX_ADDR_DEFAULT, MUX_CHANNEL, BMS_ADDR);

    tft_init();
    rl78_tool0_try_unlock();

    if (smbus_init() != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: I2C init failed. No BMS command was sent.");
        hang();
    }

    uint8_t mux = 0;
    if (smbus_find_mux(&mux) != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: TCA9548A not found at 0x70-0x77. No BMS command was sent.");
        ESP_LOGE(TAG, "Check SDA GPIO%d, SCL GPIO%d, 3k pull-ups to 3.3V, and common ground.",
                 PIN_I2C_SDA, PIN_I2C_SCL);
        hang();
    }
    if (smbus_mux_select(mux, MUX_CHANNEL) != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: TCA9548A channel %d select failed. No BMS command was sent.", MUX_CHANNEL);
        hang();
    }

    smbus_log_scan();

    rl78_smbus_boot_service();

    bool talking = false;
    esp_err_t err = bms_unlock(&talking);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stopped: %s", esp_err_to_name(err));
    }
    if (!talking) {
        hang();
    }
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        bms_log_snapshot();
    }
}
