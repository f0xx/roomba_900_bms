#pragma once

#include "esp_err.h"

esp_err_t rl78_tool0_try_unlock(void);
void rl78_reset_to_normal(void);
esp_err_t rl78_smbus_boot_service(void);
