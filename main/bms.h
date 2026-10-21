#pragma once

#include <stdbool.h>

#include "esp_err.h"

// One-shot detect and, when the gauge is in permanent-fail, unseal and clear it.
// *talking is set once Voltage() has been read, so the caller can keep logging.
esp_err_t bms_unlock(bool *talking);

// Read-only status line for the serial log after the one-shot attempt.
void bms_log_snapshot(void);
