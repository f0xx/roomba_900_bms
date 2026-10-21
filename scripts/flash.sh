#!/usr/bin/env bash
# Build if needed, flash over the UART adapter, and open the serial log at 115200.
# Usage: scripts/flash.sh [/dev/ttyUSB0]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-${ESPPORT:-/dev/ttyUSB0}}"
# shellcheck disable=SC1091
source "${ROOT}/scripts/idf-env.sh"
cd "$ROOT"

if [[ ! -f sdkconfig ]]; then
    idf set-target esp32
fi

echo "esptool pulses RTS on ${PORT}. On this Prolific cable that pin is not wired to ESP32 EN, so the module stays in the old program until the EN button is pressed."
idf -p "$PORT" -b 115200 flash monitor
