#!/usr/bin/env bash
# Configure the ESP32 target (first run) and compile.
# Uses /usr/bin/idf from Gentoo dev-embedded/esp-idf when idf.py is not on PATH.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/idf-env.sh"
cd "$ROOT"

if [[ ! -f sdkconfig ]]; then
    idf set-target esp32
fi

idf build
echo "Build finished. Flash and watch the log with: scripts/flash.sh /dev/ttyUSB0"
