#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
CACHE="${ARDUINO_SHARED_ROOT:-$ROOT/_local/arduino}"
mkdir -p "$CACHE/data" "$CACHE/user"
arduino-cli --config-dir "$CACHE/data" config init --overwrite >/dev/null
arduino-cli --config-dir "$CACHE/data" config set directories.user "$CACHE/user"
arduino-cli --config-dir "$CACHE/data" --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json core update-index
arduino-cli --config-dir "$CACHE/data" --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json core install esp32:esp32@3.3.12
for lib in 'ArduinoJson@7.4.3' 'NimBLE-Arduino@2.5.0' 'Adafruit VL53L7CX@1.0.0' 'Adafruit NeoPixel@1.15.5'; do
  arduino-cli --config-dir "$CACHE/data" lib install "$lib"
done
