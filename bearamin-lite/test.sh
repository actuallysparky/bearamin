#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$ROOT/_local/tests"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -I"$ROOT/third_party/ArduinoJson/src" "$ROOT/tests/test_bearamin_lite_midi.cpp" -o "$ROOT/_local/tests/midi"
"$ROOT/_local/tests/midi"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -I"$ROOT/third_party/bearfield-core/include" "$ROOT/test/test_bearcv_firmware.cpp" -o "$ROOT/_local/tests/firmware"
"$ROOT/_local/tests/firmware"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror "$ROOT/tests/test_bearcv_dac_boot_policy.cpp" -o "$ROOT/_local/tests/dac"
"$ROOT/_local/tests/dac"
