#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
: "${PICO_SDK_PATH:?Set PICO_SDK_PATH to the Pico SDK}"
INSTALLER_PUBLISH="${INSTALLER_PUBLISH:-0}"
[[ "$INSTALLER_PUBLISH" == 0 || "$INSTALLER_PUBLISH" == 1 ]] || { echo 'INSTALLER_PUBLISH must be 0 or 1' >&2; exit 2; }
build_dir_overridden="${BUILD_DIR:+yes}"
BUILD_DIR="${BUILD_DIR:-$ROOT/_local/rp2040-build}"
# Preserve caches made before the canonical project move.
if [[ -z "$build_dir_overridden" && -f "$BUILD_DIR/CMakeCache.txt" ]]; then
  cached_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BUILD_DIR/CMakeCache.txt")"
  if [[ "$cached_source" != "$ROOT/firmware/BearaminVirtual2040" ]]; then
    BUILD_DIR="$ROOT/_local/rp2040-build-canonical"
    echo "Using canonical build cache: $BUILD_DIR; older cache retained."
  fi
fi
cmake -S "$ROOT/firmware/BearaminVirtual2040" -B "$BUILD_DIR" -G Ninja -DPICO_BOARD=pico -DPICO_PLATFORM=rp2040 -DPICO_NO_PICOTOOL=1
cmake --build "$BUILD_DIR" -j
arm-none-eabi-objcopy -O binary "$BUILD_DIR/bearamin_virtual_2040.elf" "$BUILD_DIR/Bearamin-Virtual-MIDI.bin"
arm-none-eabi-objcopy -O binary "$BUILD_DIR/bearamin_virtual_loader.elf" "$BUILD_DIR/bearamin_virtual_loader.bin"
python3 - "$BUILD_DIR" <<'PY_SIZE'
from pathlib import Path
import sys
p=Path(sys.argv[1]);app=(p/'Bearamin-Virtual-MIDI.bin').read_bytes();loader=(p/'bearamin_virtual_loader.bin').read_bytes()
if len(app)>0x100000-0x4000 or len(loader)>0x4000:
    raise SystemExit('Application or recovery loader overlaps reserved flash')
(p/'bearamin_virtual_2040.bin').write_bytes(loader.ljust(0x4000,b'\xff')+app)
PY_SIZE
python3 "$ROOT/firmware/BearaminVirtual2040/bin_to_uf2.py" "$BUILD_DIR/bearamin_virtual_2040.bin" "$BUILD_DIR/bearamin_virtual_2040.uf2" --clear-update-intent

if [[ "$INSTALLER_PUBLISH" == 1 ]]; then
  echo "Publish through the family release workflow after verifying this build." >&2
  exit 2
else
  echo 'Virtual build retained locally; installer publication disabled.'
fi
