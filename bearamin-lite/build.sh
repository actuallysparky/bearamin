#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
ENGINEERING="$ROOT/_local"
CACHE="${ARDUINO_SHARED_ROOT:-$ROOT/_local/arduino}"
BUILD="$ROOT/_local/arduino/bearamin-lite-midi"
OUT="$ROOT/_local/latest-roms/bearamin-lite-midi"
mkdir -p "$BUILD" "$OUT"
FQBN='esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=cdc,PSRAM=enabled,FlashSize=4M,PartitionScheme=default'
BUILD_ID="${LITE_BUILD_ID:-$(date -u +%s)}"
VERSION="${LITE_VERSION:-0.2.0-beta.$BUILD_ID}"
[[ "$BUILD_ID" =~ ^[0-9]+$ && "$VERSION" =~ ^[a-zA-Z0-9.+-]+$ ]] || exit 2
arduino-cli --config-dir "$CACHE/data" core list --format json > "$BUILD/core.json"
arduino-cli --config-dir "$CACHE/data" lib list --format json > "$BUILD/libraries.json"
python3 - "$BUILD" "$CACHE" <<'PY'
import json,pathlib,sys
p=pathlib.Path(sys.argv[1]);cache=pathlib.Path(sys.argv[2]);assert 'version=7.4.3' in (cache/'user/libraries/ArduinoJson/library.properties').read_text(), 'Requires ArduinoJson@7.4.3';core=json.loads((p/'core.json').read_text());libs=json.loads((p/'libraries.json').read_text())
assert any(x['id']=='esp32:esp32' and x['releases'].get('3.3.12',{}).get('installed') for x in core['platforms']), 'Requires esp32:esp32@3.3.12'
assert any(x['library']['name']=='Adafruit VL53L7CX' and x['library']['version']=='1.0.0' for x in libs['installed_libraries']), 'Requires Adafruit VL53L7CX@1.0.0'
assert any(x['library']['name']=='Adafruit NeoPixel' and x['library']['version']=='1.15.5' for x in libs['installed_libraries']), 'Requires Adafruit NeoPixel@1.15.5'
assert any(x['library']['name']=='NimBLE-Arduino' and x['library']['version']=='2.5.0' for x in libs['installed_libraries']), 'Requires NimBLE-Arduino@2.5.0'
PY
FLAGS="-ffile-prefix-map=$HOME=/source -fmacro-prefix-map=$HOME=/source -DLITE_BUILD_ID=${BUILD_ID}UL -DLITE_VERSION=\"$VERSION\" -DVL53L7CX_DISABLE_AMBIENT_PER_SPAD -DVL53L7CX_DISABLE_NB_SPADS_ENABLED -DVL53L7CX_DISABLE_SIGNAL_PER_SPAD -DVL53L7CX_DISABLE_RANGE_SIGMA_MM -DVL53L7CX_DISABLE_REFLECTANCE_PERCENT -DVL53L7CX_DISABLE_MOTION_INDICATOR -DCONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED -DCONFIG_BT_NIMBLE_ROLE_OBSERVER_DISABLED -DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=1"
arduino-cli --config-dir "$CACHE/data" compile --fqbn "$FQBN" --libraries "$CACHE/user/libraries" --build-property "compiler.cpp.extra_flags=$FLAGS" --build-property "compiler.c.extra_flags=$FLAGS" --build-path "$BUILD/build" --output-dir "$BUILD/output" "$ROOT/firmware/BearaminLiteMidi"
nm -C "$BUILD/build/BearaminLiteMidi.ino.elf" > "$BUILD/symbols.txt"
if rg -q 'WiFiClass|esp_wifi_init|esp_wifi_start|WebServer::|MDNSResponder::' "$BUILD/symbols.txt"; then echo 'Forbidden Wi-Fi dependency' >&2; exit 1; fi
for symbol in USBMIDI NimBLEServer Adafruit_VL53L7CX; do rg -q "$symbol" "$BUILD/symbols.txt" || exit 1; done
cp "$BUILD/output/BearaminLiteMidi.ino.merged.bin" "$OUT/Bearamin-Lite-MIDI--full.bin"
cp "$BUILD/output/BearaminLiteMidi.ino.bin" "$OUT/Bearamin-Lite-MIDI--app.bin"
cp "$BUILD/output/BearaminLiteMidi.ino.partitions.bin" "$OUT/Bearamin-Lite-MIDI--partitions.bin"
python3 - "$ROOT" "$OUT" "$FQBN" "$BUILD_ID" "$VERSION" <<'PY'
import hashlib,json,pathlib,subprocess,sys
root,out=map(pathlib.Path,sys.argv[1:3]);files={p.name:{'size':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in out.glob('*.bin')}
sources={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['firmware/BearaminLiteMidi','firmware/BearCV','firmware/common','firmware/targets'] for p in sorted((root/folder).rglob('*')) if p.is_file() and p.suffix in {'.ino','.h','.cpp','.c','.csv'}}
manifest=dict(target='bearamin-lite-midi-prototype',hardware='bearcv-dual-mcp4728-v1',fqbn=sys.argv[3],build=int(sys.argv[4]),version=sys.argv[5],usb='TinyUSB MIDI + CDC',ble='NimBLE 2.5.0; bonded BLE MIDI',update=False,core='3.3.12',core_commit=(root/'third_party/bearfield-core/REVISION').read_text().strip(),dependencies={'NimBLE-Arduino':'2.5.0','Adafruit VL53L7CX':'1.0.0','Adafruit NeoPixel':'1.15.5','ArduinoJson':'7.4.3'},commit=subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),source_dirty=bool(subprocess.check_output(['git','-C',str(root),'status','--porcelain','--','firmware','build-lite-midi.sh'],text=True).strip()),files=files,sources=sources,manual_full_flash_offset='0x0',app_offset='0x10000')
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(files,indent=2))
PY
printf 'Beta firmware: %s\n' "$OUT"

echo 'Firmware build complete. The release workflow publishes the verified result.'
