#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
ENGINEERING_ROOT="$PROJECT_ROOT/_local"
ARDUINO_SHARED_ROOT="${ARDUINO_SHARED_ROOT:-$ENGINEERING_ROOT/_cache/esp/arduino-shared}"
LOCAL_ROOT="${LOCAL_ROOT:-$PROJECT_ROOT/_local}"
LOCAL_ARDUINO_ROOT="${ARDUINO_BUILD_ROOT:-$LOCAL_ROOT/arduino/bearamin}"
ARDUINO_CONFIG_DIR="${ARDUINO_CONFIG_DIR:-$ARDUINO_SHARED_ROOT/data}"
ARDUINO_USER_DIR="${ARDUINO_USER_DIR:-$LOCAL_ARDUINO_ROOT/user}"
ARDUINO_BUILD_CACHE_DIR="${ARDUINO_BUILD_CACHE_DIR:-$ARDUINO_SHARED_ROOT/build-cache}"
ARDUINO_BUILD_DIR="${ARDUINO_BUILD_DIR:-$LOCAL_ARDUINO_ROOT/build}"
ARDUINO_OUTPUT_DIR="${ARDUINO_OUTPUT_DIR:-$LOCAL_ARDUINO_ROOT/output}"
LATEST_ROMS_DIR="${LATEST_ROMS_DIR:-$LOCAL_ROOT/latest-roms}"
DPT_PLATFORM_ROOT="${DPT_PLATFORM_ROOT:-$PROJECT_ROOT/_local/dpt-platform}"
DPT_BUILD_DIR="${DPT_BUILD_DIR:-$LOCAL_ROOT/dpt/DPT.multimode/build-qspi}"
BEARAMIN_ALL_COMPONENTS="sidecar sensor-c5 sensor-c5-revc matrixcore matrixcore-revc matrixcore-8x dpt-multimode"
BEARAMIN_COMPONENTS="${BEARAMIN_COMPONENTS:-$BEARAMIN_ALL_COMPONENTS}"
BEARAMIN_SIDECAR_FQBN="${BEARAMIN_SIDECAR_FQBN:-esp32:esp32:XIAO_ESP32S3:PSRAM=opi,LoopCore=1,EventsCore=1}"
BEARAMIN_SENSOR_C5_FQBN="${BEARAMIN_SENSOR_C5_FQBN:-esp32:esp32:XIAO_ESP32C5:PSRAM=enabled,PartitionScheme=default_8MB,CPUFreq=240,CDCOnBoot=cdc}"
BEARAMIN_MATRIXCORE_FQBN="${BEARAMIN_MATRIXCORE_FQBN:-esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=enabled,FlashSize=4M,PartitionScheme=default}"
BEARAMIN_MATRIXCORE_8X_FQBN="${BEARAMIN_MATRIXCORE_8X_FQBN:-esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=enabled,FlashSize=4M,PartitionScheme=min_spiffs}"
BEARAMIN_AMOLED_FQBN="${BEARAMIN_AMOLED_FQBN:-esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PSRAM=opi,FlashSize=16M,PartitionScheme=custom}"
PUBLIC_PATH_FLAGS="-ffile-prefix-map=$HOME=/source -fmacro-prefix-map=$HOME=/source"
VL53_OUTPUT_FLAGS="${VL53_OUTPUT_FLAGS:--DVL53L7CX_DISABLE_AMBIENT_PER_SPAD -DVL53L7CX_DISABLE_NB_SPADS_ENABLED -DVL53L7CX_DISABLE_SIGNAL_PER_SPAD -DVL53L7CX_DISABLE_RANGE_SIGMA_MM -DVL53L7CX_DISABLE_REFLECTANCE_PERCENT -DVL53L7CX_DISABLE_MOTION_INDICATOR -DVL53L8CX_DISABLE_AMBIENT_PER_SPAD -DVL53L8CX_DISABLE_NB_SPADS_ENABLED -DVL53L8CX_DISABLE_NB_TARGET_DETECTED -DVL53L8CX_DISABLE_SIGNAL_PER_SPAD -DVL53L8CX_DISABLE_RANGE_SIGMA_MM -DVL53L8CX_DISABLE_REFLECTANCE_PERCENT -DVL53L8CX_DISABLE_MOTION_INDICATOR}"
VL53L8CX_PATCH="$PROJECT_ROOT/firmware/patches/stm32duino-vl53l8cx-2.1.0-bounded-i2c.patch"
VL53L8CX_READ_PATCH="$PROJECT_ROOT/firmware/patches/stm32duino-vl53l8cx-2.1.0-bounded-i2c-read.patch"
VL53L8CX_WRITE_PATCH="$PROJECT_ROOT/firmware/patches/stm32duino-vl53l8cx-2.1.0-checked-i2c-write.patch"

list_targets() {
  printf '%s\n' \
    bearamin-sidecar-s3 \
    bearamin-sidecar-c5 \
    bearamin-sidecar-c5-revc-s3layout \
    bearamin-matrix-vl53l7cx-s3 \
    bearamin-matrix-vl53l7cx-revc-s3 \
    bearamin-matrix-vl53l8cx-s3 \
    bearamin-dpt-multimode
}

component_for_target() {
  case "$1" in
    bearamin-sidecar-s3) printf '%s\n' sidecar ;;
    bearamin-sidecar-c5) printf '%s\n' sensor-c5 ;;
    bearamin-sidecar-c5-revc-s3layout) printf '%s\n' sensor-c5-revc ;;
    bearamin-matrix-vl53l7cx-s3) printf '%s\n' matrixcore ;;
    bearamin-matrix-vl53l7cx-revc-s3) printf '%s\n' matrixcore-revc ;;
    bearamin-matrix-vl53l8cx-s3) printf '%s\n' matrixcore-8x ;;
    bearamin-dpt-multimode) printf '%s\n' dpt-multimode ;;
    *)
      printf 'Unknown Bearamin target: %s\n' "$1" >&2
      list_targets >&2
      return 2
      ;;
  esac
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      BEARAMIN_COMPONENTS="$BEARAMIN_ALL_COMPONENTS"
      shift
      ;;
    --list-targets)
      list_targets
      exit 0
      ;;
    --target)
      [[ $# -ge 2 ]] || { printf '%s\n' '--target requires a target ID' >&2; exit 2; }
      BEARAMIN_COMPONENTS="$(component_for_target "$2")"
      shift 2
      ;;
    *)
      printf 'Usage: %s [--all | --list-targets | --target TARGET_ID]\n' "$0" >&2
      exit 2
      ;;
  esac
done

has_component() {
  local wanted="$1"
  local component
  for component in $BEARAMIN_COMPONENTS; do
    [[ "$component" == "$wanted" ]] && return 0
  done
  return 1
}

needs_arduino() {
  has_component sidecar || has_component sensor-c5 || has_component sensor-c5-revc || has_component matrixcore || has_component matrixcore-revc ||
    has_component matrixcore-8x || has_component amoled
}

setup_arduino() {
  mkdir -p "$ARDUINO_CONFIG_DIR" "$ARDUINO_USER_DIR" "$ARDUINO_BUILD_CACHE_DIR" "$ARDUINO_BUILD_DIR" "$ARDUINO_OUTPUT_DIR" "$LATEST_ROMS_DIR"
  arduino-cli --config-dir "$ARDUINO_CONFIG_DIR" config init --overwrite >/dev/null
  arduino-cli --config-dir "$ARDUINO_CONFIG_DIR" config set \
    build_cache.path "$ARDUINO_BUILD_CACHE_DIR"
  arduino-cli --config-dir "$ARDUINO_CONFIG_DIR" config set \
    directories.user "$ARDUINO_USER_DIR"
  arduino-cli \
    --config-dir "$ARDUINO_CONFIG_DIR" \
    --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json \
    core update-index
  arduino-cli \
    --config-dir "$ARDUINO_CONFIG_DIR" \
    --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json \
    core install esp32:esp32@3.3.11
  arduino-cli --config-dir "$ARDUINO_CONFIG_DIR" lib install "Adafruit VL53L7CX@1.0.0"
  arduino-cli --config-dir "$ARDUINO_CONFIG_DIR" lib install "STM32duino VL53L8CX@2.1.0"
  arduino-cli --config-dir "$ARDUINO_CONFIG_DIR" lib install "Adafruit NeoPixel@1.11.0"

  local vl53l8cx_header="$ARDUINO_USER_DIR/libraries/STM32duino_VL53L8CX/src/vl53l8cx.h"
  if ! rg -q 'transmission_attempts >= 3' "$vl53l8cx_header"; then
    patch --batch --forward -d "$(dirname "$(dirname "$vl53l8cx_header")")" \
      -p1 < "$VL53L8CX_PATCH"
  fi
  rg -q 'transmission_attempts >= 3' "$vl53l8cx_header" || {
    printf '%s\n' 'Failed to apply the bounded VL53L8CX I2C retry patch' >&2
    return 1
  }
  if ! rg -q 'received_bytes != current_read_size' "$vl53l8cx_header"; then
    patch --batch --forward -d "$(dirname "$(dirname "$vl53l8cx_header")")" \
      -p1 < "$VL53L8CX_READ_PATCH"
  fi
  rg -q 'received_bytes != current_read_size' "$vl53l8cx_header" || {
    printf '%s\n' 'Failed to apply the bounded VL53L8CX I2C read patch' >&2
    return 1
  }
  if ! rg -q 'written_bytes != current_write_size' "$vl53l8cx_header"; then
    patch --batch --forward -d "$(dirname "$(dirname "$vl53l8cx_header")")" \
      -p1 < "$VL53L8CX_WRITE_PATCH"
  fi
  rg -q 'written_bytes != current_write_size' "$vl53l8cx_header" || {
    printf '%s\n' 'Failed to apply the checked VL53L8CX I2C write patch' >&2
    return 1
  }
}

compile_arduino() {
  local component="$1"
  local sketch="$2"
  local fqbn="$3"
  local target_flags="${4:-}"
  local local_library="${5:-}"
  local target_id="${6:-$component}"
  local board_id="${7:-unknown}"
  local sensor_id="${8:-none}"
  local backends="${9:-none}"
  local orientation="${10:-none}"
  local feel_profile="${11:-none}"
  local sketch_dir="$PROJECT_ROOT/firmware/$sketch"
  local build_dir="$ARDUINO_BUILD_DIR/$component"
  local output_dir="$ARDUINO_OUTPUT_DIR/$component"

  mkdir -p "$build_dir" "$output_dir" "$LATEST_ROMS_DIR"
  local library_args=()
  if [[ -n "$local_library" ]]; then
    library_args=(--library "$local_library")
  fi
  arduino-cli \
    --config-dir "$ARDUINO_CONFIG_DIR" \
    compile \
    --fqbn "$fqbn" \
    --build-property "compiler.cpp.extra_flags=$PUBLIC_PATH_FLAGS $VL53_OUTPUT_FLAGS $target_flags" \
    --build-property "compiler.c.extra_flags=$PUBLIC_PATH_FLAGS $VL53_OUTPUT_FLAGS $target_flags" \
    "${library_args[@]}" \
    --build-path "$build_dir" \
    --output-dir "$output_dir" \
    "$sketch_dir"
  cp "$output_dir/$sketch.ino.merged.bin" "$LATEST_ROMS_DIR/$target_id.bin"
  cp "$output_dir/$sketch.ino.bin" \
    "$LATEST_ROMS_DIR/$target_id--app.bin"

  if [[ "$component" == "amoled" ]]; then
    local app_bytes
    app_bytes="$(wc -c < "$output_dir/$sketch.ino.bin" | tr -d ' ')"
    if ((app_bytes > 4194304)); then
      printf 'AMOLED app exceeds a 4 MiB OTA slot: %s bytes\n' "$app_bytes" >&2
      return 1
    fi
    printf 'AMOLED OTA slot usage: %s / 4194304 bytes\n' "$app_bytes"
  fi

  local forbidden=''
  local required=''
  case "$component" in
    sidecar|sensor-c5)
      forbidden='VL53L|Adafruit_NeoPixel|Daisy|Audio'
      ;;
    matrixcore|matrixcore-revc)
      forbidden='VL53L0X|VL6180X|VL53L8CX'
      required='Adafruit_VL53L7CX'
      ;;
    matrixcore-8x)
      forbidden='VL53L0X|VL6180X|Adafruit_VL53L7CX'
      required='VL53L8CX'
      ;;
    amoled)
      forbidden='VL53L0X|VL6180X|VL53L8CX|BearaminSidecar'
      required='Adafruit_VL53L7CX'
      ;;
  esac
  nm -C "$build_dir/$sketch.ino.elf" >"$build_dir/$target_id.symbols.txt"
  if [[ -n "$forbidden" ]] &&
     grep -Eq "$forbidden" "$build_dir/$target_id.symbols.txt"; then
    printf 'Forbidden sensor driver linked into %s (%s)\n' \
      "$component" "$forbidden" >&2
    return 1
  fi
  if [[ -n "$required" ]] &&
     ! grep -Eq "$required" "$build_dir/$target_id.symbols.txt"; then
    printf 'Required sensor driver absent from %s (%s)\n' \
      "$component" "$required" >&2
    return 1
  fi

  write_manifest "$target_id" "$board_id" "$sensor_id" "$backends" \
    "$orientation" "$feel_profile"
}

write_manifest() {
  local target_id="$1"
  local board_id="$2"
  local sensor_id="$3"
  local backends="$4"
  local orientation="$5"
  local feel_profile="$6"
  local orientation_hash
  local core_commit
  local firmware_commit
  local firmware_dirty=false
  if [[ -f "$orientation" ]]; then
    orientation_hash="$(shasum -a 256 "$orientation" | awk '{print $1}')"
  else
    orientation_hash="$(printf '%s' "$orientation" | shasum -a 256 | awk '{print $1}')"
  fi
  core_commit="$(cat "$PROJECT_ROOT/third_party/bearfield-core/REVISION")"
  firmware_commit="$(git -C "$PROJECT_ROOT" rev-parse HEAD)"
  [[ -z "$(git -C "$PROJECT_ROOT" status --porcelain --untracked-files=no -- \
    build.sh firmware third_party/bearfield-core dpt/DPT.multimode)" ]] || \
    firmware_dirty=true
  printf '{\n  "target_id": "%s",\n  "board": "%s",\n  "matrix_sensor": "%s",\n  "backends": "%s",\n  "orientation_hash": "%s",\n  "feel_profile": "%s",\n  "core_commit": "%s",\n  "firmware_commit": "%s",\n  "firmware_dirty": %s\n}\n' \
    "$target_id" "$board_id" "$sensor_id" "$backends" \
    "$orientation_hash" "$feel_profile" "$core_commit" "$firmware_commit" \
    "$firmware_dirty" >"$LATEST_ROMS_DIR/$target_id.manifest.json"
}

if needs_arduino; then
  setup_arduino
fi

if has_component sidecar; then
  compile_arduino sidecar BearaminSidecar "$BEARAMIN_SIDECAR_FQBN" \
    "" "$PROJECT_ROOT/firmware/libraries/BearaminSidecar" \
    bearamin-sidecar-s3 xiao-esp32s3 none wifi-web-ota none none
fi

if has_component sensor-c5; then
  compile_arduino sensor-c5 BearaminSensorC5 "$BEARAMIN_SENSOR_C5_FQBN" \
    "-DBEARAMIN_TARGET_SIDECAR_C5=1" \
    "$PROJECT_ROOT/firmware/libraries/BearaminSidecar" \
    bearamin-sidecar-c5 xiao-esp32c5 none wifi-web-ota none none
fi

if has_component sensor-c5-revc; then
  compile_arduino sensor-c5-revc BearaminSensorC5 "$BEARAMIN_SENSOR_C5_FQBN" \
    "-DBEARAMIN_TARGET_SIDECAR_C5_REVC_S3_LAYOUT=1" \
    "$PROJECT_ROOT/firmware/libraries/BearaminSidecar" \
    bearamin-sidecar-c5-revc-s3layout xiao-esp32c5 none wifi-web-ota none none
fi

if has_component matrixcore; then
  compile_arduino matrixcore BearaminMatrixCore "$BEARAMIN_MATRIXCORE_FQBN" \
    "" "$PROJECT_ROOT/firmware/libraries/BearaminMatrixCore" \
    bearamin-matrix-vl53l7cx-s3 esp32s3-matrix vl53l7cx \
    display-imu-dpt-sidecar \
    "$PROJECT_ROOT/firmware/libraries/BearaminMatrixCore/src/targets/vl53l7cx_s3.h" \
    BearaminV2
fi

if has_component matrixcore-revc; then
  compile_arduino matrixcore-revc BearaminMatrixCore "$BEARAMIN_MATRIXCORE_FQBN" \
    "-DBEARAMIN_TARGET_MATRIX_VL53L7CX_REVC=1" \
    "$PROJECT_ROOT/firmware/libraries/BearaminMatrixCore" \
    bearamin-matrix-vl53l7cx-revc-s3 esp32s3-matrix vl53l7cx \
    display-imu-dpt-sidecar \
    "$PROJECT_ROOT/firmware/libraries/BearaminMatrixCore/src/targets/vl53l7cx_revc_s3.h" \
    DptSensorRevC
fi

if has_component matrixcore-8x; then
  compile_arduino matrixcore-8x BearaminMatrixCore8x "$BEARAMIN_MATRIXCORE_8X_FQBN" \
    "-DBEARAMIN_TARGET_MATRIX_VL53L8CX=1" \
    "$PROJECT_ROOT/firmware/libraries/BearaminMatrixCore" \
    bearamin-matrix-vl53l8cx-s3 esp32s3-matrix vl53l8cx \
    display-imu-dpt-sidecar \
    "$PROJECT_ROOT/firmware/libraries/BearaminMatrixCore/src/targets/vl53l8cx_s3.h" \
    BearaminV2
fi


if has_component dpt-multimode; then
  mkdir -p "$DPT_BUILD_DIR" "$LATEST_ROMS_DIR"
  make -C "$DPT_PLATFORM_ROOT/libDaisy"
  make -C "$PROJECT_ROOT/dpt/DPT.multimode" \
    DPT_PLATFORM_ROOT="$DPT_PLATFORM_ROOT" \
    BEARAMIN_ROOT="$PROJECT_ROOT" \
    BUILD_DIR="$DPT_BUILD_DIR"
  nm -C "$DPT_BUILD_DIR/DPT.multimode.elf" \
    >"$DPT_BUILD_DIR/bearamin-dpt-multimode.symbols.txt"
  if grep -Eq 'VL53L|Adafruit_NeoPixel' \
       "$DPT_BUILD_DIR/bearamin-dpt-multimode.symbols.txt"; then
    printf '%s\n' 'Forbidden sensor/display driver linked into DPT target' >&2
    exit 1
  fi
  cp "$DPT_BUILD_DIR/DPT.multimode.bin" "$LATEST_ROMS_DIR/bearamin-dpt-multimode.bin"
  write_manifest bearamin-dpt-multimode daisy-patch-submodule none \
    audio-cv-gates-presets none none
fi
