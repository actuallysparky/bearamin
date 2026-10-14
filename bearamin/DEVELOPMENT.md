# Bearamin development

Bearamin keeps the realtime gesture path in MatrixCore, audio/CV/gates in the DPT app, and browser/Wi-Fi duties in the sidecar. The DPT’s physical CV8 selector chooses the mode; the browser reflects it.

## Build and test

Install Arduino CLI and an ARM embedded toolchain (`arm-none-eabi-gcc`, `make`). From this folder:

```sh
./setup-dpt.sh
./test.sh
./build.sh --all
```

The bootstrap fetches pinned upstream DPT/libDaisy sources and applies the tracked bootloader and bounded SD patches. It preserves an existing checkout; use `DPT_PLATFORM_ROOT` to select one you have already prepared. ESP32 builds install Arduino ESP32 3.3.11 and the pinned sensor/LED libraries. No network credentials are distributed in the release images. Configure your own sidecar network through its SD configuration, or keep a local ignored `firmware/common/wifi_config.h` for a private build.

Select a target with `./build.sh --list-targets` or `./build.sh --target TARGET`. Generated images and manifests stay in `_local/latest-roms/`. AMOLED is deprecated and is not part of this build.

## Source layout

- `firmware/BearaminMatrixCore/` and `BearaminMatrixCore8x/`: sensor-core entrypoints.
- `firmware/BearaminSidecar/` and `BearaminSensorC5/`: XIAO sidecar entrypoints.
- `firmware/libraries/`: app implementations and target pin maps.
- `firmware/common/`: protocols, gesture/control logic and preset storage.
- `dpt/DPT.multimode/`: Rings/FM audio, CV, gates and presets.
- `third_party/bearfield-core/`: pinned portable depth-field processing.
- `third_party/mutable/`: licensed Rings/stmlib source and provenance.
- `hw/`: current v6 sensor-expander and Rev E sidecar designs.
- `test/`: host and source contracts; `tools/obs_overlay/` supplies the optional observation bridge.

MatrixCore sends CRC-framed control to DPT at 38400 baud and communicates with the sidecar at 460800 baud. The boot default lives in MatrixCore NVS; named presets require the sidecar SD card. All three processors need compatible firmware.

See [wiring](WIRING.md) before connecting hardware and [the family README](../README.md) for credits and licensing. The hardware and firmware releases are beta releases.
