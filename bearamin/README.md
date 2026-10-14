# Bearamin

The full meal deal for droners: a specialist nerd tool with optical gesture control, five sound and CV modes, saved presets, and a browser interface for your Eurorack patch.

A VL53L7CX depth sensor and Waveshare ESP32-S3 Matrix track your hand. A XIAO Wi-Fi sidecar hosts the controls, while the [dsp.coffee DPT](https://github.com/joemisra/dpt) makes the audio, CV and gates.

## Play

CV8 selects Modal, Sympathetic Strings, Karplus-Strong, FM drone, or direct CV mode. The first three voices build on Mutable Instruments Rings. Move above the sensor to shape X/Y/Z, pitch and roll; proximity and impact gestures provide gates. The sidecar browser interface follows the physical mode selector and edits its mappings and sound controls.

Your boot default is saved in MatrixCore. Named presets use the sidecar SD card.

## Sources

- `firmware/`: MatrixCore, XIAO sidecar, shared protocols and supporting target variants.
- `dpt/DPT.multimode/`: Bearamin's DPT application.
- `hw/`: current sidecar, sensor-expander and panel designs.
- `third_party/`: Bearfield processing and licensed Rings/stmlib source.

These are beta releases. AMOLED is deprecated and is excluded from this publication. The Rev E sidecar PCB currently uses its Rev D connectivity sheet.

See [development](DEVELOPMENT.md) and [wiring](WIRING.md) before building. The DPT application requires the external DPT platform, libDaisy, DaisySP and an ARM embedded toolchain. ESP32 targets use Arduino ESP32 3.3.11.

[Family, credits and licensing](../README.md)

## Build

Run `./setup-dpt.sh`, then `./build.sh --all` from this folder. Run `./test.sh` for the host contracts. The bootstrap pins the DPT and libDaisy revisions and applies the included boot/SD patches. Build output stays in `_local/`; provide your own network configuration for the sidecar.
