# Bearamin Virtual

An optical gesture controller for VCV Rack and USB MIDI. The display lives in Rack; the hardware stays small.

A Waveshare RP2040-Zero connects to a VL53L7CX 8×8 depth sensor through the six-pin carrier. It presents USB serial and class-compliant USB MIDI at the same time, so Rack and your DAW can share the controller without a bridge process.

## Play

In VCV Rack 2, add **Bearamin Virtual**, then select its serial port in the module context menu. The module shows the depth field and provides X/Y/Z unipolar and bipolar CV, Presence and Spank outputs.

In your DAW, choose **Bearamin Virtual** as a MIDI input. Defaults: channel 1, CC 20/21/22 for X/Y/Z, CC 23 for Presence, and note 60 for Spank. Virtual uses USB; it has no Bluetooth or Wi-Fi.

Open the [Bearamin Field Guide](https://actuallysparky.github.io/bearamin/) to change MIDI and gesture settings or install the current firmware. After the initial UF2 installation, compatible application updates transfer through MIDI and preserve saved settings. The full UF2 includes the recovery loader; firmware before the new loader requires that initial installation.

## Sources

- `firmware/BearaminVirtual2040/`: Pico SDK firmware, sensor driver and recovery loader.
- `vcv/BearaminVirtual/`: Rack 2 plugin and panel assets.
- `hw/bearamin-virtual/`: current RP2040-Zero carrier, schematic and footprints.

These are beta releases. Sensor wiring is GP12 SDA, GP9 SCL, GP14 LPn and GP13 INT. The six-pin sensor header also carries 3.3 V and ground.

Firmware uses Pico SDK 2.1.1. Rack builds require the Rack 2 SDK and Jansson. MIDI updates verify SHA-256 integrity and accept a trusted USB host; they are not signed updates.

[Family, credits and licensing](../README.md)

## Build

Set `PICO_SDK_PATH` to Pico SDK 2.1.1 with its submodules, then run `./test.sh` and `./build.sh`. Build the Rack plugin with `make -C vcv/BearaminVirtual RACK_DIR=/path/to/Rack-SDK`, followed by its `dist` target.
