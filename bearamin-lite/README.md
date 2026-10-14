# Bearamin Lite

A small optical controller for your Eurorack patch, with USB MIDI and Bluetooth MIDI for the rest of your studio.

Move above the VL53L7CX sensor to control X/Y/Z CV in unipolar and bipolar ranges, Presence and Spank gates. The ESP32-S3 Matrix supplies the depth display; Range and Slew keep the gesture response close at hand. Lite uses two MCP4728 DACs and has no DPT dependency.

## MIDI and settings

Connect USB, or connect Bluetooth MIDI in your computer's MIDI settings, then open the [Bearamin Field Guide](https://actuallysparky.github.io/bearamin/). The guide discovers your device and lets you change MIDI mapping, gesture response, orientation and LED brightness.

Defaults: channel 1, CC 20/21/22 for X/Y/Z, CC 23 for Presence, and note 60 for Spank. Bluetooth discovery opens for two minutes after startup; resetting pairings opens another two-minute window. Lite has no Wi-Fi interface.

## Sources

`firmware/BearaminLiteMidi/` contains the MIDI firmware; its production CV behavior shares `firmware/BearCV/`, `firmware/common/`, `firmware/targets/` and Bearfield processing. `hw/bearcv-10hp/` contains the current Bourns Rev C main board, schematic and matching faceplate.

These are beta releases. Rev C retains its recorded potentiometer-cutout clearance issue; review that geometry before ordering a board.

The MIDI build requires Arduino ESP32 3.3.12, ArduinoJson 7.4.3, NimBLE-Arduino 2.5.0, Adafruit VL53L7CX 1.0.0 and NeoPixel 1.15.5. First installation replaces internal settings and Bluetooth pairings.

[Family, credits and licensing](../README.md)

## Build

With Arduino CLI installed, run `./setup.sh`, `./test.sh`, then `./build.sh` from this folder. The setup installs the pinned dependencies; images and build metadata stay in `_local/latest-roms/bearamin-lite-midi/`.
