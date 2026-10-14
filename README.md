# Bearamin

A whole copse of Bearamin friends. I made this family of optical instruments to turn a little motion into a lot of musical possibility.

Move your paw above an 8×8 depth sensor to shape X, Y and Z, find a note, open a gate, or give it a Spank. Pick the Bearamin that fits your patch:

| Bearamin | What it does | What you need |
| --- | --- | --- |
| [Bearamin Lite](bearamin-lite/README.md) | Gesture-controlled Eurorack CV and gates, plus USB and Bluetooth MIDI | Optical VL53L7CX sensor  paired with an ESP32-S3 Matrix |
| [Bearamin Virtual](bearamin-virtual/README.md) | Gesture control for VCV Rack and USB MIDI for your DAW | VL53L7CX sensor and an RP2040-Zero carrier |
| [Bearamin](bearamin/README.md) | A specialist nerd tool for droners: five sound and CV modes, presets, and a browser interface | Sensor head, MatrixCore, Wi-Fi sidecar, and a dsp.coffee DPT |

## Find your sound

The shared **[Bearamin Field Guide](https://actuallysparky.github.io/bearamin/)** hangs out in a cave by Salmon Lake and connects to Bearamin Lite and Bearamin Virtual, helping you map motion to MIDI, get the latest firmware for your device, and generally be cool af. USB MIDI works with Lite and Virtual; Bluetooth MIDI works with Lite. Use a browser that supports Web MIDI, such as Chrome or Edge.

## See Bearamin in action

[![Hands playing Bearamin above a Eurorack system](bearamin/media/bearamin-jazz.png)](bearamin/media/bearamin-jazz.mp4?raw=1)

**[Watch “Bearamin Jazz” (2:29)](bearamin/media/bearamin-jazz.mp4?raw=1)**

## Make a Bearamin

Want one but don't wanna singe your fur with a soldering iron? Stay tuned - built modules available soon from Hive Mind Synthesis. 

Each product folder contains its firmware source and current editable KiCad hardware designs. Virtual also includes its VCV Rack 2 plugin. The releases are **beta releases**; start with the instructions for your hardware and the matching firmware.

Build reports, patches, and musical experiments are welcome. [Open an issue](https://github.com/actuallysparky/bearamin/issues) and tell me what you’re making.

## A deep bow to Mutable Instruments

Bearamin’s resonator modes build on **Émilie Gillet’s extraordinary work at Mutable Instruments**. Rings and stmlib come from [pichenettes/eurorack](https://github.com/pichenettes/eurorack); their original copyright and MIT license notices are retained. Bearamin is an independent derivative project, with enormous gratitude for the generosity and imagination behind that work.

## Full credit to dsp.coffee

The full instrument is built on the **dsp.coffee DPT**, designed and openly shared by **Joe Misra**. It provides the Daisy-based audio engine, CV and gate I/O, and the expansion connection Bearamin uses. Explore the [DPT source](https://github.com/joemisra/dpt) and [dsp.coffee](https://dsp.coffee/).

## Other upstream work

This family also builds on the [Electro-Smith Daisy ecosystem](https://github.com/electro-smith/libDaisy), [Espressif Arduino core](https://github.com/espressif/arduino-esp32), [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), [TinyUSB](https://github.com/hathach/tinyusb), [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino), [ArduinoJson](https://github.com/bblanchon/ArduinoJson), ST’s VL53L7CX sensor driver, and Adafruit’s [VL53L7CX](https://github.com/adafruit/Adafruit_VL53L7CX) and [NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) libraries. Virtual’s predecessor is [BearaminVirtual](https://github.com/actuallysparky/BearaminVirtual).

## License

The existing Bearamin source and hardware use the [MIT License](LICENSE). Third-party files retain their upstream licenses and notices. Virtual’s Rack plugin currently identifies its license as proprietary in `plugin.json`; publication does not change that designation.
