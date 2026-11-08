# Bearamin Lite

- This is the standalone optical Eurorack CV controller; it has no DPT or audio-generation dependency.
- Preserve the dual-MCP4728 configuration, sensor processing, output scaling, boot-safe zero behavior, slew, presence hysteresis and Spank hold/release semantics.
- Keep component choices, schematic nets, PCB pads and assembly BOM/CPL consistent. Never regenerate hand-routed hardware as a side effect of a source change.
- Use ./test.sh and inspect ./build.sh --list-targets before building. Disable publishing side effects for an offline/source-only task; do not flash without explicit device authorization.
- A successful firmware build or DAC provisioning receipt does not prove output voltage, rail health, calibration, orientation or physical behavior. Keep those acceptance checks distinct.
- Preserve manufacturing/review lineage and accepted limitations; a prior release or GO is not automatic approval of a changed board.
