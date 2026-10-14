# Bearamin Sensor Expander

The canonical publication hardware is the v6 sidecar board:

- `bearamin-dpt-sensor-v6.kicad_pro`
- `bearamin-dpt-sensor-v6.kicad_pcb`

The PCB contains the XIAO ESP32-S3 Wi-Fi sidecar, ESP32-S3 MatrixCore,
VL53L7CX 8x8 sensor connection, DPT link, and power components. It has no
retired single-point sensor footprint or connection.

The current board is routed and KiCad reports no unconnected pads. Its current
DRC report contains 14 warnings for local footprint overrides and silkscreen
size/overlap, so review and resolve those items before fabrication.

The v6 release is board-authoritative and does not attach the older v5
schematic. The v5 board, its schematic, earlier revisions, autorouter
intermediates, and generated fabrication exports remain local design history
and are not part of the publication bundle.
