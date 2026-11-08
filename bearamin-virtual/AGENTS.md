# Bearamin Virtual

- Plan of record: Waveshare RP2040-Zero carrier and firmware/BearaminVirtual2040 for simultaneous CDC/MIDI. Historical C3 variants are separate; never fit both MCU variants.
- Preserve direct 3V3 supply, intentionally omitted R1/R2, unused GP2/VSYS, SDA/SCL GP12/GP9 (I2C0), and LPn/INT GP14/GP13 unless a reviewed hardware change updates the contract.
- Read the protocol and supply documentation before changes. Never run a board generator over saved placement/routing.
- Preserve unrelated work and ignored build/history artifacts. Run ./test.sh and build with the configured Pico SDK; inspect build-side publication behavior before running offline tasks.
- Separate source/build, flash, runtime and physical evidence. Host tests and a compiled installer do not establish physical sensor, MIDI, voltage, timing or flash acceptance.
- Authorized firmware changes require a new version when the bytes change; a version must not identify conflicting payloads. Device writes require exact target verification.
