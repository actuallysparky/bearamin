# DPT.multimode

Five-mode firmware for the dsp.coffee DPT. The bottom CV/pot,
`CV_8`, selects five performance apps directionally across the observed
panel-control travel: one end selects slot 1, the other end selects slot 5.

## Modes

1. Modal: the shared Bearamin Rings resonator path using
   `RESONATOR_MODEL_MODAL`.
2. Sympathetic Strings: the existing Bearamin resonator implementation,
   unchanged, using `RESONATOR_MODEL_SYMPATHETIC_STRING`.
3. Karplus-Strong: the shared Bearamin Rings resonator path using the library's
   corresponding physical-string model, `RESONATOR_MODEL_STRING`.

   All three Rings modes use the same stereo-to-mono input, strum and impact
   excitation, brightness/damping/structure/position controls, CV5 reverb,
   C0-C4 frequency, CV7 gesture amount, shared Bearamin lane mappings, web
   feedback/drone/reverb controls, CV outputs, gates, and stereo output path.
   The Rings model selected above is their only processing difference. Stereo
   input is summed to mono, boosted, and fed into the external-input resonator
   path. Gate input 1 injects a short excitation impulse for strums. CV1-CV4
   set brightness, damping, structure, and position. CV5 controls live Rings
   reverb wet amount, and CV6 maps frequency from C0 to C4. A saved,
   default-off quantizer offers Chromatic, Major, and Pentatonic Major. In a
   quantized mode, CV6 alone selects the displayed root/key; Bearamin XYZ
   frequency modulation moves among notes in that rooted scale. Bearamin XYZ can
   digitally offset any of CV1-CV6 from inline web UI axis dropdowns with a
   signed `-100..100%` depth before CV7 applies the global Bearamin gesture
   amount. CV outs 1-3 remain the envelope pair and resonator pitch; CV outs
   4-6 are continuous bipolar Bearamin X/Y/Z. The output is the resonator
   stereo pair, not unity dry pass-through. Resonator-only browser controls can
   also mix in bounded delayed feedback or a pitch-tracking sine or band-limited
   saw drone before the resonator. Reverb time, diffusion, and low-pass are
   separate controls. Each browser parameter has a saved base value plus an
   optional signed Bearamin/CV5 modulation depth.
4. Bearamin FM drone: left audio is a sine carrier, right audio is a triangle
   carrier. CV1 is the first modulation oscillator, scaled into the carriers
   by CV2. CV3 modulates CV1's oscillator rate, scaled by CV4. CV5 sets base
   pitch, CV6 sets stereo spread, and CV7 is the universal Bearamin gesture
   amount for this mode. With CV5 fully down, the base carrier rate is
   sub-audio, around 0.2 Hz. CV outs 1-2 expose the raw modulation oscillators as bipolar
   `-5..+5V` signals, CV out 3 exposes the cooked bipolar modulation signal
   that bends the carriers after both FM amount stages, and CV outs 4-6 expose
   fresh Bearamin X/Y/Z when available so those axes can be patched manually.
   Stereo audio input is summed into the generated carrier outputs at unity.
5. Bearamin CV: receives the external Bearamin sensor head stream over
   the EXP `AUX_1` / `AUX_2` rows at 38400 baud. CV outs 1-3 are unipolar
   3D X, 3D Y, and 3D Z at `0..10 V`; CV outs 4-6 are the same axes as
   bipolar `-5..+5 V` signals. The native DAC path drives CV outs 1-2 while
   the DAC7554 path drives CV outs 3-6. CV4-6 use direct logical voltage
   commands so `-5/0/+5 V` reaches the jacks without an offset. CV3 retains
   its pre-existing `+2 V` compatibility compensation; that mode-owned output
   is deliberately outside the fixed XYZ bank.
   Gate out 1 follows the Bearamin ToF detection gate; gate out 2 follows
   IMU impact. While this mode is selected, the DPT sends a `DPT\n` probe back to
   the sensor head every 500 ms so the reverse UART line can be checked.
   During the existing presence hold, all axes retain their last position.
   After presence ends, X/Y follow the packet centroid as it drifts to center
   and Z relaxes to center at the matrix polling cadence. A stale or lost link
   relaxes all three axes to center, which is `0 V` on CV4-6 and `5 V` on
   CV1-3. The former no-packet jack diagnostics are carried in additive
   `DPTS` status bits instead of consuming CV outputs.
   Gate out 1 / the user LED blink with the mode heartbeat; gate out 2 follows
   fresh IMU impact. Stereo audio input passes through to the outputs at
   unity, aside from optional mode announcements.
## Knob/CV control map

Each `CV_n` entry is the associated DPT pot plus any patched CV at that input.
Most controls use absolute control magnitude, so turning away from center
increases the parameter unless noted. The three Rings modes use the physical
panel knobs' native `0..1` travel directly, so fully counterclockwise is the
true minimum for CV1-CV6. `CV_8` is reserved globally for directional mode
selection across five equal 20% bands.

| Slot | Mode | CV_1 | CV_2 | CV_3 | CV_4 | CV_5 | CV_6 | CV_7 | CV_8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Modal | Brightness plus physical CV offset | Damping plus physical CV offset | Structure plus physical CV offset | Position plus physical CV offset | Reverb wet mix, 0-100% | Frequency C0-C4 | Bearamin gesture amount, CCW none to CW full | Mode select |
| 2 | Sympathetic Strings | Brightness plus physical CV offset | Damping plus physical CV offset | Structure plus physical CV offset | Position plus physical CV offset | Reverb wet mix, 0-100% | Frequency C0-C4 | Bearamin gesture amount, CCW none to CW full | Mode select |
| 3 | Karplus-Strong | Brightness plus physical CV offset | Damping plus physical CV offset | Structure plus physical CV offset | Position plus physical CV offset | Reverb wet mix, 0-100% | Frequency C0-C4 | Bearamin gesture amount, CCW none to CW full | Mode select |
| 4 | Bearamin FM drone | Mod oscillator 1 rate | FM depth from mod 1 into carriers | Mod oscillator 2 rate | FM depth from mod 2 into mod 1 | Base carrier pitch; fully down reaches about 0.2 Hz | Stereo spread, bipolar | Bearamin gesture amount, CCW none to CW full | Mode select |
| 5 | Bearamin CV | unused | unused | unused | unused | unused | unused | unused | Mode select |
## SD card mode announcements

Mode changes can optionally play short voice prompts from the SD card. The
System page's **Announce modes** checkbox controls playback and is saved in
presets. It defaults off, so mode changes are seamless unless the user enables
it. The files should be
48 kHz, 16-bit WAVs. Mono files are duplicated to both outputs; stereo files are
played as-is. Sample-rate conversion is not performed, so record or export the
clips at the firmware audio rate.

Place the clips here:

```text
dpt-modes/bearamin_fm_drone.wav
dpt-modes/modal.wav
dpt-modes/bearamin_resonator.wav
dpt-modes/karplus_strong.wav
dpt-modes/bearamin.wav
```

Missing files or a missing SD card do not stop the firmware; the selected mode
still runs and the announcement is skipped.

## Bearamin bridge wiring

The DPT listens on the observed Bearamin bridge pair:

```text
Matrix GPIO33 / TX             -> DPT EXP AUX_1 / A2 / PA1 / UART4_RX
Matrix GPIO34 / RX             <- DPT EXP AUX_2 / A3 / PA0 / UART4_TX
Matrix GND                     -> DPT GND

Legacy XIAO D6 / GPIO43 / TX   -> DPT EXP AUX_1 / A2 / PA1 / UART4_RX
Legacy XIAO D7 / GPIO44 / RX   <- DPT EXP AUX_2 / A3 / PA0 / UART4_TX
```

The DPT sends a four-byte `DPT\n` diagnostic probe and compact
`DPTS,3,<mode_hex2>,<knobs_hex16>,<flags_hex4>,<resonator_revision_hex2>`
status on the
AUX_2/A3/PA0 return UART once per second. `DPTS` lets the Bearamin web UI reflect
the selected mode and eight CV input positions without sending any label text.
The return path uses hardware UART4 TX on A3; A2 remains UART4 RX and is never
reconfigured as a software or half-duplex transmitter.
The final byte acknowledges the atomically applied resonator web-control
revision. Earlier status formats are intentionally rejected.
Status bits 0-1 retain settings-fresh and Bearamin-fresh; bits 2-6 report RX
line high, low seen, transition seen, recent transition, and packet seen.
The CV bytes are display-normalized from the DPT panel's physical `0..1`
control range, so full counter-clockwise is `00`, noon is about `80`, and full
clockwise is `FF`. This quantization is used only by `DPTS` telemetry and the
web display; synthesis continues to consume the original floating-point
control values.
The Bearamin sensor head streams CV data continuously without needing a command
from the DPT. In all three Rings modes and FM drone, current
Bearamin firmware packs six five-bit lane axis/range mappings across
`reserved0/reserved1/source_flags2` and marks them with `reserved2 == 0x53`:
CV1-CV6 can each choose none, X/Y/Z bipolar, X/Y/Z unipolar, Pitch, Roll, or
Spank. Pitch and Roll are centered bipolar orientation sources. Spank is a
latched `0/1` gate source; it does not replace either physical gate output.
Six additional signed bytes carry exact `-100..100` depth attenuverter values.
The Bearamin web UI can save and load these
mappings as part of its JSON settings snapshot, either via browser
download/upload or on the DPT SD card under `/presets/<name>.json`.
`/presets/default.json` is requested when MatrixCore reports that no sidecar has
initialized controls since its boot, and applied if present; transient read
transfer failures are retried. A sidecar-only reboot imports MatrixCore's live
runtime controls instead of transmitting defaults. DPT reads and writes preset storage incrementally without stopping
audio, while MatrixCore interleaves fresh gesture packets between replies. Each lane depth is
applied to the selected XYZ value first, then CV7 scales that digital offset
from zero to full before clamping. The
physical DPT `CV_1..CV_6` controls already include each knob plus patched jack
CV in the DPT analog/control path; the Bearamin offsets are added after that
combined value in firmware.

When Matrix presence ends, the short dropout hold keeps the last complete XYZ
sample. After the hold expires, X/Y follow the MatrixCore centroid drift while
Z returns directly to the normalized center. Bipolar mappings therefore settle
at neutral before CV7 is raised again; unipolar mappings intentionally settle
at their midpoint.

The bridge packet is version 10. Its 64-byte payload retains the sensor, impact,
and six-lane mapping fields, then carries a one-byte resonator control revision,
a 16-byte resonator control block, the six signed lane depths, Pitch and Roll
12-bit values, and orientation confidence. Spank is carried in a new flag bit.
Earlier bridge versions are intentionally rejected.
The sidecar sends that block to MatrixCore as three revisioned pages, and
MatrixCore publishes it only after every page for one revision has arrived.

In each Rings mode the signal path is dry input plus pre-reverb feedback plus
the optional drone, then a soft limiter, the selected Rings model, and the live
reverb. Feedback always uses the pre-reverb resonator output and is capped at
95%.
The drone follows the resonator's current C0-C4 pitch. Sine and saw are
continuous-phase. A 100% drone setting is scaled to 0.25 full scale before the
resonator.

Feedback amount, drone level, reverb time, reverb diffusion, and reverb
low-pass each resolve as saved base plus signed modulation depth. Sources are
none, Bearamin X/Y/Z bipolar, Bearamin X/Y/Z unipolar, and the physical CV5
control. Bearamin sources still pass through CV7's global gesture amount. CV5
is intentionally offered only for these resonator detail controls and remains
the always-on reverb wet/dry control. Feedback, drone, and reverb parameter
changes are smoothed at about 50 ms to avoid zipper noise.

In all three Rings modes, CV out 1 is a positive envelope follower of the
module audio output, CV out 2 is the negative mirror, CV out 3 is the current
resonator frequency as V/oct relative to C0, and CV outs 4-6 are the raw
continuous bipolar Bearamin X/Y/Z sensor mirrors. The per-lane mappings and
CV7 still affect resonator parameters exactly as before; they no longer
replace the three dedicated axis output jacks.

Performance constraint: status formatting and UART writes stay out of the DPT
audio callback. The audio callback must remain allocation-free and label-free;
mode labels and panel documentation live in the Bearamin browser metadata.
Foreground bridge parsing publishes complete packets through a two-slot
release/acquire snapshot so the audio interrupt cannot observe a partially
copied packet. Versions 2 through 8 remain decodable; pre-v9 packets supply
centered Pitch/Roll and low Spank defaults.

The Cortex-M7 cycle counter continuously records audio callback runtime and
start gaps, plus the longest interrupt-off byte and complete single-wire return
transmission. A compact `DPTP` line is returned every five seconds and forwarded
to sidecar `/status.json`. The retained counters are maxima/totals only; no
allocation, formatting, or UART work occurs in the callback.

## DAC7554 transport and calibration lesson

The DPT's CV3-CV6 DAC7554 accepted valid codes but did not accept the platform
SPI transaction shape used by `WriteCvOutExp()` in this app. Repeated hardware
tests produced nearly identical `~1.1 V` values on all four jacks even when the
firmware requested a static `1/2/3/4 V` staircase. Changing polarity, phase,
blocking behavior, and software-controlled SYNC did not make the hardware SPI
path independent.

A direct GPIO diagnostic established the working wire protocol: clock idle
low, data set before the rising edge, MSB first, and one uninterrupted 16-bit
word while SYNC is low. With 2 us setup/hold intervals, the physical outputs
became clean, independent `1/2/3/4 V` lines. Production multimode firmware now
uses that proven framing on DPT pins D1/SYNC, D10/SCLK, and D9/MOSI from the
2 ms foreground CV service. The audio callback only publishes desired CV
values; it never bit-bangs GPIO or waits on those delays. The shared DPT driver
is unchanged.

Once the transport was correct, idle CV4-CV6 measured `~2 V`. That exposed a
second, independent defect: the legacy `+2 V` expander compensation had been
applied to the Bearamin XYZ bank. CV4-CV6 now bypass that compensation and
clamp directly to `-5..+5 V`; CV3 keeps its existing behavior. Final physical
validation showed zero-centered, continuous, independently moving CV4-CV6.

The reusable diagnostic order is:

1. Send distinct static values to every DAC channel before tuning scale or
   offset. Equal outputs indicate framing, channel addressing, or latch timing.
2. Prove a simple staircase at the physical jacks, then validate `-5/0/+5 V`.
3. Only after channels are independent should calibration compensation be
   inferred or changed.
4. Keep slow peripheral transport in the foreground loop so audio behavior is
   isolated from hardware experiments.

## Build and test

This app is built for the Daisy bootloader by default:

```makefile
APP_TYPE = BOOT_QSPI
```

That keeps the Daisy bootloader in internal flash and places the app at QSPI
address `0x90040000`. The Bearamin root wrapper copies the bootloader-ready
binary to `_local/latest-roms/bearamin-dpt-multimode.bin`.

```sh
cd /path/to/bearamin
./test.sh
./build.sh
```

The 2026-08-10 five-mode build links with the Daisy bootloader/QSPI app
layout. The current build uses 178580 bytes of QSPI app code, 305560 bytes
of 512 KiB SRAM, 18652 bytes of RAM_D2_DMA, and no SDRAM. The live Rings reverb
reuses the already allocated Rings delay buffer. The remaining fit constraint is
normal SRAM/code size rather than the
bootloader-enabled external QSPI app storage.

First-time bootloader installation still requires the STM32 ROM DFU gesture:
hold BOOT, tap RESET, then release BOOT. With this local DPT, verify DFU serial
`200364500000` or `200364500070` and flash the Daisy bootloader to internal
flash:

```sh
dfu-util -d 0483:df11 -S <stm32-rom-dfu-serial> -a 0 \
  -s 0x08000000:leave \
  -D ../dpt/libDaisy/core/dsy_bootloader_v6_3-intdfu-2000ms.bin
```

After the Daisy bootloader is installed, flash this app to QSPI:

```sh
dfu-util -d 0483:df11 -S <daisy-bootloader-dfu-serial> -a 0 \
  -s 0x90040000:leave \
  -D .build/dpt/DPT.multimode/DPT.multimode.bin
```

On 2026-07-19 the Daisy bootloader enumerated as serial `348237613233` with QSPI
layout `@Flash /0x90000000/64*4Kg/0x90040000/60*64Kg/0x90400000/60*64Kg`.
STLink/OpenOCD QSPI flashing is also validated through
`tools/openocd/dpt-daisy-seed-qspi.cfg`:

```sh
openocd -s /path/to/openocd/scripts \
  -f tools/openocd/dpt-daisy-seed-qspi.cfg \
  -c "adapter speed 1000" \
  -c "init" \
  -c "reset halt" \
  -c "dpt_daisy_seed_qspi_init" \
  -c "flash probe 1" \
  -c "flash write_image erase .build/dpt/DPT.multimode/DPT.multimode.bin 0x90040000 bin" \
  -c "verify_image .build/dpt/DPT.multimode/DPT.multimode.bin 0x90040000 bin" \
  -c "reset run" \
  -c "exit"
```

The bootloader also scans the SD card root for a `.bin` file on startup. Keep
mode announcement WAVs under `dpt-modes/`; only place an app `.bin` at the SD
card root when intentionally updating firmware.
