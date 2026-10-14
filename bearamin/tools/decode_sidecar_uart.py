#!/usr/bin/env python3
"""Decode Bearamin MatrixCore <-> sidecar UART frames.

This is intentionally small and self-contained so it can run on the bench
without building the firmware tree.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import BinaryIO, Iterable


START = b"\xBA\x4D"
VERSION = 5

FRAME_STATE = 1
FRAME_COMMAND = 2
FRAME_ACK = 3
FRAME_MATRIX_CELLS = 4
FRAME_PRESET = 5
FRAME_SENSOR_DIAGNOSTICS = 6
FRAME_RUNTIME_SETTINGS = 7
FRAME_MATRIX_PERFORMANCE = 8

STATE_PAYLOAD_SIZE = 90
COMMAND_PAYLOAD_SIZE = 12
SENSOR_DIAGNOSTICS_PAYLOAD_SIZE = 16
RUNTIME_SETTINGS_PAYLOAD_SIZE = 36
MATRIX_PERFORMANCE_PAYLOAD_SIZE = 40
PRESET_LINE_PAYLOAD_SIZE = 96
PRESET_PAYLOAD_SIZE = 4 + PRESET_LINE_PAYLOAD_SIZE
MATRIX_CELLS_PAYLOAD_SIZE = 134

FRAME_SIZES = {
    FRAME_STATE: 2 + 1 + 1 + STATE_PAYLOAD_SIZE + 2,
    FRAME_COMMAND: 2 + 1 + 1 + COMMAND_PAYLOAD_SIZE + 2,
    FRAME_ACK: 2 + 1 + 1 + COMMAND_PAYLOAD_SIZE + 2,
    FRAME_MATRIX_CELLS: 2 + 1 + 1 + MATRIX_CELLS_PAYLOAD_SIZE + 2,
    FRAME_PRESET: 2 + 1 + 1 + PRESET_PAYLOAD_SIZE + 2,
    FRAME_SENSOR_DIAGNOSTICS: 2 + 1 + 1 + SENSOR_DIAGNOSTICS_PAYLOAD_SIZE + 2,
    FRAME_RUNTIME_SETTINGS: 2 + 1 + 1 + RUNTIME_SETTINGS_PAYLOAD_SIZE + 2,
    FRAME_MATRIX_PERFORMANCE: 2 + 1 + 1 + MATRIX_PERFORMANCE_PAYLOAD_SIZE + 2,
}

STATE_FLAGS = {
    0: "dpt_ready",
    3: "matrix_ready",
    4: "matrix_present",
    5: "imu_ready",
    6: "impact_gate",
    7: "matrix_ota_mode",
    8: "matrix_ota_active",
}

DPT_FLAGS = {
    1: "matrix_valid",
    2: "impact_gate",
    4: "gesture_gate",
    5: "stale",
    6: "matrix_detected",
    7: "matrix_warming",
}

COMMAND_NAMES = {
    1: "set_dpt_mode",
    2: "set_control_flags",
    3: "set_cv_slew",
    4: "set_sidecar_status",
    5: "enter_matrix_ota",
    6: "set_engineering_stream",
    7: "set_resonator_web_page",
    8: "set_lane_depths",
    9: "set_mode_announcements",
    10: "request_runtime_settings",
    11: "capture_start",
    12: "capture_stop",
}


@dataclass
class DecodedFrame:
    kind: int
    payload: bytes
    raw: bytes


@dataclass
class ParserStats:
    frames: int = 0
    crc_errors: int = 0
    skipped_bytes: int = 0
    unknown_headers: int = 0


class SidecarParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = ParserStats()

    def push(self, byte: int) -> DecodedFrame | None:
        if not self._buffer and byte != START[0]:
            self.stats.skipped_bytes += 1
            return None

        if len(self._buffer) == 1 and byte != START[1]:
            self._buffer = bytearray([START[0]]) if byte == START[0] else bytearray()
            self.stats.skipped_bytes += 1
            return None

        self._buffer.append(byte)

        if len(self._buffer) == 4:
            if self._buffer[2] != VERSION or self._buffer[3] not in FRAME_SIZES:
                self.stats.unknown_headers += 1
                self._buffer.clear()
                return None

        if len(self._buffer) < 4:
            return None

        expected = FRAME_SIZES.get(self._buffer[3])
        if expected is None or len(self._buffer) < expected:
            return None

        raw = bytes(self._buffer)
        self._buffer.clear()
        expected_crc = u16(raw, expected - 2)
        actual_crc = crc16_ccitt(raw[:-2])
        if expected_crc != actual_crc:
            self.stats.crc_errors += 1
            return None

        self.stats.frames += 1
        return DecodedFrame(kind=raw[3], payload=raw[4:-2], raw=raw)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def flag_names(value: int, names: dict[int, str]) -> str:
    active = [name for bit, name in names.items() if value & (1 << bit)]
    return ",".join(active) if active else "none"


def pct(byte: int) -> str:
    return f"{(byte / 255.0) * 100:5.1f}%"


def compact_controls(values: Iterable[int]) -> str:
    return " ".join(f"{index + 1}:{value:3d}/{pct(value)}" for index, value in enumerate(values))


def format_state(payload: bytes, wall_time: float, previous_sequence: int | None) -> tuple[int, str]:
    sequence = u16(payload, 0)
    state_flags = u16(payload, 2)
    uptime_ms = u32(payload, 4)
    matrix_z_mm = u16(payload, 12)
    matrix_x_milli = u16(payload, 14)
    matrix_y_milli = u16(payload, 16)
    matrix_z_norm = u16(payload, 18)
    matrix_x_norm = u16(payload, 20)
    matrix_y_norm = u16(payload, 22)
    matrix_valid_count = payload[24]
    matrix_blob_count = payload[25]
    impact_velocity_norm = u16(payload, 26)
    impact_age_ms = u16(payload, 28)
    accel_x = i16(payload, 30)
    accel_y = i16(payload, 32)
    accel_z = i16(payload, 34)
    dpt_mode_index = payload[36]
    dpt_protocol_version = payload[37]
    dpt_flags = u16(payload, 38)
    dpt_control_flags = u16(payload, 40)
    sensor_age_ms = u16(payload, 42)
    frame_errors = u16(payload, 44)
    i2c_recoveries = u16(payload, 46)
    dpt_probe_tx_count = u16(payload, 48)
    first_cells = [u16(payload, 50 + index * 2) for index in range(8)]
    dpt_frame_tx_count = u16(payload, 66)
    dpt_preset_tx_count = u16(payload, 68)
    dpt_rx_byte_count = u16(payload, 70)
    dpt_status_line_count = u16(payload, 72)
    dpt_preset_line_count = u16(payload, 74)
    dpt_controls = list(payload[76:84])
    resonator_revision = payload[84]
    pitch_norm = u16(payload, 85)
    roll_norm = u16(payload, 87)
    orientation_confidence = payload[89]

    delta = "first" if previous_sequence is None else str((sequence - previous_sequence) & 0xFFFF)
    line = (
        f"{time.strftime('%H:%M:%S', time.localtime(wall_time))} "
        f"state seq={sequence:5d} delta={delta:>5} uptime={uptime_ms / 1000:8.1f}s "
        f"mode={dpt_mode_index} proto={dpt_protocol_version} "
        f"flags=[{flag_names(state_flags, STATE_FLAGS)}] "
        f"dpt=[{flag_names(dpt_flags, DPT_FLAGS)}] "
        f"matrix=({matrix_x_milli:4d},{matrix_y_milli:4d}) z={matrix_z_mm:4d}mm "
        f"norm=({matrix_x_norm:4d},{matrix_y_norm:4d},{matrix_z_norm:4d}) "
        f"valid={matrix_valid_count:2d} blobs={matrix_blob_count:2d} age={sensor_age_ms:4d}ms "
        f"impact={impact_velocity_norm:4d}/{impact_age_ms:4d}ms "
        f"accel=({accel_x:5d},{accel_y:5d},{accel_z:5d})mg "
        f"controls=[{compact_controls(dpt_controls)}] "
        f"res_rev={resonator_revision} orientation=({pitch_norm},{roll_norm})/{orientation_confidence} "
        f"dpt_counts=tx:{dpt_frame_tx_count} rx_bytes:{dpt_rx_byte_count} "
        f"status:{dpt_status_line_count} preset_tx:{dpt_preset_tx_count} "
        f"preset_lines:{dpt_preset_line_count} probe:{dpt_probe_tx_count} "
        f"errors=sidecar:{frame_errors} i2c:{i2c_recoveries} "
        f"control_flags=0x{dpt_control_flags:04x} cells0-7={first_cells}"
    )
    return sequence, line


def format_command(kind: int, payload: bytes, wall_time: float) -> str:
    sequence = u16(payload, 0)
    command = payload[2]
    args = [u16(payload, offset) for offset in (4, 6, 8, 10)]
    kind_name = "ack" if kind == FRAME_ACK else "command"
    command_name = COMMAND_NAMES.get(command, f"unknown_{command}")
    return (
        f"{time.strftime('%H:%M:%S', time.localtime(wall_time))} "
        f"{kind_name} seq={sequence} command={command_name} args={args}"
    )


def format_cells(payload: bytes, wall_time: float) -> str:
    sequence = u16(payload, 0)
    valid_count = payload[2]
    blob_count = payload[3]
    sensor_age_ms = u16(payload, 4)
    cells = [u16(payload, 6 + index * 2) for index in range(64)]
    nonzero = [value for value in cells if value]
    closest = min(nonzero) if nonzero else 0
    farthest = max(nonzero) if nonzero else 0
    center = [cells[index] for index in (27, 28, 35, 36)]
    return (
        f"{time.strftime('%H:%M:%S', time.localtime(wall_time))} "
        f"cells seq={sequence:5d} valid={valid_count:2d} blobs={blob_count:2d} "
        f"age={sensor_age_ms:4d}ms range={closest:4d}..{farthest:4d}mm "
        f"center={center}"
    )


def format_preset(payload: bytes, wall_time: float) -> str:
    sequence = u16(payload, 0)
    op = payload[2]
    length = min(payload[3], PRESET_LINE_PAYLOAD_SIZE)
    text = payload[4 : 4 + length].decode("utf-8", errors="replace").rstrip()
    return (
        f"{time.strftime('%H:%M:%S', time.localtime(wall_time))} "
        f"preset seq={sequence} op={op} length={length} data={text!r}"
    )


def iter_bytes(stream: BinaryIO, chunk_size: int = 256) -> Iterable[int]:
    while True:
        chunk = stream.read(chunk_size)
        if not chunk:
            return
        yield from chunk


def open_serial(port: str, baud: int) -> BinaryIO:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required for serial ports. Try running with "
            "/Library/Developer/CommandLineTools/usr/bin/python3 or install pyserial."
        ) from exc
    return serial.Serial(port, baudrate=baud, timeout=0.2)


def run(args: argparse.Namespace) -> int:
    parser = SidecarParser()
    previous_state_sequence: int | None = None
    last_state_print = 0.0
    last_cells_print = 0.0

    stream = sys.stdin.buffer if args.port == "-" else open_serial(args.port, args.baud)
    print(
        f"Decoding Bearamin sidecar UART from {'stdin' if args.port == '-' else args.port} "
        f"at {args.baud} baud. Ctrl-C to stop.",
        flush=True,
    )

    try:
        for byte in iter_bytes(stream):
            frame = parser.push(byte)
            if frame is None:
                continue

            now = time.time()
            if frame.kind == FRAME_STATE:
                previous_state_sequence, line = format_state(frame.payload, now, previous_state_sequence)
                if args.all or now - last_state_print >= args.state_interval:
                    print(line, flush=True)
                    last_state_print = now
            elif frame.kind in (FRAME_COMMAND, FRAME_ACK):
                print(format_command(frame.kind, frame.payload, now), flush=True)
            elif frame.kind == FRAME_MATRIX_CELLS:
                if not args.no_cells and (args.all or now - last_cells_print >= args.cells_interval):
                    print(format_cells(frame.payload, now), flush=True)
                    last_cells_print = now
            elif frame.kind == FRAME_PRESET:
                print(format_preset(frame.payload, now), flush=True)
            elif args.all:
                print(
                    f"{time.strftime('%H:%M:%S', time.localtime(now))} "
                    f"frame type={frame.kind} payload_bytes={len(frame.payload)}",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        if stream is not sys.stdin.buffer:
            stream.close()

    print(
        f"Stopped. frames={parser.stats.frames} crc_errors={parser.stats.crc_errors} "
        f"skipped_bytes={parser.stats.skipped_bytes} unknown_headers={parser.stats.unknown_headers}",
        flush=True,
    )
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "port",
        nargs="?",
        default="/dev/cu.wchusbserial20120",
        help="Serial port to read, or '-' for stdin. Default: /dev/cu.wchusbserial20120",
    )
    parser.add_argument("--baud", type=int, default=460800, help="Serial baud rate. Default: 460800")
    parser.add_argument(
        "--state-interval",
        type=float,
        default=0.5,
        help="Seconds between printed state summaries. Default: 0.5",
    )
    parser.add_argument(
        "--cells-interval",
        type=float,
        default=1.0,
        help="Seconds between printed matrix-cell summaries. Default: 1.0",
    )
    parser.add_argument("--all", action="store_true", help="Print every decoded frame.")
    parser.add_argument("--no-cells", action="store_true", help="Suppress matrix-cell frame summaries.")
    return parser


def main() -> int:
    return run(build_arg_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
