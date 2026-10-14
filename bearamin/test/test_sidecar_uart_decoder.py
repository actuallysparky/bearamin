#!/usr/bin/env python3

import importlib.util
import pathlib
import struct
import sys


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
DECODER_PATH = PROJECT_ROOT / "tools" / "decode_sidecar_uart.py"

spec = importlib.util.spec_from_file_location("decode_sidecar_uart", DECODER_PATH)
decoder = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = decoder
spec.loader.exec_module(decoder)


def frame(kind, payload):
    raw = bytearray()
    raw.extend(decoder.START)
    raw.append(decoder.VERSION)
    raw.append(kind)
    raw.extend(payload)
    raw.extend(struct.pack("<H", decoder.crc16_ccitt(bytes(raw))))
    return bytes(raw)


def state_payload():
    payload = bytearray(decoder.STATE_PAYLOAD_SIZE)
    struct.pack_into("<H", payload, 0, 42)
    struct.pack_into("<H", payload, 2, (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5))
    struct.pack_into("<I", payload, 4, 123456)
    struct.pack_into("<H", payload, 8, 321)
    struct.pack_into("<H", payload, 10, 654)
    struct.pack_into("<H", payload, 12, 777)
    struct.pack_into("<H", payload, 14, 250)
    struct.pack_into("<H", payload, 16, 750)
    struct.pack_into("<H", payload, 18, 888)
    struct.pack_into("<H", payload, 20, 111)
    struct.pack_into("<H", payload, 22, 222)
    payload[24] = 17
    payload[25] = 2
    struct.pack_into("<H", payload, 26, 333)
    struct.pack_into("<H", payload, 28, 44)
    struct.pack_into("<h", payload, 30, -10)
    struct.pack_into("<h", payload, 32, 20)
    struct.pack_into("<h", payload, 34, 1001)
    payload[36] = 3
    payload[37] = 1
    struct.pack_into("<H", payload, 38, (1 << 1) | (1 << 6))
    struct.pack_into("<H", payload, 40, 0x5300)
    struct.pack_into("<H", payload, 42, 55)
    struct.pack_into("<H", payload, 44, 1)
    struct.pack_into("<H", payload, 46, 2)
    struct.pack_into("<H", payload, 48, 9)
    for index in range(8):
        struct.pack_into("<H", payload, 50 + index * 2, 100 + index)
    struct.pack_into("<H", payload, 66, 1000)
    struct.pack_into("<H", payload, 68, 4)
    struct.pack_into("<H", payload, 70, 2000)
    struct.pack_into("<H", payload, 72, 5)
    struct.pack_into("<H", payload, 74, 6)
    payload[76:84] = bytes([0, 32, 64, 96, 128, 160, 192, 255])
    return bytes(payload)


def test_state_frame_decodes_and_formats():
    parser = decoder.SidecarParser()
    parsed = None
    for byte in b"noise" + frame(decoder.FRAME_STATE, state_payload()):
        parsed = parser.push(byte)
    assert parsed is not None
    assert parsed.kind == decoder.FRAME_STATE
    assert parser.stats.skipped_bytes == 5

    sequence, line = decoder.format_state(parsed.payload, 1000.0, None)
    assert sequence == 42
    assert "uptime=   123.5s" in line
    assert "flags=[dpt_ready,matrix_ready,imu_ready]" in line
    assert "dpt=[matrix_valid,matrix_detected]" in line
    assert "matrix=( 250, 750)" in line
    assert "controls=[1:  0/  0.0%" in line
    assert "8:255/100.0%" in line


def test_crc_error_is_rejected():
    parser = decoder.SidecarParser()
    bad = bytearray(frame(decoder.FRAME_STATE, state_payload()))
    bad[-1] ^= 0xFF
    for byte in bad:
        parsed = parser.push(byte)
    assert parsed is None
    assert parser.stats.crc_errors == 1


def test_matrix_cells_summary():
    payload = bytearray(decoder.MATRIX_CELLS_PAYLOAD_SIZE)
    struct.pack_into("<H", payload, 0, 7)
    payload[2] = 3
    payload[3] = 1
    struct.pack_into("<H", payload, 4, 12)
    for index in range(64):
        struct.pack_into("<H", payload, 6 + index * 2, 0 if index % 5 else 200 + index)

    line = decoder.format_cells(bytes(payload), 1000.0)
    assert "cells seq=    7" in line
    assert "valid= 3 blobs= 1" in line
    assert "range= 200.. 260mm" in line


if __name__ == "__main__":
    test_state_frame_decodes_and_formats()
    test_crc_error_is_rejected()
    test_matrix_cells_summary()
