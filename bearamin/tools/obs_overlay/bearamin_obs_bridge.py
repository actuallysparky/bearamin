#!/usr/bin/env python3
"""Receive Bearamin UDP telemetry and serve a localhost-only OBS overlay."""

from __future__ import annotations

import argparse
import http.server
import json
import pathlib
import queue
import random
import signal
import socket
import struct
import threading
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Any

MAGIC = b"BOV1"
VERSION_V3 = 3
VERSION_V4 = 4
VERSION = VERSION_V3
PACKET_SUBSCRIBE = 1
PACKET_STATE = 2
PACKET_CELLS = 3
STREAM_STATE = 1
STREAM_CELLS = 2
DEVICE_PORT = 51337
MULTICAST_GROUP = "239.255.77.66"
MULTICAST_PORT = 51338
HTTP_PORT = 8765
DEFAULT_DEVICE_HOSTS = ("bearamin.lan", "bearamin-c5.lan")
TRUE_GRID_SPACE = "bearamin_true_grid_v1"
# This is the installed OBS display profile. It is deliberately source-owned:
# updating it requires deploying and restarting this bridge, never a browser
# URL parameter or an operator calibration control.
TRUE_GRID_TO_OBS = {
    "rotation_quarters": 0,
    "mirror_x": False,
    "mirror_y": True,
}
HEADER = struct.Struct("<4sBBHHH")
SUBSCRIBE_PAYLOAD = struct.Struct("<B3xI")
STATE_PAYLOAD = struct.Struct("<I12H2B")
STATE_PAYLOAD_V4 = struct.Struct("<I12H2B6H")
CELLS_PAYLOAD = struct.Struct("<IH2B64H")
SUBSCRIBE_SIZE = HEADER.size + SUBSCRIBE_PAYLOAD.size + 2
STATE_SIZE = HEADER.size + STATE_PAYLOAD.size + 2
STATE_SIZE_V4 = HEADER.size + STATE_PAYLOAD_V4.size + 2
CELLS_SIZE = HEADER.size + CELLS_PAYLOAD.size + 2
VALID_MODES = {"crosshair", "dots", "combined", "mappings"}
MAPPING_VALUE_MODES = {"modulation", "total"}
GESTURE_AMOUNT_LOW_DEAD_ZONE = 0.025
MAPPING_DEPTH_GAIN = 0.03
MODE_MAPPINGS = (
    {
        "name": "Modal",
        "key": "rings_resonator",
        "gain": 1.0 / 3.0,
        "lanes": ("Brightness", "Damping", "Structure", "Position", "Reverb mix", "Frequency"),
    },
    {
        "name": "Sympathetic Strings",
        "key": "rings_resonator",
        "gain": 1.0 / 3.0,
        "lanes": ("Brightness", "Damping", "Structure", "Position", "Reverb mix", "Frequency"),
    },
    {
        "name": "Karplus-Strong",
        "key": "rings_resonator",
        "gain": 1.0 / 3.0,
        "lanes": ("Brightness", "Damping", "Structure", "Position", "Reverb mix", "Frequency"),
    },
    {
        "name": "Bearamin FM drone",
        "key": "bearamin_fm",
        "gain": 2.0 / 3.0,
        "lanes": ("Mod 1 rate", "FM1 depth", "Mod 2 rate", "FM2 depth", "Base pitch", "Stereo spread"),
    },
    {
        "name": "Bearamin CV",
        "key": None,
        "gain": 0.0,
        "lanes": ("Unused", "Unused", "Unused", "Unused", "Unused", "Unused"),
    },
)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_subscribe(
    streams: int, sequence: int, nonce: int, version: int = VERSION_V3
) -> bytes:
    if version not in {VERSION_V3, VERSION_V4}:
        raise ValueError("unsupported subscription version")
    payload = SUBSCRIBE_PAYLOAD.pack(streams, nonce)
    packet = HEADER.pack(MAGIC, version, PACKET_SUBSCRIBE, len(payload), sequence, 0) + payload
    return packet + struct.pack("<H", crc16_ccitt(packet))


def _validate_packet(
    data: bytes, packet_type: int, payload_size: int, expected_version: int
) -> tuple[int, bytes]:
    expected_size = HEADER.size + payload_size + 2
    if len(data) != expected_size:
        raise ValueError(f"wrong packet size {len(data)} != {expected_size}")
    magic, version, actual_type, actual_payload, sequence, _flags = HEADER.unpack_from(data)
    if magic != MAGIC or version not in {VERSION_V3, VERSION_V4} or actual_type != packet_type:
        raise ValueError("wrong packet identity")
    if actual_payload != payload_size:
        raise ValueError("wrong payload size")
    if crc16_ccitt(data[:-2]) != struct.unpack_from("<H", data, len(data) - 2)[0]:
        raise ValueError("CRC mismatch")
    if version != expected_version:
        raise ValueError("wrong packet version")
    return sequence, data[HEADER.size:-2]


def parse_packet(data: bytes) -> tuple[str, int, dict[str, Any]]:
    if len(data) < HEADER.size:
        raise ValueError("short packet")
    version = data[4]
    packet_type = data[5]
    if packet_type == PACKET_STATE:
        payload_struct = STATE_PAYLOAD_V4 if version == VERSION_V4 else STATE_PAYLOAD
        sequence, payload = _validate_packet(
            data, PACKET_STATE, payload_struct.size, version
        )
        values = payload_struct.unpack(payload)
        keys = (
            "source_uptime_ms", "state_flags", "sensor_age_ms",
            "matrix_x_norm", "matrix_y_norm", "matrix_z_norm",
            "matrix_x_milli", "matrix_y_milli", "matrix_z_mm",
            "impact_velocity_norm", "impact_age_ms", "pitch_norm",
            "roll_norm", "matrix_valid_count", "matrix_blob_count",
        )
        decoded = dict(zip(keys, values))
        decoded["protocol_version"] = version
        if version == VERSION_V4:
            decoded["trajectory_start_norm"] = list(values[15:20])
            decoded["slew_tau_ms"] = values[20]
            if not 43 <= decoded["slew_tau_ms"] <= 420:
                raise ValueError("invalid trajectory tau")
        else:
            decoded["trajectory_start_norm"] = [
                decoded["matrix_x_norm"], decoded["matrix_y_norm"],
                decoded["matrix_z_norm"], decoded["pitch_norm"],
                decoded["roll_norm"],
            ]
            decoded["slew_tau_ms"] = None
        decoded["matrix_ready"] = bool(decoded["state_flags"] & (1 << 3))
        decoded["matrix_present"] = bool(decoded["state_flags"] & (1 << 4))
        decoded["impact_gate"] = bool(decoded["state_flags"] & (1 << 6))
        decoded["spank_gate"] = bool(decoded["state_flags"] & (1 << 12))
        return "state", sequence, decoded
    if packet_type == PACKET_CELLS:
        sequence, payload = _validate_packet(
            data, PACKET_CELLS, CELLS_PAYLOAD.size, version
        )
        values = CELLS_PAYLOAD.unpack(payload)
        decoded = {
            "source_uptime_ms": values[0],
            "sensor_age_ms": values[1],
            "matrix_valid_count": values[2],
            "matrix_blob_count": values[3],
            "cells_mm": list(values[4:]),
            "protocol_version": version,
        }
        return "cells", sequence, decoded
    raise ValueError(f"unsupported packet type {packet_type}")


def clamp(value: float, minimum: float = 0.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


def transform_true_grid_point(
    x: float,
    y: float,
    profile: dict[str, Any] = TRUE_GRID_TO_OBS,
) -> tuple[float, float]:
    """Rotate clockwise around the grid centre, then mirror local axes."""
    input_x = clamp(x)
    input_y = clamp(y)
    rotation = int(profile["rotation_quarters"])
    if rotation == 1:
        x, y = 1.0 - input_y, input_x
    elif rotation == 2:
        x, y = 1.0 - input_x, 1.0 - input_y
    elif rotation == 3:
        x, y = input_y, 1.0 - input_x
    else:
        x, y = input_x, input_y
    if profile["mirror_x"]:
        x = 1.0 - x
    if profile["mirror_y"]:
        y = 1.0 - y
    return x, y


def transform_true_grid_cell(
    x: int,
    y: int,
    profile: dict[str, Any] = TRUE_GRID_TO_OBS,
) -> tuple[int, int]:
    """Apply the shared transform to a true-grid 8x8 cell coordinate."""
    rotation = int(profile["rotation_quarters"])
    if rotation == 1:
        x, y = 7 - y, x
    elif rotation == 2:
        x, y = 7 - x, 7 - y
    elif rotation == 3:
        x, y = y, 7 - x
    if profile["mirror_x"]:
        x = 7 - x
    if profile["mirror_y"]:
        y = 7 - y
    return x, y


def gesture_amount(knob_byte: Any) -> float:
    value = clamp(float(knob_byte) / 255.0)
    if value <= GESTURE_AMOUNT_LOW_DEAD_ZONE:
        return 0.0
    return (value - GESTURE_AMOUNT_LOW_DEAD_ZONE) / (
        1.0 - GESTURE_AMOUNT_LOW_DEAD_ZONE
    )


def mapping_source_value(state: dict[str, Any], source: str) -> float:
    if source == "spank":
        return 1.0 if state.get("spank_gate") else 0.0
    unipolar = source.endswith("_unipolar")
    axis = source.removesuffix("_unipolar").removesuffix("_bipolar")
    field = {
        "x": "matrix_x_norm",
        "y": "matrix_y_norm",
        "z": "matrix_z_norm",
        "pitch": "pitch_norm",
        "roll": "roll_norm",
    }.get(axis)
    if not field:
        return 0.0
    unit = clamp(float(state.get(field, 2048)) / 4095.0)
    return unit if unipolar else clamp(unit * 2.0 - 1.0, -1.0, 1.0)


def build_mapping_snapshot(
    status: dict[str, Any],
    state: dict[str, Any],
    value_mode: str,
) -> dict[str, Any]:
    mode_index = int(status.get("dpt_mode", 0))
    if mode_index < 0 or mode_index >= len(MODE_MAPPINGS):
        mode_index = 0
    mode = MODE_MAPPINGS[mode_index]
    knobs = list(status.get("dpt_knobs") or [])
    knobs.extend([0] * (8 - len(knobs)))
    amount = gesture_amount(knobs[6])
    mapping_key = mode["key"]
    lanes = []
    for index, name in enumerate(mode["lanes"]):
        source = "none"
        depth = 0
        if mapping_key:
            prefix = f"map_{mapping_key}_{index}"
            source = str(status.get(f"{prefix}_axis", "none"))
            try:
                depth = int(status.get(f"{prefix}_depth", 0))
            except (TypeError, ValueError):
                depth = 0
        depth = max(-100, min(100, depth))
        base = clamp(float(knobs[index]) / 255.0)
        raw_modulation = (
            mapping_source_value(state, source)
            * depth
            * MAPPING_DEPTH_GAIN
            * amount
            * float(mode["gain"])
        )
        total = clamp(base + raw_modulation)
        modulation = total - base
        lanes.append(
            {
                "index": index,
                "name": name,
                "source": source,
                "depth_percent": depth,
                "base": round(base, 6),
                "modulation": round(modulation, 6),
                "total": round(total, 6),
                "display_value": round(
                    total if value_mode == "total" else modulation, 6
                ),
            }
        )
    return {
        "mode_index": mode_index,
        "mode_name": mode["name"],
        "mapping_key": mapping_key,
        "mode_gain": float(mode["gain"]),
        "mapping_depth_gain": MAPPING_DEPTH_GAIN,
        "value_mode": value_mode,
        "gesture_amount": round(amount, 6),
        "dpt_ready": bool(status.get("dpt_ready")),
        "core_link_stale": bool(status.get("core_link_stale", True)),
        "status_received_unix_ms": status.get("_mapping_status_received_unix_ms", 0),
        "state_received_unix_ms": state.get("received_unix_ms", 0),
        # Keep the producer keyframe intact.  The browser evaluates v4 once per
        # animation frame; the precomputed lane values above remain the v3
        # fallback for older producers.
        "protocol_version": state.get("protocol_version", 3),
        "matrix_x_norm": state.get("matrix_x_norm", 2048),
        "matrix_y_norm": state.get("matrix_y_norm", 2048),
        "matrix_z_norm": state.get("matrix_z_norm", 2048),
        "pitch_norm": state.get("pitch_norm", 2048),
        "roll_norm": state.get("roll_norm", 2048),
        "trajectory_start_norm": state.get("trajectory_start_norm"),
        "slew_tau_ms": state.get("slew_tau_ms"),
        "sensor_age_ms": state.get("sensor_age_ms", 0),
        "matrix_present": bool(state.get("matrix_present")),
        "spank_gate": bool(state.get("spank_gate")),
        "lanes": lanes,
    }


@dataclass
class Client:
    mode: str
    events: queue.Queue[tuple[str, dict[str, Any]]] = field(
        default_factory=lambda: queue.Queue(maxsize=4)
    )


class BridgeState:
    def __init__(
        self,
        multicast_group: str = MULTICAST_GROUP,
        multicast_port: int = MULTICAST_PORT,
        mapping_value_mode: str = "modulation",
    ) -> None:
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.clients: dict[int, Client] = {}
        self.next_client_id = 1
        self.latest: dict[str, dict[str, Any]] = {}
        self.last_packet_at: dict[str, float] = {}
        self.last_sequence: dict[str, int] = {}
        self.sequence_gaps: dict[str, int] = {"state": 0, "cells": 0}
        self.packet_counts: dict[str, int] = {"state": 0, "cells": 0}
        self.malformed_packets = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.heartbeats_sent = 0
        self.resolve_failures = 0
        self.send_failures = 0
        self.source_rejections = 0
        self.multicast_joined = False
        self.multicast_join_failures = 0
        self.multicast_group = multicast_group
        self.multicast_port = multicast_port
        self.device_hosts: tuple[str, ...] = ()
        self.device_addresses: dict[str, str] = {}
        self.device_address = ""
        self.active_source_host = ""
        self.active_source_address = ""
        self.source_switches = 0
        self.mapping_value_mode = mapping_value_mode
        self.mapping_status: dict[str, Any] = {}
        self.mapping_status_fetches = 0
        self.mapping_status_errors = 0

    def add_client(self, mode: str) -> tuple[int, Client]:
        if mode not in VALID_MODES:
            mode = "crosshair"
        with self.lock:
            client_id = self.next_client_id
            self.next_client_id += 1
            client = Client(mode)
            self.clients[client_id] = client
            latest = dict(self.latest)
        for event_type, payload in latest.items():
            if self.client_wants(client, event_type):
                client.events.put_nowait((event_type, payload))
        return client_id, client

    def remove_client(self, client_id: int) -> None:
        with self.lock:
            self.clients.pop(client_id, None)

    @staticmethod
    def client_wants(client: Client, event_type: str) -> bool:
        return (event_type == "state" and client.mode != "mappings") or (
            event_type == "cells" and client.mode in {"dots", "combined"}
        ) or (
            event_type == "mappings" and client.mode == "mappings"
        )

    @staticmethod
    def enqueue(client: Client, event_type: str, payload: dict[str, Any]) -> None:
        try:
            client.events.put_nowait((event_type, payload))
        except queue.Full:
            try:
                client.events.get_nowait()
            except queue.Empty:
                pass
            client.events.put_nowait((event_type, payload))

    def has_mapping_clients(self) -> bool:
        with self.lock:
            return any(client.mode == "mappings" for client in self.clients.values())

    def requested_streams(self) -> int:
        with self.lock:
            modes = {client.mode for client in self.clients.values()}
        if not modes:
            return 0
        streams = STREAM_STATE
        if modes & {"dots", "combined"}:
            streams |= STREAM_CELLS
        return streams

    def configure_sources(self, device_hosts: tuple[str, ...]) -> None:
        with self.lock:
            self.device_hosts = device_hosts

    def update_device_addresses(
        self, addresses: dict[str, tuple[str, int]]
    ) -> None:
        formatted = {
            host: f"{address[0]}:{address[1]}"
            for host, address in addresses.items()
        }
        with self.lock:
            self.device_addresses = formatted
            if self.active_source_host in formatted:
                self.device_address = formatted[self.active_source_host]

    def active_status_url(self) -> str:
        with self.lock:
            if not self.active_source_address:
                return ""
            return f"http://{self.active_source_address}/status.json"

    def record_packet(
        self,
        event_type: str,
        sequence: int,
        payload: dict[str, Any],
        source_host: str = "",
        source_address: str = "",
    ) -> None:
        now = time.monotonic()
        payload = dict(payload)
        payload["received_unix_ms"] = int(time.time() * 1000)
        payload["sequence"] = sequence
        with self.lock:
            source_changed = bool(
                source_host and source_host != self.active_source_host
            )
            if source_changed:
                if self.active_source_host:
                    self.source_switches += 1
                self.active_source_host = source_host
                self.active_source_address = source_address
                self.device_address = self.device_addresses.get(
                    source_host, source_address
                )
                # A mapping status belongs to one producer. Do not combine it
                # with a newly selected BOV1 source while the next poll is due.
                self.mapping_status = {}
            previous = self.last_sequence.get(event_type)
            previous_source = self.last_sequence.get(f"{event_type}_source")
            if previous is not None and previous_source == source_host:
                delta = (sequence - previous) & 0xFFFF
                if delta > 1:
                    self.sequence_gaps[event_type] += delta - 1
            self.last_sequence[event_type] = sequence
            self.last_sequence[f"{event_type}_source"] = source_host
            self.latest[event_type] = payload
            self.last_packet_at[event_type] = now
            self.packet_counts[event_type] += 1
            clients = list(self.clients.values())
            mapping_payload = None
            if event_type == "state" and self.mapping_status:
                mapping_payload = build_mapping_snapshot(
                    self.mapping_status, payload, self.mapping_value_mode
                )
                mapping_payload["received_unix_ms"] = payload["received_unix_ms"]
                self.latest["mappings"] = mapping_payload
                self.last_packet_at["mappings"] = now
        for client in clients:
            if self.client_wants(client, event_type):
                self.enqueue(client, event_type, payload)
            if mapping_payload is not None and client.mode == "mappings":
                self.enqueue(client, "mappings", mapping_payload)

    def record_mapping_status(self, status: dict[str, Any]) -> None:
        status = dict(status)
        status["_mapping_status_received_unix_ms"] = int(time.time() * 1000)
        now = time.monotonic()
        with self.lock:
            self.mapping_status = status
            self.mapping_status_fetches += 1
            current_state = self.latest.get("state")
            clients = list(self.clients.values())
            mapping_payload = None
            if current_state:
                mapping_payload = build_mapping_snapshot(
                    status, current_state, self.mapping_value_mode
                )
                mapping_payload["received_unix_ms"] = int(time.time() * 1000)
                self.latest["mappings"] = mapping_payload
                self.last_packet_at["mappings"] = now
        if mapping_payload is not None:
            for client in clients:
                if client.mode == "mappings":
                    self.enqueue(client, "mappings", mapping_payload)

    def record_mapping_status_error(self) -> None:
        with self.lock:
            self.mapping_status_errors += 1

    def health(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            ages = {
                key: round((now - received) * 1000)
                for key, received in self.last_packet_at.items()
            }
            stale = ages.get("state", 1000000) > 500
            modes = {mode: 0 for mode in sorted(VALID_MODES)}
            for client in self.clients.values():
                modes[client.mode] += 1
            return {
                "ok": not stale,
                "stale": stale,
                "coordinate_space": TRUE_GRID_SPACE,
                "true_grid_to_local": dict(TRUE_GRID_TO_OBS),
                "uptime_ms": round((now - self.started) * 1000),
                "device_address": self.device_address,
                "device_hosts": list(self.device_hosts),
                "device_addresses": dict(self.device_addresses),
                "active_source_host": self.active_source_host,
                "active_source_address": self.active_source_address,
                "source_switches": self.source_switches,
                "active_browser_clients": len(self.clients),
                "browser_modes": modes,
                "requested_streams": self.requested_streams_unlocked(),
                "packet_age_ms": ages,
                "packet_counts": dict(self.packet_counts),
                "sequence_gaps": dict(self.sequence_gaps),
                "malformed_packets": self.malformed_packets,
                "crc_errors": self.crc_errors,
                "format_errors": self.format_errors,
                "heartbeats_sent": self.heartbeats_sent,
                "resolve_failures": self.resolve_failures,
                "send_failures": self.send_failures,
                "source_rejections": self.source_rejections,
                "multicast_group": self.multicast_group,
                "multicast_port": self.multicast_port,
                "multicast_joined": self.multicast_joined,
                "multicast_join_failures": self.multicast_join_failures,
                "mapping_value_mode": self.mapping_value_mode,
                "mapping_status_fetches": self.mapping_status_fetches,
                "mapping_status_errors": self.mapping_status_errors,
            }

    def requested_streams_unlocked(self) -> int:
        if not self.clients:
            return 0
        streams = STREAM_STATE
        if any(client.mode in {"dots", "combined"} for client in self.clients.values()):
            streams |= STREAM_CELLS
        return streams


class UdpWorker(threading.Thread):
    def __init__(
        self,
        state: BridgeState,
        stop_event: threading.Event,
        device_hosts: tuple[str, ...] | list[str] | str,
        device_port: int,
        multicast_group: str,
        multicast_port: int,
    ) -> None:
        super().__init__(name="bearamin-udp", daemon=True)
        self.state = state
        self.stop_event = stop_event
        self.device_hosts = (
            (device_hosts,) if isinstance(device_hosts, str) else tuple(device_hosts)
        )
        self.device_port = device_port
        self.multicast_group = multicast_group
        self.multicast_port = multicast_port
        self.nonce = random.SystemRandom().getrandbits(32)
        self.sequence = 0
        self.addresses: dict[str, tuple[str, int]] = {}
        self.state.configure_sources(self.device_hosts)

    def resolve_all(self) -> dict[str, tuple[str, int]]:
        addresses: dict[str, tuple[str, int]] = {}
        for host in self.device_hosts:
            try:
                addresses[host] = socket.getaddrinfo(
                    host, self.device_port, socket.AF_INET, socket.SOCK_DGRAM
                )[0][4]
            except OSError:
                self.state.resolve_failures += 1
        self.addresses = addresses
        self.state.update_device_addresses(addresses)
        return addresses

    def join_multicast(self, udp: socket.socket) -> bool:
        try:
            membership = (
                socket.inet_aton(self.multicast_group) +
                socket.inet_aton("0.0.0.0")
            )
            udp.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                membership,
            )
            self.state.multicast_joined = True
            return True
        except OSError:
            self.state.multicast_joined = False
            self.state.multicast_join_failures += 1
            return False

    def leave_multicast(self, udp: socket.socket) -> None:
        try:
            membership = (
                socket.inet_aton(self.multicast_group) +
                socket.inet_aton("0.0.0.0")
            )
            udp.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_DROP_MEMBERSHIP,
                membership,
            )
        except OSError:
            pass
        self.state.multicast_joined = False

    @staticmethod
    def source_allowed(
        addresses: dict[str, tuple[str, int]],
        source: tuple[str, int],
    ) -> bool:
        return any(address[0] == source[0] for address in addresses.values())

    def source_host(self, source: tuple[str, int]) -> str:
        for host, address in self.addresses.items():
            if address[0] == source[0]:
                return host
        return source[0]

    def send_heartbeats(self, udp: socket.socket, streams: int) -> None:
        for address in self.addresses.values():
            try:
                for version in (VERSION_V3, VERSION_V4):
                    udp.sendto(
                        encode_subscribe(streams, self.sequence, self.nonce, version),
                        address,
                    )
                    self.sequence = (self.sequence + 1) & 0xFFFF
                    self.state.heartbeats_sent += 1
            except OSError:
                self.state.send_failures += 1

    def run(self) -> None:
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind(("0.0.0.0", self.multicast_port))
        udp.settimeout(0.1)
        multicast_joined = self.join_multicast(udp)
        next_resolve = 0.0
        next_multicast_join = 0.0
        next_heartbeat = 0.0
        while not self.stop_event.is_set():
            now = time.monotonic()
            streams = self.state.requested_streams()
            if not multicast_joined and now >= next_multicast_join:
                multicast_joined = self.join_multicast(udp)
                next_multicast_join = now + 5.0
            if now >= next_resolve:
                self.resolve_all()
                next_resolve = now + 5.0
            if streams and self.addresses and now >= next_heartbeat:
                self.send_heartbeats(udp, streams)
                next_heartbeat = now + 1.0
            try:
                data, source = udp.recvfrom(CELLS_SIZE + 32)
            except socket.timeout:
                continue
            except OSError:
                if not self.stop_event.is_set():
                    self.state.send_failures += 1
                continue
            if not self.source_allowed(self.addresses, source):
                self.state.source_rejections += 1
                continue
            try:
                event_type, sequence, payload = parse_packet(data)
            except ValueError as error:
                self.state.malformed_packets += 1
                if str(error) == "CRC mismatch":
                    self.state.crc_errors += 1
                else:
                    self.state.format_errors += 1
                continue
            self.state.record_packet(
                event_type,
                sequence,
                payload,
                self.source_host(source),
                source[0],
            )
        if multicast_joined:
            self.leave_multicast(udp)
        udp.close()


class StatusWorker(threading.Thread):
    def __init__(
        self,
        state: BridgeState,
        stop_event: threading.Event,
        status_url: str,
        poll_interval: float,
    ) -> None:
        super().__init__(name="bearamin-status", daemon=True)
        self.state = state
        self.stop_event = stop_event
        self.status_url = status_url
        self.poll_interval = max(0.5, poll_interval)

    def fetch(self, status_url: str) -> dict[str, Any]:
        request = urllib.request.Request(
            status_url,
            headers={"Accept": "application/json", "User-Agent": "BearaminOBS/2"},
        )
        with urllib.request.urlopen(request, timeout=0.8) as response:
            status = json.loads(response.read(65536))
        if not isinstance(status, dict):
            raise ValueError("status response is not an object")
        if "dpt_mode" not in status or "dpt_knobs" not in status:
            raise ValueError("status response lacks DPT mapping fields")
        return status

    def run(self) -> None:
        next_poll = 0.0
        while not self.stop_event.is_set():
            if not self.state.has_mapping_clients():
                next_poll = 0.0
                self.stop_event.wait(0.2)
                continue
            now = time.monotonic()
            if now < next_poll:
                self.stop_event.wait(min(0.2, next_poll - now))
                continue
            status_url = self.status_url or self.state.active_status_url()
            if status_url:
                try:
                    self.state.record_mapping_status(self.fetch(status_url))
                except (OSError, ValueError, json.JSONDecodeError):
                    self.state.record_mapping_status_error()
            next_poll = time.monotonic() + self.poll_interval


class OverlayHandler(http.server.BaseHTTPRequestHandler):
    server_version = "BearaminOBS/2"

    @property
    def bridge(self) -> "OverlayHttpServer":
        return self.server  # type: ignore[return-value]

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.bridge.verbose:
            super().log_message(fmt, *args)

    def send_bytes(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in {"/", "/index.html"}:
            self.send_bytes(200, "text/html; charset=utf-8", self.bridge.overlay_html)
            return
        if parsed.path == "/mappings.html":
            self.send_bytes(
                200, "text/html; charset=utf-8", self.bridge.mapping_overlay_html
            )
            return
        if parsed.path == "/logo.png" and self.bridge.logo_png:
            self.send_bytes(200, "image/png", self.bridge.logo_png)
            return
        if parsed.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            return
        if parsed.path == "/health.json":
            body = json.dumps(self.bridge.state.health(), separators=(",", ":")).encode()
            self.send_bytes(200, "application/json; charset=utf-8", body)
            return
        if parsed.path == "/events":
            mode = urllib.parse.parse_qs(parsed.query).get("mode", ["crosshair"])[0]
            self.serve_events(mode)
            return
        self.send_bytes(404, "text/plain; charset=utf-8", b"not found\n")

    def serve_events(self, mode: str) -> None:
        client_id, client = self.bridge.state.add_client(mode)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            self.wfile.write(b"retry: 1000\n\n")
            self.wfile.flush()
            while not self.bridge.stop_event.is_set():
                try:
                    event_type, payload = client.events.get(timeout=10)
                    body = json.dumps(payload, separators=(",", ":")).encode()
                    self.wfile.write(b"event: " + event_type.encode() + b"\n")
                    self.wfile.write(b"data: " + body + b"\n\n")
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.bridge.state.remove_client(client_id)


class OverlayHttpServer(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        state: BridgeState,
        stop_event: threading.Event,
        overlay_html: bytes,
        mapping_overlay_html: bytes,
        logo_png: bytes,
        verbose: bool,
    ) -> None:
        super().__init__(address, OverlayHandler)
        self.state = state
        self.stop_event = stop_event
        self.overlay_html = overlay_html
        self.mapping_overlay_html = mapping_overlay_html
        self.logo_png = logo_png
        self.verbose = verbose

    def handle_error(self, request: Any, client_address: Any) -> None:
        if self.stop_event.is_set() or not self.verbose:
            return
        super().handle_error(request, client_address)


def default_logo_path() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parent / "logo.png"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device-host",
        action="append",
        dest="device_hosts",
        metavar="HOST",
        help=(
            "BOV1 producer hostname; repeat to accept multiple sources "
            "(defaults to bearamin.lan and bearamin-c5.lan)"
        ),
    )
    parser.add_argument("--device-port", type=int, default=DEVICE_PORT)
    parser.add_argument("--multicast-group", default=MULTICAST_GROUP)
    parser.add_argument("--multicast-port", type=int, default=MULTICAST_PORT)
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)
    parser.add_argument("--device-status-url", default="")
    parser.add_argument(
        "--mapping-value-mode",
        choices=sorted(MAPPING_VALUE_MODES),
        default="modulation",
        help="show effective Bearamin contribution or final knob-plus-modulation value",
    )
    parser.add_argument("--mapping-status-interval", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    device_hosts = tuple(args.device_hosts or DEFAULT_DEVICE_HOSTS)

    root = pathlib.Path(__file__).resolve().parent
    overlay_html = (root / "overlay.html").read_text().replace(
        "__BEARAMIN_TRUE_GRID_TO_OBS__", json.dumps(TRUE_GRID_TO_OBS)
    ).encode()
    mapping_overlay_html = (root / "mappings.html").read_bytes()
    logo_path = default_logo_path()
    logo_png = logo_path.read_bytes() if logo_path.exists() else b""
    state = BridgeState(
        args.multicast_group, args.multicast_port, args.mapping_value_mode
    )
    stop_event = threading.Event()
    udp = UdpWorker(
        state,
        stop_event,
        device_hosts,
        args.device_port,
        args.multicast_group,
        args.multicast_port,
    )
    status = StatusWorker(
        state,
        stop_event,
        args.device_status_url,
        args.mapping_status_interval,
    )
    httpd = OverlayHttpServer(
        ("127.0.0.1", args.http_port),
        state,
        stop_event,
        overlay_html,
        mapping_overlay_html,
        logo_png,
        args.verbose,
    )

    def stop(_signum: int, _frame: Any) -> None:
        stop_event.set()
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    udp.start()
    status.start()
    print(
        "Bearamin OBS overlay: "
        f"http://127.0.0.1:{args.http_port}/?mode=crosshair "
        f"and http://127.0.0.1:{args.http_port}/mappings.html "
        f"(mapping values: {args.mapping_value_mode}; "
        f"sources {', '.join(device_hosts)}; "
        f"multicast {args.multicast_group}:{args.multicast_port})",
        flush=True,
    )
    try:
        httpd.serve_forever(poll_interval=0.25)
    finally:
        stop_event.set()
        httpd.server_close()
        udp.join(timeout=2)
        status.join(timeout=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
