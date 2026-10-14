#!/usr/bin/env python3

import importlib.util
import pathlib
import queue
import struct
import sys
import threading

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools/obs_overlay/bearamin_obs_bridge.py"
SPEC = importlib.util.spec_from_file_location("bearamin_obs_bridge", MODULE_PATH)
assert SPEC and SPEC.loader
bridge = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bridge
SPEC.loader.exec_module(bridge)


def make_packet(packet_type, sequence, payload, version=bridge.VERSION_V3):
    header = bridge.HEADER.pack(
        bridge.MAGIC, version, packet_type, len(payload), sequence, 0
    )
    packet = header + payload
    return packet + struct.pack("<H", bridge.crc16_ccitt(packet))


def test_subscribe():
    assert bridge.MULTICAST_GROUP == "239.255.77.66"
    assert bridge.MULTICAST_PORT == 51338
    packet = bridge.encode_subscribe(
        bridge.STREAM_STATE | bridge.STREAM_CELLS, 0x3344, 0x12345678
    )
    assert len(packet) == bridge.SUBSCRIBE_SIZE
    assert packet[:4] == b"BOV1"
    assert struct.unpack_from("<H", packet, 8)[0] == 0x3344
    assert bridge.crc16_ccitt(packet[:-2]) == struct.unpack_from("<H", packet, len(packet) - 2)[0]


def test_state_packet():
    values = (
        0x01020304, 0x1058, 17, 1024, 3072, 2048, 250, 750, 321,
        1234, 25, 1536, 2560, 48, 12,
    )
    event, sequence, decoded = bridge.parse_packet(
        make_packet(bridge.PACKET_STATE, 9, bridge.STATE_PAYLOAD.pack(*values))
    )
    assert event == "state" and sequence == 9
    assert decoded["matrix_ready"] and decoded["matrix_present"]
    assert decoded["matrix_x_norm"] == 1024
    assert decoded["pitch_norm"] == 1536
    assert decoded["roll_norm"] == 2560
    assert "matrix_mode" not in decoded
    assert decoded["spank_gate"]


def test_v4_state_packet_and_subscription():
    values = (
        0x01020304, 0x1058, 17, 1024, 3072, 2048, 250, 750, 321,
        1234, 25, 1536, 2560, 48, 12,
        100, 200, 300, 400, 500, 232,
    )
    event, sequence, decoded = bridge.parse_packet(make_packet(
        bridge.PACKET_STATE, 10, bridge.STATE_PAYLOAD_V4.pack(*values),
        bridge.VERSION_V4,
    ))
    assert event == "state" and sequence == 10
    assert decoded["protocol_version"] == bridge.VERSION_V4
    assert decoded["trajectory_start_norm"] == [100, 200, 300, 400, 500]
    assert decoded["slew_tau_ms"] == 232
    v3 = bridge.encode_subscribe(bridge.STREAM_STATE, 1, 99, bridge.VERSION_V3)
    v4 = bridge.encode_subscribe(bridge.STREAM_STATE, 2, 99, bridge.VERSION_V4)
    assert v3[4] == bridge.VERSION_V3 and v4[4] == bridge.VERSION_V4


def test_cells_packet_and_malformed():
    values = (99, 7, 64, 16, *range(100, 164))
    packet = make_packet(
        bridge.PACKET_CELLS, 10, bridge.CELLS_PAYLOAD.pack(*values)
    )
    event, sequence, decoded = bridge.parse_packet(packet)
    assert event == "cells" and sequence == 10
    assert decoded["cells_mm"][0] == 100 and decoded["cells_mm"][63] == 163
    try:
        bridge.parse_packet(packet[:-1] + bytes([packet[-1] ^ 0x80]))
        raise AssertionError("bad CRC accepted")
    except ValueError:
        pass
    try:
        bridge.parse_packet(packet[:-1])
        raise AssertionError("short packet accepted")
    except ValueError:
        pass


def test_true_grid_transform_vectors():
    # Rotation is clockwise about the grid centre, followed by local mirrors.
    vectors = (
        (0, False, False, 1, 2), (0, True, False, 6, 2),
        (0, False, True, 1, 5), (0, True, True, 6, 5),
        (1, False, False, 5, 1), (1, True, False, 2, 1),
        (1, False, True, 5, 6), (1, True, True, 2, 6),
        (2, False, False, 6, 5), (2, True, False, 1, 5),
        (2, False, True, 6, 2), (2, True, True, 1, 2),
        (3, False, False, 2, 6), (3, True, False, 5, 6),
        (3, False, True, 2, 1), (3, True, True, 5, 1),
    )
    for rotation, mirror_x, mirror_y, expected_x, expected_y in vectors:
        profile = {
            "rotation_quarters": rotation,
            "mirror_x": mirror_x,
            "mirror_y": mirror_y,
        }
        assert bridge.transform_true_grid_cell(1, 2, profile) == (
            expected_x, expected_y,
        )
        point_x, point_y = bridge.transform_true_grid_point(1 / 7, 2 / 7, profile)
        assert abs(point_x - expected_x / 7) < 0.000001
        assert abs(point_y - expected_y / 7) < 0.000001
    assert bridge.transform_true_grid_point(0.0, 0.0, {
        "rotation_quarters": 1, "mirror_x": False, "mirror_y": True,
    }) == (1.0, 1.0)
    assert bridge.transform_true_grid_cell(0, 0, {
        "rotation_quarters": 1, "mirror_x": False, "mirror_y": True,
    }) == (7, 7)
    for rotation in range(4):
        for mirror_x in (False, True):
            for mirror_y in (False, True):
                profile = {
                    "rotation_quarters": rotation,
                    "mirror_x": mirror_x,
                    "mirror_y": mirror_y,
                }
                cells = {
                    bridge.transform_true_grid_cell(x, y, profile)
                    for y in range(8) for x in range(8)
                }
                assert len(cells) == 64
def test_client_modes_fanout_and_gaps():
    state = bridge.BridgeState()
    cross_id, cross = state.add_client("crosshair")
    dots_id, dots = state.add_client("dots")
    combined_id, combined = state.add_client("combined")
    mappings_id, mappings = state.add_client("mappings")
    assert state.requested_streams() == bridge.STREAM_STATE | bridge.STREAM_CELLS
    state.record_mapping_status({
        "dpt_mode": 1,
        "dpt_knobs": [128, 128, 128, 128, 128, 128, 255, 0],
        "dpt_ready": True,
        "core_link_stale": False,
        "map_rings_resonator_0_axis": "x_bipolar",
        "map_rings_resonator_0_depth": 100,
    })
    state.record_packet("state", 65535, {"matrix_x_norm": 1})
    state.record_packet("state", 1, {"matrix_x_norm": 2})
    assert state.sequence_gaps["state"] == 1
    for client in (cross, combined, dots):
        assert client.events.get_nowait()[0] == "state"
        assert client.events.get_nowait()[0] == "state"
    mapping_event, mapping_payload = mappings.events.get_nowait()
    assert mapping_event == "mappings"
    assert mapping_payload["mode_name"] == "Sympathetic Strings"
    state.record_packet("cells", 2, {"cells_mm": [1] * 64})
    assert dots.events.get_nowait()[0] == "cells"
    state.remove_client(dots_id)
    state.remove_client(combined_id)
    assert state.requested_streams() == bridge.STREAM_STATE
    state.remove_client(cross_id)
    assert state.requested_streams() == bridge.STREAM_STATE
    state.remove_client(mappings_id)
    assert state.requested_streams() == 0
    health = state.health()
    assert health["multicast_group"] == bridge.MULTICAST_GROUP
    assert health["multicast_port"] == bridge.MULTICAST_PORT
    assert not health["multicast_joined"]
    assert not health["stale"]
    assert health["crc_errors"] == 0
    assert health["format_errors"] == 0
    assert health["coordinate_space"] == "bearamin_true_grid_v1"
    assert health["true_grid_to_local"] == bridge.TRUE_GRID_TO_OBS


def test_mapping_snapshot_math_and_modes():
    status = {
        "dpt_mode": 1,
        "dpt_knobs": [128, 128, 128, 128, 128, 128, 255, 0],
        "dpt_ready": True,
        "core_link_stale": False,
        "map_rings_resonator_0_axis": "x_bipolar",
        "map_rings_resonator_0_depth": 100,
        "map_rings_resonator_1_axis": "z_unipolar",
        "map_rings_resonator_1_depth": -50,
    }
    state = {
        "protocol_version": 4,
        "matrix_x_norm": 4095,
        "matrix_y_norm": 2048,
        "matrix_z_norm": 2048,
        "pitch_norm": 2048,
        "roll_norm": 2048,
        "spank_gate": False,
        "trajectory_start_norm": [3000, 2048, 2048, 2048, 2048],
        "slew_tau_ms": 232,
    }
    modulation = bridge.build_mapping_snapshot(status, state, "modulation")
    assert modulation["mode_name"] == "Sympathetic Strings"
    assert modulation["gesture_amount"] == 1.0
    assert abs(modulation["lanes"][0]["total"] - 1.0) < 0.000001
    assert abs(modulation["lanes"][0]["modulation"] - (1.0 - 128 / 255)) < 0.000001
    assert abs(modulation["lanes"][1]["modulation"] + 0.250061) < 0.00001
    assert modulation["lanes"][0]["display_value"] == modulation["lanes"][0]["modulation"]
    assert modulation["protocol_version"] == state["protocol_version"]
    assert modulation["trajectory_start_norm"] == state["trajectory_start_norm"]
    assert modulation["slew_tau_ms"] == state["slew_tau_ms"]
    total = bridge.build_mapping_snapshot(status, state, "total")
    assert total["lanes"][0]["display_value"] == total["lanes"][0]["total"]
    status["dpt_mode"] = 3
    status["map_bearamin_fm_0_axis"] = "x_bipolar"
    status["map_bearamin_fm_0_depth"] = 100
    fm = bridge.build_mapping_snapshot(status, state, "modulation")
    assert fm["mode_name"] == "Bearamin FM drone"
    assert fm["lanes"][0]["total"] == 1.0
    assert bridge.gesture_amount(4) == 0.0


def test_multicast_join_leave_and_source_validation():
    class FakeSocket:
        def __init__(self):
            self.options = []
            self.sent = []

        def setsockopt(self, level, option, value):
            self.options.append((level, option, value))

        def sendto(self, packet, address):
            self.sent.append((packet, address))

    state = bridge.BridgeState()
    assert state.health()["stale"]
    worker = bridge.UdpWorker(
        state,
        threading.Event(),
        bridge.DEFAULT_DEVICE_HOSTS,
        bridge.DEVICE_PORT,
        bridge.MULTICAST_GROUP,
        bridge.MULTICAST_PORT,
    )
    udp = FakeSocket()
    assert worker.join_multicast(udp)
    assert state.multicast_joined
    assert udp.options[-1][1] == bridge.socket.IP_ADD_MEMBERSHIP
    worker.leave_multicast(udp)
    assert not state.multicast_joined
    assert udp.options[-1][1] == bridge.socket.IP_DROP_MEMBERSHIP
    addresses = {
        "bearamin.lan": ("192.168.8.178", bridge.DEVICE_PORT),
        "bearamin-c5.lan": ("192.168.8.179", bridge.DEVICE_PORT),
    }
    assert worker.source_allowed(
        addresses,
        ("192.168.8.178", bridge.MULTICAST_PORT),
    )
    assert worker.source_allowed(
        addresses,
        ("192.168.8.179", bridge.MULTICAST_PORT),
    )
    assert not worker.source_allowed(
        addresses,
        ("192.168.8.99", bridge.MULTICAST_PORT),
    )
    assert not worker.source_allowed({}, ("192.168.8.178", bridge.MULTICAST_PORT))
    worker.addresses = addresses
    state.update_device_addresses(addresses)
    worker.send_heartbeats(udp, bridge.STREAM_STATE)
    assert len(udp.sent) == 4
    assert {address for _, address in udp.sent} == set(addresses.values())
    assert [packet[4] for packet, _ in udp.sent] == [
        bridge.VERSION_V3,
        bridge.VERSION_V4,
        bridge.VERSION_V3,
        bridge.VERSION_V4,
    ]
    assert state.health()["heartbeats_sent"] == 4


def test_dual_source_state_and_sequence_transition():
    state = bridge.BridgeState()
    addresses = {
        "bearamin.lan": ("192.168.8.178", bridge.DEVICE_PORT),
        "bearamin-c5.lan": ("192.168.8.179", bridge.DEVICE_PORT),
    }
    state.configure_sources(bridge.DEFAULT_DEVICE_HOSTS)
    state.update_device_addresses(addresses)
    state.record_packet(
        "state", 10, {"matrix_x_norm": 1}, "bearamin.lan", "192.168.8.178"
    )
    state.record_packet(
        "state", 12, {"matrix_x_norm": 2}, "bearamin.lan", "192.168.8.178"
    )
    assert state.sequence_gaps["state"] == 1
    state.record_packet(
        "state", 3, {"matrix_x_norm": 3}, "bearamin-c5.lan", "192.168.8.179"
    )
    health = state.health()
    assert health["active_source_host"] == "bearamin-c5.lan"
    assert health["active_source_address"] == "192.168.8.179"
    assert health["device_address"] == "192.168.8.179:51337"
    assert health["device_hosts"] == list(bridge.DEFAULT_DEVICE_HOSTS)
    assert health["source_switches"] == 1
    assert state.sequence_gaps["state"] == 1
    assert state.active_status_url() == "http://192.168.8.179/status.json"


def test_overlay_contract():
    html = (ROOT / "tools/obs_overlay/overlay.html").read_text()
    for token in (
        "new EventSource('/events?mode='",
        "age<=250",
        "age>=500",
        "mode==='crosshair'",
        "mode==='dots'",
        "mode==='combined'",
        "const trueGridToObs=__BEARAMIN_TRUE_GRID_TO_OBS__",
        "function transformTrueGridPoint(x,y)",
        "function transformTrueGridCell(column,row)",
        "const mapped=transformTrueGridPoint(next.matrix_x_norm/4095,next.matrix_y_norm/4095)",
        "expandLedAxis(mapped.x)",
        "expandLedAxis(mapped.y)",
        "index=row*8+col",
        "screen=transformTrueGridCell(col,row)",
        "showLogo",
        "showOrientation",
        "showDebug",
        "next.pitch_norm??2048",
        "next.roll_norm??2048",
        "drawPitchDial",
        "drawRollDial",
        "drawSpankIcon",
        "state&&state.spank_gate",
        "function zBarHeight(z){return 1-clamp(z)}",
        "lit=Math.round(zBarHeight(rendered.z)*segments)",
        "zeroY=by+bh-zBarHeight(2048/4095)*bh",
        "ctx.strokeStyle='rgba(220,0,255,.95)'",
        "Math.min(w,h)*.78*scale",
        "ctx.lineWidth=7",
        'id="glass"',
        "backdrop-filter:blur(9px)",
        "function layoutGlass(w,h)",
        "z-index:2",
        "width:18vmin;height:18vmin",
    ):
        assert token in html
    assert "matrix_mode" not in html
    assert "mirrorX" not in html
    assert "mirrorY" not in html
    assert "ledDisplayRows" not in html
    assert "dotScreenCoordinate" not in html
    assert "4x4" not in html
    assert "lit=Math.round(rendered.z*segments)" not in html


def test_mapping_overlay_contract():
    html = (ROOT / "tools/obs_overlay/mappings.html").read_text()
    for token in (
        "background:transparent",
        "#strip{width:100%;height:150px",
        "grid-template-columns:minmax(235px,.95fr) repeat(6",
        "new EventSource('/events?mode=mappings')",
        "frame.value_mode==='total'?'Final · knob + Bearamin':'Bearamin contribution'",
        "<span>atten ${lane.depth_percent>0?'+':''}${lane.depth_percent}%</span>",
        "Math.abs(mod)*.5",
        "orbSize=11+Math.abs",
        "function trajectoryAxes(frame)",
        "Math.exp(-elapsed/tau)",
        "function nativeMappingValue(frame,axes,source)",
        "function frame(){if(latestFrame)render(latestFrame,trajectoryAxes(latestFrame));requestAnimationFrame(frame)}",
        "trajectory_start_norm",
        "slew_tau_ms",
        ".meter{position:relative;align-self:center;height:18px",
        "background:linear-gradient(145deg,rgba(4,12,15,.92),rgba(11,20,24,.78))",
        ".lane-name{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:22px",
        ".current{font:850 40px/1",
        "CV7 amount",
    ):
        assert token in html


def test_sidecar_orchestration_contract():
    source = (ROOT / "firmware/libraries/BearaminSidecar/src/BearaminSidecarApp.cpp").read_text()
    for token in (
        "obsTelemetryUdp.begin(bearamin_obs::kUdpPort)",
        "pollMatrixUart();\n  expireObsSubscriptions(now);",
        "expireObsSubscriptions(now);",
        "sendPendingObsTelemetry(now);",
        "reconcileEngineeringStream(now);",
        "obsSubscriberLeases.update(",
        "obsSubscriberLeases.streamUnion(now)",
        "bearamin_obs::kMulticastPort",
        "table_full_rejections",
        "active_subscribers",
        "multicast_state_packets",
        "multicast_cell_packets",
        "obsStatePending = true;",
        "obsCellsPending = true;",
        "obs_overlay",
    ):
        assert token in source
    assert source.index("pollMatrixUart();") < source.index("pollObsSubscriptions(now);")
    assert source.index("expireObsSubscriptions(now);") < source.index("pollObsSubscriptions(now);")
    assert source.index("pollObsSubscriptions(now);") < source.index("sendPendingObsTelemetry(now);")


if __name__ == "__main__":
    test_subscribe()
    test_state_packet()
    test_v4_state_packet_and_subscription()
    test_cells_packet_and_malformed()
    test_true_grid_transform_vectors()
    test_client_modes_fanout_and_gaps()
    test_mapping_snapshot_math_and_modes()
    test_multicast_join_leave_and_source_validation()
    test_dual_source_state_and_sequence_transition()
    test_overlay_contract()
    test_mapping_overlay_contract()
    test_sidecar_orchestration_contract()
