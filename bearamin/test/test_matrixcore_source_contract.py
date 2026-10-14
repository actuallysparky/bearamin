#!/usr/bin/env python3
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MATRIXCORE = PROJECT_ROOT / "firmware/libraries/BearaminMatrixCore/src/BearaminMatrixCoreApp.cpp"
MATRIXCORE_V2_CONFIG = PROJECT_ROOT / "firmware/libraries/BearaminMatrixCore/src/targets/vl53l7cx_s3.h"
MATRIXCORE_REVC_CONFIG = PROJECT_ROOT / "firmware/libraries/BearaminMatrixCore/src/targets/vl53l7cx_revc_s3.h"
MATRIXCORE_8X = PROJECT_ROOT / "firmware" / "BearaminMatrixCore8x" / "BearaminMatrixCore8x.ino"
MATRIXCORE_8X_CONFIG = PROJECT_ROOT / "firmware/libraries/BearaminMatrixCore/src/targets/vl53l8cx_s3.h"
SIDECAR_PROTOCOL = PROJECT_ROOT / "firmware" / "common" / "sidecar_protocol.h"
SIDECAR = PROJECT_ROOT / "firmware/libraries/BearaminSidecar/src/BearaminSidecarApp.cpp"


def main() -> None:
    source = MATRIXCORE.read_text()
    source_v2_config = MATRIXCORE_V2_CONFIG.read_text()
    source_revc_config = MATRIXCORE_REVC_CONFIG.read_text()
    source_8x = MATRIXCORE_8X.read_text()
    source_8x_config = MATRIXCORE_8X_CONFIG.read_text()
    protocol = SIDECAR_PROTOCOL.read_text()
    sidecar = SIDECAR.read_text()

    for contract in (
        '#include "../../../../third_party/bearfield-core/include/bearfield/bearfield.h"',
        "bearfield::Processor fieldProcessor(matrixFeelProfile())",
        "profile.depthOffsetMm = kMatrixDepthOffsetMm",
        "profile.idleAxisNorm = bearfield::kNormCenter",
        "profile.idleDepthNorm = 0",
        "profile.centroidMinimumX = BEARAMIN_TRUE_GRID_CENTROID_MIN_X",
        "bearfield::orientFrame(rawFrame, sensorMount)",
        "fieldProcessor.process(canonical, capturedUs)",
        "processedField = fieldProcessor.target()",
        "bearamin_trajectory::Authority trajectoryAuthority",
        "trajectoryAuthority.publishTarget",
        "bearamin_trajectory::evaluateAtSourceTime",
        "constexpr uint8_t kMatrixSourceHz = 15;",
        "constexpr uint8_t kProcessedOutputHz = 30;",
        "matrixSensor.setResolution(64)",
        "matrixSensor.setRangingFrequency(kMatrixSourceHz)",
        "matrixSensor.set_resolution(64)",
        "matrixSensor.set_ranging_frequency_hz(kMatrixSourceHz)",
        "BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK",
        "BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS",
        "bearfield::displayPixelIndex(column, row, mount)",
        "performance.sourceHz = kMatrixSourceHz;",
        "performance.outputHz = kProcessedOutputHz;",
    ):
        assert contract in source, f"missing fixed 8x8/core contract: {contract}"

    for removed in (
        "4x4",
        "MATRIX_MODE",
        "MatrixReconfigure",
        "matrixModeSwitching",
        "COMMAND_SET_MATRIX_MODE",
        "expandToCanonical8x8",
        "matrixModeStateFlags",
    ):
        assert removed not in source, f"obsolete matrix mode remains in MatrixCore: {removed}"
        assert removed not in protocol, f"obsolete matrix mode remains in protocol: {removed}"

    for retired_sensor in ("VL53L0X", "VL6180X", "BEARAMIN_ENABLE_LEGACY_NEAR_TOF"):
        assert retired_sensor not in source
    assert "fieldProcessor.tick" not in source
    assert "clearMatrixMusicalPresence" not in source
    assert "optical.matrixXNorm = trajectoryOutputAxes" not in source
    assert "optical.matrixYNorm = trajectoryOutputAxes" not in source
    assert "optical.matrixZNorm = trajectoryOutputAxes" not in source
    assert "const uint16_t evaluatedXMilli = trajectoryOutputValid" in source
    assert "const uint16_t evaluatedYMilli = trajectoryOutputValid" in source
    assert "constexpr uint16_t kCvSlewMaxStep = 32;" in source
    assert source.index("matrixSensor.setResolution(64)") < source.index(
        "matrixSensor.setRangingFrequency(kMatrixSourceHz)"
    )
    for contract in (
        "#include <WebServer.h>",
        'matrixIdentityServer.on("/device.json", HTTP_GET, sendMatrixIdentityJson)',
        'matrixIdentityServer.on("/status.json", HTTP_GET, sendMatrixIdentityJson)',
        '"\\\"mac\\\":\\\"%s\\\"',
        'MDNS.addService("http", "tcp", 80)',
        "matrixIdentityServer.handleClient()",
    ):
        assert contract in source, f"missing Matrix OTA identity contract: {contract}"

    for contract in (
        "#define BEARAMIN_MATRIX_SENSOR_VL53L7CX 1",
        "#define BEARAMIN_MATRIX_LPR_PIN 5",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS 1",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X 0",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y 1",
        "#define BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK UINT64_MAX",
        "#define BEARAMIN_TRUE_GRID_CENTROID_MIN_X 0",
        "#define BEARAMIN_TRUE_GRID_CENTROID_MAX_X 7",
        "#define BEARAMIN_TRUE_GRID_CENTROID_MIN_Y 0",
        "#define BEARAMIN_TRUE_GRID_CENTROID_MAX_Y 7",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS 2",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X 0",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y 0",
        "#define BEARAMIN_OPTICAL_CLOCK_HZ 400000",
        '#define MATRIXCORE_MDNS_HOSTNAME "bearamin-matrixcore"',
    ):
        assert contract in source_v2_config, f"missing Matrix v2 target contract: {contract}"

    for contract in (
        "#define BEARAMIN_MATRIX_SENSOR_VL53L7CX 1",
        "#define BEARAMIN_MATRIX_LPR_PIN 3",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS 0",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X 0",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y 1",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS 2",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X 0",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y 0",
        "#define BEARAMIN_STATUS_USES_DISPLAY_PROFILE 1",
        '#define MATRIXCORE_MDNS_HOSTNAME "bearamin-matrixcore"',
    ):
        assert contract in source_revc_config, f"missing Matrix Rev C target contract: {contract}"

    for contract in (
        "#define BEARAMIN_MATRIX_SENSOR_VL53L8CX 1",
        "#define BEARAMIN_MATRIX_LPR_PIN 5",
        "#define BEARAMIN_MERGED_MATRIX_SENSOR_STATUS 1",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS 1",
        "#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y 1",
        "#define BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK UINT64_MAX",
        "#define BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS 1",
        "#define BEARAMIN_STATUS_USES_DISPLAY_PROFILE 1",
        "#define BEARAMIN_OPTICAL_CLOCK_HZ 400000",
        '#define MATRIXCORE_MDNS_HOSTNAME "bearamin-matrixcore-8x"',
    ):
        assert contract in source_8x_config, f"missing VL53L8CX target contract: {contract}"
    assert "prepareVl53l8cxOpticalBusForDeferredInit(optical);" in source
    assert "constexpr uint8_t kMatrixLprPin = BEARAMIN_MATRIX_LPR_PIN;" in source
    assert '#include <BearaminMatrixCoreApp.h>' in source_8x
    assert "4x4" not in source_8x
    assert "60 Hz" not in source_8x
    assert "BearaminMatrixCore.ino" not in source_8x
    assert "Matrix Core v3's complete panel, including its board-status row, follows" in source
    assert "Matrix Core v2 status indicators are board hardware UI, not true-grid" in source
    assert "#if BEARAMIN_STATUS_USES_DISPLAY_PROFILE" in source
    assert "pixelIndex(columnFromLeft, kMatrixWidth - 1)" in source
    assert "(kMatrixWidth - 1) * kMatrixWidth +" in source
    assert "(kMatrixWidth - 1 - columnFromLeft)" in source
    assert "BEARAMIN_LED_ROTATE_180" not in source
    assert "BEARAMIN_LED_ROTATE_180" not in source_8x_config
    assert "COMMAND_ENTER_LORENZ = 13" in protocol
    assert "chaosUnlock.force()" in source
    assert "startChaosActivationFeedback(now)" in source
    assert "startChaosActivationFeedback(millis())" in source
    assert "renderChaosActivationFeedback(now)" in source
    assert "kChaosRainbowDurationMs = 1000" in source
    packet_builder = source[source.index("bearamin_bridge::Packet buildDptPacket()"):
                            source.index("void sendDpt(uint32_t now)")]
    assert "const bool chaosActive = chaosUnlock.latched();" in packet_builder
    assert "packet.flags |= bearamin_bridge::FLAG_CHAOS_ACTIVE;" in packet_builder
    assert "packet.matrixXNorm = chaosAxes.x;" in packet_builder
    assert "packet.matrixYNorm = chaosAxes.y;" in packet_builder
    assert "packet.matrixZNorm = chaosAxes.z;" in packet_builder
    assert "packet.trajectoryStartNorm[bearamin_trajectory::AXIS_X] = chaosAxes.x;" in packet_builder
    assert "} else if (trajectoryAuthority.hasKeyframe()) {" in packet_builder
    assert "void handleLorenz()" in sidecar
    assert "COMMAND_ENTER_LORENZ" in sidecar
    assert 'server.on("/lorenz", HTTP_GET, handleLorenz)' in sidecar


if __name__ == "__main__":
    main()
