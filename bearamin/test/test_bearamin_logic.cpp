#include <cassert>
#include <cstring>

#include "../firmware/common/bridge_protocol.h"
#include "../firmware/common/cv_logic.h"
#include "../firmware/common/obs_overlay_protocol.h"
#include "../firmware/common/preset_quiet.h"
#include "../firmware/common/preset_response_split.h"
#include "../firmware/common/sensor_recovery.h"
#include "../firmware/common/sidecar_protocol.h"
#include "../firmware/common/wifi_band_logic.h"
#include "../dpt/DPT.multimode/PresetProtocol.h"

int main() {
  assert(!bearamin_wifi::is5GhzChannel(1));
  assert(!bearamin_wifi::is5GhzChannel(14));
  assert(bearamin_wifi::is5GhzChannel(36));
  assert(bearamin_wifi::is5GhzChannel(161));
  assert(bearamin_wifi::fiveGhzPulseBrightness(0) == 28);
  assert(bearamin_wifi::fiveGhzPulseBrightness(700) == 54);
  assert(bearamin_wifi::fiveGhzPulseBrightness(1400) == 80);
  assert(bearamin_wifi::fiveGhzPulseBrightness(2100) == 54);
  assert(bearamin_wifi::fiveGhzPulseBrightness(2800) == 28);
  assert(bearamin_sidecar::SIDECAR_STATUS_WIFI_5GHZ == (1u << 4));

  using bearamin_sensor_recovery::nextRetryMs;
  using bearamin_sensor_recovery::sensorMonitoringStartMs;
  using bearamin_sensor_recovery::streamNeedsRecovery;

  assert(!streamNeedsRecovery(false, 0, 100, 10000, 1500, 1000));
  assert(!streamNeedsRecovery(true, 0, 100, 1600, 1500, 1000));
  assert(streamNeedsRecovery(true, 0, 100, 1601, 1500, 1000));
  assert(!streamNeedsRecovery(true, 1200, 100, 2200, 1500, 1000));
  assert(streamNeedsRecovery(true, 1200, 100, 2201, 1500, 1000));
  assert(nextRetryMs(5000, 300000) == 10000);
  assert(nextRetryMs(160000, 300000) == 300000);
  assert(nextRetryMs(300000, 300000) == 300000);
  assert(sensorMonitoringStartMs(false, 12000) == 0);
  assert(sensorMonitoringStartMs(true, 12000) == 12000);
  assert(!streamNeedsRecovery(true, 0, sensorMonitoringStartMs(true, 12000),
                              12001, 1500, 1000));

  assert(bearamin_preset::kQuietWatchdogMs == 200);
  assert(!bearamin_preset::quietWindowActive(0, 1000));
  assert(bearamin_preset::quietWindowActive(16000, 1000));
  assert(!bearamin_preset::quietWindowActive(16000, 16000));
  assert(bearamin_preset::containsTerminalResponse("BPLS,7,END"));
  assert(bearamin_preset::containsTerminalResponse("BPRD,8,END,C085"));
  assert(bearamin_preset::containsTerminalResponse("BPWR,9,OK"));
  assert(bearamin_preset::containsTerminalResponse("BPERR,10,timeout"));
  assert(bearamin_preset::containsTerminalResponse(
      "BPD,8,0,7B7DBPRD,8,END,C085"));
  assert(!bearamin_preset::containsTerminalResponse("BPLS,7,ITEM,default"));
  assert(!bearamin_preset::containsTerminalResponse("BPRD,8,BEGIN,512,C085"));
  assert(!bearamin_preset::containsTerminalResponse("BPWR,9,READY"));
  assert(!bearamin_preset::containsTerminalResponse("BPAK,9,32"));
  {
    const char single[] = "BPD,18,0,7B226D6F6465223A30";
    size_t cursor = 0;
    bearamin_sidecar::PresetResponseSpan span = {};
    assert(bearamin_sidecar::nextPresetResponseSpan(
        single, strlen(single), cursor, span));
    assert(span.start == 0);
    assert(span.length == strlen(single));
    assert(!bearamin_sidecar::nextPresetResponseSpan(
        single, strlen(single), cursor, span));
  }

  {
    const char adjacent[] =
        "BPRD,18,BEGIN,32,ACADBPD,18,0,7B226D6F6465223A30BPRD,18,END,ACAD";
    const char *expected[] = {
        "BPRD,18,BEGIN,32,ACAD",
        "BPD,18,0,7B226D6F6465223A30",
        "BPRD,18,END,ACAD",
    };
    size_t cursor = 0;
    bearamin_sidecar::PresetResponseSpan span = {};
    for (const char *response : expected) {
      assert(bearamin_sidecar::nextPresetResponseSpan(
          adjacent, strlen(adjacent), cursor, span));
      assert(span.length == strlen(response));
      assert(memcmp(adjacent + span.start, response, span.length) == 0);
    }
    assert(!bearamin_sidecar::nextPresetResponseSpan(
        adjacent, strlen(adjacent), cursor, span));
  }

  {
    const char mixed[] = "noiseBPAK,9,0BPD,9,0,7B7DBPERR,9,read";
    const char *expected[] = {"BPAK,9,0", "BPD,9,0,7B7D", "BPERR,9,read"};
    size_t cursor = 0;
    bearamin_sidecar::PresetResponseSpan span = {};
    for (const char *response : expected) {
      assert(bearamin_sidecar::nextPresetResponseSpan(
          mixed, strlen(mixed), cursor, span));
      assert(span.length == strlen(response));
      assert(memcmp(mixed + span.start, response, span.length) == 0);
    }
    assert(!bearamin_sidecar::nextPresetResponseSpan(
        mixed, strlen(mixed), cursor, span));
  }

  assert(cvDutyTargetForDistance(0, false, 5, 75, 255) == 0);
  assert(cvDutyTargetForDistance(0, true, 5, 75, 255) == 0);
  assert(cvDutyTargetForDistance(5, true, 5, 75, 255) == 255);
  assert(cvDutyTargetForDistance(75, true, 5, 75, 255) == 0);
  assert(cvDutyTargetForDistance(40, true, 5, 75, 255) == 127);
  assert(cvDutyTargetForDistance(5, true, 5, 50, 255) == 255);
  assert(cvDutyTargetForDistance(50, true, 5, 50, 255) == 0);
  assert(cvDutyTargetForDistance(75, true, 5, 50, 255) == 0);
  assert(cvDutyTargetForDistance(0, true, 0, 50, 255) == 0);
  assert(!cvOpticalDistanceUsableMm(0, 2000));
  assert(cvOpticalDistanceUsableMm(25, 2000));
  assert(cvOpticalDistanceUsableMm(50, 2000));
  assert(cvOpticalDistanceUsableMm(2000, 2000));
  assert(!cvOpticalDistanceUsableMm(2001, 2000));
  assert(cvPlayableDistanceWithCloseClampMm(0, true, 25, 2000) == 25);
  assert(cvPlayableDistanceWithCloseClampMm(1, true, 25, 2000) == 25);
  assert(cvPlayableDistanceWithCloseClampMm(24, true, 25, 2000) == 25);
  assert(cvPlayableDistanceWithCloseClampMm(25, true, 25, 2000) == 25);
  assert(cvPlayableDistanceWithCloseClampMm(26, true, 25, 2000) == 26);
  assert(cvPlayableDistanceWithCloseClampMm(2000, true, 25, 2000) == 2000);
  assert(cvPlayableDistanceWithCloseClampMm(2001, true, 25, 2000) == 0);
  assert(cvPlayableDistanceWithCloseClampMm(24, false, 25, 2000) == 0);
  assert(cvPlayableDistanceWithCloseClampMm(19, true, 60, 705) == 60);
  assert(cvPlayableDistanceWithCloseClampMm(59, true, 60, 705) == 60);
  assert(cvPlayableDistanceWithCloseClampMm(60, true, 60, 705) == 60);
  assert(cvPlayableDistanceWithCloseClampMm(61, true, 60, 705) == 61);
  assert(cvPlayableDistanceWithCloseClampMm(706, true, 60, 705) == 0);
  assert(bearamin_bridge::encodeResonatorControls(
             bearamin_bridge::RESONATOR_AXIS_NONE,
             bearamin_bridge::RESONATOR_AXIS_Y,
             bearamin_bridge::RESONATOR_AXIS_X,
             bearamin_bridge::RESONATOR_AXIS_Z)
         == 0x00D8);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_NONE,
             bearamin_bridge::RESONATOR_STRENGTH_POS_FULL)
         == 0);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_X,
             bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL)
         == 1);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_X,
             bearamin_bridge::RESONATOR_STRENGTH_POS_HALF)
         == 3);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_Z,
             bearamin_bridge::RESONATOR_STRENGTH_POS_FULL)
         == 12);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_X,
             true,
             bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL)
         == 13);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_Z,
             true,
             bearamin_bridge::RESONATOR_STRENGTH_POS_FULL)
         == 24);
  assert(bearamin_bridge::resonatorMappingAxis(7) == bearamin_bridge::RESONATOR_AXIS_Y);
  assert(bearamin_bridge::resonatorMappingStrength(7)
         == bearamin_bridge::RESONATOR_STRENGTH_POS_HALF);
  assert(bearamin_bridge::resonatorMappingAxis(13) == bearamin_bridge::RESONATOR_AXIS_X);
  assert(bearamin_bridge::resonatorMappingUnipolar(13));
  assert(bearamin_bridge::resonatorMappingStrength(13)
         == bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL);
  assert(bearamin_bridge::resonatorMappingAxis(25) == bearamin_bridge::RESONATOR_AXIS_PITCH);
  assert(bearamin_bridge::resonatorMappingAxis(26) == bearamin_bridge::RESONATOR_AXIS_ROLL);
  assert(bearamin_bridge::resonatorMappingAxis(27) == bearamin_bridge::RESONATOR_AXIS_SPANK);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_PITCH, false,
             bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL) == 25);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_ROLL, false,
             bearamin_bridge::RESONATOR_STRENGTH_POS_FULL) == 26);
  assert(bearamin_bridge::encodeResonatorMapping(
             bearamin_bridge::RESONATOR_AXIS_SPANK, false,
             bearamin_bridge::RESONATOR_STRENGTH_POS_HALF) == 27);
  assert(!bearamin_bridge::resonatorMappingUnipolar(25));
  assert(bearamin_bridge::resonatorMappingStrength(25)
         == bearamin_bridge::RESONATOR_STRENGTH_POS_HALF);
  assert(bearamin_bridge::resonatorMappingDepthPercent(1) == -100);
  assert(bearamin_bridge::resonatorMappingDepthPercent(3) == 50);
  assert(bearamin_bridge::resonatorMappingDepthPercent(24) == 100);
  const uint32_t packedMappings = bearamin_bridge::packResonatorMappings(1, 8, 13, 18, 24, 0);
  assert((packedMappings & 0x1Fu) == 1);
  assert(((packedMappings >> 5) & 0x1Fu) == 8);
  assert(((packedMappings >> 10) & 0x1Fu) == 13);
  assert(((packedMappings >> 15) & 0x1Fu) == 18);
  assert(((packedMappings >> 20) & 0x1Fu) == 24);
  assert(((packedMappings >> 25) & 0x1Fu) == 0);
  assert(cvDutyTargetForDistance(1, true, 0, 50, 255) == 249);
  assert(cvDutyTargetForDistance(25, true, 0, 50, 255) == 127);
  assert(cvDutyTargetForDistanceQuadratic(50, true, 0, 50, 255) == 0);
  assert(cvDutyTargetForDistanceQuadratic(25, true, 0, 50, 255) == 63);
  assert(cvDutyTargetForDistanceQuadratic(1, true, 0, 50, 255) == 243);
  assert(cvDutyTargetForDistanceCurvedWithFade(0, false, 5, 61, 76, 255, 13) == 0);
  assert(cvDutyTargetForDistanceCurvedWithFade(5, true, 5, 61, 76, 255, 13) == 255);
  assert(cvDutyTargetForDistanceCurvedWithFade(61, true, 5, 61, 76, 255, 13) == 13);
  assert(cvDutyTargetForDistanceCurvedWithFade(62, true, 5, 61, 76, 255, 13) == 12);
  assert(cvDutyTargetForDistanceCurvedWithFade(68, true, 5, 61, 76, 255, 13) == 7);
  assert(cvDutyTargetForDistanceCurvedWithFade(76, true, 5, 61, 76, 255, 13) == 0);
  assert(cvDutyTargetForDistanceCurvedWithFade(90, true, 5, 61, 76, 255, 13) == 0);
  assert(cvScaleDutyToBridgeValue(0, 255, 4095) == 0);
  assert(cvScaleDutyToBridgeValue(255, 255, 4095) == 4095);
  assert(cvScaleDutyToBridgeValue(127, 255, 4095) == 2039);
  assert(cvLevelForBridgeValue(0, 4095, 8) == 0);
  assert(cvLevelForBridgeValue(1, 4095, 8) == 1);
  assert(cvLevelForBridgeValue(512, 4095, 8) == 2);
  assert(cvLevelForBridgeValue(2048, 4095, 8) == 5);
  assert(cvLevelForBridgeValue(4095, 4095, 8) == 8);
  assert(cvLevelForBridgeValue(4095, 4095, 0) == 0);
  assert(cvWindowStartForPosition(500, 2, 8) == 3);
  assert(cvWindowStartForPosition(0, 2, 8) == 0);
  assert(cvWindowStartForPosition(1000, 2, 8) == 6);
  assert(cvWindowStartForPosition(500, 4, 8) == 2);
  assert(cvWindowStartForPosition(1000, 8, 8) == 0);
  assert(cvWindowStartForPosition(1200, 3, 8) == 5);
  assert(cvSignedDifference(420, 350) == 70);
  assert(cvSignedDifference(350, 420) == -70);
  assert(cvAbsSignedDifference(-70) == 70);
  assert(cvAbsSignedDifference(70) == 70);
  assert(cvCenteredOffsetMilli(0) == -500);
  assert(cvCenteredOffsetMilli(500) == 0);
  assert(cvCenteredOffsetMilli(1000) == 500);
  assert(cvCenteredOffsetMilli(1200) == 500);
  assert(cvSpatialDeltaFromCenterMilli(500, 500) == 0);
  assert(cvSpatialDeltaFromCenterMilli(0, 1000) == 1000);
  assert(cvSpatialDeltaFromCenterMilli(250, 750) == 500);
  assert(cvBridgeValueForAxisPosition(0, false, false, 4095) == 0);
  assert(cvBridgeValueForAxisPosition(0, true, false, 4095) == 0);
  assert(cvBridgeValueForAxisPosition(500, true, false, 4095) == 2048);
  assert(cvBridgeValueForAxisPosition(1000, true, false, 4095) == 4095);
  assert(cvBridgeValueForAxisPosition(1000, true, true, 4095) == 0);
  assert(cvBridgeValueForAxisPosition(1200, true, false, 4095) == 4095);
  assert(cvSlewBridgeValue(1000, 1300, 128) == 1128);
  assert(cvSlewBridgeValue(1000, 1050, 128) == 1050);
  assert(cvSlewBridgeValue(1000, 700, 128) == 872);
  assert(cvSlewBridgeValue(1000, 700, 0) == 700);
  assert(cvBridgeCenterValue(4095) == 2048);
  assert(cvBridgeCenterValue(4094) == 2047);
  assert(cvBridgeCenterValue(0) == 0);
  CvBridgeIdleState idle =
      cvBridgeIdleStateForCentroid(500, 500, 4095);
  assert(idle.x == 2048);
  assert(idle.y == 2048);
  assert(idle.z == 2048);
  idle = cvBridgeIdleStateForCentroid(250, 750, 4095);
  assert(idle.x == 1024);
  assert(idle.y == 1024);
  assert(idle.z == 2048);

  CvBridgeXyState xy = cvBridgeXyForMatrixOrFallback(0, 4095, true, 250, 750, true, 2048, 4095, 256);
  assert(xy.targetX == 1024);
  assert(xy.targetY == 1024);
  assert(xy.x == 256);
  assert(xy.y == 3839);
  xy = cvBridgeXyForMatrixOrFallback(1800, 2300, false, 0, 0, true, 2048, 4095, 512);
  assert(xy.targetX == 2047);
  assert(xy.targetY == 2047);
  assert(xy.x == 2047);
  assert(xy.y == 2047);
  xy = cvBridgeXyForMatrixOrFallback(1800, 2300, false, 0, 0, true, 0, 4095, 512);
  assert(xy.targetX == 0);
  assert(xy.targetY == 0);
  assert(xy.x == 0);
  assert(xy.y == 0);

  assert(cvDutyTargetForDistanceQuadratic(75, true, 31, 75, 255) == 0);
  assert(cvDutyTargetForDistanceQuadratic(53, true, 31, 75, 255) == 63);
  assert(cvDutyTargetForDistanceQuadratic(31, true, 31, 75, 255) == 255);
  assert(cvDutyTargetForDistanceQuadratic(30, true, 31, 75, 255) == 255);

  CvRgb pixel = cvNeoPixelColorForDistance(31, true, 31, 75, 48, 12);
  assert(pixel.red == 48 && pixel.green == 0 && pixel.blue == 0);
  pixel = cvNeoPixelColorForDistance(53, true, 31, 75, 48, 12);
  assert(pixel.red == 24 && pixel.green == 24 && pixel.blue == 0);
  pixel = cvNeoPixelColorForDistance(75, true, 31, 75, 48, 12);
  assert(pixel.red == 0 && pixel.green == 48 && pixel.blue == 0);
  pixel = cvNeoPixelColorForDistance(76, true, 31, 75, 48, 12);
  assert(pixel.red == 0 && pixel.green == 24 && pixel.blue == 48);
  pixel = cvNeoPixelColorForDistance(30, true, 31, 75, 48, 12);
  assert(pixel.red == 0 && pixel.green == 24 && pixel.blue == 48);
  pixel = cvNeoPixelColorForDistance(0, false, 31, 75, 48, 12);
  assert(pixel.red == 12 && pixel.green == 12 && pixel.blue == 0);

  assert(cvConstrainDistanceCm(0, false, 5, 75) == 0);
  assert(cvConstrainDistanceCm(3, true, 5, 75) == 5);
  assert(cvConstrainDistanceCm(90, true, 5, 75) == 75);
  assert(cvConstrainDistanceCm(75, true, 5, 50) == 50);
  assert(cvConstrainDistanceCm(1, true, 0, 50) == 1);

  assert(cvSlewDuty(100, 140, 16) == 116);
  assert(cvSlewDuty(100, 110, 16) == 110);
  assert(cvSlewDuty(100, 50, 16) == 84);
  assert(cvSlewDuty(100, 50, 0) == 50);

  bearamin_bridge::Packet packet = {};
  packet.sequence = 77;
  packet.flags = bearamin_bridge::FLAG_MATRIX_VALID |
                 bearamin_bridge::FLAG_IMPACT_GATE;
  packet.reservedZMm = 310;
  packet.reservedZNorm = 2048;
  packet.matrixZMm = 450;
  packet.matrixXMilli = 333;
  packet.matrixYMilli = 667;
  packet.matrixZNorm = 1024;
  packet.matrixXNorm = 3072;
  packet.matrixYNorm = 4095;
  packet.reserved0 = 0x1234;
  packet.reserved1 = 0x56;
  packet.reserved2 = 0x78;
  packet.matrixValidCount = 48;
  packet.matrixBlobCount = 5;
  packet.ageMs = 12;
  packet.controlFlags = bearamin_bridge::CONTROL_FM_DRONE_RANGER;
  packet.impactVelocityNorm = 3072;
  packet.impactAgeMs = 18;
  packet.impactFlags = 0x01;
  packet.sourceFlags2 = 0xA0;
  bearamin_bridge::setDefaultResonatorWebControls(packet.resonatorWebControls);
  packet.resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] =
      static_cast<uint8_t>(
          bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_MAJOR <<
          bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_SHIFT);
  packet.laneDepths[0] = -37;
  packet.laneDepths[1] = 83;
  packet.pitchNorm = 1200;
  packet.rollNorm = 3300;
  packet.orientationConfidence = 207;

  uint8_t frame[bearamin_bridge::kFrameSize] = {};
  assert(bearamin_bridge::encodeFrame(packet, frame, sizeof(frame)) == bearamin_bridge::kFrameSize);

  bearamin_bridge::Packet decoded = {};
  assert(bearamin_bridge::decodeFrame(frame, sizeof(frame), decoded));
  assert(decoded.sequence == packet.sequence);
  assert(decoded.matrixYMilli == 667);
  assert(decoded.matrixYNorm == 4095);
  assert(decoded.reserved0 == 0x1234);
  assert(decoded.reserved1 == 0x56);
  assert(decoded.reserved2 == 0x78);
  assert(decoded.matrixBlobCount == 5);
  assert(decoded.controlFlags == bearamin_bridge::CONTROL_FM_DRONE_RANGER);
  assert(decoded.impactVelocityNorm == 3072);
  assert(decoded.impactAgeMs == 18);
  assert(decoded.impactFlags == 0x01);
  assert(decoded.sourceFlags2 == 0xA0);
  assert((decoded.resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] &
          bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_MASK) ==
         (bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_MAJOR <<
          bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_SHIFT));
  assert(decoded.laneDepths[0] == -37);
  assert(decoded.laneDepths[1] == 83);
  assert(decoded.pitchNorm == 1200);
  assert(decoded.rollNorm == 3300);
  assert(decoded.orientationConfidence == 207);

  bearamin_bridge::Parser parser;
  bearamin_bridge::Packet parsed = {};
  bool complete = false;
  for (size_t i = 0; i < sizeof(frame); ++i) {
    complete = parser.Push(frame[i], parsed);
  }
  assert(complete);

  uint8_t incompatibleFrame[bearamin_bridge::kFrameSize] = {};
  memcpy(incompatibleFrame, frame, sizeof(frame));
  incompatibleFrame[2] = 9;
  assert(!bearamin_bridge::decodeFrame(
      incompatibleFrame, sizeof(incompatibleFrame), decoded));
  assert(parsed.reservedZMm == 310);

  frame[7] ^= 0x01;
  assert(!bearamin_bridge::decodeFrame(frame, sizeof(frame), decoded));

  bearamin_bridge::Packet v11 = packet;
  v11.sourceEpochMs = 0x11223344;
  v11.slewTauMs = 232;
  const uint16_t starts[] = {1000, 1100, 1200, 1300, 1400};
  for (uint8_t axis = 0; axis < 5; ++axis) {
    v11.trajectoryStartNorm[axis] = starts[axis];
  }
  uint8_t v11Frame[bearamin_bridge::kFrameSizeV11] = {};
  assert(bearamin_bridge::encodeFrameV11(
             v11, v11Frame, sizeof(v11Frame)) ==
         bearamin_bridge::kFrameSizeV11);
  assert(v11Frame[2] == bearamin_bridge::kVersionV11);
  assert(v11Frame[3] == bearamin_bridge::kPayloadSizeV11);
  assert(bearamin_bridge::decodeFrame(
      v11Frame, sizeof(v11Frame), decoded));
  assert(decoded.protocolVersion == bearamin_bridge::kVersionV11);
  assert(decoded.sourceEpochMs == v11.sourceEpochMs);
  assert(decoded.slewTauMs == 232);
  assert(decoded.trajectoryStartNorm[4] == 1400);
  bearamin_bridge::Parser v11Parser;
  complete = false;
  for (size_t i = 0; i < sizeof(v11Frame); ++i) {
    complete = v11Parser.Push(v11Frame[i], parsed);
  }
  assert(complete && parsed.protocolVersion == bearamin_bridge::kVersionV11);
  bearamin_bridge::writeLe16(v11Frame, 4 + 74, 0);
  bearamin_bridge::writeLe16(
      v11Frame, 4 + bearamin_bridge::kPayloadSizeV11,
      bearamin_bridge::crc16Ccitt(
          v11Frame, 4 + bearamin_bridge::kPayloadSizeV11));
  assert(!bearamin_bridge::decodeFrame(
      v11Frame, sizeof(v11Frame), decoded));

  bearamin_bridge::DptProbeParser probeParser;
  assert(!probeParser.Push('D'));
  assert(!probeParser.Push('P'));
  assert(!probeParser.Push('T'));
  assert(probeParser.Push('\n'));
  assert(!probeParser.Push('x'));
  assert(!probeParser.Push('D'));
  assert(!probeParser.Push('D'));
  assert(!probeParser.Push('P'));
  assert(!probeParser.Push('T'));
  assert(probeParser.Push('\n'));

  bearamin_bridge::DptStatus dptStatus = {};
  assert(bearamin_bridge::decodeDptStatusLine(
      "DPTS,3,04,0011223380AAFEFF,0003,2A", dptStatus));
  assert(dptStatus.protocolVersion == 3);
  assert(dptStatus.modeIndex == 4);
  assert(dptStatus.hasControlSnapshot);
  assert(dptStatus.controls[0] == 0x00);
  assert(dptStatus.controls[3] == 0x33);
  assert(dptStatus.controls[4] == 0x80);
  assert(dptStatus.controls[7] == 0xFF);
  assert(dptStatus.flags == 0x0003);
  assert(dptStatus.resonatorControlsRevision == 0x2A);

  assert(!bearamin_bridge::decodeDptStatusLine("DPTM,4", dptStatus));
  assert(!bearamin_bridge::decodeDptStatusLine("DPTS,2,05,0011223380AAFEFF,0003", dptStatus));
  assert(!bearamin_bridge::decodeDptStatusLine("DPTS,1,05,0011223380AAFEF,0003", dptStatus));
  assert(!bearamin_bridge::decodeDptStatusLine("DPTS,3,04,0011223380AAFEFF,0003", dptStatus));

  bearamin_bridge::DptPerformance dptPerformance = {};
  assert(bearamin_bridge::decodeDptPerformanceLine(
      "DPTP,1,01234567,00000321,00000002,00000003,00000004,"
      "00000456,89ABCDEF,000000F0,00005208",
      dptPerformance));
  assert(dptPerformance.protocolVersion == 1);
  assert(dptPerformance.callbackCount == 0x01234567u);
  assert(dptPerformance.maxCallbackUs == 0x321u);
  assert(dptPerformance.deadlineMisses == 2);
  assert(dptPerformance.lateStarts == 3);
  assert(dptPerformance.missedPeriods == 4);
  assert(dptPerformance.maxStartGapUs == 0x456u);
  assert(dptPerformance.txCallCount == 0x89ABCDEFu);
  assert(dptPerformance.maxIrqOffUs == 0xF0u);
  assert(dptPerformance.maxTxCallUs == 0x5208u);
  assert(!bearamin_bridge::decodeDptPerformanceLine(
      "DPTP,2,01234567,00000321,00000002,00000003,00000004,"
      "00000456,00000003,00123456,00000321",
      dptPerformance));
  assert(!bearamin_bridge::decodeDptPerformanceLine(
      "DPTP,1,01234567,00000321,00000002,00000003,00000004,"
      "00000456,89ABCDEF,000000F0,0000520",
      dptPerformance));

  bearamin_bridge::DptTrajectoryDiagnostics dptTrajectoryDiagnostics = {};
  assert(bearamin_bridge::decodeDptTrajectoryDiagnosticsLine(
      "DPTD,1,0B,00E8,11223344,0011,00000001,00000002,"
      "00000003,00000004,00000005,00000006,00000007",
      dptTrajectoryDiagnostics));
  assert(dptTrajectoryDiagnostics.bridgeProtocolVersion == 11);
  assert(dptTrajectoryDiagnostics.slewTauMs == 232);
  assert(dptTrajectoryDiagnostics.sourceEpochMs == 0x11223344u);
  assert(dptTrajectoryDiagnostics.sourceAgeMs == 17);
  assert(dptTrajectoryDiagnostics.v10PacketCount == 1);
  assert(dptTrajectoryDiagnostics.v11PacketCount == 2);
  assert(dptTrajectoryDiagnostics.correctionCount == 3);
  assert(dptTrajectoryDiagnostics.maxCorrectionNorm == 4);
  assert(dptTrajectoryDiagnostics.invalidKeyframeCount == 5);
  assert(dptTrajectoryDiagnostics.staleTransitions == 6);
  assert(dptTrajectoryDiagnostics.deadlineMisses == 7);
  assert(!bearamin_bridge::decodeDptTrajectoryDiagnosticsLine(
      "DPTD,1,0B,00E8,11223344,0011,00000001,00000002,"
      "00000003,00000004,00000005,00000006,0000000",
      dptTrajectoryDiagnostics));

  bearamin_sidecar::PresetFrame preset = {};
  preset.sequence = 22;
  preset.op = bearamin_sidecar::PRESET_OP_LINE;
  const char *presetLine = "BPLS,22\n";
  preset.length = static_cast<uint8_t>(strlen(presetLine));
  memcpy(preset.data, presetLine, preset.length);
  uint8_t presetFrame[bearamin_sidecar::kPresetFrameSize] = {};
  assert(bearamin_sidecar::encodePresetFrame(preset, presetFrame, sizeof(presetFrame)) ==
         bearamin_sidecar::kPresetFrameSize);
  bearamin_sidecar::PresetFrame decodedPreset = {};
  assert(bearamin_sidecar::decodePresetFrame(presetFrame, sizeof(presetFrame), decodedPreset));
  assert(decodedPreset.sequence == 22);
  assert(decodedPreset.op == bearamin_sidecar::PRESET_OP_LINE);
  assert(decodedPreset.length == strlen(presetLine));
  assert(memcmp(decodedPreset.data, presetLine, decodedPreset.length) == 0);
  bearamin_sidecar::Parser sidecarPresetParser;
  bearamin_sidecar::StateFrame unusedStateForPreset = {};
  bearamin_sidecar::CommandFrame unusedCommandForPreset = {};
  bearamin_sidecar::MatrixCellsFrame unusedCellsForPreset = {};
  uint8_t presetParsedType = 0;
  for (size_t i = 0; i < sizeof(presetFrame); ++i) {
    presetParsedType = sidecarPresetParser.Push(
        presetFrame[i], unusedStateForPreset, unusedCommandForPreset, unusedCellsForPreset, decodedPreset);
  }
  assert(presetParsedType == bearamin_sidecar::FRAME_PRESET);
  presetFrame[9] ^= 0x01;
  assert(!bearamin_sidecar::decodePresetFrame(presetFrame, sizeof(presetFrame), decodedPreset));

  bearamin_sidecar::RuntimeSettingsFrame runtimeSettings = {};
  runtimeSettings.sequence = 91;
  runtimeSettings.dptModeIndex = 3;
  runtimeSettings.modeAnnouncementsEnabled = true;
  runtimeSettings.controlsInitializedBySidecar = true;
  runtimeSettings.cvSlewStep = 27;
  runtimeSettings.dptControlFlags = 0x02A5;
  runtimeSettings.dptReserved0 = 0xBEEF;
  runtimeSettings.dptReserved1 = 0x34;
  runtimeSettings.dptReserved2 =
      bearamin_bridge::CONTROL_RESONATOR_MAPPING_MAGIC;
  runtimeSettings.dptSourceFlags2 = 0x12;
  runtimeSettings.resonatorControlsRevision = 0x5A;
  for (uint8_t i = 0; i < bearamin_bridge::kLaneDepthCount; ++i) {
    runtimeSettings.laneDepths[i] = static_cast<int8_t>(-75 + i * 25);
  }
  for (uint8_t i = 0;
       i < bearamin_bridge::kResonatorWebControlSize;
       ++i) {
    runtimeSettings.resonatorWebControls[i] =
        static_cast<uint8_t>(i * 7);
  }
  uint8_t runtimeFrame[bearamin_sidecar::kRuntimeSettingsFrameSize] = {};
  assert(bearamin_sidecar::encodeRuntimeSettingsFrame(
             runtimeSettings, runtimeFrame, sizeof(runtimeFrame)) ==
         bearamin_sidecar::kRuntimeSettingsFrameSize);
  bearamin_sidecar::RuntimeSettingsFrame decodedRuntimeSettings = {};
  assert(bearamin_sidecar::decodeRuntimeSettingsFrame(
      runtimeFrame, sizeof(runtimeFrame), decodedRuntimeSettings));
  assert(decodedRuntimeSettings.sequence == runtimeSettings.sequence);
  assert(decodedRuntimeSettings.dptModeIndex == runtimeSettings.dptModeIndex);
  assert(decodedRuntimeSettings.modeAnnouncementsEnabled);
  assert(decodedRuntimeSettings.controlsInitializedBySidecar);
  assert(decodedRuntimeSettings.cvSlewStep == runtimeSettings.cvSlewStep);
  assert(decodedRuntimeSettings.dptControlFlags ==
         runtimeSettings.dptControlFlags);
  assert(decodedRuntimeSettings.dptReserved0 == runtimeSettings.dptReserved0);
  assert(decodedRuntimeSettings.dptReserved1 == runtimeSettings.dptReserved1);
  assert(decodedRuntimeSettings.dptReserved2 == runtimeSettings.dptReserved2);
  assert(decodedRuntimeSettings.dptSourceFlags2 ==
         runtimeSettings.dptSourceFlags2);
  assert(decodedRuntimeSettings.resonatorControlsRevision ==
         runtimeSettings.resonatorControlsRevision);
  assert(memcmp(decodedRuntimeSettings.laneDepths,
                runtimeSettings.laneDepths,
                sizeof(runtimeSettings.laneDepths)) == 0);
  assert(memcmp(decodedRuntimeSettings.resonatorWebControls,
                runtimeSettings.resonatorWebControls,
                sizeof(runtimeSettings.resonatorWebControls)) == 0);
  bearamin_sidecar::Parser runtimeParser;
  bearamin_sidecar::SensorDiagnosticsFrame unusedDiagnosticsForRuntime = {};
  uint8_t runtimeParsedType = 0;
  for (size_t i = 0; i < sizeof(runtimeFrame); ++i) {
    runtimeParsedType = runtimeParser.Push(
        runtimeFrame[i],
        unusedStateForPreset,
        unusedCommandForPreset,
        unusedCellsForPreset,
        decodedPreset,
        unusedDiagnosticsForRuntime,
        decodedRuntimeSettings);
  }
  assert(runtimeParsedType == bearamin_sidecar::FRAME_RUNTIME_SETTINGS);
  runtimeFrame[12] ^= 0x01;
  assert(!bearamin_sidecar::decodeRuntimeSettingsFrame(
      runtimeFrame, sizeof(runtimeFrame), decodedRuntimeSettings));
  assert(dpt_multimode::IsValidPresetName("default"));
  assert(dpt_multimode::IsValidPresetName("Patch_01-A"));
  assert(!dpt_multimode::IsValidPresetName(""));
  assert(!dpt_multimode::IsValidPresetName("../bad"));
  const uint8_t presetBytes[] = {0x7B, 0x22, 0x78, 0x22, 0x7D};
  char presetHex[16] = {};
  dpt_multimode::PresetHexEncode(presetBytes, sizeof(presetBytes), presetHex, sizeof(presetHex));
  assert(strcmp(presetHex, "7B2278227D") == 0);
  uint8_t decodedPresetBytes[8] = {};
  size_t decodedPresetSize = 0;
  assert(dpt_multimode::PresetHexDecode(
      presetHex, decodedPresetBytes, sizeof(decodedPresetBytes), decodedPresetSize));
  assert(decodedPresetSize == sizeof(presetBytes));
  assert(memcmp(decodedPresetBytes, presetBytes, sizeof(presetBytes)) == 0);
  assert(!dpt_multimode::PresetHexDecode("7B2", decodedPresetBytes, sizeof(decodedPresetBytes), decodedPresetSize));

  bearamin_sidecar::StateFrame sidecarState = {};
  sidecarState.sequence = 9;
  sidecarState.stateFlags = bearamin_sidecar::STATE_DPT_READY |
                            bearamin_sidecar::STATE_IMU_READY |
                            bearamin_sidecar::STATE_IMPACT_GATE |
                            bearamin_sidecar::STATE_MATRIX_OTA_MODE |
                            bearamin_sidecar::STATE_MATRIX_OTA_ACTIVE |
                            bearamin_sidecar::STATE_DPT_CONTROL_TX_QUIET |
                            bearamin_sidecar::STATE_SPANK_GATE;
  sidecarState.uptimeMs = 123456;
  sidecarState.reserved0 = 321;
  sidecarState.matrixXNorm = 2048;
  sidecarState.matrixYNorm = 4095;
  sidecarState.impactVelocityNorm = 3000;
  sidecarState.impactAgeMs = 17;
  sidecarState.accelXMilliG = -120;
  sidecarState.accelYMilliG = 30;
  sidecarState.accelZMilliG = 1010;
  sidecarState.dptModeIndex = 5;
  sidecarState.dptProtocolVersion = 1;
  sidecarState.dptProbeTxCount = 11;
  sidecarState.dptFrameTxCount = 22;
  sidecarState.dptPresetTxCount = 33;
  sidecarState.dptRxByteCount = 44;
  sidecarState.dptStatusLineCount = 55;
  sidecarState.dptPresetLineCount = 66;
  sidecarState.dptControls[0] = 0;
  sidecarState.dptControls[1] = 16;
  sidecarState.dptControls[2] = 64;
  sidecarState.dptControls[3] = 96;
  sidecarState.dptControls[4] = 128;
  sidecarState.dptControls[5] = 192;
  sidecarState.dptControls[6] = 224;
  sidecarState.dptControls[7] = 255;
  sidecarState.dptResonatorControlsRevision = 0x2A;
  sidecarState.pitchNorm = 1234;
  sidecarState.rollNorm = 3456;
  sidecarState.orientationConfidence = 219;
  sidecarState.trajectoryStartNorm[0] = 100;
  sidecarState.trajectoryStartNorm[4] = 400;
  sidecarState.trajectoryTauMs = 232;
  sidecarState.cellsMm[0] = 100;
  sidecarState.cellsMm[7] = 800;
  uint8_t sidecarFrame[bearamin_sidecar::kStateFrameSize] = {};
  assert(bearamin_sidecar::encodeStateFrame(sidecarState, sidecarFrame, sizeof(sidecarFrame)) ==
         bearamin_sidecar::kStateFrameSize);
  bearamin_sidecar::StateFrame decodedState = {};
  assert(bearamin_sidecar::decodeStateFrame(sidecarFrame, sizeof(sidecarFrame), decodedState));
  assert(decodedState.sequence == 9);
  assert(decodedState.uptimeMs == 123456);
  assert((decodedState.stateFlags & bearamin_sidecar::STATE_MATRIX_OTA_MODE) != 0);
  assert(decodedState.dptProbeTxCount == 11);
  assert(decodedState.dptFrameTxCount == 22);
  assert(decodedState.dptPresetTxCount == 33);
  assert(decodedState.dptRxByteCount == 44);
  assert(decodedState.dptStatusLineCount == 55);
  assert(decodedState.dptPresetLineCount == 66);
  assert(decodedState.dptControls[0] == 0);
  assert(decodedState.dptControls[4] == 128);
  assert(decodedState.dptControls[7] == 255);
  assert(decodedState.dptResonatorControlsRevision == 0x2A);
  assert(decodedState.pitchNorm == 1234);
  assert(decodedState.rollNorm == 3456);
  assert(decodedState.orientationConfidence == 219);
  assert(decodedState.trajectoryValid);
  assert(decodedState.trajectoryStartNorm[4] == 400);
  assert(decodedState.trajectoryTauMs == 232);
  assert((decodedState.stateFlags & bearamin_sidecar::STATE_SPANK_GATE) != 0);
  assert((decodedState.stateFlags & bearamin_sidecar::STATE_MATRIX_OTA_ACTIVE) != 0);
  assert((decodedState.stateFlags &
          bearamin_sidecar::STATE_DPT_CONTROL_TX_QUIET) != 0);
  assert(decodedState.impactVelocityNorm == 3000);
  assert(decodedState.accelXMilliG == -120);
  assert(decodedState.cellsMm[7] == 800);

  uint8_t mismatchedState[bearamin_sidecar::kStateFrameSize] = {};
  memcpy(mismatchedState, sidecarFrame, sizeof(mismatchedState));
  mismatchedState[2] = static_cast<uint8_t>(bearamin_sidecar::kVersion - 1);
  bearamin_sidecar::writeLe16(
      mismatchedState,
      4 + bearamin_sidecar::kStatePayloadSize,
      bearamin_sidecar::crc16Ccitt(
          mismatchedState, 4 + bearamin_sidecar::kStatePayloadSize));
  assert(!bearamin_sidecar::decodeStateFrame(
      mismatchedState, sizeof(mismatchedState), decodedState));
  bearamin_sidecar::Parser mismatchParser;
  bearamin_sidecar::CommandFrame mismatchCommand = {};
  uint8_t mismatchType = 0;
  for (uint8_t byte : mismatchedState) {
    mismatchType = mismatchParser.Push(byte, decodedState, mismatchCommand);
  }
  assert(mismatchType == 0);

  bearamin_sidecar::CommandFrame command = {};
  command.sequence = 10;
  command.command = bearamin_sidecar::COMMAND_ENTER_MATRIX_OTA;
  command.arg0 = 1;
  uint8_t commandFrame[bearamin_sidecar::kCommandFrameSize] = {};
  assert(bearamin_sidecar::encodeCommandFrame(
             command, bearamin_sidecar::FRAME_COMMAND, commandFrame, sizeof(commandFrame)) ==
         bearamin_sidecar::kCommandFrameSize);
  bearamin_sidecar::Parser sidecarParser;
  bearamin_sidecar::CommandFrame decodedCommand = {};
  uint8_t parsedType = sidecarParser.Push(0xFF, decodedState, decodedCommand);
  assert(parsedType == 0);
  for (size_t i = 0; i < sizeof(commandFrame); ++i) {
    parsedType = sidecarParser.Push(commandFrame[i], decodedState, decodedCommand);
  }
  assert(parsedType == bearamin_sidecar::FRAME_COMMAND);
  assert(decodedCommand.sequence == 10);
  assert(decodedCommand.command == bearamin_sidecar::COMMAND_ENTER_MATRIX_OTA);
  commandFrame[2] = static_cast<uint8_t>(bearamin_sidecar::kVersion - 1);
  bearamin_sidecar::writeLe16(
      commandFrame,
      4 + bearamin_sidecar::kCommandPayloadSize,
      bearamin_sidecar::crc16Ccitt(
          commandFrame, 4 + bearamin_sidecar::kCommandPayloadSize));
  assert(!bearamin_sidecar::decodeCommandFrame(
      commandFrame, sizeof(commandFrame),
      bearamin_sidecar::FRAME_COMMAND, decodedCommand));

  bearamin_sidecar::MatrixCellsFrame cells = {};
  cells.sequence = 11;
  cells.validCount = 12;
  cells.blobCount = 9;
  cells.sensorAgeMs = 34;
  cells.cellsMm[0] = 101;
  cells.cellsMm[8] = 202;
  cells.cellsMm[63] = 909;
  uint8_t cellsFrame[bearamin_sidecar::kMatrixCellsFrameSize] = {};
  assert(bearamin_sidecar::encodeMatrixCellsFrame(cells, cellsFrame, sizeof(cellsFrame)) ==
         bearamin_sidecar::kMatrixCellsFrameSize);
  bearamin_sidecar::MatrixCellsFrame decodedCells = {};
  assert(bearamin_sidecar::decodeMatrixCellsFrame(cellsFrame, sizeof(cellsFrame), decodedCells));
  assert(decodedCells.sequence == 11);
  assert(decodedCells.validCount == 12);
  assert(decodedCells.blobCount == 9);
  assert(decodedCells.sensorAgeMs == 34);
  assert(decodedCells.cellsMm[0] == 101);
  assert(decodedCells.cellsMm[8] == 202);
  assert(decodedCells.cellsMm[63] == 909);

  for (size_t i = 0; i < sizeof(cellsFrame); ++i) {
    parsedType = sidecarParser.Push(cellsFrame[i], decodedState, decodedCommand, decodedCells);
  }
  assert(parsedType == bearamin_sidecar::FRAME_MATRIX_CELLS);
  assert(decodedCells.sequence == 11);
  assert(decodedCells.cellsMm[63] == 909);

  bearamin_sidecar::SensorDiagnosticsFrame diagnostics = {};
  diagnostics.sequence = 12;
  diagnostics.reserved0 = 17;
  diagnostics.reserved1 = 3;
  diagnostics.recoveryFailures = 2;
  diagnostics.reserved2 = 44;
  diagnostics.matrixAgeMs = 55;
  diagnostics.recoveryReason =
      bearamin_sidecar::SENSOR_RECOVERY_MATRIX_NO_FIRST_SAMPLE;
  diagnostics.reserved3 = 2;
  diagnostics.reserved4 = 4;
  diagnostics.busFlags =
      bearamin_sidecar::SENSOR_BUS_SDA_HIGH |
      bearamin_sidecar::SENSOR_BUS_SCL_HIGH |
      bearamin_sidecar::SENSOR_BUS_RESERVED_3;
  uint8_t diagnosticsFrame[bearamin_sidecar::kSensorDiagnosticsFrameSize] = {};
  assert(bearamin_sidecar::encodeSensorDiagnosticsFrame(
             diagnostics, diagnosticsFrame, sizeof(diagnosticsFrame)) ==
         bearamin_sidecar::kSensorDiagnosticsFrameSize);
  bearamin_sidecar::SensorDiagnosticsFrame decodedDiagnostics = {};
  assert(bearamin_sidecar::decodeSensorDiagnosticsFrame(
      diagnosticsFrame, sizeof(diagnosticsFrame), decodedDiagnostics));
  assert(decodedDiagnostics.sequence == 12);
  assert(decodedDiagnostics.reserved0 == 17);
  assert(decodedDiagnostics.recoveryFailures == 2);
  assert(decodedDiagnostics.recoveryReason ==
         bearamin_sidecar::SENSOR_RECOVERY_MATRIX_NO_FIRST_SAMPLE);
  assert(decodedDiagnostics.reserved3 == 2);
  assert((decodedDiagnostics.busFlags &
          bearamin_sidecar::SENSOR_BUS_RESERVED_3) != 0);
  for (size_t i = 0; i < sizeof(diagnosticsFrame); ++i) {
    parsedType = sidecarParser.Push(
        diagnosticsFrame[i],
        decodedState,
        decodedCommand,
        decodedCells,
        decodedPreset,
        decodedDiagnostics);
  }
  assert(parsedType == bearamin_sidecar::FRAME_SENSOR_DIAGNOSTICS);
  assert(decodedDiagnostics.reserved1 == 3);

  command.sequence = 13;
  command.command = bearamin_sidecar::COMMAND_SET_ENGINEERING_STREAM;
  command.arg0 = 1;
  assert(bearamin_sidecar::encodeCommandFrame(
             command, bearamin_sidecar::FRAME_COMMAND, commandFrame, sizeof(commandFrame)) ==
         bearamin_sidecar::kCommandFrameSize);
  for (size_t i = 0; i < sizeof(commandFrame); ++i) {
    parsedType = sidecarParser.Push(commandFrame[i], decodedState, decodedCommand);
  }
  assert(parsedType == bearamin_sidecar::FRAME_COMMAND);
  assert(decodedCommand.sequence == 13);
  assert(decodedCommand.command == bearamin_sidecar::COMMAND_SET_ENGINEERING_STREAM);
  assert(decodedCommand.arg0 == 1);

  command.sequence = 14;
  command.command = bearamin_sidecar::COMMAND_SET_MODE_ANNOUNCEMENTS;
  command.arg0 = 1;
  assert(bearamin_sidecar::encodeCommandFrame(
             command, bearamin_sidecar::FRAME_COMMAND, commandFrame, sizeof(commandFrame)) ==
         bearamin_sidecar::kCommandFrameSize);
  for (size_t i = 0; i < sizeof(commandFrame); ++i) {
    parsedType = sidecarParser.Push(commandFrame[i], decodedState, decodedCommand);
  }
  assert(parsedType == bearamin_sidecar::FRAME_COMMAND);
  assert(decodedCommand.command ==
         bearamin_sidecar::COMMAND_SET_MODE_ANNOUNCEMENTS);
  assert(decodedCommand.arg0 == 1);

  command.sequence = 15;
  command.command = bearamin_sidecar::COMMAND_SET_RESONATOR_WEB_PAGE;
  command.arg0 = 0x0217;
  command.arg1 = 0x0100;
  command.arg2 = 0x0302;
  command.arg3 = 0x0504;
  assert(bearamin_sidecar::encodeCommandFrame(
             command, bearamin_sidecar::FRAME_COMMAND, commandFrame, sizeof(commandFrame)) ==
         bearamin_sidecar::kCommandFrameSize);
  for (size_t i = 0; i < sizeof(commandFrame); ++i) {
    parsedType = sidecarParser.Push(commandFrame[i], decodedState, decodedCommand);
  }
  assert(parsedType == bearamin_sidecar::FRAME_COMMAND);
  assert(decodedCommand.command == bearamin_sidecar::COMMAND_SET_RESONATOR_WEB_PAGE);
  assert(decodedCommand.arg0 == 0x0217);
  assert(decodedCommand.arg3 == 0x0504);

  bearamin_sidecar::ResonatorWebPageAssembler assembler;
  uint8_t assembledRevision = 0;
  uint8_t assembledControls[bearamin_bridge::kResonatorWebControlSize] = {};
  command.command = bearamin_sidecar::COMMAND_SET_RESONATOR_WEB_PAGE;
  command.arg0 = 0x0001;
  command.arg1 = 0x0100;
  command.arg2 = 0x0302;
  command.arg3 = 0x0504;
  assert(!assembler.Apply(command, assembledRevision, assembledControls));
  assert(assembler.PageMask() == 0x01);
  command.arg0 = 0x0102;
  command.arg1 = 0x0706;
  command.arg2 = 0x0908;
  command.arg3 = 0x0B0A;
  assert(!assembler.Apply(command, assembledRevision, assembledControls));
  assert(assembler.PageMask() == 0x02);
  command.arg0 = 0x0202;
  command.arg1 = 0x0D0C;
  command.arg2 = 0x0F0E;
  command.arg3 = 0;
  assert(!assembler.Apply(command, assembledRevision, assembledControls));
  command.arg0 = 0x0002;
  command.arg1 = 0x0100;
  command.arg2 = 0x0302;
  command.arg3 = 0x0504;
  assert(assembler.Apply(command, assembledRevision, assembledControls));
  assert(assembledRevision == 2);
  for (uint8_t i = 0; i < bearamin_bridge::kResonatorWebControlSize; ++i) {
    assert(assembledControls[i] == i);
  }

  bearamin_sidecar::MatrixPerformanceFrame performance = {};
  performance.sequence = 41;
  performance.effectiveRateMilliHz = 59875;
  performance.intervalMaxUs = 18123;
  performance.readMaxUs = 3456;
  performance.processMaxUs = 789;
  performance.loopGapMaxUs = 4321;
  performance.freeHeapBytes = 120000;
  performance.minimumFreeHeapBytes = 91000;
  performance.readErrors = 2;
  performance.recoveryStackWords = 777;
  performance.sourceHz = 15;
  performance.outputHz = 30;
  performance.readyPollStatus = 0x84;
  performance.readyPollValue = 1;
  uint8_t performanceFrame[bearamin_sidecar::kMatrixPerformanceFrameSize] = {};
  assert(bearamin_sidecar::encodeMatrixPerformanceFrame(
             performance, performanceFrame, sizeof(performanceFrame)) ==
         bearamin_sidecar::kMatrixPerformanceFrameSize);
  bearamin_sidecar::MatrixPerformanceFrame decodedPerformance = {};
  assert(bearamin_sidecar::decodeMatrixPerformanceFrame(
      performanceFrame, sizeof(performanceFrame), decodedPerformance));
  assert(decodedPerformance.effectiveRateMilliHz == 59875);
  assert(decodedPerformance.intervalMaxUs == 18123);
  assert(decodedPerformance.sourceHz == 15);
  assert(decodedPerformance.outputHz == 30);
  assert(decodedPerformance.readyPollStatus == 0x84);
  assert(decodedPerformance.readyPollValue == 1);

  bearamin_obs::Subscribe subscription = {};
  subscription.streams =
      bearamin_obs::STREAM_STATE | bearamin_obs::STREAM_CELLS;
  subscription.clientNonce = 0x12345678u;
  uint8_t subscribePacket[bearamin_obs::kSubscribePacketSize] = {};
  assert(bearamin_obs::encodeSubscribe(
             subscription, 0x3344, subscribePacket, sizeof(subscribePacket)) ==
         bearamin_obs::kSubscribePacketSize);
  assert(subscribePacket[0] == 'B' && subscribePacket[3] == '1');
  assert(bearamin_obs::readLe16(subscribePacket + 8) == 0x3344);
  bearamin_obs::Subscribe decodedSubscription = {};
  assert(bearamin_obs::decodeSubscribe(
      subscribePacket, sizeof(subscribePacket), decodedSubscription));
  assert(decodedSubscription.streams == subscription.streams);
  assert(decodedSubscription.clientNonce == subscription.clientNonce);
  subscribePacket[4] = 99;
  assert(!bearamin_obs::decodeSubscribe(
      subscribePacket, sizeof(subscribePacket), decodedSubscription));
  subscribePacket[4] = bearamin_obs::kVersion;
  subscribePacket[sizeof(subscribePacket) - 1] ^= 0x80;
  assert(!bearamin_obs::decodeSubscribe(
      subscribePacket, sizeof(subscribePacket), decodedSubscription));

  bearamin_obs::State overlayState = {};
  overlayState.sourceUptimeMs = 0x01020304u;
  overlayState.stateFlags =
      bearamin_sidecar::STATE_MATRIX_READY |
      bearamin_sidecar::STATE_MATRIX_PRESENT;
  overlayState.sensorAgeMs = 17;
  overlayState.matrixXNorm = 1024;
  overlayState.matrixYNorm = 3072;
  overlayState.matrixZNorm = 2048;
  overlayState.matrixXMilli = 250;
  overlayState.matrixYMilli = 750;
  overlayState.matrixZMm = 321;
  overlayState.impactVelocityNorm = 1234;
  overlayState.impactAgeMs = 25;
  overlayState.pitchNorm = 1536;
  overlayState.rollNorm = 2560;
  overlayState.matrixValidCount = 48;
  overlayState.matrixBlobCount = 12;
  uint8_t statePacket[bearamin_obs::kStatePacketSize] = {};
  assert(bearamin_obs::encodeState(
             overlayState, 9, statePacket, sizeof(statePacket)) ==
         bearamin_obs::kStatePacketSize);
  assert(bearamin_obs::validPacket(
      statePacket, sizeof(statePacket),
      bearamin_obs::PACKET_STATE, bearamin_obs::kStatePayloadSize));
  assert(bearamin_obs::readLe32(
             statePacket + bearamin_obs::kHeaderSize) == 0x01020304u);
  assert(bearamin_obs::readLe16(
             statePacket + bearamin_obs::kHeaderSize + 8) == 1024);
  assert(bearamin_obs::readLe16(
             statePacket + bearamin_obs::kHeaderSize + 24) == 1536);
  assert(bearamin_obs::readLe16(
             statePacket + bearamin_obs::kHeaderSize + 26) == 2560);
  for (uint8_t axis = 0; axis < 5; ++axis) {
    overlayState.trajectoryStartNorm[axis] = static_cast<uint16_t>(500 + axis);
  }
  overlayState.slewTauMs = 232;
  uint8_t statePacketV4[bearamin_obs::kStatePacketSizeV4] = {};
  assert(bearamin_obs::encodeStateV4(
             overlayState, 10, statePacketV4, sizeof(statePacketV4)) ==
         bearamin_obs::kStatePacketSizeV4);
  assert(statePacketV4[4] == bearamin_obs::kVersionV4);
  assert(bearamin_obs::validPacket(
      statePacketV4, sizeof(statePacketV4), bearamin_obs::PACKET_STATE,
      bearamin_obs::kStatePayloadSizeV4, bearamin_obs::kVersionV4));
  bearamin_obs::State decodedOverlayState = {};
  uint8_t overlayVersion = 0;
  assert(bearamin_obs::decodeState(
      statePacketV4, sizeof(statePacketV4), decodedOverlayState, overlayVersion));
  assert(overlayVersion == bearamin_obs::kVersionV4);
  assert(decodedOverlayState.trajectoryStartNorm[4] == 504);
  assert(decodedOverlayState.slewTauMs == 232);
  statePacketV4[sizeof(statePacketV4) - 1] ^= 0x80;
  assert(!bearamin_obs::decodeState(
      statePacketV4, sizeof(statePacketV4), decodedOverlayState, overlayVersion));

  bearamin_obs::Cells overlayCells = {};
  overlayCells.sourceUptimeMs = 99;
  overlayCells.sensorAgeMs = 7;
  overlayCells.matrixValidCount = 64;
  overlayCells.matrixBlobCount = 16;
  for (uint8_t i = 0; i < 64; ++i) {
    overlayCells.cellsMm[i] = static_cast<uint16_t>(100 + i);
  }
  uint8_t cellsPacket[bearamin_obs::kCellsPacketSize] = {};
  assert(bearamin_obs::encodeCells(
             overlayCells, 10, cellsPacket, sizeof(cellsPacket)) ==
         bearamin_obs::kCellsPacketSize);
  assert(bearamin_obs::validPacket(
      cellsPacket, sizeof(cellsPacket),
      bearamin_obs::PACKET_CELLS, bearamin_obs::kCellsPayloadSize));
  assert(bearamin_obs::readLe16(
             cellsPacket + bearamin_obs::kHeaderSize + 8 + (63 * 2)) == 163);
  assert(!bearamin_obs::leaseActive(1000, 0));
  assert(bearamin_obs::leaseActive(1000, 1001));
  assert(!bearamin_obs::leaseActive(1001, 1001));
  assert(bearamin_obs::leaseActive(0xFFFFFFF0u, 0x00000010u));
  assert(bearamin_obs::kMulticastAddress[0] == 239);
  assert(bearamin_obs::kMulticastPort == 51338);

  bearamin_obs::SubscriberLeaseTable leaseTable;
  assert(leaseTable.activeCount(1000) == 0);
  assert(leaseTable.streamUnion(1000) == 0);
  assert(leaseTable.update(
             0x01020304u,
             11,
             bearamin_obs::STREAM_STATE,
             1000) == bearamin_obs::LEASE_INSERTED);
  assert(leaseTable.update(
             0x05060708u,
             22,
             bearamin_obs::STREAM_STATE |
                 bearamin_obs::STREAM_CELLS,
             1000) == bearamin_obs::LEASE_INSERTED);
  assert(leaseTable.activeCount(1000) == 2);
  assert(leaseTable.streamUnion(1000) ==
         (bearamin_obs::STREAM_STATE |
          bearamin_obs::STREAM_CELLS));
  assert(leaseTable.update(
             0x01020304u, 11, bearamin_obs::STREAM_STATE,
             bearamin_obs::kVersionV4, 2000) ==
         bearamin_obs::LEASE_REFRESHED);
  assert(leaseTable.streamUnionForVersion(2000, bearamin_obs::kVersionV4) ==
         bearamin_obs::STREAM_STATE);
  assert(leaseTable.activeCountForVersion(
             2000, bearamin_obs::kVersionV4) == 1);
  assert(leaseTable.update(
             0x01020304u,
             11,
             bearamin_obs::STREAM_STATE,
             2000) == bearamin_obs::LEASE_REFRESHED);
  assert(leaseTable.streamUnionForVersion(2000, bearamin_obs::kVersionV3) ==
         (bearamin_obs::STREAM_STATE | bearamin_obs::STREAM_CELLS));
  assert(leaseTable.activeCountForVersion(
             2000, bearamin_obs::kVersionV3) == 1);
  assert(leaseTable.update(
             0x090A0B0Cu,
             11,
             bearamin_obs::STREAM_STATE,
             2000) == bearamin_obs::LEASE_INSERTED);
  assert(leaseTable.update(
             0x0D0E0F10u,
             44,
             bearamin_obs::STREAM_STATE,
             2000) == bearamin_obs::LEASE_INSERTED);
  assert(leaseTable.update(
             0x11121314u,
             55,
             bearamin_obs::STREAM_STATE,
             2000) == bearamin_obs::LEASE_TABLE_FULL);
  const uint8_t pendingPacketTypes =
      ((leaseTable.streamUnion(2000) &
        bearamin_obs::STREAM_STATE) != 0 ? 1 : 0) +
      ((leaseTable.streamUnion(2000) &
        bearamin_obs::STREAM_CELLS) != 0 ? 1 : 0);
  assert(pendingPacketTypes == 2);
  assert(leaseTable.expire(4000) == 1);
  assert(leaseTable.activeCount(4000) == 3);
  assert(leaseTable.streamUnion(4000) ==
         bearamin_obs::STREAM_STATE);
  assert(leaseTable.expire(5000) == 3);
  assert(leaseTable.activeCount(5000) == 0);

  return 0;
}
