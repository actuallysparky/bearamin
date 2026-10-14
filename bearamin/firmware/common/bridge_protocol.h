#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bearamin_bridge {

static constexpr uint8_t kStart0 = 0xA5;
static constexpr uint8_t kStart1 = 0x5A;
static constexpr uint8_t kVersionV10 = 10;
static constexpr uint8_t kVersionV11 = 11;
// Keep these aliases for v10 callers while a producer/receiver pair migrates.
static constexpr uint8_t kVersion = kVersionV10;
static constexpr uint8_t kPayloadSizeV10 = 64;
static constexpr uint8_t kPayloadSizeV11 = 76;
static constexpr uint8_t kPayloadSize = kPayloadSizeV10;
static constexpr size_t kFrameSizeV10 = 2 + 1 + 1 + kPayloadSizeV10 + 2;
static constexpr size_t kFrameSizeV11 = 2 + 1 + 1 + kPayloadSizeV11 + 2;
static constexpr size_t kFrameSize = kFrameSizeV10;
static constexpr size_t kMaxFrameSize = kFrameSizeV11;
static_assert(kFrameSizeV10 == 70, "Bridge v10 frame size drift");
static_assert(kFrameSizeV11 == 82, "Bridge v11 frame size drift");
static constexpr uint8_t kDptProbe[] = {'D', 'P', 'T', '\n'};
static constexpr size_t kDptProbeSize = sizeof(kDptProbe);
static constexpr uint8_t kDptStatusProtocolVersion = 3;
static constexpr uint8_t kDptControlCount = 8;
static constexpr uint8_t kResonatorWebControlSize = 16;
static constexpr uint8_t kLaneDepthCount = 6;

enum ResonatorWebSource : uint8_t {
  RESONATOR_WEB_SOURCE_NONE = 0,
  RESONATOR_WEB_SOURCE_X_BIPOLAR = 1,
  RESONATOR_WEB_SOURCE_Y_BIPOLAR = 2,
  RESONATOR_WEB_SOURCE_Z_BIPOLAR = 3,
  RESONATOR_WEB_SOURCE_X_UNIPOLAR = 4,
  RESONATOR_WEB_SOURCE_Y_UNIPOLAR = 5,
  RESONATOR_WEB_SOURCE_Z_UNIPOLAR = 6,
  RESONATOR_WEB_SOURCE_CV5_WET = 7,
  RESONATOR_WEB_SOURCE_PITCH = 8,
  RESONATOR_WEB_SOURCE_ROLL = 9,
  RESONATOR_WEB_SOURCE_SPANK = 10,
};

enum ResonatorWebFlags : uint8_t {
  RESONATOR_WEB_DRONE_ENABLED = 1u << 0,
  RESONATOR_WEB_DRONE_WAVEFORM_SHIFT = 1,
  RESONATOR_WEB_DRONE_WAVEFORM_MASK = 0x02,
  RESONATOR_WEB_FREQUENCY_QUANTIZATION_SHIFT = 5,
  RESONATOR_WEB_FREQUENCY_QUANTIZATION_MASK = 0x60,
};

enum ResonatorFrequencyQuantization : uint8_t {
  RESONATOR_FREQUENCY_QUANTIZATION_OFF = 0,
  RESONATOR_FREQUENCY_QUANTIZATION_CHROMATIC = 1,
  RESONATOR_FREQUENCY_QUANTIZATION_MAJOR = 2,
  RESONATOR_FREQUENCY_QUANTIZATION_MAJOR_PENTATONIC = 3,
};

enum ResonatorWebOffset : uint8_t {
  RESONATOR_WEB_FLAGS = 0,
  RESONATOR_WEB_FEEDBACK_BASE = 1,
  RESONATOR_WEB_FEEDBACK_SOURCE = 2,
  RESONATOR_WEB_FEEDBACK_DEPTH = 3,
  RESONATOR_WEB_DRONE_LEVEL = 4,
  RESONATOR_WEB_DRONE_SOURCE = 5,
  RESONATOR_WEB_DRONE_DEPTH = 6,
  RESONATOR_WEB_REVERB_TIME = 7,
  RESONATOR_WEB_REVERB_TIME_SOURCE = 8,
  RESONATOR_WEB_REVERB_TIME_DEPTH = 9,
  RESONATOR_WEB_REVERB_DIFFUSION = 10,
  RESONATOR_WEB_REVERB_DIFFUSION_SOURCE = 11,
  RESONATOR_WEB_REVERB_DIFFUSION_DEPTH = 12,
  RESONATOR_WEB_REVERB_LP = 13,
  RESONATOR_WEB_REVERB_LP_SOURCE = 14,
  RESONATOR_WEB_REVERB_LP_DEPTH = 15,
};

inline void setDefaultResonatorWebControls(uint8_t controls[kResonatorWebControlSize]) {
  for (uint8_t i = 0; i < kResonatorWebControlSize; ++i) {
    controls[i] = 0;
  }
  controls[RESONATOR_WEB_DRONE_LEVEL] = 20;
  controls[RESONATOR_WEB_REVERB_TIME] = 67;
  controls[RESONATOR_WEB_REVERB_DIFFUSION] = 63;
  controls[RESONATOR_WEB_REVERB_LP] = 60;
}

enum Flags : uint16_t {
  // A hidden Lorenz run owns XYZ modulation but deliberately leaves the
  // physical presence and impact gates under their sensor owners.
  FLAG_CHAOS_ACTIVE = 1u << 0,
  FLAG_MATRIX_VALID = 1u << 1,
  FLAG_IMPACT_GATE = 1u << 2,
  FLAG_RESERVED_3 = 1u << 3,
  FLAG_GESTURE_GATE = 1u << 4,
  FLAG_STALE = 1u << 5,
  FLAG_MATRIX_DETECTED = 1u << 6,
  FLAG_MATRIX_WARMING = 1u << 7,
  FLAG_MODE_ANNOUNCEMENTS_ENABLED = 1u << 8,
  FLAG_SPANK_GATE = 1u << 9,
};

enum ControlFlags : uint16_t {
  CONTROL_FM_DRONE_RANGER = 1u << 0,
  CONTROL_FM_MAPPING_MASK = 0x000E | 0x0070 | 0x0380,
  CONTROL_FM_X_SHIFT = 1,
  CONTROL_FM_Y_SHIFT = 4,
  CONTROL_FM_Z_SHIFT = 7,
  CONTROL_CV_X_UNIPOLAR = 1u << 0,
  CONTROL_CV_Y_UNIPOLAR = 1u << 1,
  CONTROL_CV_Z_UNIPOLAR = 1u << 2,
  CONTROL_RESONATOR_BRIGHTNESS_SHIFT = 0,
  CONTROL_RESONATOR_DAMPING_SHIFT = 2,
  CONTROL_RESONATOR_STRUCTURE_SHIFT = 4,
  CONTROL_RESONATOR_POSITION_SHIFT = 6,
  CONTROL_RESONATOR_REVERB_SHIFT = 8,
  CONTROL_RESONATOR_FREQUENCY_SHIFT = 10,
  CONTROL_RESONATOR_MAPPING_MAGIC = 0x53,
};

enum FmParam : uint8_t {
  FM_PARAM_PITCH = 0,
  FM_PARAM_FM1_AMOUNT = 1,
  FM_PARAM_FM1_FREQUENCY = 2,
  FM_PARAM_FM2_AMOUNT = 3,
  FM_PARAM_FM2_FREQUENCY = 4,
  FM_PARAM_STEREO = 5,
  FM_PARAM_CV_SLEW = 6,
};

enum ResonatorAxis : uint8_t {
  RESONATOR_AXIS_NONE = 0,
  RESONATOR_AXIS_X = 1,
  RESONATOR_AXIS_Y = 2,
  RESONATOR_AXIS_Z = 3,
  RESONATOR_AXIS_PITCH = 4,
  RESONATOR_AXIS_ROLL = 5,
  RESONATOR_AXIS_SPANK = 6,
};

enum ResonatorStrength : uint8_t {
  RESONATOR_STRENGTH_NEG_FULL = 0,
  RESONATOR_STRENGTH_NEG_HALF = 1,
  RESONATOR_STRENGTH_POS_HALF = 2,
  RESONATOR_STRENGTH_POS_FULL = 3,
};

inline uint16_t encodeFmDroneControls(bool enabled, FmParam x, FmParam y, FmParam z) {
  uint16_t flags = enabled ? CONTROL_FM_DRONE_RANGER : 0;
  flags |= (static_cast<uint16_t>(x) & 0x7u) << CONTROL_FM_X_SHIFT;
  flags |= (static_cast<uint16_t>(y) & 0x7u) << CONTROL_FM_Y_SHIFT;
  flags |= (static_cast<uint16_t>(z) & 0x7u) << CONTROL_FM_Z_SHIFT;
  return flags;
}

inline uint16_t encodeResonatorControls(ResonatorAxis brightness,
                                        ResonatorAxis damping,
                                        ResonatorAxis structure,
                                        ResonatorAxis position,
                                        ResonatorAxis reverb = RESONATOR_AXIS_NONE,
                                        ResonatorAxis frequency = RESONATOR_AXIS_NONE) {
  uint16_t flags = 0;
  flags |= (static_cast<uint16_t>(brightness) & 0x3u) << CONTROL_RESONATOR_BRIGHTNESS_SHIFT;
  flags |= (static_cast<uint16_t>(damping) & 0x3u) << CONTROL_RESONATOR_DAMPING_SHIFT;
  flags |= (static_cast<uint16_t>(structure) & 0x3u) << CONTROL_RESONATOR_STRUCTURE_SHIFT;
  flags |= (static_cast<uint16_t>(position) & 0x3u) << CONTROL_RESONATOR_POSITION_SHIFT;
  flags |= (static_cast<uint16_t>(reverb) & 0x3u) << CONTROL_RESONATOR_REVERB_SHIFT;
  flags |= (static_cast<uint16_t>(frequency) & 0x3u) << CONTROL_RESONATOR_FREQUENCY_SHIFT;
  return flags;
}

inline uint8_t encodeResonatorMapping(ResonatorAxis axis, bool unipolar, uint8_t strength) {
  if (axis == RESONATOR_AXIS_NONE) {
    return 0;
  }
  if (axis == RESONATOR_AXIS_PITCH) return 25;
  if (axis == RESONATOR_AXIS_ROLL) return 26;
  if (axis == RESONATOR_AXIS_SPANK) return 27;
  if (axis > RESONATOR_AXIS_Z) return 0;
  if (strength > RESONATOR_STRENGTH_POS_FULL) {
    strength = RESONATOR_STRENGTH_POS_HALF;
  }
  const uint8_t rangeOffset = unipolar ? 3 : 0;
  return static_cast<uint8_t>(
      1 + ((static_cast<uint8_t>(axis) - 1 + rangeOffset) * 4) + strength);
}

inline uint8_t encodeResonatorMapping(ResonatorAxis axis, uint8_t strength) {
  return encodeResonatorMapping(axis, false, strength);
}

inline ResonatorAxis resonatorMappingAxis(uint8_t code) {
  if (code == 25) return RESONATOR_AXIS_PITCH;
  if (code == 26) return RESONATOR_AXIS_ROLL;
  if (code == 27) return RESONATOR_AXIS_SPANK;
  if (code == 0 || code > 24) {
    return RESONATOR_AXIS_NONE;
  }
  return static_cast<ResonatorAxis>((((code - 1) / 4) % 3) + 1);
}

inline bool resonatorMappingUnipolar(uint8_t code) {
  if (code == 0 || code > 24) {
    return false;
  }
  return ((code - 1) / 4) >= 3;
}

inline uint8_t resonatorMappingStrength(uint8_t code) {
  if (code == 0 || code > 24) {
    return RESONATOR_STRENGTH_POS_HALF;
  }
  return static_cast<uint8_t>((code - 1) % 4);
}

inline int8_t resonatorMappingDepthPercent(uint8_t code) {
  if (code == 0 || code > 24) return 0;
  switch (resonatorMappingStrength(code)) {
    case RESONATOR_STRENGTH_NEG_FULL: return -100;
    case RESONATOR_STRENGTH_NEG_HALF: return -50;
    case RESONATOR_STRENGTH_POS_FULL: return 100;
    case RESONATOR_STRENGTH_POS_HALF:
    default: return 50;
  }
}

inline uint32_t packResonatorMappings(uint8_t brightness,
                                      uint8_t damping,
                                      uint8_t structure,
                                      uint8_t position,
                                      uint8_t reverb,
                                      uint8_t frequency) {
  uint32_t packed = 0;
  const uint8_t codes[6] = {brightness, damping, structure, position, reverb, frequency};
  for (int i = 0; i < 6; ++i) {
    packed |= static_cast<uint32_t>(codes[i] & 0x1Fu) << (i * 5);
  }
  return packed;
}

struct Packet {
  uint8_t protocolVersion = kVersionV10;
  uint16_t sequence;
  uint16_t flags;
  uint16_t reservedZMm;
  uint16_t reservedZNorm;
  uint16_t matrixZMm;
  uint16_t matrixXMilli;
  uint16_t matrixYMilli;
  uint16_t matrixZNorm;
  uint16_t matrixXNorm;
  uint16_t matrixYNorm;
  uint16_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t matrixValidCount;
  uint8_t matrixBlobCount;
  uint16_t ageMs;
  uint16_t controlFlags;
  uint16_t impactVelocityNorm;
  uint16_t impactAgeMs;
  uint8_t impactFlags;
  uint8_t sourceFlags2;
  uint8_t resonatorControlsRevision;
  uint8_t resonatorWebControls[kResonatorWebControlSize];
  int8_t laneDepths[kLaneDepthCount];
  uint16_t pitchNorm;
  uint16_t rollNorm;
  uint8_t orientationConfidence;
  // v11 reuses v10's retired old-Z bytes as one source epoch and appends these
  // trajectory fields after the unchanged 64-byte v10 payload.
  uint32_t sourceEpochMs = 0;
  uint16_t trajectoryStartNorm[5] = {};
  uint16_t slewTauMs = 43;
};

struct DptStatus {
  uint8_t protocolVersion;
  uint8_t modeIndex;
  uint8_t controls[kDptControlCount];
  uint16_t flags;
  bool hasControlSnapshot;
  uint8_t resonatorControlsRevision;
};

struct DptPerformance {
  uint8_t protocolVersion;
  uint32_t callbackCount;
  uint32_t maxCallbackUs;
  uint32_t deadlineMisses;
  uint32_t lateStarts;
  uint32_t missedPeriods;
  uint32_t maxStartGapUs;
  uint32_t txCallCount;
  uint32_t maxIrqOffUs;
  uint32_t maxTxCallUs;
};

struct DptTrajectoryDiagnostics {
  uint8_t bridgeProtocolVersion = 0;
  uint16_t slewTauMs = 0;
  uint32_t sourceEpochMs = 0;
  uint16_t sourceAgeMs = 0;
  uint32_t v10PacketCount = 0;
  uint32_t v11PacketCount = 0;
  uint32_t correctionCount = 0;
  uint32_t maxCorrectionNorm = 0;
  uint32_t invalidKeyframeCount = 0;
  uint32_t staleTransitions = 0;
  uint32_t deadlineMisses = 0;
};

inline uint16_t clampU16(uint32_t value, uint16_t maxValue) {
  return value > maxValue ? maxValue : static_cast<uint16_t>(value);
}

inline void writeLe16(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void writeLe32(uint8_t *buffer, size_t offset, uint32_t value) {
  writeLe16(buffer, offset, static_cast<uint16_t>(value & 0xFFFFu));
  writeLe16(buffer, offset + 2, static_cast<uint16_t>(value >> 16));
}

inline uint16_t readLe16(const uint8_t *buffer, size_t offset) {
  return static_cast<uint16_t>(buffer[offset]) |
         (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

inline uint32_t readLe32(const uint8_t *buffer, size_t offset) {
  return static_cast<uint32_t>(readLe16(buffer, offset)) |
         (static_cast<uint32_t>(readLe16(buffer, offset + 2)) << 16);
}

inline int hexValue(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return 10 + c - 'a';
  }
  if (c >= 'A' && c <= 'F') {
    return 10 + c - 'A';
  }
  return -1;
}

inline bool parseHexByte(const char *text, uint8_t &value) {
  const int hi = hexValue(text[0]);
  const int lo = hexValue(text[1]);
  if (hi < 0 || lo < 0) {
    return false;
  }
  value = static_cast<uint8_t>((hi << 4) | lo);
  return true;
}

inline bool parseHexU16(const char *text, uint16_t &value) {
  uint16_t parsed = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    const int nibble = hexValue(text[i]);
    if (nibble < 0) {
      return false;
    }
    parsed = static_cast<uint16_t>((parsed << 4) | static_cast<uint16_t>(nibble));
  }
  value = parsed;
  return true;
}

inline bool parseHexU32(const char *text, uint32_t &value) {
  uint32_t parsed = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    const int nibble = hexValue(text[i]);
    if (nibble < 0) {
      return false;
    }
    parsed = (parsed << 4) | static_cast<uint32_t>(nibble);
  }
  value = parsed;
  return true;
}

inline bool decodeDptStatusLine(const char *line, DptStatus &status) {
  if (line == nullptr) {
    return false;
  }

  if (!(line[0] == 'D' && line[1] == 'P' && line[2] == 'T' && line[3] == 'S' && line[4] == ',')) {
    return false;
  }
  if (line[5] != '3' || line[6] != ',') {
    return false;
  }

  uint8_t mode = 255;
  if (!parseHexByte(line + 7, mode) || line[9] != ',') {
    return false;
  }

  uint8_t controls[kDptControlCount] = {};
  size_t offset = 10;
  for (uint8_t i = 0; i < kDptControlCount; ++i) {
    if (!parseHexByte(line + offset, controls[i])) {
      return false;
    }
    offset += 2;
  }
  if (line[offset] != ',') {
    return false;
  }
  ++offset;

  uint16_t flags = 0;
  if (!parseHexU16(line + offset, flags)) {
    return false;
  }
  offset += 4;
  uint8_t resonatorRevision = 0;
  if (line[offset] != ',' || !parseHexByte(line + offset + 1, resonatorRevision)) {
    return false;
  }
  offset += 3;
  if (line[offset] != '\0') {
    return false;
  }

  status.protocolVersion = kDptStatusProtocolVersion;
  status.modeIndex = mode;
  status.flags = flags;
  status.hasControlSnapshot = true;
  status.resonatorControlsRevision = resonatorRevision;
  for (uint8_t i = 0; i < kDptControlCount; ++i) {
    status.controls[i] = controls[i];
  }
  return true;
}

inline bool decodeDptPerformanceLine(const char *line, DptPerformance &performance) {
  if (line == nullptr ||
      line[0] != 'D' || line[1] != 'P' || line[2] != 'T' || line[3] != 'P' ||
      line[4] != ',' || line[5] != '1' || line[6] != ',') {
    return false;
  }

  uint32_t fields[9] = {};
  size_t offset = 7;
  for (uint8_t field = 0; field < 9; ++field) {
    if (!parseHexU32(line + offset, fields[field])) {
      return false;
    }
    offset += 8;
    if (field + 1 < 9) {
      if (line[offset] != ',') {
        return false;
      }
      ++offset;
    }
  }
  if (line[offset] != '\0') {
    return false;
  }

  performance.protocolVersion = 1;
  performance.callbackCount = fields[0];
  performance.maxCallbackUs = fields[1];
  performance.deadlineMisses = fields[2];
  performance.lateStarts = fields[3];
  performance.missedPeriods = fields[4];
  performance.maxStartGapUs = fields[5];
  performance.txCallCount = fields[6];
  performance.maxIrqOffUs = fields[7];
  performance.maxTxCallUs = fields[8];
  return true;
}

inline bool decodeDptTrajectoryDiagnosticsLine(
    const char *line, DptTrajectoryDiagnostics &diagnostics) {
  if (line == nullptr || line[0] != 'D' || line[1] != 'P' ||
      line[2] != 'T' || line[3] != 'D' || line[4] != ',' ||
      line[5] != '1' || line[6] != ',') {
    return false;
  }
  uint8_t protocol = 0;
  uint16_t tau = 0;
  uint32_t epoch = 0;
  uint16_t age = 0;
  uint32_t fields[7] = {};
  size_t offset = 7;
  if (!parseHexByte(line + offset, protocol)) return false;
  offset += 2;
  if (line[offset++] != ',' || !parseHexU16(line + offset, tau)) return false;
  offset += 4;
  if (line[offset++] != ',' || !parseHexU32(line + offset, epoch)) return false;
  offset += 8;
  if (line[offset++] != ',' || !parseHexU16(line + offset, age)) return false;
  offset += 4;
  for (uint8_t field = 0; field < 7; ++field) {
    if (line[offset++] != ',' || !parseHexU32(line + offset, fields[field])) {
      return false;
    }
    offset += 8;
  }
  if (line[offset] != '\0') return false;
  diagnostics.bridgeProtocolVersion = protocol;
  diagnostics.slewTauMs = tau;
  diagnostics.sourceEpochMs = epoch;
  diagnostics.sourceAgeMs = age;
  diagnostics.v10PacketCount = fields[0];
  diagnostics.v11PacketCount = fields[1];
  diagnostics.correctionCount = fields[2];
  diagnostics.maxCorrectionNorm = fields[3];
  diagnostics.invalidKeyframeCount = fields[4];
  diagnostics.staleTransitions = fields[5];
  diagnostics.deadlineMisses = fields[6];
  return true;
}

inline uint16_t crc16Ccitt(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline void encodePayload(const Packet &packet, uint8_t *payload) {
  writeLe16(payload, 0, packet.sequence);
  writeLe16(payload, 2, packet.flags);
  writeLe16(payload, 4, packet.reservedZMm);
  writeLe16(payload, 6, packet.reservedZNorm);
  writeLe16(payload, 8, packet.matrixZMm);
  writeLe16(payload, 10, packet.matrixXMilli);
  writeLe16(payload, 12, packet.matrixYMilli);
  writeLe16(payload, 14, packet.matrixZNorm);
  writeLe16(payload, 16, packet.matrixXNorm);
  writeLe16(payload, 18, packet.matrixYNorm);
  writeLe16(payload, 20, packet.reserved0);
  payload[22] = packet.reserved1;
  payload[23] = packet.reserved2;
  payload[24] = packet.matrixValidCount;
  payload[25] = packet.matrixBlobCount;
  writeLe16(payload, 26, packet.ageMs);
  writeLe16(payload, 28, packet.controlFlags);
  writeLe16(payload, 30, packet.impactVelocityNorm);
  writeLe16(payload, 32, packet.impactAgeMs);
  payload[34] = packet.impactFlags;
  payload[35] = packet.sourceFlags2;
  payload[36] = packet.resonatorControlsRevision;
  for (uint8_t i = 0; i < kResonatorWebControlSize; ++i) {
    payload[37 + i] = packet.resonatorWebControls[i];
  }
  for (uint8_t i = 0; i < kLaneDepthCount; ++i) {
    payload[53 + i] = static_cast<uint8_t>(packet.laneDepths[i]);
  }
  writeLe16(payload, 59, packet.pitchNorm);
  writeLe16(payload, 61, packet.rollNorm);
  payload[63] = packet.orientationConfidence;
}

inline Packet decodePayload(const uint8_t *payload) {
  Packet packet = {};
  packet.pitchNorm = 2048;
  packet.rollNorm = 2048;
  packet.sequence = readLe16(payload, 0);
  packet.flags = readLe16(payload, 2);
  packet.reservedZMm = readLe16(payload, 4);
  packet.reservedZNorm = readLe16(payload, 6);
  packet.matrixZMm = readLe16(payload, 8);
  packet.matrixXMilli = readLe16(payload, 10);
  packet.matrixYMilli = readLe16(payload, 12);
  packet.matrixZNorm = readLe16(payload, 14);
  packet.matrixXNorm = readLe16(payload, 16);
  packet.matrixYNorm = readLe16(payload, 18);
  packet.reserved0 = readLe16(payload, 20);
  packet.reserved1 = payload[22];
  packet.reserved2 = payload[23];
  packet.matrixValidCount = payload[24];
  packet.matrixBlobCount = payload[25];
  packet.ageMs = readLe16(payload, 26);
  packet.controlFlags = readLe16(payload, 28);
  packet.impactVelocityNorm = readLe16(payload, 30);
  packet.impactAgeMs = readLe16(payload, 32);
  packet.impactFlags = payload[34];
  packet.sourceFlags2 = payload[35];
  packet.resonatorControlsRevision = payload[36];
  for (uint8_t i = 0; i < kResonatorWebControlSize; ++i) {
    packet.resonatorWebControls[i] = payload[37 + i];
  }
  for (uint8_t i = 0; i < kLaneDepthCount; ++i) {
    packet.laneDepths[i] = static_cast<int8_t>(payload[53 + i]);
  }
  packet.pitchNorm = readLe16(payload, 59);
  packet.rollNorm = readLe16(payload, 61);
  packet.orientationConfidence = payload[63];
  packet.protocolVersion = kVersionV10;
  return packet;
}

inline bool validTrajectoryPacket(const Packet &packet) {
  if (packet.slewTauMs < 43 || packet.slewTauMs > 420) return false;
  const uint16_t targetNorm[5] = {
      packet.matrixXNorm, packet.matrixYNorm, packet.matrixZNorm,
      packet.pitchNorm, packet.rollNorm};
  for (uint8_t axis = 0; axis < 5; ++axis) {
    if (packet.trajectoryStartNorm[axis] > 4095 ||
        targetNorm[axis] > 4095) {
      return false;
    }
  }
  return true;
}

inline size_t encodeFrame(const Packet &packet, uint8_t *frame, size_t capacity) {
  if (capacity < kFrameSize) {
    return 0;
  }

  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = kPayloadSize;
  encodePayload(packet, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kPayloadSize);
  writeLe16(frame, 4 + kPayloadSize, crc);
  return kFrameSize;
}

inline size_t encodeFrameV11(const Packet &packet,
                             uint8_t *frame,
                             size_t capacity) {
  if (capacity < kFrameSizeV11 || !validTrajectoryPacket(packet)) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersionV11;
  frame[3] = kPayloadSizeV11;
  encodePayload(packet, frame + 4);
  // These offsets were reserved/retired old-Z fields in v10.  Reusing them
  // keeps the v11 growth to exactly five starts plus tau (12 bytes).
  writeLe32(frame + 4, 4, packet.sourceEpochMs);
  for (uint8_t axis = 0; axis < 5; ++axis) {
    writeLe16(frame + 4, 64 + axis * 2u, packet.trajectoryStartNorm[axis]);
  }
  writeLe16(frame + 4, 74, packet.slewTauMs);
  const uint16_t crc = crc16Ccitt(frame, 4 + kPayloadSizeV11);
  writeLe16(frame, 4 + kPayloadSizeV11, crc);
  return kFrameSizeV11;
}

inline bool decodeFrame(const uint8_t *frame, size_t size, Packet &packet) {
  if (frame == nullptr || size < 6 ||
      frame[0] != kStart0 || frame[1] != kStart1) {
    return false;
  }
  const uint8_t version = frame[2];
  const uint8_t payloadSize = frame[3];
  const size_t expectedSize = version == kVersionV10
      ? kFrameSizeV10 : version == kVersionV11 ? kFrameSizeV11 : 0;
  const uint8_t expectedPayloadSize = version == kVersionV10
      ? kPayloadSizeV10 : version == kVersionV11 ? kPayloadSizeV11 : 0;
  if (expectedSize == 0 || size != expectedSize ||
      payloadSize != expectedPayloadSize) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + payloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + payloadSize);
  if (expected != actual) {
    return false;
  }
  packet = decodePayload(frame + 4);
  if (version == kVersionV11) {
    packet.protocolVersion = kVersionV11;
    packet.sourceEpochMs = readLe32(frame + 4, 4);
    for (uint8_t axis = 0; axis < 5; ++axis) {
      packet.trajectoryStartNorm[axis] = readLe16(frame + 4, 64 + axis * 2u);
    }
    packet.slewTauMs = readLe16(frame + 4, 74);
    if (!validTrajectoryPacket(packet)) return false;
  }
  return true;
}

class Parser {
 public:
  Parser() : index_(0), expectedSize_(0) {}

  bool Push(uint8_t byte, Packet &packet) {
    if (index_ == 0 && byte != kStart0) {
      return false;
    }
    if (index_ == 1 && byte != kStart1) {
      index_ = byte == kStart0 ? 1 : 0;
      return false;
    }

    buffer_[index_++] = byte;
    if (index_ == 4) {
      const uint8_t version = buffer_[2];
      const uint8_t payloadSize = buffer_[3];
      if (version == kVersionV10 && payloadSize == kPayloadSizeV10) {
        expectedSize_ = kFrameSizeV10;
      } else if (version == kVersionV11 && payloadSize == kPayloadSizeV11) {
        expectedSize_ = kFrameSizeV11;
      } else {
        Reset();
        return false;
      }
    }

    if (expectedSize_ == 0 || index_ < expectedSize_) {
      return false;
    }

    const bool ok = decodeFrame(buffer_, expectedSize_, packet);
    Reset();
    return ok;
  }

  void Reset() {
    index_ = 0;
    expectedSize_ = 0;
  }

 private:
  uint8_t buffer_[kMaxFrameSize];
  size_t index_;
  size_t expectedSize_;
};

class DptProbeParser {
 public:
  DptProbeParser() : index_(0) {}

  bool Push(uint8_t byte) {
    if (byte == kDptProbe[index_]) {
      ++index_;
      if (index_ == kDptProbeSize) {
        index_ = 0;
        return true;
      }
      return false;
    }

    index_ = byte == kDptProbe[0] ? 1 : 0;
    return false;
  }

  void Reset() { index_ = 0; }

 private:
  size_t index_;
};

}  // namespace bearamin_bridge
