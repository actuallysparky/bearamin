#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bridge_protocol.h"

namespace bearamin_sidecar {

static constexpr uint8_t kStart0 = 0xBA;
static constexpr uint8_t kStart1 = 0x4D;
static constexpr uint8_t kVersion = 5;
static constexpr uint8_t kStatePayloadSize = 90;
static constexpr uint8_t kCommandPayloadSize = 12;
static constexpr uint8_t kSensorDiagnosticsPayloadSize = 16;
static constexpr uint8_t kRuntimeSettingsPayloadSize = 36;
static constexpr uint8_t kMatrixPerformancePayloadSize = 40;
static constexpr uint8_t kPresetLinePayloadSize = 96;
static constexpr uint8_t kPresetPayloadSize = 4 + kPresetLinePayloadSize;
static constexpr size_t kMatrixCellsPayloadSize = 134;
static constexpr size_t kStateFrameSize = 2 + 1 + 1 + kStatePayloadSize + 2;
static constexpr size_t kCommandFrameSize = 2 + 1 + 1 + kCommandPayloadSize + 2;
static constexpr size_t kSensorDiagnosticsFrameSize =
    2 + 1 + 1 + kSensorDiagnosticsPayloadSize + 2;
static constexpr size_t kRuntimeSettingsFrameSize =
    2 + 1 + 1 + kRuntimeSettingsPayloadSize + 2;
static constexpr size_t kMatrixPerformanceFrameSize =
    2 + 1 + 1 + kMatrixPerformancePayloadSize + 2;
static constexpr size_t kPresetFrameSize = 2 + 1 + 1 + kPresetPayloadSize + 2;
static constexpr size_t kMatrixCellsFrameSize = 2 + 1 + 1 + kMatrixCellsPayloadSize + 2;
static constexpr size_t kMaxFrameSize = kMatrixCellsFrameSize > kPresetFrameSize
                                            ? kMatrixCellsFrameSize
                                            : kPresetFrameSize;

enum FrameType : uint8_t {
  FRAME_STATE = 1,
  FRAME_COMMAND = 2,
  FRAME_ACK = 3,
  FRAME_MATRIX_CELLS = 4,
  FRAME_PRESET = 5,
  FRAME_SENSOR_DIAGNOSTICS = 6,
  FRAME_RUNTIME_SETTINGS = 7,
  FRAME_MATRIX_PERFORMANCE = 8,
};

enum SensorRecoveryReason : uint8_t {
  SENSOR_RECOVERY_BOOT = 0,
  SENSOR_RECOVERY_RESERVED_1 = 1,
  SENSOR_RECOVERY_RESERVED_2 = 2,
  SENSOR_RECOVERY_MATRIX_NO_FIRST_SAMPLE = 3,
  SENSOR_RECOVERY_MATRIX_STALE = 4,
  SENSOR_RECOVERY_RESERVED_5 = 5,
  SENSOR_RECOVERY_RESERVED_6 = 6,
  SENSOR_RECOVERY_MATRIX_NOT_READY = 7,
  SENSOR_RECOVERY_RESERVED_8 = 8,
};

enum SensorBusFlags : uint8_t {
  SENSOR_BUS_SDA_HIGH = 1u << 0,
  SENSOR_BUS_SCL_HIGH = 1u << 1,
  SENSOR_BUS_DEFAULT_ADDRESS_ACK = 1u << 2,
  SENSOR_BUS_RESERVED_3 = 1u << 3,
  SENSOR_BUS_RESERVED_4 = 1u << 4,
};

enum StateFlags : uint16_t {
  STATE_DPT_READY = 1u << 0,
  STATE_RESERVED_1 = 1u << 1,
  STATE_RESERVED_2 = 1u << 2,
  STATE_MATRIX_READY = 1u << 3,
  STATE_MATRIX_PRESENT = 1u << 4,
  STATE_IMU_READY = 1u << 5,
  STATE_IMPACT_GATE = 1u << 6,
  STATE_MATRIX_OTA_MODE = 1u << 7,
  STATE_MATRIX_OTA_ACTIVE = 1u << 8,
  STATE_DPT_CONTROL_TX_QUIET = 1u << 9,
  STATE_MODE_ANNOUNCEMENTS_ENABLED = 1u << 10,
  STATE_OPTICAL_RECOVERY_ACTIVE = 1u << 11,
  STATE_SPANK_GATE = 1u << 12,
  STATE_SYNTHETIC_GATE = 1u << 13,
};

enum CommandId : uint8_t {
  COMMAND_SET_DPT_MODE = 1,
  COMMAND_SET_CONTROL_FLAGS = 2,
  COMMAND_SET_CV_SLEW = 3,
  COMMAND_SET_SIDECAR_STATUS = 4,
  COMMAND_ENTER_MATRIX_OTA = 5,
  COMMAND_SET_ENGINEERING_STREAM = 6,
  COMMAND_SET_RESONATOR_WEB_PAGE = 7,
  COMMAND_SET_LANE_DEPTHS = 8,
  COMMAND_SET_MODE_ANNOUNCEMENTS = 9,
  COMMAND_REQUEST_RUNTIME_SETTINGS = 10,
  COMMAND_CAPTURE_START = 11,
  COMMAND_CAPTURE_STOP = 12,
};

enum PresetOp : uint8_t {
  PRESET_OP_LINE = 1,
};

enum SidecarStatusFlags : uint16_t {
  SIDECAR_STATUS_WIFI_READY = 1u << 0,
  SIDECAR_STATUS_WEB_READY = 1u << 1,
  SIDECAR_STATUS_OTA_READY = 1u << 2,
  SIDECAR_STATUS_SOFTAP_READY = 1u << 3,
  SIDECAR_STATUS_WIFI_5GHZ = 1u << 4,
};

struct StateFrame {
  uint16_t sequence;
  uint16_t stateFlags;
  uint32_t uptimeMs;
  uint16_t reserved0;
  uint16_t reserved1;
  uint16_t matrixZMm;
  uint16_t matrixXMilli;
  uint16_t matrixYMilli;
  uint16_t matrixZNorm;
  uint16_t matrixXNorm;
  uint16_t matrixYNorm;
  uint8_t matrixValidCount;
  uint8_t matrixBlobCount;
  uint16_t impactVelocityNorm;
  uint16_t impactAgeMs;
  int16_t accelXMilliG;
  int16_t accelYMilliG;
  int16_t accelZMilliG;
  uint8_t dptModeIndex;
  uint8_t dptProtocolVersion;
  uint16_t dptFlags;
  uint16_t dptControlFlags;
  uint16_t sensorAgeMs;
  uint16_t frameErrors;
  uint16_t i2cRecoveries;
  uint16_t dptProbeTxCount;
  uint16_t cellsMm[8];
  uint16_t dptFrameTxCount;
  uint16_t dptPresetTxCount;
  uint16_t dptRxByteCount;
  uint16_t dptStatusLineCount;
  uint16_t dptPresetLineCount;
  uint8_t dptControls[bearamin_bridge::kDptControlCount];
  uint8_t dptResonatorControlsRevision;
  uint16_t pitchNorm;
  uint16_t rollNorm;
  uint8_t orientationConfidence;
};

struct CommandFrame {
  uint16_t sequence;
  uint8_t command;
  uint8_t reserved0;
  uint16_t arg0;
  uint16_t arg1;
  uint16_t arg2;
  uint16_t arg3;
};

class ResonatorWebPageAssembler {
 public:
  ResonatorWebPageAssembler() : revision_(0), pageMask_(0) {
    bearamin_bridge::setDefaultResonatorWebControls(staged_);
  }

  bool Apply(const CommandFrame &command,
             uint8_t &revision,
             uint8_t controls[bearamin_bridge::kResonatorWebControlSize]) {
    if (command.command != COMMAND_SET_RESONATOR_WEB_PAGE) {
      return false;
    }
    const uint8_t nextRevision = static_cast<uint8_t>(command.arg0 & 0xFFu);
    const uint8_t page = static_cast<uint8_t>((command.arg0 >> 8) & 0xFFu);
    if (page >= 3) {
      return false;
    }
    if (nextRevision != revision_) {
      revision_ = nextRevision;
      pageMask_ = 0;
    }
    const uint16_t words[3] = {command.arg1, command.arg2, command.arg3};
    const uint8_t start = static_cast<uint8_t>(page * 6);
    for (uint8_t i = 0; i < 6; ++i) {
      const uint8_t index = static_cast<uint8_t>(start + i);
      if (index < bearamin_bridge::kResonatorWebControlSize) {
        staged_[index]
            = static_cast<uint8_t>((words[i / 2] >> ((i % 2) * 8)) & 0xFFu);
      }
    }
    pageMask_ |= static_cast<uint8_t>(1u << page);
    if (pageMask_ != 0x07u) {
      return false;
    }
    for (uint8_t i = 0; i < bearamin_bridge::kResonatorWebControlSize; ++i) {
      controls[i] = staged_[i];
    }
    revision = revision_;
    return true;
  }

  uint8_t PageMask() const { return pageMask_; }

 private:
  uint8_t revision_;
  uint8_t pageMask_;
  uint8_t staged_[bearamin_bridge::kResonatorWebControlSize];
};

struct MatrixCellsFrame {
  uint16_t sequence;
  uint8_t validCount;
  uint8_t blobCount;
  uint16_t sensorAgeMs;
  uint16_t cellsMm[64];
};

struct PresetFrame {
  uint16_t sequence;
  uint8_t op;
  uint8_t length;
  uint8_t data[kPresetLinePayloadSize];
};

struct SensorDiagnosticsFrame {
  uint16_t sequence;
  uint16_t reserved0;
  uint16_t reserved1;
  uint16_t recoveryFailures;
  uint16_t reserved2;
  uint16_t matrixAgeMs;
  uint8_t recoveryReason;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t busFlags;
};

struct RuntimeSettingsFrame {
  uint16_t sequence;
  uint8_t dptModeIndex;
  bool modeAnnouncementsEnabled;
  bool controlsInitializedBySidecar;
  uint16_t cvSlewStep;
  uint16_t dptControlFlags;
  uint16_t dptReserved0;
  uint8_t dptReserved1;
  uint8_t dptReserved2;
  uint8_t dptSourceFlags2;
  uint8_t resonatorControlsRevision;
  int8_t laneDepths[bearamin_bridge::kLaneDepthCount];
  uint8_t resonatorWebControls[bearamin_bridge::kResonatorWebControlSize];
};

struct MatrixPerformanceFrame {
  uint16_t sequence;
  uint32_t effectiveRateMilliHz;
  uint32_t intervalMaxUs;
  uint32_t readMaxUs;
  uint32_t processMaxUs;
  uint32_t loopGapMaxUs;
  uint32_t freeHeapBytes;
  uint32_t minimumFreeHeapBytes;
  uint32_t readErrors;
  uint16_t recoveryStackWords;
  uint8_t sourceHz;
  uint8_t outputHz;
  uint8_t readyPollStatus;
  uint8_t readyPollValue;
};

inline void writeLe16(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline uint16_t readLe16(const uint8_t *buffer, size_t offset) {
  return static_cast<uint16_t>(buffer[offset]) |
         (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

inline void writeLe32(uint8_t *buffer, size_t offset, uint32_t value) {
  writeLe16(buffer, offset, static_cast<uint16_t>(value & 0xFFFFu));
  writeLe16(buffer, offset + 2, static_cast<uint16_t>((value >> 16) & 0xFFFFu));
}

inline uint32_t readLe32(const uint8_t *buffer, size_t offset) {
  return static_cast<uint32_t>(readLe16(buffer, offset)) |
         (static_cast<uint32_t>(readLe16(buffer, offset + 2)) << 16);
}

inline uint16_t crc16Ccitt(const uint8_t *data, size_t size) {
  return bearamin_bridge::crc16Ccitt(data, size);
}

inline void encodeStatePayload(const StateFrame &state, uint8_t *payload) {
  writeLe16(payload, 0, state.sequence);
  writeLe16(payload, 2, state.stateFlags);
  writeLe32(payload, 4, state.uptimeMs);
  writeLe16(payload, 8, state.reserved0);
  writeLe16(payload, 10, state.reserved1);
  writeLe16(payload, 12, state.matrixZMm);
  writeLe16(payload, 14, state.matrixXMilli);
  writeLe16(payload, 16, state.matrixYMilli);
  writeLe16(payload, 18, state.matrixZNorm);
  writeLe16(payload, 20, state.matrixXNorm);
  writeLe16(payload, 22, state.matrixYNorm);
  payload[24] = state.matrixValidCount;
  payload[25] = state.matrixBlobCount;
  writeLe16(payload, 26, state.impactVelocityNorm);
  writeLe16(payload, 28, state.impactAgeMs);
  writeLe16(payload, 30, static_cast<uint16_t>(state.accelXMilliG));
  writeLe16(payload, 32, static_cast<uint16_t>(state.accelYMilliG));
  writeLe16(payload, 34, static_cast<uint16_t>(state.accelZMilliG));
  payload[36] = state.dptModeIndex;
  payload[37] = state.dptProtocolVersion;
  writeLe16(payload, 38, state.dptFlags);
  writeLe16(payload, 40, state.dptControlFlags);
  writeLe16(payload, 42, state.sensorAgeMs);
  writeLe16(payload, 44, state.frameErrors);
  writeLe16(payload, 46, state.i2cRecoveries);
  writeLe16(payload, 48, state.dptProbeTxCount);
  for (uint8_t i = 0; i < 8; ++i) {
    writeLe16(payload, 50 + i * 2, state.cellsMm[i]);
  }
  writeLe16(payload, 66, state.dptFrameTxCount);
  writeLe16(payload, 68, state.dptPresetTxCount);
  writeLe16(payload, 70, state.dptRxByteCount);
  writeLe16(payload, 72, state.dptStatusLineCount);
  writeLe16(payload, 74, state.dptPresetLineCount);
  for (uint8_t i = 0; i < bearamin_bridge::kDptControlCount; ++i) {
    payload[76 + i] = state.dptControls[i];
  }
  payload[84] = state.dptResonatorControlsRevision;
  writeLe16(payload, 85, state.pitchNorm);
  writeLe16(payload, 87, state.rollNorm);
  payload[89] = state.orientationConfidence;
}

inline StateFrame decodeStatePayload(const uint8_t *payload) {
  StateFrame state = {};
  state.pitchNorm = 2048;
  state.rollNorm = 2048;
  state.sequence = readLe16(payload, 0);
  state.stateFlags = readLe16(payload, 2);
  state.uptimeMs = readLe32(payload, 4);
  state.reserved0 = readLe16(payload, 8);
  state.reserved1 = readLe16(payload, 10);
  state.matrixZMm = readLe16(payload, 12);
  state.matrixXMilli = readLe16(payload, 14);
  state.matrixYMilli = readLe16(payload, 16);
  state.matrixZNorm = readLe16(payload, 18);
  state.matrixXNorm = readLe16(payload, 20);
  state.matrixYNorm = readLe16(payload, 22);
  state.matrixValidCount = payload[24];
  state.matrixBlobCount = payload[25];
  state.impactVelocityNorm = readLe16(payload, 26);
  state.impactAgeMs = readLe16(payload, 28);
  state.accelXMilliG = static_cast<int16_t>(readLe16(payload, 30));
  state.accelYMilliG = static_cast<int16_t>(readLe16(payload, 32));
  state.accelZMilliG = static_cast<int16_t>(readLe16(payload, 34));
  state.dptModeIndex = payload[36];
  state.dptProtocolVersion = payload[37];
  state.dptFlags = readLe16(payload, 38);
  state.dptControlFlags = readLe16(payload, 40);
  state.sensorAgeMs = readLe16(payload, 42);
  state.frameErrors = readLe16(payload, 44);
  state.i2cRecoveries = readLe16(payload, 46);
  state.dptProbeTxCount = readLe16(payload, 48);
  for (uint8_t i = 0; i < 8; ++i) {
    state.cellsMm[i] = readLe16(payload, 50 + i * 2);
  }
  state.dptFrameTxCount = readLe16(payload, 66);
  state.dptPresetTxCount = readLe16(payload, 68);
  state.dptRxByteCount = readLe16(payload, 70);
  state.dptStatusLineCount = readLe16(payload, 72);
  state.dptPresetLineCount = readLe16(payload, 74);
  for (uint8_t i = 0; i < bearamin_bridge::kDptControlCount; ++i) {
    state.dptControls[i] = payload[76 + i];
  }
  state.dptResonatorControlsRevision = payload[84];
  state.pitchNorm = readLe16(payload, 85);
  state.rollNorm = readLe16(payload, 87);
  state.orientationConfidence = payload[89];
  return state;
}

inline void encodeCommandPayload(const CommandFrame &command, uint8_t *payload) {
  writeLe16(payload, 0, command.sequence);
  payload[2] = command.command;
  payload[3] = command.reserved0;
  writeLe16(payload, 4, command.arg0);
  writeLe16(payload, 6, command.arg1);
  writeLe16(payload, 8, command.arg2);
  writeLe16(payload, 10, command.arg3);
}

inline CommandFrame decodeCommandPayload(const uint8_t *payload) {
  CommandFrame command = {};
  command.sequence = readLe16(payload, 0);
  command.command = payload[2];
  command.reserved0 = payload[3];
  command.arg0 = readLe16(payload, 4);
  command.arg1 = readLe16(payload, 6);
  command.arg2 = readLe16(payload, 8);
  command.arg3 = readLe16(payload, 10);
  return command;
}

inline void encodeMatrixCellsPayload(const MatrixCellsFrame &cells, uint8_t *payload) {
  writeLe16(payload, 0, cells.sequence);
  payload[2] = cells.validCount;
  payload[3] = cells.blobCount;
  writeLe16(payload, 4, cells.sensorAgeMs);
  for (uint8_t i = 0; i < 64; ++i) {
    writeLe16(payload, 6 + i * 2, cells.cellsMm[i]);
  }
}

inline MatrixCellsFrame decodeMatrixCellsPayload(const uint8_t *payload) {
  MatrixCellsFrame cells = {};
  cells.sequence = readLe16(payload, 0);
  cells.validCount = payload[2];
  cells.blobCount = payload[3];
  cells.sensorAgeMs = readLe16(payload, 4);
  for (uint8_t i = 0; i < 64; ++i) {
    cells.cellsMm[i] = readLe16(payload, 6 + i * 2);
  }
  return cells;
}

inline void encodePresetPayload(const PresetFrame &preset, uint8_t *payload) {
  writeLe16(payload, 0, preset.sequence);
  payload[2] = preset.op;
  payload[3] = preset.length <= kPresetLinePayloadSize ? preset.length : kPresetLinePayloadSize;
  for (uint8_t i = 0; i < kPresetLinePayloadSize; ++i) {
    payload[4 + i] = i < payload[3] ? preset.data[i] : 0;
  }
}

inline PresetFrame decodePresetPayload(const uint8_t *payload) {
  PresetFrame preset = {};
  preset.sequence = readLe16(payload, 0);
  preset.op = payload[2];
  preset.length = payload[3] <= kPresetLinePayloadSize ? payload[3] : kPresetLinePayloadSize;
  for (uint8_t i = 0; i < preset.length; ++i) {
    preset.data[i] = payload[4 + i];
  }
  return preset;
}

inline void encodeSensorDiagnosticsPayload(const SensorDiagnosticsFrame &diagnostics,
                                           uint8_t *payload) {
  writeLe16(payload, 0, diagnostics.sequence);
  writeLe16(payload, 2, diagnostics.reserved0);
  writeLe16(payload, 4, diagnostics.reserved1);
  writeLe16(payload, 6, diagnostics.recoveryFailures);
  writeLe16(payload, 8, diagnostics.reserved2);
  writeLe16(payload, 10, diagnostics.matrixAgeMs);
  payload[12] = diagnostics.recoveryReason;
  payload[13] = diagnostics.reserved3;
  payload[14] = diagnostics.reserved4;
  payload[15] = diagnostics.busFlags;
}

inline SensorDiagnosticsFrame decodeSensorDiagnosticsPayload(const uint8_t *payload) {
  SensorDiagnosticsFrame diagnostics = {};
  diagnostics.sequence = readLe16(payload, 0);
  diagnostics.reserved0 = readLe16(payload, 2);
  diagnostics.reserved1 = readLe16(payload, 4);
  diagnostics.recoveryFailures = readLe16(payload, 6);
  diagnostics.reserved2 = readLe16(payload, 8);
  diagnostics.matrixAgeMs = readLe16(payload, 10);
  diagnostics.recoveryReason = payload[12];
  diagnostics.reserved3 = payload[13];
  diagnostics.reserved4 = payload[14];
  diagnostics.busFlags = payload[15];
  return diagnostics;
}

inline void encodeRuntimeSettingsPayload(const RuntimeSettingsFrame &settings,
                                         uint8_t *payload) {
  writeLe16(payload, 0, settings.sequence);
  payload[2] = settings.dptModeIndex;
  payload[3] = (settings.modeAnnouncementsEnabled ? 1u : 0u) |
               (settings.controlsInitializedBySidecar ? 2u : 0u);
  writeLe16(payload, 4, settings.cvSlewStep);
  writeLe16(payload, 6, settings.dptControlFlags);
  writeLe16(payload, 8, settings.dptReserved0);
  payload[10] = settings.dptReserved1;
  payload[11] = settings.dptReserved2;
  payload[12] = settings.dptSourceFlags2;
  payload[13] = settings.resonatorControlsRevision;
  for (uint8_t i = 0; i < bearamin_bridge::kLaneDepthCount; ++i) {
    payload[14 + i] = static_cast<uint8_t>(settings.laneDepths[i]);
  }
  for (uint8_t i = 0; i < bearamin_bridge::kResonatorWebControlSize; ++i) {
    payload[20 + i] = settings.resonatorWebControls[i];
  }
}

inline RuntimeSettingsFrame decodeRuntimeSettingsPayload(const uint8_t *payload) {
  RuntimeSettingsFrame settings = {};
  settings.sequence = readLe16(payload, 0);
  settings.dptModeIndex = payload[2];
  settings.modeAnnouncementsEnabled = (payload[3] & 1u) != 0;
  settings.controlsInitializedBySidecar = (payload[3] & 2u) != 0;
  settings.cvSlewStep = readLe16(payload, 4);
  settings.dptControlFlags = readLe16(payload, 6);
  settings.dptReserved0 = readLe16(payload, 8);
  settings.dptReserved1 = payload[10];
  settings.dptReserved2 = payload[11];
  settings.dptSourceFlags2 = payload[12];
  settings.resonatorControlsRevision = payload[13];
  for (uint8_t i = 0; i < bearamin_bridge::kLaneDepthCount; ++i) {
    settings.laneDepths[i] = static_cast<int8_t>(payload[14 + i]);
  }
  for (uint8_t i = 0; i < bearamin_bridge::kResonatorWebControlSize; ++i) {
    settings.resonatorWebControls[i] = payload[20 + i];
  }
  return settings;
}

inline void encodeMatrixPerformancePayload(const MatrixPerformanceFrame &performance,
                                           uint8_t *payload) {
  writeLe16(payload, 0, performance.sequence);
  writeLe32(payload, 2, performance.effectiveRateMilliHz);
  writeLe32(payload, 6, performance.intervalMaxUs);
  writeLe32(payload, 10, performance.readMaxUs);
  writeLe32(payload, 14, performance.processMaxUs);
  writeLe32(payload, 18, performance.loopGapMaxUs);
  writeLe32(payload, 22, performance.freeHeapBytes);
  writeLe32(payload, 26, performance.minimumFreeHeapBytes);
  writeLe32(payload, 30, performance.readErrors);
  writeLe16(payload, 34, performance.recoveryStackWords);
  payload[36] = performance.sourceHz;
  payload[37] = performance.outputHz;
  payload[38] = performance.readyPollStatus;
  payload[39] = performance.readyPollValue;
}

inline MatrixPerformanceFrame decodeMatrixPerformancePayload(const uint8_t *payload) {
  MatrixPerformanceFrame performance = {};
  performance.sequence = readLe16(payload, 0);
  performance.effectiveRateMilliHz = readLe32(payload, 2);
  performance.intervalMaxUs = readLe32(payload, 6);
  performance.readMaxUs = readLe32(payload, 10);
  performance.processMaxUs = readLe32(payload, 14);
  performance.loopGapMaxUs = readLe32(payload, 18);
  performance.freeHeapBytes = readLe32(payload, 22);
  performance.minimumFreeHeapBytes = readLe32(payload, 26);
  performance.readErrors = readLe32(payload, 30);
  performance.recoveryStackWords = readLe16(payload, 34);
  performance.sourceHz = payload[36];
  performance.outputHz = payload[37];
  performance.readyPollStatus = payload[38];
  performance.readyPollValue = payload[39];
  return performance;
}

inline size_t encodeStateFrame(const StateFrame &state, uint8_t *frame, size_t capacity) {
  if (capacity < kStateFrameSize) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = FRAME_STATE;
  encodeStatePayload(state, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kStatePayloadSize);
  writeLe16(frame, 4 + kStatePayloadSize, crc);
  return kStateFrameSize;
}

inline size_t encodeCommandFrame(const CommandFrame &command, uint8_t type, uint8_t *frame, size_t capacity) {
  if (capacity < kCommandFrameSize || (type != FRAME_COMMAND && type != FRAME_ACK)) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = type;
  encodeCommandPayload(command, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kCommandPayloadSize);
  writeLe16(frame, 4 + kCommandPayloadSize, crc);
  return kCommandFrameSize;
}

inline size_t encodeMatrixCellsFrame(const MatrixCellsFrame &cells, uint8_t *frame, size_t capacity) {
  if (capacity < kMatrixCellsFrameSize) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = FRAME_MATRIX_CELLS;
  encodeMatrixCellsPayload(cells, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kMatrixCellsPayloadSize);
  writeLe16(frame, 4 + kMatrixCellsPayloadSize, crc);
  return kMatrixCellsFrameSize;
}

inline size_t encodePresetFrame(const PresetFrame &preset, uint8_t *frame, size_t capacity) {
  if (capacity < kPresetFrameSize) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = FRAME_PRESET;
  encodePresetPayload(preset, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kPresetPayloadSize);
  writeLe16(frame, 4 + kPresetPayloadSize, crc);
  return kPresetFrameSize;
}

inline size_t encodeSensorDiagnosticsFrame(const SensorDiagnosticsFrame &diagnostics,
                                           uint8_t *frame,
                                           size_t capacity) {
  if (capacity < kSensorDiagnosticsFrameSize) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = FRAME_SENSOR_DIAGNOSTICS;
  encodeSensorDiagnosticsPayload(diagnostics, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kSensorDiagnosticsPayloadSize);
  writeLe16(frame, 4 + kSensorDiagnosticsPayloadSize, crc);
  return kSensorDiagnosticsFrameSize;
}

inline size_t encodeRuntimeSettingsFrame(const RuntimeSettingsFrame &settings,
                                         uint8_t *frame,
                                         size_t capacity) {
  if (capacity < kRuntimeSettingsFrameSize) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = FRAME_RUNTIME_SETTINGS;
  encodeRuntimeSettingsPayload(settings, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kRuntimeSettingsPayloadSize);
  writeLe16(frame, 4 + kRuntimeSettingsPayloadSize, crc);
  return kRuntimeSettingsFrameSize;
}

inline size_t encodeMatrixPerformanceFrame(const MatrixPerformanceFrame &performance,
                                           uint8_t *frame,
                                           size_t capacity) {
  if (capacity < kMatrixPerformanceFrameSize) {
    return 0;
  }
  frame[0] = kStart0;
  frame[1] = kStart1;
  frame[2] = kVersion;
  frame[3] = FRAME_MATRIX_PERFORMANCE;
  encodeMatrixPerformancePayload(performance, frame + 4);
  const uint16_t crc = crc16Ccitt(frame, 4 + kMatrixPerformancePayloadSize);
  writeLe16(frame, 4 + kMatrixPerformancePayloadSize, crc);
  return kMatrixPerformanceFrameSize;
}

inline bool decodeStateFrame(const uint8_t *frame, size_t size, StateFrame &state) {
  if (size != kStateFrameSize || frame[0] != kStart0 ||
      frame[1] != kStart1 || frame[2] != kVersion ||
      frame[3] != FRAME_STATE) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kStatePayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kStatePayloadSize);
  if (expected != actual) {
    return false;
  }
  state = decodeStatePayload(frame + 4);
  return true;
}

inline bool decodeCommandFrame(const uint8_t *frame, size_t size, uint8_t type, CommandFrame &command) {
  if (size != kCommandFrameSize || frame[0] != kStart0 || frame[1] != kStart1 ||
      frame[2] != kVersion ||
      frame[3] != type) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kCommandPayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kCommandPayloadSize);
  if (expected != actual) {
    return false;
  }
  command = decodeCommandPayload(frame + 4);
  return true;
}

inline bool decodeMatrixCellsFrame(const uint8_t *frame, size_t size, MatrixCellsFrame &cells) {
  if (size != kMatrixCellsFrameSize || frame[0] != kStart0 || frame[1] != kStart1 ||
      frame[2] != kVersion ||
      frame[3] != FRAME_MATRIX_CELLS) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kMatrixCellsPayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kMatrixCellsPayloadSize);
  if (expected != actual) {
    return false;
  }
  cells = decodeMatrixCellsPayload(frame + 4);
  return true;
}

inline bool decodePresetFrame(const uint8_t *frame, size_t size, PresetFrame &preset) {
  if (size != kPresetFrameSize || frame[0] != kStart0 || frame[1] != kStart1 ||
      frame[2] != kVersion ||
      frame[3] != FRAME_PRESET) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kPresetPayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kPresetPayloadSize);
  if (expected != actual) {
    return false;
  }
  preset = decodePresetPayload(frame + 4);
  return true;
}

inline bool decodeSensorDiagnosticsFrame(const uint8_t *frame,
                                         size_t size,
                                         SensorDiagnosticsFrame &diagnostics) {
  if (size != kSensorDiagnosticsFrameSize ||
      frame[0] != kStart0 || frame[1] != kStart1 ||
      frame[2] != kVersion ||
      frame[3] != FRAME_SENSOR_DIAGNOSTICS) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kSensorDiagnosticsPayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kSensorDiagnosticsPayloadSize);
  if (expected != actual) {
    return false;
  }
  diagnostics = decodeSensorDiagnosticsPayload(frame + 4);
  return true;
}

inline bool decodeRuntimeSettingsFrame(const uint8_t *frame,
                                       size_t size,
                                       RuntimeSettingsFrame &settings) {
  if (size != kRuntimeSettingsFrameSize ||
      frame[0] != kStart0 || frame[1] != kStart1 ||
      frame[2] != kVersion ||
      frame[3] != FRAME_RUNTIME_SETTINGS) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kRuntimeSettingsPayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kRuntimeSettingsPayloadSize);
  if (expected != actual) {
    return false;
  }
  settings = decodeRuntimeSettingsPayload(frame + 4);
  return true;
}

inline bool decodeMatrixPerformanceFrame(const uint8_t *frame,
                                         size_t size,
                                         MatrixPerformanceFrame &performance) {
  if (size != kMatrixPerformanceFrameSize ||
      frame[0] != kStart0 || frame[1] != kStart1 ||
      frame[2] != kVersion || frame[3] != FRAME_MATRIX_PERFORMANCE) {
    return false;
  }
  const uint16_t expected = readLe16(frame, 4 + kMatrixPerformancePayloadSize);
  const uint16_t actual = crc16Ccitt(frame, 4 + kMatrixPerformancePayloadSize);
  if (expected != actual) {
    return false;
  }
  performance = decodeMatrixPerformancePayload(frame + 4);
  return true;
}

class Parser {
 public:
  Parser() : index_(0), expectedSize_(0) {}

  uint8_t Push(uint8_t byte,
               StateFrame &state,
               CommandFrame &command,
               MatrixCellsFrame &cells,
               PresetFrame &preset,
               SensorDiagnosticsFrame &diagnostics,
               RuntimeSettingsFrame &runtimeSettings,
               MatrixPerformanceFrame &matrixPerformance) {
    if (index_ == 0 && byte != kStart0) {
      return 0;
    }
    if (index_ == 1 && byte != kStart1) {
      index_ = byte == kStart0 ? 1 : 0;
      return 0;
    }

    buffer_[index_++] = byte;
    if (index_ == 4) {
      if (buffer_[2] != kVersion) {
        Reset();
        return 0;
      }
      expectedSize_ = buffer_[3] == FRAME_STATE ? kStateFrameSize :
                      buffer_[3] == FRAME_MATRIX_CELLS ? kMatrixCellsFrameSize :
                      buffer_[3] == FRAME_PRESET ? kPresetFrameSize :
                      buffer_[3] == FRAME_SENSOR_DIAGNOSTICS
                          ? kSensorDiagnosticsFrameSize :
                      buffer_[3] == FRAME_RUNTIME_SETTINGS
                          ? kRuntimeSettingsFrameSize :
                      buffer_[3] == FRAME_MATRIX_PERFORMANCE
                          ? kMatrixPerformanceFrameSize :
                      (buffer_[3] == FRAME_COMMAND || buffer_[3] == FRAME_ACK) ? kCommandFrameSize : 0;
      if (expectedSize_ == 0) {
        Reset();
        return 0;
      }
    }

    if (expectedSize_ == 0 || index_ < expectedSize_) {
      return 0;
    }

    const uint8_t type = buffer_[3];
    bool ok = false;
    if (type == FRAME_STATE) {
      ok = decodeStateFrame(buffer_, expectedSize_, state);
    } else if (type == FRAME_MATRIX_CELLS) {
      ok = decodeMatrixCellsFrame(buffer_, expectedSize_, cells);
    } else if (type == FRAME_PRESET) {
      ok = decodePresetFrame(buffer_, expectedSize_, preset);
    } else if (type == FRAME_SENSOR_DIAGNOSTICS) {
      ok = decodeSensorDiagnosticsFrame(buffer_, expectedSize_, diagnostics);
    } else if (type == FRAME_RUNTIME_SETTINGS) {
      ok = decodeRuntimeSettingsFrame(buffer_, expectedSize_, runtimeSettings);
    } else if (type == FRAME_MATRIX_PERFORMANCE) {
      ok = decodeMatrixPerformanceFrame(buffer_, expectedSize_, matrixPerformance);
    } else {
      ok = decodeCommandFrame(buffer_, expectedSize_, type, command);
    }
    Reset();
    return ok ? type : 0;
  }

  uint8_t Push(uint8_t byte,
               StateFrame &state,
               CommandFrame &command,
               MatrixCellsFrame &cells,
               PresetFrame &preset,
               SensorDiagnosticsFrame &diagnostics,
               RuntimeSettingsFrame &runtimeSettings) {
    MatrixPerformanceFrame unusedMatrixPerformance = {};
    return Push(byte, state, command, cells, preset, diagnostics,
                runtimeSettings, unusedMatrixPerformance);
  }

  uint8_t Push(uint8_t byte,
               StateFrame &state,
               CommandFrame &command,
               MatrixCellsFrame &cells,
               PresetFrame &preset,
               SensorDiagnosticsFrame &diagnostics) {
    RuntimeSettingsFrame unusedRuntimeSettings = {};
    return Push(byte,
                state,
                command,
                cells,
                preset,
                diagnostics,
                unusedRuntimeSettings);
  }

  uint8_t Push(uint8_t byte, StateFrame &state, CommandFrame &command) {
    MatrixCellsFrame unusedCells = {};
    PresetFrame unusedPreset = {};
    SensorDiagnosticsFrame unusedDiagnostics = {};
    return Push(byte, state, command, unusedCells, unusedPreset, unusedDiagnostics);
  }

  uint8_t Push(uint8_t byte, StateFrame &state, CommandFrame &command, MatrixCellsFrame &cells) {
    PresetFrame unusedPreset = {};
    SensorDiagnosticsFrame unusedDiagnostics = {};
    return Push(byte, state, command, cells, unusedPreset, unusedDiagnostics);
  }

  uint8_t Push(uint8_t byte,
               StateFrame &state,
               CommandFrame &command,
               MatrixCellsFrame &cells,
               PresetFrame &preset) {
    SensorDiagnosticsFrame unusedDiagnostics = {};
    return Push(byte, state, command, cells, preset, unusedDiagnostics);
  }

  void Reset() {
    index_ = 0;
    expectedSize_ = 0;
  }

  bool InFrame() const { return index_ != 0; }

 private:
  uint8_t buffer_[kMaxFrameSize];
  size_t index_;
  size_t expectedSize_;
};

}  // namespace bearamin_sidecar
