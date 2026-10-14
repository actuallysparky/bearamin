#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bearamin_capture {

static constexpr uint8_t kMagic[4] = {'B', 'C', 'A', 'P'};
static constexpr uint8_t kVersion = 1;
// Session, metadata, and terminal records retain schema 1. Processed sensor
// snapshots version independently because their payload grew in place.
static constexpr uint16_t kSchemaVersion = 1;
static constexpr uint16_t kProcessedSnapshotV1SchemaVersion = 1;
static constexpr uint16_t kProcessedSnapshotV2SchemaVersion = 2;
static constexpr size_t kHeaderSize = 22;
static constexpr size_t kTrailerSize = 4;
static constexpr size_t kRecordOverhead = kHeaderSize + kTrailerSize;
static constexpr size_t kMaxPayloadSize = 224;
static constexpr size_t kMaxRecordSize = kRecordOverhead + kMaxPayloadSize;

enum RecordType : uint8_t {
  RECORD_SESSION_START = 1,
  RECORD_RESERVED_2 = 2,
  RECORD_MATRIX_FRAME = 3,
  RECORD_IMU_BATCH = 4,
  RECORD_PROCESSED_SNAPSHOT_V1 = 5,
  RECORD_DPT_CONTROL = 6,
  RECORD_DIAGNOSTIC = 7,
  RECORD_GAP = 8,
  RECORD_SESSION_END = 9,
  RECORD_CAPTURE_METADATA = 10,
  RECORD_PROCESSED_SNAPSHOT_V2 = 11,
};
static constexpr uint8_t kRecordTypeCount = 12;

enum RecordFlags : uint16_t {
  FLAG_NONE = 0,
  FLAG_SIDECAR_GENERATED = 1u << 0,
  FLAG_PARTIAL = 1u << 1,
};

enum SensorFlags : uint16_t {
  SENSOR_RESERVED_0 = 1u << 0,
  SENSOR_RESERVED_1 = 1u << 1,
  SENSOR_MATRIX_READY = 1u << 2,
  SENSOR_MATRIX_PRESENT = 1u << 3,
  SENSOR_IMU_READY = 1u << 4,
  SENSOR_IMPACT_GATE = 1u << 5,
  SENSOR_SPANK_GATE = 1u << 6,
};

enum GapStage : uint8_t {
  GAP_MATRIXCORE_RING = 1,
  GAP_SIDECAR_RING = 2,
  GAP_BROWSER_SEQUENCE = 3,
};

enum ControlKind : uint8_t {
  CONTROL_DPT_FRAME = 1,
  CONTROL_SIDECAR_COMMAND = 2,
  CONTROL_DPT_STATUS = 3,
};

static constexpr size_t kSessionStartPayloadSize = 24;
static constexpr size_t kMatrixPayloadSize = 208;
static constexpr size_t kImuSampleSize = 14;
static constexpr uint8_t kMaxImuBatchSamples = 8;
static constexpr size_t kImuBatchHeaderSize = 8;
static constexpr size_t kMaxImuBatchPayloadSize =
    kImuBatchHeaderSize + kMaxImuBatchSamples * kImuSampleSize;
static constexpr size_t kProcessedSnapshotV1PayloadSize = 34;
static constexpr size_t kProcessedSnapshotV2PayloadSize = 40;
static constexpr size_t kGapPayloadSize = 16;
static constexpr size_t kSessionEndPayloadSize = 56;
static constexpr size_t kDiagnosticPayloadSize = 24;

struct RecordView {
  uint8_t type;
  uint16_t flags;
  uint16_t payloadSize;
  uint32_t sequence;
  uint64_t coreUs;
  const uint8_t *payload;
};

struct MutableRecordView {
  RecordView record;
  uint8_t *payload;
};

struct ProcessedSensorSnapshotV1 {
  uint16_t sensorFlags;
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
  uint16_t sensorAgeMs;
};

struct ProcessedSensorSnapshotV2 : ProcessedSensorSnapshotV1 {
  uint16_t pitchNorm;
  uint16_t rollNorm;
  uint8_t orientationConfidence;
  uint8_t reserved0;
};

inline void writeLe16(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset] = static_cast<uint8_t>(value & 0xFFu);
  buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
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

inline void writeLe64(uint8_t *buffer, size_t offset, uint64_t value) {
  writeLe32(buffer, offset, static_cast<uint32_t>(value & 0xFFFFFFFFULL));
  writeLe32(buffer, offset + 4, static_cast<uint32_t>(value >> 32));
}

inline uint64_t readLe64(const uint8_t *buffer, size_t offset) {
  return static_cast<uint64_t>(readLe32(buffer, offset)) |
         (static_cast<uint64_t>(readLe32(buffer, offset + 4)) << 32);
}

inline uint32_t crc32(const uint8_t *data, size_t size) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = static_cast<uint32_t>(
          -static_cast<int32_t>(crc & 1u));
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

inline size_t recordSizeForPayload(uint16_t payloadSize) {
  return payloadSize <= kMaxPayloadSize
             ? kRecordOverhead + static_cast<size_t>(payloadSize)
             : 0;
}

inline size_t encodeRecord(uint8_t type,
                           uint16_t flags,
                           uint32_t sequence,
                           uint64_t coreUs,
                           const uint8_t *payload,
                           uint16_t payloadSize,
                           uint8_t *output,
                           size_t capacity) {
  const size_t totalSize = recordSizeForPayload(payloadSize);
  if (totalSize == 0 || capacity < totalSize ||
      (payloadSize > 0 && payload == nullptr)) {
    return 0;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    output[i] = kMagic[i];
  }
  output[4] = kVersion;
  output[5] = type;
  writeLe16(output, 6, flags);
  writeLe16(output, 8, payloadSize);
  writeLe32(output, 10, sequence);
  writeLe64(output, 14, coreUs);
  for (uint16_t i = 0; i < payloadSize; ++i) {
    output[kHeaderSize + i] = payload[i];
  }
  writeLe32(
      output, kHeaderSize + payloadSize, crc32(output, kHeaderSize + payloadSize));
  return totalSize;
}

inline bool hasMagic(const uint8_t *data, size_t size) {
  if (size < 4) {
    return false;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    if (data[i] != kMagic[i]) {
      return false;
    }
  }
  return true;
}

inline bool decodeRecord(const uint8_t *data,
                         size_t size,
                         RecordView &record,
                         size_t *consumed = nullptr) {
  if (size < kRecordOverhead || !hasMagic(data, size) ||
      data[4] != kVersion) {
    return false;
  }
  const uint16_t payloadSize = readLe16(data, 8);
  const size_t totalSize = recordSizeForPayload(payloadSize);
  if (totalSize == 0 || size < totalSize) {
    return false;
  }
  const uint32_t expected = readLe32(data, kHeaderSize + payloadSize);
  if (crc32(data, kHeaderSize + payloadSize) != expected) {
    return false;
  }
  record.type = data[5];
  record.flags = readLe16(data, 6);
  record.payloadSize = payloadSize;
  record.sequence = readLe32(data, 10);
  record.coreUs = readLe64(data, 14);
  record.payload = data + kHeaderSize;
  if (consumed != nullptr) {
    *consumed = totalSize;
  }
  return true;
}

inline void encodeProcessedSnapshotV1(
    const ProcessedSensorSnapshotV1 &snapshot,
    uint8_t payload[kProcessedSnapshotV1PayloadSize]) {
  writeLe16(payload, 0, kProcessedSnapshotV1SchemaVersion);
  writeLe16(payload, 2, snapshot.sensorFlags);
  writeLe16(payload, 4, snapshot.reserved0);
  writeLe16(payload, 6, snapshot.reserved1);
  writeLe16(payload, 8, snapshot.matrixZMm);
  writeLe16(payload, 10, snapshot.matrixXMilli);
  writeLe16(payload, 12, snapshot.matrixYMilli);
  writeLe16(payload, 14, snapshot.matrixZNorm);
  writeLe16(payload, 16, snapshot.matrixXNorm);
  writeLe16(payload, 18, snapshot.matrixYNorm);
  payload[20] = snapshot.matrixValidCount;
  payload[21] = snapshot.matrixBlobCount;
  writeLe16(payload, 22, snapshot.impactVelocityNorm);
  writeLe16(payload, 24, snapshot.impactAgeMs);
  writeLe16(payload, 26, static_cast<uint16_t>(snapshot.accelXMilliG));
  writeLe16(payload, 28, static_cast<uint16_t>(snapshot.accelYMilliG));
  writeLe16(payload, 30, static_cast<uint16_t>(snapshot.accelZMilliG));
  writeLe16(payload, 32, snapshot.sensorAgeMs);
}

inline bool decodeProcessedSnapshotV1(
    const uint8_t *payload,
    size_t payloadSize,
    ProcessedSensorSnapshotV1 &snapshot) {
  if (payloadSize != kProcessedSnapshotV1PayloadSize ||
      readLe16(payload, 0) != kProcessedSnapshotV1SchemaVersion) {
    return false;
  }
  snapshot.sensorFlags = readLe16(payload, 2);
  snapshot.reserved0 = readLe16(payload, 4);
  snapshot.reserved1 = readLe16(payload, 6);
  snapshot.matrixZMm = readLe16(payload, 8);
  snapshot.matrixXMilli = readLe16(payload, 10);
  snapshot.matrixYMilli = readLe16(payload, 12);
  snapshot.matrixZNorm = readLe16(payload, 14);
  snapshot.matrixXNorm = readLe16(payload, 16);
  snapshot.matrixYNorm = readLe16(payload, 18);
  snapshot.matrixValidCount = payload[20];
  snapshot.matrixBlobCount = payload[21];
  snapshot.impactVelocityNorm = readLe16(payload, 22);
  snapshot.impactAgeMs = readLe16(payload, 24);
  snapshot.accelXMilliG = static_cast<int16_t>(readLe16(payload, 26));
  snapshot.accelYMilliG = static_cast<int16_t>(readLe16(payload, 28));
  snapshot.accelZMilliG = static_cast<int16_t>(readLe16(payload, 30));
  snapshot.sensorAgeMs = readLe16(payload, 32);
  return true;
}

inline void encodeProcessedSnapshotV2(
    const ProcessedSensorSnapshotV2 &snapshot,
    uint8_t payload[kProcessedSnapshotV2PayloadSize]) {
  ProcessedSensorSnapshotV1 base = snapshot;
  encodeProcessedSnapshotV1(base, payload);
  writeLe16(payload, 0, kProcessedSnapshotV2SchemaVersion);
  writeLe16(payload, 34, snapshot.pitchNorm);
  writeLe16(payload, 36, snapshot.rollNorm);
  payload[38] = snapshot.orientationConfidence;
  payload[39] = snapshot.reserved0;
}

inline bool decodeProcessedSnapshotV2(
    const uint8_t *payload,
    size_t payloadSize,
    ProcessedSensorSnapshotV2 &snapshot) {
  if (payloadSize != kProcessedSnapshotV2PayloadSize ||
      readLe16(payload, 0) != kProcessedSnapshotV2SchemaVersion) {
    return false;
  }
  uint8_t basePayload[kProcessedSnapshotV1PayloadSize];
  for (size_t i = 0; i < kProcessedSnapshotV1PayloadSize; ++i) {
    basePayload[i] = payload[i];
  }
  writeLe16(basePayload, 0, kProcessedSnapshotV1SchemaVersion);
  ProcessedSensorSnapshotV1 base = {};
  if (!decodeProcessedSnapshotV1(basePayload, sizeof(basePayload), base)) {
    return false;
  }
  static_cast<ProcessedSensorSnapshotV1 &>(snapshot) = base;
  snapshot.pitchNorm = readLe16(payload, 34);
  snapshot.rollNorm = readLe16(payload, 36);
  snapshot.orientationConfidence = payload[38];
  snapshot.reserved0 = payload[39];
  return true;
}

template <size_t Capacity>
class ByteRing {
 public:
  ByteRing() : head_(0), tail_(0), used_(0), highWater_(0) {}

  size_t CapacityBytes() const { return Capacity; }
  size_t Used() const { return used_; }
  size_t Free() const { return Capacity - used_; }
  size_t HighWater() const { return highWater_; }

  bool TryPush(const uint8_t *data, size_t size) {
    if (data == nullptr || size > Free()) {
      return false;
    }
    for (size_t i = 0; i < size; ++i) {
      bytes_[head_] = data[i];
      head_ = (head_ + 1) % Capacity;
    }
    used_ += size;
    if (used_ > highWater_) {
      highWater_ = used_;
    }
    return true;
  }

  size_t Peek(uint8_t *output, size_t maxSize) const {
    if (output == nullptr) {
      return 0;
    }
    const size_t count = used_ < maxSize ? used_ : maxSize;
    size_t cursor = tail_;
    for (size_t i = 0; i < count; ++i) {
      output[i] = bytes_[cursor];
      cursor = (cursor + 1) % Capacity;
    }
    return count;
  }

  size_t ContiguousSize() const {
    if (used_ == 0) {
      return 0;
    }
    const size_t untilEnd = Capacity - tail_;
    return used_ < untilEnd ? used_ : untilEnd;
  }

  const uint8_t *ContiguousData() const {
    return used_ == 0 ? nullptr : bytes_ + tail_;
  }

  void Consume(size_t size) {
    if (size > used_) {
      size = used_;
    }
    tail_ = (tail_ + size) % Capacity;
    used_ -= size;
  }

  void Clear() {
    head_ = 0;
    tail_ = 0;
    used_ = 0;
    highWater_ = 0;
  }

 private:
  uint8_t bytes_[Capacity];
  size_t head_;
  size_t tail_;
  size_t used_;
  size_t highWater_;
};

template <size_t MaxRecordSize = kMaxRecordSize>
class StreamParser {
 public:
  StreamParser() : size_(0), expected_(0), crcErrors_(0), framingErrors_(0) {}

  bool Push(uint8_t byte, RecordView &record) {
    if (size_ < 4) {
      if (byte == kMagic[size_]) {
        buffer_[size_++] = byte;
      } else {
        if (size_ > 0) {
          ++framingErrors_;
        }
        size_ = byte == kMagic[0] ? 1 : 0;
        if (size_ == 1) {
          buffer_[0] = byte;
        }
      }
      return false;
    }

    if (size_ >= MaxRecordSize) {
      ++framingErrors_;
      resetWithPossibleMagic(byte);
      return false;
    }
    buffer_[size_++] = byte;
    if (size_ == 10) {
      const uint16_t payloadSize = readLe16(buffer_, 8);
      expected_ = recordSizeForPayload(payloadSize);
      if (expected_ == 0 || expected_ > MaxRecordSize) {
        ++framingErrors_;
        resetWithPossibleMagic(byte);
        return false;
      }
    }
    if (expected_ == 0 || size_ < expected_) {
      return false;
    }
    const bool valid = decodeRecord(buffer_, expected_, record);
    if (!valid) {
      ++crcErrors_;
    }
    size_ = 0;
    expected_ = 0;
    return valid;
  }

  uint32_t CrcErrors() const { return crcErrors_; }
  uint32_t FramingErrors() const { return framingErrors_; }
  bool InFrame() const { return size_ != 0; }

 private:
  void resetWithPossibleMagic(uint8_t byte) {
    size_ = byte == kMagic[0] ? 1 : 0;
    expected_ = 0;
    if (size_ == 1) {
      buffer_[0] = byte;
    }
  }

  uint8_t buffer_[MaxRecordSize];
  size_t size_;
  size_t expected_;
  uint32_t crcErrors_;
  uint32_t framingErrors_;
};

}  // namespace bearamin_capture
