#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bearamin_obs {

static constexpr uint8_t kMagic[4] = {'B', 'O', 'V', '1'};
// STATE positions and CELLS use row-major bearamin_true_grid_v1: (0,0) is
// physical lower-left, X increases right, Y increases up. This is semantic
// metadata only; BOV1 v3 bytes remain unchanged.
static constexpr char kCoordinateSpace[] = "bearamin_true_grid_v1";
static constexpr uint8_t kVersion = 3;
static constexpr uint16_t kUdpPort = 51337;
static constexpr uint8_t kMulticastAddress[4] = {239, 255, 77, 66};
static constexpr uint16_t kMulticastPort = 51338;
static constexpr uint32_t kLeaseMs = 3000;
static constexpr size_t kMaxSubscribers = 4;
static constexpr size_t kHeaderSize = 12;
static constexpr size_t kCrcSize = 2;
static constexpr size_t kSubscribePayloadSize = 8;
static constexpr size_t kStatePayloadSize = 30;
static constexpr size_t kCellsPayloadSize = 136;
static constexpr size_t kSubscribePacketSize =
    kHeaderSize + kSubscribePayloadSize + kCrcSize;
static constexpr size_t kStatePacketSize =
    kHeaderSize + kStatePayloadSize + kCrcSize;
static constexpr size_t kCellsPacketSize =
    kHeaderSize + kCellsPayloadSize + kCrcSize;

enum PacketType : uint8_t {
  PACKET_SUBSCRIBE = 1,
  PACKET_STATE = 2,
  PACKET_CELLS = 3,
};

enum StreamMask : uint8_t {
  STREAM_STATE = 1u << 0,
  STREAM_CELLS = 1u << 1,
};

struct Subscribe {
  uint8_t streams;
  uint32_t clientNonce;
};

struct State {
  uint32_t sourceUptimeMs;
  uint16_t stateFlags;
  uint16_t sensorAgeMs;
  uint16_t matrixXNorm;
  uint16_t matrixYNorm;
  uint16_t matrixZNorm;
  uint16_t matrixXMilli;
  uint16_t matrixYMilli;
  uint16_t matrixZMm;
  uint16_t impactVelocityNorm;
  uint16_t impactAgeMs;
  uint16_t pitchNorm;
  uint16_t rollNorm;
  uint8_t matrixValidCount;
  uint8_t matrixBlobCount;
};

struct Cells {
  uint32_t sourceUptimeMs;
  uint16_t sensorAgeMs;
  uint8_t matrixValidCount;
  uint8_t matrixBlobCount;
  uint16_t cellsMm[64];
};

inline void writeLe16(uint8_t *target, uint16_t value) {
  target[0] = static_cast<uint8_t>(value & 0xFFu);
  target[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

inline void writeLe32(uint8_t *target, uint32_t value) {
  target[0] = static_cast<uint8_t>(value & 0xFFu);
  target[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  target[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  target[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

inline uint16_t readLe16(const uint8_t *source) {
  return static_cast<uint16_t>(source[0]) |
         static_cast<uint16_t>(source[1] << 8);
}

inline uint32_t readLe32(const uint8_t *source) {
  return static_cast<uint32_t>(source[0]) |
         (static_cast<uint32_t>(source[1]) << 8) |
         (static_cast<uint32_t>(source[2]) << 16) |
         (static_cast<uint32_t>(source[3]) << 24);
}

inline uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline void encodeHeader(uint8_t type,
                         uint16_t payloadSize,
                         uint16_t sequence,
                         uint8_t *packet) {
  for (uint8_t i = 0; i < 4; ++i) {
    packet[i] = kMagic[i];
  }
  packet[4] = kVersion;
  packet[5] = type;
  writeLe16(packet + 6, payloadSize);
  writeLe16(packet + 8, sequence);
  writeLe16(packet + 10, 0);
}

inline bool validPacket(const uint8_t *packet,
                        size_t length,
                        uint8_t expectedType,
                        size_t expectedPayloadSize) {
  const size_t expectedSize = kHeaderSize + expectedPayloadSize + kCrcSize;
  if (packet == nullptr || length != expectedSize ||
      packet[0] != kMagic[0] || packet[1] != kMagic[1] ||
      packet[2] != kMagic[2] || packet[3] != kMagic[3] ||
      packet[4] != kVersion || packet[5] != expectedType ||
      readLe16(packet + 6) != expectedPayloadSize) {
    return false;
  }
  return crc16Ccitt(packet, length - kCrcSize) ==
         readLe16(packet + length - kCrcSize);
}

inline size_t encodeSubscribe(const Subscribe &subscription,
                              uint16_t sequence,
                              uint8_t *packet,
                              size_t capacity) {
  if (capacity < kSubscribePacketSize) {
    return 0;
  }
  encodeHeader(PACKET_SUBSCRIBE, kSubscribePayloadSize, sequence, packet);
  uint8_t *payload = packet + kHeaderSize;
  payload[0] = subscription.streams;
  payload[1] = 0;
  payload[2] = 0;
  payload[3] = 0;
  writeLe32(payload + 4, subscription.clientNonce);
  writeLe16(packet + kSubscribePacketSize - kCrcSize,
            crc16Ccitt(packet, kSubscribePacketSize - kCrcSize));
  return kSubscribePacketSize;
}

inline bool decodeSubscribe(const uint8_t *packet,
                            size_t length,
                            Subscribe &subscription) {
  if (!validPacket(
          packet, length, PACKET_SUBSCRIBE, kSubscribePayloadSize)) {
    return false;
  }
  const uint8_t streams = packet[kHeaderSize];
  if ((streams & STREAM_STATE) == 0 ||
      (streams & ~(STREAM_STATE | STREAM_CELLS)) != 0) {
    return false;
  }
  subscription.streams = streams;
  subscription.clientNonce = readLe32(packet + kHeaderSize + 4);
  return true;
}

inline size_t encodeState(const State &state,
                          uint16_t sequence,
                          uint8_t *packet,
                          size_t capacity) {
  if (capacity < kStatePacketSize) {
    return 0;
  }
  encodeHeader(PACKET_STATE, kStatePayloadSize, sequence, packet);
  uint8_t *payload = packet + kHeaderSize;
  writeLe32(payload, state.sourceUptimeMs);
  writeLe16(payload + 4, state.stateFlags);
  writeLe16(payload + 6, state.sensorAgeMs);
  writeLe16(payload + 8, state.matrixXNorm);
  writeLe16(payload + 10, state.matrixYNorm);
  writeLe16(payload + 12, state.matrixZNorm);
  writeLe16(payload + 14, state.matrixXMilli);
  writeLe16(payload + 16, state.matrixYMilli);
  writeLe16(payload + 18, state.matrixZMm);
  writeLe16(payload + 20, state.impactVelocityNorm);
  writeLe16(payload + 22, state.impactAgeMs);
  writeLe16(payload + 24, state.pitchNorm);
  writeLe16(payload + 26, state.rollNorm);
  payload[28] = state.matrixValidCount;
  payload[29] = state.matrixBlobCount;
  writeLe16(packet + kStatePacketSize - kCrcSize,
            crc16Ccitt(packet, kStatePacketSize - kCrcSize));
  return kStatePacketSize;
}

inline size_t encodeCells(const Cells &cells,
                          uint16_t sequence,
                          uint8_t *packet,
                          size_t capacity) {
  if (capacity < kCellsPacketSize) {
    return 0;
  }
  encodeHeader(PACKET_CELLS, kCellsPayloadSize, sequence, packet);
  uint8_t *payload = packet + kHeaderSize;
  writeLe32(payload, cells.sourceUptimeMs);
  writeLe16(payload + 4, cells.sensorAgeMs);
  payload[6] = cells.matrixValidCount;
  payload[7] = cells.matrixBlobCount;
  for (uint8_t i = 0; i < 64; ++i) {
    writeLe16(payload + 8 + (i * 2), cells.cellsMm[i]);
  }
  writeLe16(packet + kCellsPacketSize - kCrcSize,
            crc16Ccitt(packet, kCellsPacketSize - kCrcSize));
  return kCellsPacketSize;
}

inline bool leaseActive(uint32_t now, uint32_t expiresAtMs) {
  return expiresAtMs != 0 &&
         static_cast<int32_t>(expiresAtMs - now) > 0;
}

enum LeaseUpdateResult : uint8_t {
  LEASE_INSERTED = 1,
  LEASE_REFRESHED = 2,
  LEASE_TABLE_FULL = 3,
};

struct SubscriberLease {
  uint32_t address = 0;
  uint32_t clientNonce = 0;
  uint32_t expiresAtMs = 0;
  uint8_t streams = 0;
};

class SubscriberLeaseTable {
 public:
  LeaseUpdateResult update(uint32_t address,
                           uint32_t clientNonce,
                           uint8_t streams,
                           uint32_t now) {
    SubscriberLease *available = nullptr;
    for (SubscriberLease &lease : leases_) {
      if (leaseActive(now, lease.expiresAtMs)) {
        if (lease.address == address &&
            lease.clientNonce == clientNonce) {
          lease.streams = streams;
          lease.expiresAtMs = now + kLeaseMs;
          return LEASE_REFRESHED;
        }
      } else if (available == nullptr) {
        available = &lease;
      }
    }
    if (available == nullptr) {
      return LEASE_TABLE_FULL;
    }
    available->address = address;
    available->clientNonce = clientNonce;
    available->streams = streams;
    available->expiresAtMs = now + kLeaseMs;
    return LEASE_INSERTED;
  }

  uint8_t expire(uint32_t now) {
    uint8_t expired = 0;
    for (SubscriberLease &lease : leases_) {
      if (lease.expiresAtMs != 0 &&
          !leaseActive(now, lease.expiresAtMs)) {
        lease = {};
        ++expired;
      }
    }
    return expired;
  }

  uint8_t activeCount(uint32_t now) const {
    uint8_t count = 0;
    for (const SubscriberLease &lease : leases_) {
      if (leaseActive(now, lease.expiresAtMs)) {
        ++count;
      }
    }
    return count;
  }

  uint8_t streamUnion(uint32_t now) const {
    uint8_t streams = 0;
    for (const SubscriberLease &lease : leases_) {
      if (leaseActive(now, lease.expiresAtMs)) {
        streams |= lease.streams;
      }
    }
    return streams;
  }

 private:
  SubscriberLease leases_[kMaxSubscribers] = {};
};

}  // namespace bearamin_obs
