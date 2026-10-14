#pragma once

#include <cstring>
#include "sidecar_protocol.h"

// Matrix-owned, single-key NVS record. No SD or DPT relay operations here.
namespace bearamin_default {
constexpr size_t kMaxBytes = 12000;
constexpr size_t kHeaderBytes = 8;
constexpr uint8_t kWriteBegin = 16;
constexpr uint8_t kWriteChunk = 17;
constexpr uint8_t kCommit = 18;
constexpr uint8_t kReadBegin = 19;
constexpr uint8_t kReadChunk = 20;
constexpr uint8_t kOk = 0;
constexpr uint8_t kMissing = 1;
constexpr uint8_t kInvalid = 2;
constexpr uint8_t kStorage = 3;
constexpr size_t kWriteChunkBytes = bearamin_sidecar::kPresetLinePayloadSize - 2;
constexpr size_t kReadChunkBytes = bearamin_sidecar::kPresetLinePayloadSize - 3;

inline bool isOperation(uint8_t op) { return op >= kWriteBegin && op <= kReadChunk; }

// Storage supplies Read(buffer, length) and Write(buffer, length). A failed
// atomic Write must retain the previous committed record. One foreground
// transaction at a time; chunks are request-driven so UART cannot overflow.
template <class Storage>
class Store {
 public:
  explicit Store(Storage &storage) : storage_(storage) {}

  bearamin_sidecar::PresetFrame Handle(const bearamin_sidecar::PresetFrame &request) {
    bearamin_sidecar::PresetFrame reply = {};
    reply.sequence = request.sequence;
    reply.op = request.op;
    reply.length = 1;
    reply.data[0] = kInvalid;
    if (request.length > bearamin_sidecar::kPresetLinePayloadSize) return reply;
    using namespace bearamin_sidecar;
    if (request.op == kWriteBegin && request.length == 4) {
      size_ = readLe16(request.data, 0);
      crc_ = readLe16(request.data, 2);
      received_ = 0;
      committed_ = false;
      reading_ = false;
      writing_ = size_ > 0 && size_ <= kMaxBytes;
      if (writing_) reply.data[0] = kOk;
    } else if (request.op == kWriteChunk && request.length > 2 && writing_) {
      const size_t offset = readLe16(request.data, 0);
      const size_t count = request.length - 2;
      if (offset <= received_ && offset + count <= size_) {
        if (offset == received_) {
          memcpy(record_ + kHeaderBytes + offset, request.data + 2, count);
          received_ += count;
          reply.data[0] = kOk;
        } else if (offset + count <= received_ &&
                   memcmp(record_ + kHeaderBytes + offset, request.data + 2, count) == 0) {
          reply.data[0] = kOk; // Retry of a chunk whose ACK was lost.
        }
      }
    } else if (request.op == kCommit && request.length == 0) {
      if (committed_) {
        reply.data[0] = kOk; // Retry must not write flash again.
      } else if (writing_ && received_ == size_ &&
                 crc16Ccitt(record_ + kHeaderBytes, size_) == crc_) {
        memcpy(record_, "BDP1", 4);
        writeLe16(record_, 4, static_cast<uint16_t>(size_));
        writeLe16(record_, 6, crc_);
        if (storage_.Write(record_, size_ + kHeaderBytes)) {
          committed_ = true;
          writing_ = false;
          reply.data[0] = kOk;
        } else {
          reply.data[0] = kStorage;
        }
      }
    } else if (request.op == kReadBegin && request.length == 0) {
      writing_ = false;
      committed_ = false;
      reading_ = false;
      size_t length = sizeof(record_);
      const uint8_t status = storage_.Read(record_, length);
      reply.data[0] = status;
      if (status == kOk) {
        if (length >= kHeaderBytes && memcmp(record_, "BDP1", 4) == 0) {
          size_ = readLe16(record_, 4);
          crc_ = readLe16(record_, 6);
          if (size_ > 0 && size_ <= kMaxBytes && length == size_ + kHeaderBytes &&
              crc16Ccitt(record_ + kHeaderBytes, size_) == crc_) {
            reading_ = true;
            reply.length = 5;
            writeLe16(reply.data, 1, static_cast<uint16_t>(size_));
            writeLe16(reply.data, 3, crc_);
          } else reply.data[0] = kInvalid;
        } else reply.data[0] = kInvalid;
      }
    } else if (request.op == kReadChunk && request.length == 2 && reading_) {
      const size_t offset = readLe16(request.data, 0);
      if (offset < size_) {
        const size_t count = size_ - offset < kReadChunkBytes ? size_ - offset : kReadChunkBytes;
        reply.data[0] = kOk;
        writeLe16(reply.data, 1, static_cast<uint16_t>(offset));
        memcpy(reply.data + 3, record_ + kHeaderBytes + offset, count);
        reply.length = static_cast<uint8_t>(count + 3);
      }
    }
    return reply;
  }

 private:
  Storage &storage_;
  uint8_t record_[kMaxBytes + kHeaderBytes] = {};
  size_t size_ = 0;
  size_t received_ = 0;
  uint16_t crc_ = 0;
  bool writing_ = false;
  bool committed_ = false;
  bool reading_ = false;
};
} // namespace bearamin_default
