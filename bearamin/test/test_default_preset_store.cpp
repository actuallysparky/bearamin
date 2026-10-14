#include <cassert>
#include <cstdio>
#include <vector>
#include <string>
#include "../firmware/common/default_preset_store.h"

using namespace bearamin_default;
using bearamin_sidecar::PresetFrame;
struct MemoryNvs {
  std::vector<uint8_t> committed;
  bool fail = false;
  int writes = 0;
  uint8_t Read(uint8_t *data, size_t &size) {
    if (committed.empty()) return kMissing;
    if (committed.size() > size) return kInvalid;
    size = committed.size();
    memcpy(data, committed.data(), size);
    return kOk;
  }
  bool Write(const uint8_t *data, size_t size) {
    if (fail) return false;
    committed.assign(data, data + size);
    ++writes;
    return true;
  }
};
uint16_t sequence = 0;
PresetFrame request(uint8_t op) {
  PresetFrame frame = {};
  frame.op = op;
  frame.sequence = ++sequence;
  return frame;
}
PresetFrame exchange(Store<MemoryNvs> &store, const PresetFrame &input) {
  // Exercise the actual wire codec and parser, including new op values.
  uint8_t wire[bearamin_sidecar::kPresetFrameSize];
  assert(bearamin_sidecar::encodePresetFrame(input, wire, sizeof(wire)) == sizeof(wire));
  PresetFrame decoded = {};
  assert(bearamin_sidecar::decodePresetFrame(wire, sizeof(wire), decoded));
  auto reply = store.Handle(decoded);
  assert(reply.sequence == input.sequence && reply.op == input.op);
  assert(bearamin_sidecar::encodePresetFrame(reply, wire, sizeof(wire)) == sizeof(wire));
  assert(bearamin_sidecar::decodePresetFrame(wire, sizeof(wire), decoded));
  return decoded;
}
void begin(Store<MemoryNvs> &store, const std::string &json, bool wrongCrc = false) {
  auto frame = request(kWriteBegin);
  frame.length = 4;
  bearamin_sidecar::writeLe16(frame.data, 0, json.size());
  auto crc = bearamin_sidecar::crc16Ccitt(reinterpret_cast<const uint8_t *>(json.data()), json.size());
  bearamin_sidecar::writeLe16(frame.data, 2, wrongCrc ? crc ^ 1 : crc);
  assert(exchange(store, frame).data[0] == kOk);
}
void chunks(Store<MemoryNvs> &store, const std::string &json) {
  for (size_t offset = 0; offset < json.size(); offset += kWriteChunkBytes) {
    auto frame = request(kWriteChunk);
    const size_t count = std::min(kWriteChunkBytes, json.size() - offset);
    frame.length = count + 2;
    bearamin_sidecar::writeLe16(frame.data, 0, offset);
    memcpy(frame.data + 2, json.data() + offset, count);
    assert(exchange(store, frame).data[0] == kOk);
    assert(exchange(store, frame).data[0] == kOk); // Lost ACK retry.
    frame.data[2] ^= 1;
    assert(exchange(store, frame).data[0] == kInvalid); // Conflicting duplicate.
  }
}
std::string read(Store<MemoryNvs> &store) {
  auto reply = exchange(store, request(kReadBegin));
  assert(reply.data[0] == kOk && reply.length == 5);
  const size_t size = bearamin_sidecar::readLe16(reply.data, 1);
  const uint16_t crc = bearamin_sidecar::readLe16(reply.data, 3);
  std::string json;
  while (json.size() < size) {
    auto frame = request(kReadChunk);
    frame.length = 2;
    bearamin_sidecar::writeLe16(frame.data, 0, json.size());
    reply = exchange(store, frame);
    assert(reply.data[0] == kOk && reply.length > 3);
    assert(bearamin_sidecar::readLe16(reply.data, 1) == json.size());
    const auto duplicate = exchange(store, frame);
    assert(memcmp(&reply, &duplicate, sizeof(reply)) == 0);
    json.append(reinterpret_cast<const char *>(reply.data + 3), reply.length - 3);
  }
  assert(bearamin_sidecar::crc16Ccitt(reinterpret_cast<const uint8_t *>(json.data()), json.size()) == crc);
  return json;
}
int main() {
  MemoryNvs nvs;
  Store<MemoryNvs> store(nvs);
  assert(exchange(store, request(kReadBegin)).data[0] == kMissing);
  // Full-size roundtrip includes inactive-mode settings, not just a runtime frame.
  std::string original = "{\"format\":\"bearamin-settings\",\"version\":10,\"settings\":{\"map_bearamin_fm_0_axis\":\"pitch\"}}";
  original.resize(kMaxBytes, ' ');
  begin(store, original);
  chunks(store, original);
  auto commit = request(kCommit);
  assert(exchange(store, commit).data[0] == kOk);
  assert(exchange(store, commit).data[0] == kOk && nvs.writes == 1);
  Store<MemoryNvs> coldBoot(nvs);
  assert(read(coldBoot) == original);

  const std::string changed = "{\"settings\":{\"cv_slew_step\":32}}";
  begin(coldBoot, changed);
  assert(exchange(coldBoot, request(kCommit)).data[0] == kInvalid);
  Store<MemoryNvs> interrupted(nvs); // Power loss while staging preserves old default.
  assert(read(interrupted) == original);
  begin(interrupted, changed, true);
  chunks(interrupted, changed);
  assert(exchange(interrupted, request(kCommit)).data[0] == kInvalid);
  assert(read(interrupted) == original);
  begin(interrupted, changed);
  chunks(interrupted, changed);
  nvs.fail = true;
  assert(exchange(interrupted, request(kCommit)).data[0] == kStorage);
  Store<MemoryNvs> failedSave(nvs);
  assert(read(failedSave) == original);
  nvs.fail = false;
  begin(failedSave, changed);
  chunks(failedSave, changed);
  assert(exchange(failedSave, request(kCommit)).data[0] == kOk);
  assert(read(failedSave) == changed);

  nvs.committed.back() ^= 1;
  assert(exchange(failedSave, request(kReadBegin)).data[0] == kInvalid);
  nvs.committed.resize(4);
  assert(exchange(failedSave, request(kReadBegin)).data[0] == kInvalid);
  auto oversized = request(kWriteBegin);
  oversized.length = 4;
  bearamin_sidecar::writeLe16(oversized.data, 0, kMaxBytes + 1);
  assert(exchange(failedSave, oversized).data[0] == kInvalid);
  assert(exchange(failedSave, request(kWriteChunk)).data[0] == kInvalid);
  puts("default preset storage: PASS (wire roundtrip, cold boot, interrupted/failed saves, CRC, retries)");
}
