#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace bearamin_virtual {
inline bool validZone(int distance, unsigned status) {
  return (status == 5 || status == 9) && distance > 0 && distance <= 2000;
}

// Fixed storage and explicit length keep USB traffic bounded. Callers drop
// complete messages when the CDC transmit buffer has insufficient room.
inline size_t frameLine(char *out, size_t capacity, uint32_t boot, uint32_t seq,
                        uint64_t captureUs, const int16_t *distance,
                        const uint8_t *status) {
  if (!out || !distance || !status || capacity == 0) return 0;
  int n = std::snprintf(out, capacity,
      "{\"v\":1,\"type\":\"frame\",\"boot\":%u,\"seq\":%u,\"capture_us\":%llu,\"mm\":[",
      static_cast<unsigned>(boot), static_cast<unsigned>(seq),
      static_cast<unsigned long long>(captureUs));
  if (n < 0 || static_cast<size_t>(n) >= capacity) return 0;
  size_t used = static_cast<size_t>(n);
  for (unsigned i = 0; i < 64; ++i) {
    n = validZone(distance[i], status[i])
        ? std::snprintf(out + used, capacity - used, "%s%d", i ? "," : "", distance[i])
        : std::snprintf(out + used, capacity - used, "%snull", i ? "," : "");
    if (n < 0 || static_cast<size_t>(n) >= capacity - used) return 0;
    used += static_cast<size_t>(n);
  }
  if (capacity - used < 4) return 0;
  out[used++] = ']'; out[used++] = '}'; out[used++] = '\n'; out[used] = 0;
  return used;
}

inline size_t statusLine(char *out, size_t capacity, uint32_t boot,
                         uint32_t seq, uint64_t captureUs, bool fault) {
  if (!out || capacity == 0) return 0;
  int n = std::snprintf(out, capacity,
      "{\"v\":1,\"type\":\"status\",\"boot\":%u,\"seq\":%u,\"capture_us\":%llu,\"state\":\"%s\"}\n",
      static_cast<unsigned>(boot), static_cast<unsigned>(seq),
      static_cast<unsigned long long>(captureUs), fault ? "sensor_fault" : "sensor_init");
  return n > 0 && static_cast<size_t>(n) < capacity ? static_cast<size_t>(n) : 0;
}
}
