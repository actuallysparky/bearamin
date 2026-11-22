#pragma once
#include <cstdint>

namespace virtual_editor {
// Preview always follows every fresh sensor frame. The legacy 5 Hz request
// remains an alias so an older editor cannot reduce another client's rate.
struct PreviewCadence {
  unsigned divisor = 0;
  uint32_t until = 0;
  static bool supported(double hz) { return hz == 0 || hz == 5 || hz == 15; }
  void subscribe(double hz, uint32_t now) {
    divisor = hz == 15 || hz == 5 ? 1 : 0;
    until = now + 300000;
  }
  bool active(uint32_t now) const { return divisor && int32_t(until - now) > 0; }
  bool frame(uint32_t now) {
    return active(now);
  }
  double hz() const { return divisor ? 15.0 / divisor : 0; }
};
}
