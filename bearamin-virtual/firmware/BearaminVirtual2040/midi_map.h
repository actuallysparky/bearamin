#pragma once
#include <bearfield/bearfield.h>
#include <stdint.h>

namespace bearamin_midi {
constexpr uint8_t kChannel = 0;
constexpr uint8_t kX = 20, kY = 21, kZ = 22, kPresence = 23;
constexpr uint8_t kSpankNote = 60;
inline uint8_t sevenBit(uint16_t norm12) {
  return norm12 >= 4095 ? 127 : static_cast<uint8_t>((uint32_t(norm12) * 127 + 2047) / 4095);
}
template <class Send>
void publish(const bearfield::ProcessedFieldState& field, bool& noteHeld, Send send) {
  send(0xB0 | kChannel, kX, sevenBit(field.xNorm12));
  send(0xB0 | kChannel, kY, sevenBit(field.yNorm12));
  send(0xB0 | kChannel, kZ, sevenBit(field.zNorm12));
  send(0xB0 | kChannel, kPresence, field.present ? 127 : 0);
  if (field.spankStarted && !noteHeld) {
    send(0x90 | kChannel, kSpankNote, 100);
    noteHeld = true;
  }
  if (noteHeld && !field.spankHeld) {
    send(0x80 | kChannel, kSpankNote, 0);
    noteHeld = false;
  }
}
template <class Send>
void silence(bool& noteHeld, Send send) {
  send(0xB0 | kChannel, kPresence, 0);
  if (noteHeld) { send(0x80 | kChannel, kSpankNote, 0); noteHeld = false; }
}
}
