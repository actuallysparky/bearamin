#pragma once
#include <stdint.h>

namespace bearcv_boot {
struct AddressBits { uint8_t eeprom; uint8_t input; };
enum class Result : uint8_t { kReady, kRepaired, kFailed, kAmbiguous };
constexpr uint8_t kAttempts = 3;
constexpr uint8_t kWriteBudget = 3; // Each device, each EEPROM operation type, per boot.
constexpr uint32_t kBusyTimeoutMs = 80;
constexpr uint32_t kRetryMs = 2000;
constexpr uint32_t kSweepDwellMs = 2500;
inline bool addressMatches(AddressBits value, uint8_t expected) {
  return value.eeprom == expected && value.input == expected;
}
inline bool decodeSelectedAddress(uint8_t packed, AddressBits *out) {
  // Fig. 5-6's fixed bits distinguish a real reply from floating SDA (0xff).
  if ((packed & 0x11u) != 0x10u) return false;
  *out = {static_cast<uint8_t>((packed >> 5) & 7),
          static_cast<uint8_t>((packed >> 1) & 7)};
  return true;
}
inline bool safeEeprom(const uint8_t bytes[24], uint8_t addressBits,
                       const uint16_t codes[4]) {
  for (uint8_t ch = 0; ch < 4; ++ch) {
    for (uint8_t bank = 0; bank < 2; ++bank) {
      const uint8_t i = ch * 6 + bank * 3;
      if (!(bytes[i] & 0x80) || (bytes[i] & 0x30) != (ch << 4) ||
          (bytes[i] & 7) != addressBits) return false;
      if ((bytes[i + 1] & 0xf0) != 0x90 ||
          (((bytes[i + 1] & 0x0f) << 8) | bytes[i + 2]) != codes[ch]) return false;
    }
  }
  return true;
}
// Transport performs bounded, physical LDAC selection, busy polling and recovery.
// ACK scans are cross-checks only, never a substitute for device-selected reads.
template <typename Transport> Result ensureAddresses(Transport &io) {
  bool repaired = false;
  for (uint8_t attempt = 0; attempt < kAttempts; ++attempt) {
    AddressBits bipolar = {}, unipolar = {};
    if (!io.recover() || !io.readPair(&bipolar, &unipolar)) continue;
    const uint8_t known = static_cast<uint8_t>((1u << bipolar.input) |
                                              (1u << unipolar.input));
    uint8_t mask = 0;
    if (!io.scan(&mask)) continue;
    if (mask != known) return Result::kAmbiguous;
    // UNI first also handles both devices at any shared address; a swapped pair
    // may temporarily collide, but LDAC selection continues to identify each.
    if (!addressMatches(unipolar, 1)) {
      if (!io.waitReady(unipolar.input) || !io.writeAddress(true, unipolar)) continue;
      repaired = true;
      if (!io.waitReady(1)) continue;
    }
    if (!addressMatches(bipolar, 0)) {
      if (!io.waitReady(bipolar.input) || !io.writeAddress(false, bipolar)) continue;
      repaired = true;
      if (!io.waitReady(0)) continue;
    }
    if (!io.readPair(&bipolar, &unipolar) ||
        !addressMatches(bipolar, 0) || !addressMatches(unipolar, 1) ||
        !io.scan(&mask) || mask != 0x03) continue;
    return repaired ? Result::kRepaired : Result::kReady;
  }
  return Result::kFailed;
}
inline uint16_t sweepCode(uint8_t step) {
  return step == 0 ? 0 : step == 1 ? 2048 : 4095;
}
inline bool sweepDue(uint32_t now, uint32_t start) {
  return static_cast<uint32_t>(now - start) >= kSweepDwellMs;
}
} // namespace bearcv_boot
