#pragma once

#include <stdint.h>

namespace bearamin_wifi {

constexpr bool is5GhzChannel(int32_t channel) {
  return channel > 14;
}

constexpr uint8_t fiveGhzPulseBrightness(
    uint32_t nowMs,
    uint8_t minimum = 28,
    uint8_t maximum = 80,
    uint32_t periodMs = 2800) {
  if (maximum <= minimum || periodMs < 2) {
    return maximum;
  }
  const uint32_t halfPeriodMs = periodMs / 2;
  const uint32_t phaseMs = nowMs % periodMs;
  const uint32_t triangleMs =
      phaseMs <= halfPeriodMs ? phaseMs : periodMs - phaseMs;
  const uint32_t progressMilli =
      (triangleMs * 1000UL) / halfPeriodMs;
  const uint32_t smoothMilli =
      (progressMilli * progressMilli * (3000UL - 2UL * progressMilli) +
       500000UL) /
      1000000UL;
  return static_cast<uint8_t>(
      minimum +
      ((static_cast<uint32_t>(maximum - minimum) * smoothMilli + 500UL) /
       1000UL));
}

}  // namespace bearamin_wifi
