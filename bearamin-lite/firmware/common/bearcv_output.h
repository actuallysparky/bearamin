#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "trajectory_keyframe.h"

namespace bearcv {

constexpr uint16_t kDacMaximum = 4095;
constexpr uint16_t kBipolarZeroCode = 2048;
constexpr uint16_t kBipolarBiasCode = 3472;

inline float normalizedKnob(uint16_t sample, uint16_t maximum = 4095) {
  if (maximum == 0) return 0.0f;
  float value = static_cast<float>(std::min(sample, maximum)) / maximum;
  if (value < 0.01f) return 0.0f;
  if (value > 0.99f) return 1.0f;
  return value;
}

inline uint16_t unipolarDacCode(uint16_t norm12, float range) {
  const float value = std::min<uint16_t>(norm12, kDacMaximum) *
                      std::max(0.0f, std::min(1.0f, range));
  return static_cast<uint16_t>(std::round(value));
}

inline uint16_t bipolarDacCode(uint16_t norm12, float range) {
  const float amount = std::max(0.0f, std::min(1.0f, range));
  const int32_t signedCode = static_cast<int32_t>(
      std::min<uint16_t>(norm12, kDacMaximum)) - kBipolarZeroCode;
  const int32_t result = kBipolarZeroCode +
      static_cast<int32_t>(std::round(signedCode * amount));
  return static_cast<uint16_t>(std::max<int32_t>(0, std::min<int32_t>(kDacMaximum, result)));
}

inline uint8_t trajectorySlewStep(float knob) {
  const float unit = std::max(0.0f, std::min(1.0f, knob));
  // Preserve the physical-knob direction: counter-clockwise is fast, while
  // the portable step contract uses 0 = slow and 32 = fast.
  const uint8_t fastnessStep = static_cast<uint8_t>(std::round(unit * 32.0f));
  return static_cast<uint8_t>(32u - fastnessStep);
}

inline uint16_t trajectoryTauMs(float knob) {
  return bearamin_trajectory::tauForSlewStep(trajectorySlewStep(knob));
}

struct SixCvCodes {
  uint16_t xUnipolar;
  uint16_t yUnipolar;
  uint16_t zUnipolar;
  uint16_t xBipolar;
  uint16_t yBipolar;
  uint16_t zBipolar;
};

inline SixCvCodes safeZeroCodes() {
  return {0, 0, 0, kBipolarZeroCode, kBipolarZeroCode, kBipolarZeroCode};
}

inline SixCvCodes targetCodes(bool present,
                              uint16_t xNorm,
                              uint16_t yNorm,
                              uint16_t zNorm,
                              float range) {
  if (!present) return safeZeroCodes();
  return {
      unipolarDacCode(xNorm, range),
      unipolarDacCode(yNorm, range),
      unipolarDacCode(zNorm, range),
      bipolarDacCode(xNorm, range),
      bipolarDacCode(yNorm, range),
      bipolarDacCode(zNorm, range),
  };
}

}  // namespace bearcv
