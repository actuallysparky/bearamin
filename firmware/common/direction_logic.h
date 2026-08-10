#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace espmatrix {

struct AccelSample {
  float x;
  float y;
  float z;
};

struct TiltLight {
  int column;
  int row;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  bool flat;
};

struct LiquidVector {
  float x;
  float y;
};

struct LiquidColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct ImpactConfig {
  float thresholdG = 1.2f;
  float saturationG = 8.0f;
  float baselineAlpha = 0.015f;
  uint32_t refractoryMs = 80;
  uint32_t gateMs = 30;
  uint32_t decayMs = 150;
};

struct ImpactState {
  bool gate;
  bool triggered;
  float impulseG;
  uint16_t velocityNorm;
  uint16_t ageMs;
};

inline ImpactState unavailableImpactState() {
  return {false, false, 0.0f, 0, 65535};
}

constexpr uint16_t kDepthMinMm = 20;
constexpr uint16_t kDepthMaxMm = 3500;

inline float clampFloat(float value, float minimum, float maximum) {
  return std::max(minimum, std::min(maximum, value));
}

inline uint8_t clampByte(int value) {
  return static_cast<uint8_t>(std::max(0, std::min(255, value)));
}

inline uint8_t scaleChannel(uint8_t value, uint8_t percent) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * percent + 50) / 100);
}

inline uint16_t midpointMarkerBlendMilli(uint16_t value,
                                         uint16_t maximum = 4095,
                                         uint16_t rampHalfWidth = 123) {
  if (maximum == 0 || rampHalfWidth == 0) {
    return 0;
  }
  const uint16_t clamped = std::min(value, maximum);
  const uint16_t midpoint = static_cast<uint16_t>(
      (static_cast<uint32_t>(maximum) + 1U) / 2U);
  const uint16_t distance = clamped > midpoint
                                ? static_cast<uint16_t>(clamped - midpoint)
                                : static_cast<uint16_t>(midpoint - clamped);
  if (distance >= rampHalfWidth) {
    return 0;
  }
  return static_cast<uint16_t>(
      ((static_cast<uint32_t>(rampHalfWidth - distance) * 1000U) +
       (rampHalfWidth / 2U)) /
      rampHalfWidth);
}

inline LiquidColor blendColorMilli(LiquidColor base,
                                   LiquidColor target,
                                   uint16_t targetAmountMilli) {
  const uint16_t amount = std::min<uint16_t>(targetAmountMilli, 1000);
  const uint16_t baseAmount = static_cast<uint16_t>(1000 - amount);
  return LiquidColor{
      static_cast<uint8_t>(
          (static_cast<uint32_t>(base.red) * baseAmount +
           static_cast<uint32_t>(target.red) * amount + 500U) /
          1000U),
      static_cast<uint8_t>(
          (static_cast<uint32_t>(base.green) * baseAmount +
           static_cast<uint32_t>(target.green) * amount + 500U) /
          1000U),
      static_cast<uint8_t>(
          (static_cast<uint32_t>(base.blue) * baseAmount +
           static_cast<uint32_t>(target.blue) * amount + 500U) /
          1000U),
  };
}

inline float accelMagnitudeG(const AccelSample &sample) {
  return std::sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);
}

class ImpactDetector {
 public:
  explicit ImpactDetector(ImpactConfig config = ImpactConfig())
      : config_(config),
        baseline_(1.0f),
        lastTriggerMs_(0),
        lastSampleMs_(0),
        lastVelocityNorm_(0),
        initialized_(false) {}

  void reset() {
    baseline_ = 1.0f;
    lastTriggerMs_ = 0;
    lastSampleMs_ = 0;
    lastVelocityNorm_ = 0;
    initialized_ = false;
  }

  ImpactState process(const AccelSample &sample, uint32_t nowMs) {
    const float magnitude = accelMagnitudeG(sample);
    if (!initialized_) {
      baseline_ = magnitude;
      initialized_ = true;
      lastSampleMs_ = nowMs;
      return {false, false, 0.0f, 0, 0};
    }

    const float impulse = std::max(0.0f, std::fabs(magnitude - baseline_));
    const bool refractoryClear = lastTriggerMs_ == 0 || nowMs - lastTriggerMs_ >= config_.refractoryMs;
    const bool triggered = impulse >= config_.thresholdG && refractoryClear;
    if (triggered) {
      lastTriggerMs_ = nowMs;
      lastVelocityNorm_ = impulseToVelocityNorm(impulse);
    }

    const bool gate = lastTriggerMs_ != 0 && nowMs - lastTriggerMs_ < config_.gateMs;
    uint16_t age = 65535;
    if (lastTriggerMs_ != 0) {
      const uint32_t rawAge = nowMs - lastTriggerMs_;
      age = rawAge > 65535U ? 65535U : static_cast<uint16_t>(rawAge);
    }

    uint16_t velocity = lastVelocityNorm_;
    if (!gate && lastTriggerMs_ != 0 && config_.decayMs > 0) {
      const uint32_t since = nowMs - lastTriggerMs_;
      if (since >= config_.decayMs) {
        velocity = 0;
      } else if (since > config_.gateMs) {
        const uint32_t remaining = config_.decayMs - since;
        velocity = static_cast<uint16_t>((static_cast<uint32_t>(lastVelocityNorm_) * remaining) / config_.decayMs);
      }
    }

    baseline_ += (magnitude - baseline_) * clampFloat(config_.baselineAlpha, 0.0f, 1.0f);
    lastSampleMs_ = nowMs;
    return {gate, triggered, impulse, velocity, age};
  }

  float baselineG() const { return baseline_; }

 private:
  uint16_t impulseToVelocityNorm(float impulse) const {
    const float span = std::max(0.001f, config_.saturationG - config_.thresholdG);
    const float unit = clampFloat((impulse - config_.thresholdG) / span, 0.0f, 1.0f);
    return static_cast<uint16_t>(std::round(unit * 4095.0f));
  }

  ImpactConfig config_;
  float baseline_;
  uint32_t lastTriggerMs_;
  uint32_t lastSampleMs_;
  uint16_t lastVelocityNorm_;
  bool initialized_;
};

inline uint8_t depthBrightnessForDistance(uint16_t distanceMm, bool valid = true) {
  if (!valid || distanceMm < kDepthMinMm || distanceMm > kDepthMaxMm) {
    return 0;
  }
  const uint16_t clamped = static_cast<uint16_t>(clampFloat(distanceMm, kDepthMinMm, kDepthMaxMm));
  const uint32_t inverse = static_cast<uint32_t>(kDepthMaxMm - clamped) * 150U;
  return static_cast<uint8_t>(18U + inverse / (kDepthMaxMm - kDepthMinMm));
}

inline int axisToPixel(float axis) {
  const float clamped = clampFloat(axis, -1.0f, 1.0f);
  return static_cast<int>(std::round((clamped + 1.0f) * 3.5f));
}

struct CentroidMilli {
  uint16_t x;
  uint16_t y;
};

inline CentroidMilli transposeCentroidMilli(uint16_t x, uint16_t y) {
  return CentroidMilli{y, x};
}

inline uint8_t nearestLedCoordinateForMilli(uint16_t value, uint8_t width = 8) {
  if (width <= 1) {
    return 0;
  }
  const uint16_t clamped = std::min<uint16_t>(value, 1000);
  return static_cast<uint8_t>(
      (static_cast<uint32_t>(clamped) * (width - 1) + 500U) / 1000U);
}

inline uint8_t rotatedClockwisePixelIndex(uint8_t column, uint8_t row, uint8_t width = 8) {
  const uint8_t physicalColumn = static_cast<uint8_t>(width - 1 - row);
  const uint8_t physicalRow = column;
  return static_cast<uint8_t>(physicalRow * width + physicalColumn);
}

inline uint8_t rotatedClockwisePixelIndex180(
    uint8_t column,
    uint8_t row,
    uint8_t width = 8) {
  if (width == 0 || column >= width || row >= width) {
    return 0;
  }
  return rotatedClockwisePixelIndex(
      static_cast<uint8_t>(width - 1 - column),
      static_cast<uint8_t>(width - 1 - row),
      width);
}

inline uint8_t logicalColumnForPhysicalVerticalFlip(
    uint8_t column,
    uint8_t width = 8) {
  if (width == 0 || column >= width) {
    return 0;
  }
  return static_cast<uint8_t>(width - 1 - column);
}

inline LiquidVector liquidGravityFromAccel(const AccelSample &accel) {
  return LiquidVector{
      clampFloat(accel.x, -1.0f, 1.0f),
      clampFloat(accel.y, -1.0f, 1.0f),
  };
}

inline float liquidCellScore(uint8_t column, uint8_t row, LiquidVector gravity, float wave = 0.0f, uint8_t width = 8) {
  const float center = (static_cast<float>(width) - 1.0f) * 0.5f;
  const float x = static_cast<float>(column) - center;
  const float y = static_cast<float>(row) - center;
  return gravity.x * x + gravity.y * y + wave;
}

inline float wrapUnit(float value) {
  float wrapped = value - std::floor(value);
  if (wrapped < 0.0f) {
    wrapped += 1.0f;
  }
  return wrapped;
}

inline uint8_t rainbowChannel(float hue, float offset, uint8_t peak = 120) {
  float distance = std::fabs(wrapUnit(hue - offset));
  distance = std::min(distance, 1.0f - distance);
  return static_cast<uint8_t>(std::round(clampFloat(1.0f - distance * 3.0f, 0.0f, 1.0f) * static_cast<float>(peak)));
}

inline LiquidColor liquidRainbowColor(uint8_t column, uint8_t row, uint8_t rank, float phase) {
  const float hue = wrapUnit(phase * 0.13f +
                             static_cast<float>(column) * 0.18f +
                             static_cast<float>(row) * 0.14f +
                             static_cast<float>(rank) * 0.083f);
  return LiquidColor{
      rainbowChannel(hue, 0.0f),
      rainbowChannel(hue, 0.333f),
      rainbowChannel(hue, 0.667f),
  };
}

inline float liquidAgitationFromMotion(float gyroX, float gyroY, float gyroZ, float planarMagnitude) {
  const float gyroMotion = (std::fabs(gyroX) + std::fabs(gyroY) + std::fabs(gyroZ)) / 130.0f;
  return clampFloat(gyroMotion + planarMagnitude * 0.26f, 0.0f, 1.0f);
}

inline TiltLight tiltToLight(const AccelSample &accel) {
  const float x = clampFloat(accel.x, -1.0f, 1.0f);
  const float y = clampFloat(accel.y, -1.0f, 1.0f);
  const bool flat = std::fabs(x) < 0.10f && std::fabs(y) < 0.10f;

  const int red = static_cast<int>(std::round(std::max(0.0f, x) * 180.0f));
  const int blue = static_cast<int>(std::round(std::max(0.0f, -x) * 180.0f));
  const int green = static_cast<int>(std::round(std::max(0.0f, y) * 180.0f));
  const int amber = static_cast<int>(std::round(std::max(0.0f, -y) * 140.0f));

  TiltLight light = {
      axisToPixel(-x),
      axisToPixel(-y),
      clampByte(24 + red + amber),
      clampByte(24 + green + amber / 2),
      clampByte(24 + blue),
      flat,
  };

  if (flat) {
    light.red = 32;
    light.green = 80;
    light.blue = 120;
  }

  return light;
}

}  // namespace espmatrix
