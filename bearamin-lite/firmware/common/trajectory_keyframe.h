#pragma once

#include <cmath>
#include <cstdint>

namespace bearamin_trajectory {

constexpr uint8_t kVersion = 1;
constexpr uint8_t kAxisCount = 5;
constexpr uint16_t kNormMaximum = 4095;
constexpr uint16_t kMinimumTauMs = 43;
constexpr uint16_t kMaximumTauMs = 420;
constexpr uint32_t kMaximumComparableAgeMs = 0x7fffffffu;

enum Axis : uint8_t {
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2,
  AXIS_PITCH = 3,
  AXIS_ROLL = 4,
};

struct KeyframeV1 {
  uint8_t version = kVersion;
  uint32_t sourceEpochMs = 0;
  uint16_t startNorm[kAxisCount] = {};
  uint16_t targetNorm[kAxisCount] = {};
  uint16_t slewTauMs = kMinimumTauMs;
};

struct EvaluatedAxes {
  uint16_t norm[kAxisCount] = {};
  float decay = 1.0f;
  uint32_t elapsedMs = 0;
};

inline bool validTauMs(uint16_t tauMs) {
  return tauMs >= kMinimumTauMs && tauMs <= kMaximumTauMs;
}

inline bool validNorm(uint16_t value) {
  return value <= kNormMaximum;
}

inline bool validKeyframe(const KeyframeV1 &keyframe) {
  if (keyframe.version != kVersion || !validTauMs(keyframe.slewTauMs)) {
    return false;
  }
  for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
    if (!validNorm(keyframe.startNorm[axis]) ||
        !validNorm(keyframe.targetNorm[axis])) {
      return false;
    }
  }
  return true;
}

inline bool elapsedSinceEpochMs(uint32_t nowMs,
                                uint32_t sourceEpochMs,
                                uint32_t &elapsedMs) {
  elapsedMs = nowMs - sourceEpochMs;
  return elapsedMs < kMaximumComparableAgeMs;
}

inline float decayForElapsedMs(float elapsedMs, uint16_t tauMs) {
  return std::exp(-elapsedMs / static_cast<float>(tauMs));
}

inline uint16_t roundNorm(float value) {
  if (value <= 0.0f) return 0;
  if (value >= static_cast<float>(kNormMaximum)) return kNormMaximum;
  return static_cast<uint16_t>(std::floor(value + 0.5f));
}

inline bool evaluateAtElapsedMs(const KeyframeV1 &keyframe,
                                float elapsedMs,
                                EvaluatedAxes &evaluated) {
  if (!validKeyframe(keyframe) || !(elapsedMs >= 0.0f)) {
    return false;
  }
  evaluated.decay = decayForElapsedMs(elapsedMs, keyframe.slewTauMs);
  evaluated.elapsedMs = static_cast<uint32_t>(elapsedMs);
  for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
    const float start = static_cast<float>(keyframe.startNorm[axis]);
    const float target = static_cast<float>(keyframe.targetNorm[axis]);
    evaluated.norm[axis] = roundNorm(target + (start - target) * evaluated.decay);
  }
  return true;
}

inline bool evaluateAtSourceTime(const KeyframeV1 &keyframe,
                                 uint32_t nowMs,
                                 EvaluatedAxes &evaluated) {
  uint32_t elapsedMs = 0;
  return elapsedSinceEpochMs(nowMs, keyframe.sourceEpochMs, elapsedMs) &&
         evaluateAtElapsedMs(keyframe, static_cast<float>(elapsedMs), evaluated);
}

inline bool evaluateAtReceiptTime(const KeyframeV1 &keyframe,
                                  uint32_t sourceAgeAtReceiptMs,
                                  uint32_t receiptLocalMs,
                                  uint32_t nowLocalMs,
                                  EvaluatedAxes &evaluated) {
  uint32_t localElapsedMs = 0;
  if (!elapsedSinceEpochMs(nowLocalMs, receiptLocalMs, localElapsedMs) ||
      sourceAgeAtReceiptMs >= kMaximumComparableAgeMs) {
    return false;
  }
  const uint64_t elapsedMs =
      static_cast<uint64_t>(sourceAgeAtReceiptMs) + localElapsedMs;
  return elapsedMs < kMaximumComparableAgeMs &&
         evaluateAtElapsedMs(keyframe, static_cast<float>(elapsedMs), evaluated);
}

inline bool stale(bool hasValidReceipt,
                  uint32_t nowLocalMs,
                  uint32_t lastValidReceiptLocalMs,
                  uint32_t timeoutMs) {
  uint32_t elapsedMs = 0;
  return !hasValidReceipt ||
         !elapsedSinceEpochMs(nowLocalMs, lastValidReceiptLocalMs, elapsedMs) ||
         elapsedMs > timeoutMs;
}

inline uint16_t tauForSlewStep(uint8_t slewStep) {
  const uint8_t clamped = slewStep > 32 ? 32 : slewStep;
  const uint32_t span = kMaximumTauMs - kMinimumTauMs;
  const uint32_t offset = ((32u - clamped) * span + 16u) / 32u;
  return static_cast<uint16_t>(kMinimumTauMs + offset);
}

class Authority {
 public:
  bool hasKeyframe() const { return hasKeyframe_; }

  const KeyframeV1 &keyframe() const { return keyframe_; }

  bool publishTarget(const uint16_t targetNorm[kAxisCount],
                     uint32_t sourceEpochMs,
                     uint16_t tauMs) {
    if (!validTauMs(tauMs)) return false;
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
      if (!validNorm(targetNorm[axis])) return false;
    }
    KeyframeV1 next = keyframe_;
    next.version = kVersion;
    next.sourceEpochMs = sourceEpochMs;
    next.slewTauMs = tauMs;
    if (hasKeyframe_) {
      EvaluatedAxes continued = {};
      if (!evaluateAtSourceTime(keyframe_, sourceEpochMs, continued)) {
        return false;
      }
      for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        next.startNorm[axis] = continued.norm[axis];
      }
    } else {
      for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
        next.startNorm[axis] = targetNorm[axis];
      }
    }
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
      next.targetNorm[axis] = targetNorm[axis];
    }
    keyframe_ = next;
    hasKeyframe_ = true;
    return true;
  }

  bool changeTau(uint16_t tauMs, uint32_t boundaryMs) {
    if (!hasKeyframe_ || !validTauMs(tauMs)) return false;
    if (tauMs == keyframe_.slewTauMs) return true;
    EvaluatedAxes continued = {};
    if (!evaluateAtSourceTime(keyframe_, boundaryMs, continued)) {
      return false;
    }
    for (uint8_t axis = 0; axis < kAxisCount; ++axis) {
      keyframe_.startNorm[axis] = continued.norm[axis];
    }
    keyframe_.sourceEpochMs = boundaryMs;
    keyframe_.slewTauMs = tauMs;
    return true;
  }

 private:
  KeyframeV1 keyframe_ = {};
  bool hasKeyframe_ = false;
};

}  // namespace bearamin_trajectory
