#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace bearamin_orientation {

constexpr uint8_t kMinimumSurfacePoints = 12;
constexpr uint8_t kMinimumAxisCoverage = 3;
constexpr float kMinimumInlierFraction = 0.75f;
constexpr float kMaximumPlaneRmsMm = 15.0f;
constexpr uint8_t kRobustPasses = 4;
constexpr float kMaximumAngleDegrees = 45.0f;
constexpr float kNeutralDeadbandDegrees = 2.0f;
constexpr uint32_t kOrientationHoldMs = 134;
constexpr float kOrientationRecenterTauSeconds = 0.250f;
constexpr uint16_t kSpankMaximumDepthMm = 150;
constexpr float kSpankMinimumApproachMmPerSecond = 250.0f;
constexpr uint16_t kSpankMinimumDescentMm = 25;
constexpr uint16_t kSpankReleaseRiseMm = 21;
constexpr uint32_t kSpankMaximumSampleGapMs = 120;
constexpr uint32_t kSpankMaximumVelocityWindowMs = 200;
constexpr uint32_t kSpankTrackingLossMs = 250;
constexpr uint32_t kSpankRefractoryMs = 300;

// Zone-center tangents for the VL53L7CX 60-degree square field of view.
constexpr float kRayTangent[8] = {
    -0.49314543f, -0.33945426f, -0.19891237f, -0.06554346f,
     0.06554346f,  0.19891237f,  0.33945426f,  0.49314543f,
};

struct SurfaceSample {
  uint8_t xIndex;  // Canonical instrument X: 0=left, 7=right.
  uint8_t yIndex;  // Canonical pitch axis: 0=6 o'clock, 7=noon.
  uint16_t depthMm;
};

struct SurfaceEstimate {
  bool confident = false;
  float rollDegrees = 0.0f;
  float pitchDegrees = 0.0f;
  float rmsMm = 0.0f;
  uint16_t medianDepthMm = 0;
  uint8_t confidence = 0;
  uint8_t pointCount = 0;
  uint8_t inlierCount = 0;
  uint8_t xCoverage = 0;
  uint8_t yCoverage = 0;
};

inline float clampFloat(float value, float minimum, float maximum) {
  return std::max(minimum, std::min(maximum, value));
}

inline bool solve3x3(float m[3][3], float b[3], float out[3]) {
  for (uint8_t pivot = 0; pivot < 3; ++pivot) {
    uint8_t best = pivot;
    for (uint8_t row = static_cast<uint8_t>(pivot + 1); row < 3; ++row) {
      if (std::fabs(m[row][pivot]) > std::fabs(m[best][pivot])) best = row;
    }
    if (std::fabs(m[best][pivot]) < 1.0e-6f) return false;
    if (best != pivot) {
      for (uint8_t column = 0; column < 3; ++column) {
        std::swap(m[pivot][column], m[best][column]);
      }
      std::swap(b[pivot], b[best]);
    }
    const float divisor = m[pivot][pivot];
    for (uint8_t column = pivot; column < 3; ++column) m[pivot][column] /= divisor;
    b[pivot] /= divisor;
    for (uint8_t row = 0; row < 3; ++row) {
      if (row == pivot) continue;
      const float factor = m[row][pivot];
      for (uint8_t column = pivot; column < 3; ++column) {
        m[row][column] -= factor * m[pivot][column];
      }
      b[row] -= factor * b[pivot];
    }
  }
  out[0] = b[0]; out[1] = b[1]; out[2] = b[2];
  return true;
}

inline float median(float *values, uint8_t count) {
  if (count == 0) return 0.0f;
  std::sort(values, values + count);
  return (count & 1u) != 0
             ? values[count / 2]
             : (values[count / 2 - 1] + values[count / 2]) * 0.5f;
}

inline SurfaceEstimate estimateSurface(const SurfaceSample *samples, uint8_t count) {
  SurfaceEstimate result;
  result.pointCount = count;
  if (samples == nullptr || count < kMinimumSurfacePoints || count > 64) return result;

  float weights[64] = {}, residuals[64] = {}, scratch[64] = {};
  bool xSeen[8] = {}, ySeen[8] = {};
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t xi = std::min<uint8_t>(samples[i].xIndex, 7);
    const uint8_t yi = std::min<uint8_t>(samples[i].yIndex, 7);
    weights[i] = 1.0f;
    xSeen[xi] = true;
    ySeen[yi] = true;
  }
  for (bool seen : xSeen) result.xCoverage += seen ? 1 : 0;
  for (bool seen : ySeen) result.yCoverage += seen ? 1 : 0;
  if (result.xCoverage < kMinimumAxisCoverage ||
      result.yCoverage < kMinimumAxisCoverage) return result;

  float coefficients[3] = {};
  float robustScale = 1.0f;
  for (uint8_t pass = 0; pass <= kRobustPasses; ++pass) {
    float normal[3][3] = {}, rhs[3] = {};
    for (uint8_t i = 0; i < count; ++i) {
      const float z = static_cast<float>(samples[i].depthMm);
      const float x =
          z * kRayTangent[std::min<uint8_t>(samples[i].xIndex, 7)];
      const float y =
          z * kRayTangent[std::min<uint8_t>(samples[i].yIndex, 7)];
      const float row[3] = {x, y, 1.0f};
      for (uint8_t a = 0; a < 3; ++a) {
        rhs[a] += weights[i] * row[a] * z;
        for (uint8_t b = 0; b < 3; ++b) {
          normal[a][b] += weights[i] * row[a] * row[b];
        }
      }
    }
    if (!solve3x3(normal, rhs, coefficients)) return result;
    for (uint8_t i = 0; i < count; ++i) {
      const float z = static_cast<float>(samples[i].depthMm);
      const float x =
          z * kRayTangent[std::min<uint8_t>(samples[i].xIndex, 7)];
      const float y =
          z * kRayTangent[std::min<uint8_t>(samples[i].yIndex, 7)];
      residuals[i] =
          z - (coefficients[0] * x + coefficients[1] * y + coefficients[2]);
      scratch[i] = residuals[i];
    }
    const float residualMedian = median(scratch, count);
    for (uint8_t i = 0; i < count; ++i) {
      scratch[i] = std::fabs(residuals[i] - residualMedian);
    }
    robustScale = std::max(1.0f, 1.4826f * median(scratch, count));
    if (pass == kRobustPasses) break;
    for (uint8_t i = 0; i < count; ++i) {
      const float u =
          std::fabs(residuals[i] - residualMedian) / (4.685f * robustScale);
      const float base = 1.0f - u * u;
      weights[i] = u < 1.0f ? base * base : 0.0f;
    }
  }

  const float inlierLimit = std::max(3.0f, 2.5f * robustScale);
  float squaredSum = 0.0f;
  for (uint8_t i = 0; i < count; ++i) {
    if (std::fabs(residuals[i]) > inlierLimit) continue;
    scratch[result.inlierCount++] = static_cast<float>(samples[i].depthMm);
    squaredSum += residuals[i] * residuals[i];
  }
  if (result.inlierCount == 0) return result;
  result.rmsMm = std::sqrt(squaredSum / result.inlierCount);
  result.medianDepthMm =
      static_cast<uint16_t>(std::lround(median(scratch, result.inlierCount)));
  result.rollDegrees = std::atan(coefficients[0]) * 57.295779513f;
  result.pitchDegrees = std::atan(coefficients[1]) * 57.295779513f;

  const float inlierFraction =
      static_cast<float>(result.inlierCount) / static_cast<float>(count);
  const float countScore = clampFloat((static_cast<float>(count) - 12.0f) / 24.0f, 0.0f, 1.0f);
  const float residualScore = clampFloat((15.0f - result.rmsMm) / 10.0f, 0.0f, 1.0f);
  const float inlierScore = clampFloat((inlierFraction - 0.75f) / 0.25f, 0.0f, 1.0f);
  result.confidence = static_cast<uint8_t>(
      std::lround(std::min(countScore, std::min(residualScore, inlierScore)) * 255.0f));
  result.confident =
      result.inlierCount >= kMinimumSurfacePoints &&
      inlierFraction >= kMinimumInlierFraction &&
      result.rmsMm <= kMaximumPlaneRmsMm;
  if (result.confident && result.confidence == 0) result.confidence = 1;
  return result;
}

class OneEuroFilter {
 public:
  void reset(float value = 0.0f) {
    value_ = previousRaw_ = value;
    derivative_ = 0.0f;
    initialized_ = false;
  }
  float process(float raw, float dtSeconds) {
    if (!initialized_ || dtSeconds <= 0.0f) {
      value_ = previousRaw_ = raw;
      derivative_ = 0.0f;
      initialized_ = true;
      return value_;
    }
    const float derivativeRaw = (raw - previousRaw_) / dtSeconds;
    derivative_ += alpha(1.0f, dtSeconds) * (derivativeRaw - derivative_);
    const float cutoff = 0.8f + 0.03f * std::fabs(derivative_);
    value_ += alpha(cutoff, dtSeconds) * (raw - value_);
    previousRaw_ = raw;
    return value_;
  }
 private:
  static float alpha(float cutoffHz, float dtSeconds) {
    const float tau = 1.0f / (6.283185307f * cutoffHz);
    return 1.0f / (1.0f + tau / dtSeconds);
  }
  float value_ = 0.0f, previousRaw_ = 0.0f, derivative_ = 0.0f;
  bool initialized_ = false;
};

inline uint16_t angleToNorm12(float angleDegrees) {
  const float magnitude = std::fabs(angleDegrees);
  float unit = 0.0f;
  if (magnitude > kNeutralDeadbandDegrees) {
    unit = std::copysign(
        clampFloat((magnitude - kNeutralDeadbandDegrees) /
                       (kMaximumAngleDegrees - kNeutralDeadbandDegrees),
                   0.0f, 1.0f),
        angleDegrees);
  }
  return unit <= 0.0f
             ? static_cast<uint16_t>(std::lround(2048.0f + unit * 2048.0f))
             : static_cast<uint16_t>(std::lround(2048.0f + unit * 2047.0f));
}

struct OrientationOutput {
  uint16_t pitchNorm = 2048, rollNorm = 2048;
  uint8_t confidence = 0;
  float pitchDegrees = 0.0f, rollDegrees = 0.0f;
};

class OrientationTracker {
 public:
  OrientationOutput update(const SurfaceEstimate &estimate, uint32_t nowMs) {
    const float dt = lastUpdateMs_ == 0 ? 0.0f : (nowMs - lastUpdateMs_) * 0.001f;
    lastUpdateMs_ = nowMs;
    if (estimate.confident) {
      if (lastValidMs_ == 0 || nowMs - lastValidMs_ > kOrientationHoldMs + 250u) {
        pitchFilter_.reset(pitchDegrees_);
        rollFilter_.reset(rollDegrees_);
      }
      pitchDegrees_ = pitchFilter_.process(estimate.pitchDegrees, dt);
      rollDegrees_ = rollFilter_.process(estimate.rollDegrees, dt);
      lastValidMs_ = nowMs;
      confidence_ = estimate.confidence;
    } else {
      confidence_ = 0;
      const uint32_t invalidMs = lastValidMs_ == 0 ? UINT32_MAX : nowMs - lastValidMs_;
      if (invalidMs > kOrientationHoldMs && dt > 0.0f) {
        const float alpha = 1.0f - std::exp(-dt / kOrientationRecenterTauSeconds);
        pitchDegrees_ *= 1.0f - alpha;
        rollDegrees_ *= 1.0f - alpha;
      }
    }
    return {angleToNorm12(pitchDegrees_), angleToNorm12(rollDegrees_),
            confidence_, pitchDegrees_, rollDegrees_};
  }
 private:
  OneEuroFilter pitchFilter_, rollFilter_;
  uint32_t lastUpdateMs_ = 0, lastValidMs_ = 0;
  float pitchDegrees_ = 0.0f, rollDegrees_ = 0.0f;
  uint8_t confidence_ = 0;
};

class SpankDetector {
 public:
  bool update(bool confident, uint16_t depthMm, uint32_t nowMs) {
    if (!confident) {
      if (gate_) {
        if (lastConfidentMs_ == 0 || nowMs - lastConfidentMs_ >= kSpankTrackingLossMs) {
          release(nowMs);
        }
      } else {
        clearTracking();
      }
      return gate_;
    }
    lastConfidentMs_ = nowMs;
    if (gate_) {
      closestDepthMm_ = std::min(closestDepthMm_, depthMm);
      if (depthMm >= static_cast<uint16_t>(closestDepthMm_ + kSpankReleaseRiseMm)) {
        if (++releaseFrames_ >= 2) release(nowMs);
      } else {
        releaseFrames_ = 0;
      }
      return gate_;
    }
    if (static_cast<int32_t>(nowMs - refractoryUntilMs_) < 0) {
      clearTracking();
      return false;
    }
    if (historyCount_ > 0 &&
        nowMs - historyMs_[historyCount_ - 1] > kSpankMaximumSampleGapMs) {
      clearTracking();
    }
    if (armedFrames_ < 3) {
      ++armedFrames_;
      if (armedFrames_ == 3) append(depthMm, nowMs);
      return false;
    }
    append(depthMm, nowMs);
    if (historyCount_ < 3 ||
        historyMs_[2] - historyMs_[0] > kSpankMaximumVelocityWindowMs) return false;
    const float t0 = static_cast<float>(historyMs_[0]);
    float meanT = 0.0f, meanDepth = 0.0f;
    for (uint8_t i = 0; i < 3; ++i) {
      meanT += static_cast<float>(historyMs_[i]) - t0;
      meanDepth += historyDepthMm_[i];
    }
    meanT /= 3.0f; meanDepth /= 3.0f;
    float numerator = 0.0f, denominator = 0.0f;
    for (uint8_t i = 0; i < 3; ++i) {
      const float t = static_cast<float>(historyMs_[i]) - t0;
      numerator += (t - meanT) * (static_cast<float>(historyDepthMm_[i]) - meanDepth);
      denominator += (t - meanT) * (t - meanT);
    }
    if (denominator <= 0.0f) return false;
    const float approach = -(numerator / denominator) * 1000.0f;
    const int32_t descent =
        static_cast<int32_t>(historyDepthMm_[0]) - historyDepthMm_[2];
    if (depthMm <= kSpankMaximumDepthMm &&
        approach >= kSpankMinimumApproachMmPerSecond &&
        descent >= kSpankMinimumDescentMm) {
      gate_ = true;
      closestDepthMm_ = depthMm;
      releaseFrames_ = 0;
    }
    return gate_;
  }
  bool gate() const { return gate_; }
 private:
  void append(uint16_t depthMm, uint32_t nowMs) {
    if (historyCount_ < 3) {
      historyDepthMm_[historyCount_] = depthMm;
      historyMs_[historyCount_++] = nowMs;
    } else {
      historyDepthMm_[0] = historyDepthMm_[1];
      historyDepthMm_[1] = historyDepthMm_[2];
      historyDepthMm_[2] = depthMm;
      historyMs_[0] = historyMs_[1];
      historyMs_[1] = historyMs_[2];
      historyMs_[2] = nowMs;
    }
  }
  void clearTracking() { historyCount_ = armedFrames_ = 0; }
  void release(uint32_t nowMs) {
    gate_ = false;
    refractoryUntilMs_ = nowMs + kSpankRefractoryMs;
    closestDepthMm_ = releaseFrames_ = 0;
    clearTracking();
  }
  uint16_t historyDepthMm_[3] = {};
  uint32_t historyMs_[3] = {};
  uint8_t historyCount_ = 0, armedFrames_ = 0;
  bool gate_ = false;
  uint16_t closestDepthMm_ = 0;
  uint8_t releaseFrames_ = 0;
  uint32_t lastConfidentMs_ = 0, refractoryUntilMs_ = 0;
};

}  // namespace bearamin_orientation
