#pragma once
#include <algorithm>
#include <cstdint>

// Adapted from MatrixCore's sidecar gesture detector: closest-point release,
// confirmed retreat, bounded tracking loss, and a three-sample descent fit.
namespace bearcv_spank {
constexpr uint16_t kSpankMinimumDescentMm=25;
constexpr uint16_t kSpankReleaseRiseMm=21;
constexpr uint32_t kSpankMaximumSampleGapMs=120;
constexpr uint32_t kSpankMaximumVelocityWindowMs=200;
constexpr uint32_t kSpankTrackingLossMs=250;
class Detector {
 public:
  bool update(bool confident, uint16_t depthMm, uint32_t nowMs,
              uint16_t maximumDepthMm=150, float minimumApproach=250,
              uint16_t refractoryMs=300) {
    if (!confident) {
      if (gate_) {
        if (lastConfidentMs_ == 0 || nowMs - lastConfidentMs_ >= kSpankTrackingLossMs) {
          release(nowMs, refractoryMs);
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
        if (++releaseFrames_ >= 2) release(nowMs, refractoryMs);
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
    const uint32_t latestElapsedMs=historyMs_[2]-historyMs_[1];
    const int32_t latestDescent=
        static_cast<int32_t>(historyDepthMm_[1])-historyDepthMm_[2];
    // A regression over a single step followed by a stationary/receding hand
    // must not create an onset. Also satisfy MatrixCore's latest-frame speed.
    const bool sustainedApproach=historyDepthMm_[0]>historyDepthMm_[1] &&
        latestDescent>0 && latestElapsedMs>0 &&
        float(latestDescent)*1000.0f/latestElapsedMs>=minimumApproach;
    if (depthMm <= maximumDepthMm &&
        approach >= minimumApproach &&
        descent >= kSpankMinimumDescentMm && sustainedApproach) {
      gate_ = true;
      closestDepthMm_ = depthMm;
      releaseFrames_ = 0;
    }
    return gate_;
  }
  bool gate() const { return gate_; }
  void reset() { *this = Detector{}; }
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
  void release(uint32_t nowMs, uint16_t refractoryMs) {
    gate_ = false;
    refractoryUntilMs_ = nowMs + refractoryMs;
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
} // namespace bearcv_spank
