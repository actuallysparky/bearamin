#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// A deliberately slow Lorenz source with Sloths-like, indirect influence.
// This stays local to the firmware protocol: it only supplies cooked XYZ axes.
namespace bearamin_chaos {

constexpr uint16_t kNormMaximum = 4095;

struct Axes {
  uint16_t x = 2048;
  uint16_t y = 2048;
  uint16_t z = 2048;
};

class StartupUnlock {
 public:
  void begin(uint32_t nowMs) { readyMs_ = nowMs; }
  bool active(uint32_t nowMs) const { return !latched_ && nowMs - readyMs_ <= 5000; }
  bool latched() const { return latched_; }
  void force() { latched_ = true; }
  bool event(uint32_t nowMs) {
    if (!active(nowMs)) return false;
    if (count_ && nowMs - firstMs_ > 2500) { count_ = 0; }
    if (!count_) firstMs_ = nowMs;
    if (++count_ == 5) { latched_ = true; return true; }
    return false;
  }
 private:
  uint32_t readyMs_ = 0;
  uint32_t firstMs_ = 0;
  uint8_t count_ = 0;
  bool latched_ = false;
};

class Lorenz {
 public:
  void seed(uint32_t seed) {
    const float a = static_cast<float>((seed & 0x3ffu) + 1u) / 1025.0f;
    x_ = -8.0f + 16.0f * a;
    y_ = -7.0f + 14.0f * static_cast<float>(((seed >> 10) & 0x3ffu) + 1u) / 1025.0f;
    z_ = 20.0f + 12.0f * static_cast<float>(((seed >> 20) & 0x3ffu) + 1u) / 1025.0f;
    influence_ = 0.0f;
  }
  // control is 0..1: five seconds at zero, ninety seconds at one.
  Axes tick(float elapsedSeconds, float control, float tofInfluence) {
    control = std::max(0.0f, std::min(1.0f, control));
    tofInfluence = std::max(-1.0f, std::min(1.0f, tofInfluence));
    influence_ += (tofInfluence - influence_) * std::min(1.0f, elapsedSeconds / 3.5f);
    const float period = 5.0f + 85.0f * control;
    // 0.65 Lorenz time units is the calibrated nominal lobe turn.
    const float totalDt = elapsedSeconds * 0.65f / period;
    constexpr uint8_t kSubsteps = 4;
    for (uint8_t i = 0; i < kSubsteps; ++i) step(totalDt / kSubsteps);
    if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(z_) ||
        std::fabs(x_) > 80.0f || std::fabs(y_) > 100.0f || z_ < -10.0f || z_ > 120.0f) {
      seed(0x6d2b79f5u);
    }
    return {toNorm(x_ / 24.0f), toNorm(y_ / 32.0f), toNorm((z_ - 25.0f) / 28.0f)};
  }
 private:
  static uint16_t toNorm(float value) {
    value = std::max(-1.0f, std::min(1.0f, value));
    return static_cast<uint16_t>(std::lround((value + 1.0f) * 2047.5f));
  }
  void derivatives(float px, float py, float pz, float &dx, float &dy, float &dz) const {
    constexpr float sigma = 10.0f;
    constexpr float beta = 8.0f / 3.0f;
    const float rho = 28.0f + influence_ * 1.25f;
    dx = sigma * (py - px);
    dy = px * (rho - pz) - py;
    dz = px * py - beta * pz;
  }
  void step(float dt) {
    float a, b, c, d, e, f, g, h, j, k, l, m;
    derivatives(x_, y_, z_, a, b, c);
    derivatives(x_ + a*dt/2, y_ + b*dt/2, z_ + c*dt/2, d, e, f);
    derivatives(x_ + d*dt/2, y_ + e*dt/2, z_ + f*dt/2, g, h, j);
    derivatives(x_ + g*dt, y_ + h*dt, z_ + j*dt, k, l, m);
    x_ += dt * (a + 2*d + 2*g + k) / 6.0f;
    y_ += dt * (b + 2*e + 2*h + l) / 6.0f;
    z_ += dt * (c + 2*f + 2*j + m) / 6.0f;
  }
  float x_ = 0.1f, y_ = 0.0f, z_ = 25.0f, influence_ = 0.0f;
};

}  // namespace bearamin_chaos
