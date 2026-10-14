#include <cassert>
#include <cmath>

#include "../firmware/common/orientation_logic.h"

using namespace bearamin_orientation;

static void fillPlane(SurfaceSample *samples,
                      uint8_t &count,
                      float rollDegrees,
                      float pitchDegrees,
                      float centerDepthMm,
                      bool addOutliers = false) {
  count = 0;
  const float a = std::tan(rollDegrees / 57.295779513f);
  const float b = std::tan(pitchDegrees / 57.295779513f);
  for (uint8_t x = 1; x <= 6; ++x) {
    for (uint8_t y = 1; y <= 6; ++y) {
      const float denominator =
          1.0f - a * kRayTangent[x] - b * kRayTangent[y];
      samples[count++] = {
          x, y,
          static_cast<uint16_t>(std::lround(centerDepthMm / denominator))};
    }
  }
  if (addOutliers) {
    samples[0].depthMm += 180;
    samples[35].depthMm += 120;
  }
}

int main() {
  SurfaceSample samples[64] = {};
  uint8_t count = 0;
  fillPlane(samples, count, 25.0f, -18.0f, 120.0f);
  auto estimate = estimateSurface(samples, count);
  assert(estimate.confident);
  assert(std::fabs(estimate.rollDegrees - 25.0f) < 1.0f);
  assert(std::fabs(estimate.pitchDegrees + 18.0f) < 1.0f);

  fillPlane(samples, count, -30.0f, 20.0f, 130.0f, true);
  estimate = estimateSurface(samples, count);
  assert(estimate.confident);
  assert(std::fabs(estimate.rollDegrees + 30.0f) < 2.0f);
  assert(std::fabs(estimate.pitchDegrees - 20.0f) < 2.0f);
  assert(estimate.inlierCount < estimate.pointCount);

  assert(!estimateSurface(samples, 11).confident);
  SurfaceSample singular[12] = {};
  for (uint8_t i = 0; i < 12; ++i) singular[i] = {3, 3, 100};
  assert(!estimateSurface(singular, 12).confident);

  // A partial palm at the right edge still fits when it covers three zones.
  count = 0;
  const float edgeSlope = std::tan(15.0f / 57.295779513f);
  for (uint8_t x = 5; x <= 7; ++x) {
    for (uint8_t y = 1; y <= 6; ++y) {
      samples[count++] = {
          x, y, static_cast<uint16_t>(std::lround(
              120.0f / (1.0f - edgeSlope * kRayTangent[x])))};
    }
  }
  estimate = estimateSurface(samples, count);
  assert(estimate.confident);
  assert(std::fabs(estimate.rollDegrees - 15.0f) < 1.5f);

  // A minority second surface is rejected by the robust fit.
  fillPlane(samples, count, 12.0f, -10.0f, 125.0f);
  for (uint8_t index : {0, 10, 20, 30}) samples[index].depthMm += 140;
  estimate = estimateSurface(samples, count);
  assert(estimate.confident);
  assert(std::fabs(estimate.rollDegrees - 12.0f) < 2.0f);
  assert(std::fabs(estimate.pitchDegrees + 10.0f) < 2.0f);

  assert(angleToNorm12(0.0f) == 2048);
  assert(angleToNorm12(2.0f) == 2048);
  assert(angleToNorm12(45.0f) == 4095);
  assert(angleToNorm12(60.0f) == 4095);
  assert(angleToNorm12(-45.0f) == 0);

  SurfaceEstimate missing;
  OrientationTracker tracker;
  estimate.pitchDegrees = 30.0f;
  estimate.rollDegrees = -20.0f;
  auto output = tracker.update(estimate, 1000);
  assert(output.pitchNorm > 2048 && output.rollNorm < 2048);
  output = tracker.update(missing, 1100);
  assert(output.pitchNorm > 2048);
  for (uint32_t now = 1200; now <= 2200; now += 100) {
    output = tracker.update(missing, now);
  }
  assert(std::abs(static_cast<int>(output.pitchNorm) - 2048) < 100);
  assert(std::abs(static_cast<int>(output.rollNorm) - 2048) < 100);

  SpankDetector spank;
  assert(!spank.update(true, 240, 799));
  assert(!spank.update(true, 240, 866));
  assert(!spank.update(true, 240, 933));
  assert(!spank.update(true, 210, 1000));
  assert(!spank.update(true, 180, 1067));
  assert(spank.update(true, 145, 1134));
  assert(spank.update(true, 130, 1201));
  assert(spank.update(true, 145, 1268));
  assert(spank.update(true, 152, 1335));
  assert(!spank.update(true, 155, 1402));
  assert(!spank.update(true, 120, 1469));  // Refractory.

  SpankDetector slow;
  assert(!slow.update(true, 200, 1799));
  assert(!slow.update(true, 200, 1866));
  assert(!slow.update(true, 200, 1933));
  assert(!slow.update(true, 170, 2000));
  assert(!slow.update(true, 160, 2067));
  assert(!slow.update(true, 149, 2134));

  SpankDetector tooFar;
  assert(!tooFar.update(true, 250, 2000));
  assert(!tooFar.update(true, 250, 2067));
  assert(!tooFar.update(true, 250, 2134));
  assert(!tooFar.update(true, 230, 2201));
  assert(!tooFar.update(true, 205, 2268));
  assert(!tooFar.update(true, 180, 2335));

  SpankDetector entry;
  assert(!entry.update(true, 210, 2400));
  assert(!entry.update(true, 180, 2467));
  assert(!entry.update(true, 145, 2534));  // Entry only arms.

  SpankDetector loss;
  assert(!loss.update(true, 240, 2799));
  assert(!loss.update(true, 240, 2866));
  assert(!loss.update(true, 240, 2933));
  assert(!loss.update(true, 210, 3000));
  assert(!loss.update(true, 180, 3067));
  assert(loss.update(true, 145, 3134));
  assert(loss.update(false, 0, 3383));
  assert(!loss.update(false, 0, 3384));
}
