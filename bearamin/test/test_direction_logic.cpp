#include <cassert>
#include "../firmware/common/direction_logic.h"

using espmatrix::AccelSample;
using espmatrix::ImpactConfig;
using espmatrix::ImpactDetector;
using espmatrix::LiquidColor;
using espmatrix::LiquidVector;
using espmatrix::blendColorMilli;
using espmatrix::depthBrightnessForDistance;
using espmatrix::liquidAgitationFromMotion;
using espmatrix::liquidCellScore;
using espmatrix::liquidGravityFromAccel;
using espmatrix::liquidRainbowColor;
using espmatrix::logicalColumnForPhysicalVerticalFlip;
using espmatrix::midpointMarkerBlendMilli;
using espmatrix::nearestLedCoordinateForMilli;
using espmatrix::rainbowChannel;
using espmatrix::rotatedClockwisePixelIndex;
using espmatrix::rotatedClockwisePixelIndex180;
using espmatrix::scaleChannel;
using espmatrix::transposeCentroidMilli;
using espmatrix::tiltToLight;
using espmatrix::wrapUnit;

int main() {
  const auto flat = tiltToLight(AccelSample{0.0f, 0.0f, -1.0f});
  assert(flat.flat);
  assert(flat.column == 4);
  assert(flat.row == 4);
  assert(flat.green > flat.red);

  const auto right = tiltToLight(AccelSample{1.0f, 0.0f, -0.1f});
  assert(!right.flat);
  assert(right.column == 0);
  assert(right.red > right.blue);

  const auto left = tiltToLight(AccelSample{-1.0f, 0.0f, -0.1f});
  assert(left.column == 7);
  assert(left.blue > left.red);

  const auto up = tiltToLight(AccelSample{0.0f, 1.0f, -0.1f});
  assert(up.row == 0);
  assert(up.green > up.blue);

  const auto down = tiltToLight(AccelSample{0.0f, -1.0f, -0.1f});
  assert(down.row == 7);
  assert(down.red > down.blue);

  assert(rotatedClockwisePixelIndex(0, 0) == 7);
  assert(rotatedClockwisePixelIndex(7, 0) == 63);
  assert(rotatedClockwisePixelIndex(0, 7) == 0);
  assert(rotatedClockwisePixelIndex(7, 7) == 56);
  assert(rotatedClockwisePixelIndex(3, 6) == 25);
  assert(rotatedClockwisePixelIndex180(0, 0) == 56);
  assert(rotatedClockwisePixelIndex180(7, 0) == 0);
  assert(rotatedClockwisePixelIndex180(0, 7) == 63);
  assert(rotatedClockwisePixelIndex180(7, 7) == 7);
  assert(rotatedClockwisePixelIndex180(3, 6) == 38);
  assert(rotatedClockwisePixelIndex180(0, 0, 0) == 0);

  for (uint8_t column = 0; column < 8; ++column) {
    for (uint8_t row = 0; row < 8; ++row) {
      const uint8_t before = rotatedClockwisePixelIndex(column, row);
      const uint8_t after = rotatedClockwisePixelIndex(
          logicalColumnForPhysicalVerticalFlip(column), row);
      assert(after / 8 == 7 - before / 8);
      assert(after % 8 == before % 8);
    }
  }
  assert(logicalColumnForPhysicalVerticalFlip(0) == 7);
  assert(logicalColumnForPhysicalVerticalFlip(7) == 0);
  assert(logicalColumnForPhysicalVerticalFlip(8) == 0);
  assert(logicalColumnForPhysicalVerticalFlip(0, 0) == 0);

  const auto instrumentCentroid = transposeCentroidMilli(200, 750);
  assert(instrumentCentroid.x == 750);
  assert(instrumentCentroid.y == 200);
  const auto displayCentroid =
      transposeCentroidMilli(instrumentCentroid.x, instrumentCentroid.y);
  assert(displayCentroid.x == 200);
  assert(displayCentroid.y == 750);

  assert(nearestLedCoordinateForMilli(0) == 0);
  assert(nearestLedCoordinateForMilli(499) == 3);
  assert(nearestLedCoordinateForMilli(500) == 4);
  assert(nearestLedCoordinateForMilli(1000) == 7);
  assert(nearestLedCoordinateForMilli(1200) == 7);
  assert(nearestLedCoordinateForMilli(500, 1) == 0);

  assert(scaleChannel(255, 12) == 31);
  assert(scaleChannel(56, 12) == 7);
  assert(scaleChannel(0, 12) == 0);

  assert(depthBrightnessForDistance(20) > depthBrightnessForDistance(1000));
  assert(depthBrightnessForDistance(1000) > depthBrightnessForDistance(3500));
  assert(depthBrightnessForDistance(3500) == 18);
  assert(depthBrightnessForDistance(19) == 0);
  assert(depthBrightnessForDistance(3501) == 0);
  assert(depthBrightnessForDistance(500, false) == 0);

  assert(midpointMarkerBlendMilli(0) == 0);
  assert(midpointMarkerBlendMilli(1925) == 0);
  assert(midpointMarkerBlendMilli(1986) == 496);
  assert(midpointMarkerBlendMilli(2048) == 1000);
  assert(midpointMarkerBlendMilli(2110) == 496);
  assert(midpointMarkerBlendMilli(2171) == 0);
  assert(midpointMarkerBlendMilli(4095) == 0);
  const auto purple = blendColorMilli(
      LiquidColor{110, 110, 110}, LiquidColor{220, 0, 255}, 1000);
  assert(purple.red == 220 && purple.green == 0 && purple.blue == 255);
  const auto noPurple = blendColorMilli(
      LiquidColor{110, 110, 110}, LiquidColor{220, 0, 255}, 0);
  assert(noPurple.red == 110 && noPurple.green == 110 && noPurple.blue == 110);

  const auto liquidRight = liquidGravityFromAccel(AccelSample{1.0f, 0.0f, -0.1f});
  assert(liquidCellScore(7, 3, liquidRight) > liquidCellScore(0, 3, liquidRight));

  const auto liquidLeft = liquidGravityFromAccel(AccelSample{-1.0f, 0.0f, -0.1f});
  assert(liquidCellScore(0, 3, liquidLeft) > liquidCellScore(7, 3, liquidLeft));

  const auto liquidDown = liquidGravityFromAccel(AccelSample{0.0f, 1.0f, -0.1f});
  assert(liquidCellScore(3, 7, liquidDown) > liquidCellScore(3, 0, liquidDown));

  const auto liquidUp = liquidGravityFromAccel(AccelSample{0.0f, -1.0f, -0.1f});
  assert(liquidCellScore(3, 0, liquidUp) > liquidCellScore(3, 7, liquidUp));

  assert(liquidCellScore(3, 3, LiquidVector{0.0f, 0.0f}, 0.5f) >
         liquidCellScore(3, 3, LiquidVector{0.0f, 0.0f}, 0.0f));

  assert(wrapUnit(1.25f) > 0.249f && wrapUnit(1.25f) < 0.251f);
  assert(wrapUnit(-0.25f) > 0.749f && wrapUnit(-0.25f) < 0.751f);

  assert(rainbowChannel(0.0f, 0.0f) == 120);
  assert(rainbowChannel(0.333f, 0.333f) == 120);
  assert(rainbowChannel(0.667f, 0.667f) == 120);
  assert(rainbowChannel(0.5f, 0.0f) == 0);

  assert(liquidAgitationFromMotion(0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
  assert(liquidAgitationFromMotion(20.0f, 20.0f, 20.0f, 0.0f) >
         liquidAgitationFromMotion(5.0f, 5.0f, 5.0f, 0.0f));
  assert(liquidAgitationFromMotion(100.0f, 100.0f, 100.0f, 1.0f) == 1.0f);

  ImpactConfig impactConfig;
  impactConfig.thresholdG = 1.2f;
  impactConfig.saturationG = 5.2f;
  impactConfig.baselineAlpha = 0.0f;
  impactConfig.refractoryMs = 80;
  impactConfig.gateMs = 30;
  impactConfig.decayMs = 150;
  ImpactDetector detector(impactConfig);
  auto impact = detector.process(AccelSample{0.0f, 0.0f, 1.0f}, 0);
  assert(!impact.gate);
  assert(!impact.triggered);
  impact = detector.process(AccelSample{0.0f, 0.0f, 1.8f}, 10);
  assert(!impact.gate);
  assert(!impact.triggered);
  impact = detector.process(AccelSample{0.0f, 0.0f, 2.3f}, 20);
  assert(impact.gate);
  assert(impact.triggered);
  assert(impact.velocityNorm > 0);
  const uint16_t firstVelocity = impact.velocityNorm;
  impact = detector.process(AccelSample{0.0f, 0.0f, 2.5f}, 40);
  assert(impact.gate);
  assert(!impact.triggered);
  assert(impact.velocityNorm == firstVelocity);
  impact = detector.process(AccelSample{0.0f, 0.0f, 1.0f}, 60);
  assert(!impact.gate);
  assert(impact.velocityNorm < firstVelocity);
  impact = detector.process(AccelSample{0.0f, 0.0f, 2.5f}, 90);
  assert(!impact.triggered);
  impact = detector.process(AccelSample{0.0f, 0.0f, 6.2f}, 110);
  assert(impact.triggered);
  assert(impact.velocityNorm == 4095);
  detector.reset();
  impact = detector.process(AccelSample{0.0f, 0.0f, 1.0f}, 120);
  assert(!impact.gate);
  assert(!impact.triggered);
  assert(impact.velocityNorm == 0);
  const auto unavailable = espmatrix::unavailableImpactState();
  assert(!unavailable.gate);
  assert(!unavailable.triggered);
  assert(unavailable.velocityNorm == 0);
  assert(unavailable.ageMs == 65535);

  const auto rainbowA = liquidRainbowColor(0, 0, 0, 0.0f);
  const auto rainbowB = liquidRainbowColor(7, 7, 13, 0.0f);
  const auto rainbowC = liquidRainbowColor(0, 0, 0, 4.0f);
  assert(rainbowA.red <= 120 && rainbowA.green <= 120 && rainbowA.blue <= 120);
  assert(rainbowA.red > rainbowA.green && rainbowA.red > rainbowA.blue);
  assert(rainbowB.red != rainbowA.red || rainbowB.green != rainbowA.green || rainbowB.blue != rainbowA.blue);
  assert(rainbowC.red != rainbowA.red || rainbowC.green != rainbowA.green || rainbowC.blue != rainbowA.blue);

  return 0;
}
