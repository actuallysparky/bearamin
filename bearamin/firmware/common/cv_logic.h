#pragma once

#include <stdint.h>

struct CvRgb {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

inline uint16_t cvConstrainDistanceCm(uint16_t distanceCm, bool present, uint16_t minCm, uint16_t maxCm) {
  if (!present || distanceCm == 0) {
    return 0;
  }
  if (distanceCm < minCm) {
    return minCm;
  }
  if (distanceCm > maxCm) {
    return maxCm;
  }
  return distanceCm;
}

inline bool cvOpticalDistanceUsableMm(uint16_t distanceMm, uint16_t maxMm) {
  return distanceMm > 0 && distanceMm <= maxMm;
}

inline uint16_t cvPlayableDistanceWithCloseClampMm(
    uint16_t distanceMm,
    bool measurementValid,
    uint16_t minMm,
    uint16_t maxMm) {
  if (!measurementValid || distanceMm > maxMm) {
    return 0;
  }
  return distanceMm < minMm ? minMm : distanceMm;
}

inline uint32_t cvDutyTargetForDistance(
    uint16_t distanceCm,
    bool present,
    uint16_t minCm,
    uint16_t maxCm,
    uint32_t maxDuty) {
  if (!present || distanceCm == 0) {
    return 0;
  }
  if (maxCm <= minCm) {
    return maxDuty;
  }
  if (distanceCm <= minCm) {
    return maxDuty;
  }
  if (distanceCm >= maxCm) {
    return 0;
  }

  const uint32_t span = static_cast<uint32_t>(maxCm - minCm);
  const uint32_t fromFar = static_cast<uint32_t>(maxCm - distanceCm);
  return (fromFar * maxDuty) / span;
}

inline uint32_t cvApplyQuadraticCurve(uint32_t linearDuty, uint32_t maxDuty) {
  if (linearDuty == 0 || maxDuty == 0) {
    return 0;
  }
  if (linearDuty >= maxDuty) {
    return maxDuty;
  }
  return ((linearDuty * linearDuty) + (maxDuty / 2)) / maxDuty;
}

inline uint32_t cvDutyTargetForDistanceQuadratic(
    uint16_t distanceCm,
    bool present,
    uint16_t minCm,
    uint16_t maxCm,
    uint32_t maxDuty) {
  return cvApplyQuadraticCurve(cvDutyTargetForDistance(distanceCm, present, minCm, maxCm, maxDuty), maxDuty);
}

inline uint32_t cvDutyTargetForDistanceCurvedWithFade(
    uint16_t distanceCm,
    bool present,
    uint16_t minCm,
    uint16_t playableMaxCm,
    uint16_t fadeMaxCm,
    uint32_t maxDuty,
    uint32_t farEdgeDuty) {
  if (!present || distanceCm == 0 || maxDuty == 0) {
    return 0;
  }
  if (farEdgeDuty > maxDuty) {
    farEdgeDuty = maxDuty;
  }
  if (fadeMaxCm < playableMaxCm) {
    fadeMaxCm = playableMaxCm;
  }
  if (playableMaxCm <= minCm) {
    return distanceCm <= playableMaxCm ? maxDuty : 0;
  }
  if (distanceCm <= minCm) {
    return maxDuty;
  }
  if (distanceCm <= playableMaxCm) {
    const uint32_t span = static_cast<uint32_t>(playableMaxCm - minCm);
    const uint32_t fromFar = static_cast<uint32_t>(playableMaxCm - distanceCm);
    const uint32_t nearSquaredMilli = (fromFar * fromFar * 1000UL + (span * span / 2)) / (span * span);
    return farEdgeDuty + (((maxDuty - farEdgeDuty) * nearSquaredMilli + 500UL) / 1000UL);
  }
  if (distanceCm >= fadeMaxCm) {
    return 0;
  }

  const uint32_t fadeSpan = static_cast<uint32_t>(fadeMaxCm - playableMaxCm);
  if (fadeSpan == 0) {
    return 0;
  }
  const uint32_t fadeRemaining = static_cast<uint32_t>(fadeMaxCm - distanceCm);
  return (farEdgeDuty * fadeRemaining + (fadeSpan / 2)) / fadeSpan;
}

inline uint16_t cvScaleDutyToBridgeValue(uint32_t duty, uint32_t maxDuty, uint16_t maxBridgeValue) {
  if (duty == 0 || maxDuty == 0 || maxBridgeValue == 0) {
    return 0;
  }
  if (duty >= maxDuty) {
    return maxBridgeValue;
  }
  return static_cast<uint16_t>(((duty * maxBridgeValue) + (maxDuty / 2)) / maxDuty);
}

inline uint8_t cvLevelForBridgeValue(uint16_t bridgeValue, uint16_t maxBridgeValue, uint8_t levelCount) {
  if (bridgeValue == 0 || maxBridgeValue == 0 || levelCount == 0) {
    return 0;
  }
  if (bridgeValue >= maxBridgeValue) {
    return levelCount;
  }
  return static_cast<uint8_t>(((static_cast<uint32_t>(bridgeValue) * levelCount) + maxBridgeValue - 1) /
                              maxBridgeValue);
}

inline uint8_t cvWindowStartForPosition(uint16_t positionMilli, uint8_t litCount, uint8_t stripCount) {
  if (litCount == 0 || stripCount == 0 || litCount >= stripCount) {
    return 0;
  }
  if (positionMilli > 1000) {
    positionMilli = 1000;
  }
  const uint8_t maxStart = stripCount - litCount;
  return static_cast<uint8_t>(((static_cast<uint32_t>(positionMilli) * maxStart) + 500) / 1000);
}

inline int32_t cvSignedDifference(uint16_t value, uint16_t reference) {
  return static_cast<int32_t>(value) - static_cast<int32_t>(reference);
}

inline uint32_t cvAbsSignedDifference(int32_t value) {
  return value < 0 ? static_cast<uint32_t>(-value) : static_cast<uint32_t>(value);
}

inline int16_t cvCenteredOffsetMilli(uint16_t positionMilli) {
  if (positionMilli > 1000) {
    positionMilli = 1000;
  }
  return static_cast<int16_t>(static_cast<int32_t>(positionMilli) - 500);
}

inline uint16_t cvSpatialDeltaFromCenterMilli(uint16_t xMilli, uint16_t yMilli) {
  const uint32_t x = cvAbsSignedDifference(cvCenteredOffsetMilli(xMilli));
  const uint32_t y = cvAbsSignedDifference(cvCenteredOffsetMilli(yMilli));
  const uint32_t sum = x + y;
  return sum > 1000 ? 1000 : static_cast<uint16_t>(sum);
}

inline uint16_t cvBridgeValueForAxisPosition(uint32_t positionMilli, bool present, bool invert, uint16_t maxBridgeValue) {
  if (!present || maxBridgeValue == 0) {
    return 0;
  }
  if (positionMilli > 1000) {
    positionMilli = 1000;
  }
  const uint32_t valueMilli = invert ? 1000 - positionMilli : positionMilli;
  return static_cast<uint16_t>(((valueMilli * maxBridgeValue) + 500) / 1000);
}

inline uint16_t cvSlewBridgeValue(uint16_t currentValue, uint16_t targetValue, uint16_t maxStep) {
  if (maxStep == 0 || currentValue == targetValue) {
    return targetValue;
  }
  if (currentValue < targetValue) {
    const uint32_t next = static_cast<uint32_t>(currentValue) + maxStep;
    return next > targetValue ? targetValue : static_cast<uint16_t>(next);
  }
  const uint16_t delta = currentValue - targetValue;
  return delta > maxStep ? static_cast<uint16_t>(currentValue - maxStep) : targetValue;
}

inline constexpr uint16_t cvBridgeCenterValue(uint16_t maxBridgeValue) {
  return static_cast<uint16_t>((maxBridgeValue + 1U) / 2U);
}

struct CvBridgeIdleState {
  uint16_t x;
  uint16_t y;
  uint16_t z;
};

inline CvBridgeIdleState cvBridgeIdleStateForCentroid(
    uint16_t matrixXMilli,
    uint16_t matrixYMilli,
    uint16_t maxBridgeValue) {
  return {
      cvBridgeValueForAxisPosition(
          matrixXMilli, true, false, maxBridgeValue),
      cvBridgeValueForAxisPosition(
          matrixYMilli, true, true, maxBridgeValue),
      cvBridgeCenterValue(maxBridgeValue),
  };
}

struct CvBridgeXyState {
  uint16_t targetX;
  uint16_t targetY;
  uint16_t x;
  uint16_t y;
};

inline CvBridgeXyState cvBridgeXyForMatrixOrFallback(
    uint16_t currentX,
    uint16_t currentY,
    bool matrixPresent,
    uint16_t matrixXMilli,
    uint16_t matrixYMilli,
    bool thirdDMode,
    uint16_t bridgeZ,
    uint16_t maxBridgeValue,
    uint16_t maxStep) {
  if (matrixPresent) {
    const uint16_t targetX = cvBridgeValueForAxisPosition(matrixXMilli, true, false, maxBridgeValue);
    const uint16_t targetY = cvBridgeValueForAxisPosition(matrixYMilli, true, true, maxBridgeValue);
    return {
        targetX,
        targetY,
        cvSlewBridgeValue(currentX, targetX, maxStep),
        cvSlewBridgeValue(currentY, targetY, maxStep),
    };
  }
  if (thirdDMode && bridgeZ > 0 && maxBridgeValue > 0) {
    const uint16_t center = maxBridgeValue / 2;
    return {
        center,
        center,
        cvSlewBridgeValue(currentX, center, maxStep),
        cvSlewBridgeValue(currentY, center, maxStep),
    };
  }
  return {0, 0, 0, 0};
}

inline CvRgb cvNeoPixelColorForDistance(
    uint16_t distanceCm,
    bool present,
    uint16_t minCm,
    uint16_t maxCm,
    uint8_t maxBrightness,
    uint8_t noTargetYellowBrightness) {
  if (!present || distanceCm == 0) {
    return {noTargetYellowBrightness, noTargetYellowBrightness, 0};
  }
  if (distanceCm < minCm || distanceCm > maxCm) {
    return {0, static_cast<uint8_t>(maxBrightness / 2), maxBrightness};
  }
  if (maxCm <= minCm) {
    return {maxBrightness, 0, 0};
  }

  const uint32_t span = static_cast<uint32_t>(maxCm - minCm);
  const uint32_t fromMin = static_cast<uint32_t>(distanceCm - minCm);
  const uint8_t green = static_cast<uint8_t>((fromMin * maxBrightness) / span);
  const uint8_t red = static_cast<uint8_t>(((span - fromMin) * maxBrightness) / span);
  return {red, green, 0};
}

inline uint32_t cvSlewDuty(uint32_t currentDuty, uint32_t targetDuty, uint32_t maxStep) {
  if (maxStep == 0 || currentDuty == targetDuty) {
    return targetDuty;
  }
  if (currentDuty < targetDuty) {
    const uint32_t next = currentDuty + maxStep;
    return next > targetDuty ? targetDuty : next;
  }
  const uint32_t delta = currentDuty - targetDuty;
  return delta > maxStep ? currentDuty - maxStep : targetDuty;
}
