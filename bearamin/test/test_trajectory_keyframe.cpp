#include <cassert>
#include <cmath>

#include "../firmware/common/trajectory_keyframe.h"

namespace {

using bearamin_trajectory::EvaluatedAxes;
using bearamin_trajectory::KeyframeV1;

KeyframeV1 baseKeyframe(uint16_t tauMs) {
  KeyframeV1 keyframe = {};
  keyframe.sourceEpochMs = 1000;
  keyframe.slewTauMs = tauMs;
  const uint16_t start[] = {0, 1000, 2048, 3000, 4095};
  const uint16_t target[] = {4095, 3000, 2048, 1000, 0};
  for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
    keyframe.startNorm[axis] = start[axis];
    keyframe.targetNorm[axis] = target[axis];
  }
  return keyframe;
}

void assertAxes(const EvaluatedAxes &evaluated, const uint16_t expected[5]) {
  for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
    assert(evaluated.norm[axis] == expected[axis]);
  }
}

void assertEvaluation(uint16_t tauMs,
                      float elapsedMs,
                      const uint16_t expected[5]) {
  EvaluatedAxes evaluated = {};
  assert(bearamin_trajectory::evaluateAtElapsedMs(
      baseKeyframe(tauMs), elapsedMs, evaluated));
  assertAxes(evaluated, expected);
}

}  // namespace

int main() {
  static const uint16_t kFast16[] = {1316, 1643, 2048, 2357, 2779};
  static const uint16_t kFast33[] = {2209, 2079, 2048, 1921, 1886};
  static const uint16_t kFast66[] = {3226, 2576, 2048, 1424, 869};
  static const uint16_t kMid16[] = {284, 1139, 2048, 2861, 3811};
  static const uint16_t kMid33[] = {548, 1268, 2048, 2732, 3547};
  static const uint16_t kMid66[] = {1023, 1500, 2048, 2500, 3072};
  static const uint16_t kSlow16[] = {159, 1078, 2048, 2922, 3936};
  static const uint16_t kSlow33[] = {312, 1153, 2048, 2847, 3783};
  static const uint16_t kSlow66[] = {601, 1294, 2048, 2706, 3494};
  static const uint16_t kStart[] = {0, 1000, 2048, 3000, 4095};

  assert(bearamin_trajectory::tauForSlewStep(0) == 420);
  assert(bearamin_trajectory::tauForSlewStep(16) == 232);
  assert(bearamin_trajectory::tauForSlewStep(32) == 43);
  assert(bearamin_trajectory::tauForSlewStep(255) == 43);

  bearamin_trajectory::Authority authority;
  const uint16_t firstTarget[] = {1000, 2000, 3000, 4000, 4095};
  const uint16_t secondTarget[] = {4095, 3000, 2048, 1000, 0};
  assert(authority.publishTarget(firstTarget, 1000, 420));
  assert(authority.hasKeyframe());
  assert(authority.keyframe().startNorm[0] == firstTarget[0]);
  assert(authority.publishTarget(secondTarget, 1066, 420));
  assert(authority.keyframe().startNorm[0] == 1000);
  assert(authority.changeTau(43, 1066));
  assert(authority.keyframe().slewTauMs == 43);
  assert(authority.keyframe().startNorm[0] == 1000);

  assertEvaluation(43, 0.0f, kStart);
  assertEvaluation(43, 16.667f, kFast16);
  assertEvaluation(43, 33.333f, kFast33);
  assertEvaluation(43, 66.667f, kFast66);
  assertEvaluation(232, 0.0f, kStart);
  assertEvaluation(232, 16.667f, kMid16);
  assertEvaluation(232, 33.333f, kMid33);
  assertEvaluation(232, 66.667f, kMid66);
  assertEvaluation(420, 0.0f, kStart);
  assertEvaluation(420, 16.667f, kSlow16);
  assertEvaluation(420, 33.333f, kSlow33);
  assertEvaluation(420, 66.667f, kSlow66);

  EvaluatedAxes sourceAge = {};
  static const uint16_t kSourceAge[] = {1706, 1833, 2048, 2167, 2389};
  assert(bearamin_trajectory::evaluateAtSourceTime(
      baseKeyframe(232), 1125, sourceAge));
  assertAxes(sourceAge, kSourceAge);

  KeyframeV1 beforeChange = baseKeyframe(420);
  const uint16_t beforeChangeStart[] = {1000, 2000, 3000, 4000, 4095};
  for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
    beforeChange.startNorm[axis] = beforeChangeStart[axis];
  }
  EvaluatedAxes atBoundary = {};
  assert(bearamin_trajectory::evaluateAtElapsedMs(
      beforeChange, 66.0f, atBoundary));
  static const uint16_t kBoundary[] = {1450, 2145, 2862, 3564, 3500};
  assertAxes(atBoundary, kBoundary);
  KeyframeV1 changed = baseKeyframe(43);
  for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
    changed.startNorm[axis] = atBoundary.norm[axis];
  }
  EvaluatedAxes afterChange = {};
  static const uint16_t kAfterChange[] = {2616, 2522, 2503, 2434, 1957};
  assert(bearamin_trajectory::evaluateAtElapsedMs(changed, 25.0f, afterChange));
  assertAxes(afterChange, kAfterChange);

  EvaluatedAxes repeatedHeartbeat = {};
  EvaluatedAxes lostPacket = {};
  static const uint16_t kContinued[] = {2366, 2155, 2048, 1845, 1729};
  assert(bearamin_trajectory::evaluateAtReceiptTime(
      baseKeyframe(232), 0, 5000, 5200, repeatedHeartbeat));
  assert(bearamin_trajectory::evaluateAtReceiptTime(
      baseKeyframe(232), 0, 5000, 5200, lostPacket));
  assertAxes(repeatedHeartbeat, kContinued);
  assertAxes(lostPacket, kContinued);

  KeyframeV1 correction = baseKeyframe(43);
  const uint16_t correctionStart[] = {3000, 2500, 2000, 1500, 1000};
  const uint16_t correctionTarget[] = {100, 200, 300, 400, 500};
  for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
    correction.startNorm[axis] = correctionStart[axis];
    correction.targetNorm[axis] = correctionTarget[axis];
  }
  EvaluatedAxes reconnect = {};
  static const uint16_t kReconnect[] = {1446, 1268, 1089, 911, 732};
  assert(bearamin_trajectory::evaluateAtElapsedMs(correction, 33.0f, reconnect));
  assertAxes(reconnect, kReconnect);

  KeyframeV1 wrapped = baseKeyframe(420);
  wrapped.sourceEpochMs = 0xfffffff0u;
  EvaluatedAxes wrapResult = {};
  static const uint16_t kWrap[] = {579, 1283, 2048, 2717, 3516};
  assert(bearamin_trajectory::evaluateAtSourceTime(wrapped, 48, wrapResult));
  assert(wrapResult.elapsedMs == 64);
  assertAxes(wrapResult, kWrap);
  assert(bearamin_trajectory::stale(true, 5260, 5000, 250));
  assert(!bearamin_trajectory::stale(true, 5250, 5000, 250));
  assert(bearamin_trajectory::stale(false, 5000, 5000, 250));

  KeyframeV1 invalid = baseKeyframe(232);
  invalid.version = 2;
  assert(!bearamin_trajectory::evaluateAtElapsedMs(invalid, 1.0f, sourceAge));
  invalid = baseKeyframe(42);
  assert(!bearamin_trajectory::evaluateAtElapsedMs(invalid, 1.0f, sourceAge));
  invalid = baseKeyframe(421);
  assert(!bearamin_trajectory::evaluateAtElapsedMs(invalid, 1.0f, sourceAge));
  invalid = baseKeyframe(232);
  invalid.targetNorm[0] = 4096;
  assert(!bearamin_trajectory::evaluateAtElapsedMs(invalid, 1.0f, sourceAge));
  assert(!bearamin_trajectory::elapsedSinceEpochMs(1, 2, sourceAge.elapsedMs));
  return 0;
}
