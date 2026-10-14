#include "../include/bearfield/bearfield.h"

#include <cassert>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

using namespace bearfield;

static DepthFrame8x8 blob(uint32_t sequence,
                          uint64_t capturedUs,
                          uint8_t x0,
                          uint8_t y0,
                          uint16_t depthMm) {
  DepthFrame8x8 frame = {};
  frame.sequence = sequence;
  frame.capturedUs = capturedUs;
  for (uint8_t y = y0; y < static_cast<uint8_t>(y0 + 2); ++y) {
    for (uint8_t x = x0; x < static_cast<uint8_t>(x0 + 2); ++x) {
      const uint8_t index = cellIndex(x, y);
      frame.distanceMm[index] = depthMm;
      frame.validMask |= 1ULL << index;
    }
  }
  return frame;
}

int main(int argc, char **argv) {
  assert(argc == 2);
  for (uint8_t rotation = 0; rotation < 4; ++rotation) {
    for (uint8_t mirrorX = 0; mirrorX < 2; ++mirrorX) {
      for (uint8_t mirrorY = 0; mirrorY < 2; ++mirrorY) {
        bool seen[64] = {};
        for (uint8_t y = 0; y < 8; ++y) {
          for (uint8_t x = 0; x < 8; ++x) {
            const Coordinate transformed = transformCoordinate(
                {x, y}, {static_cast<QuarterTurns>(rotation),
                         mirrorX != 0, mirrorY != 0});
            const uint8_t index = cellIndex(transformed.x, transformed.y);
            assert(index < 64 && !seen[index]);
            seen[index] = true;
          }
        }
        for (bool present : seen) assert(present);
      }
    }
  }
  assert(transformCoordinate({0, 0}, {QuarterTurns::Deg90, false, false}).x == 7);
  assert(transformCoordinate({0, 0}, {QuarterTurns::Deg90, false, false}).y == 0);
  assert(transformCoordinate({0, 0}, {QuarterTurns::Deg180, false, false}).x == 7);
  assert(transformCoordinate({0, 0}, {QuarterTurns::Deg180, false, false}).y == 7);
  assert(transformCoordinate({1, 2}, {QuarterTurns::Deg0, true, false}).x == 6);

  DepthFrame8x8 raw = {};
  raw.validMask = 1ULL << cellIndex(0, 0);
  raw.distanceMm[cellIndex(0, 0)] = 123;
  const DepthFrame8x8 oriented = orientFrame(
      raw, {{QuarterTurns::Deg90, false, false}, UINT64_MAX});
  assert(oriented.distanceMm[cellIndex(7, 0)] == 123);
  assert(oriented.validMask == (1ULL << cellIndex(7, 0)));

  DisplayMountConfig serpentine = {};
  serpentine.pixelOrder = PixelOrder::Serpentine;
  assert(displayPixelIndex(0, 0, serpentine) == 0);
  assert(displayPixelIndex(0, 1, serpentine) == 15);
  uint8_t reversedLut[64] = {};
  for (uint8_t index = 0; index < 64; ++index) reversedLut[index] = 63 - index;
  DisplayMountConfig lookup = {};
  lookup.pixelOrder = PixelOrder::LookupTable;
  lookup.pixelLut = reversedLut;
  assert(displayPixelIndex(0, 0, lookup) == 63);
  assert(displayPixelIndex(7, 7, lookup) == 0);

  Processor processor(BearaminV2());
  auto first = processor.process(blob(1, 66667, 0, 0, 400), 66667);
  assert(first.present);
  assert(first.sourceSequence == 1);
  assert(first.xNorm12 < kNormCenter);
  assert(first.yNorm12 < kNormCenter);

  const auto halfTick = processor.tick(100000);
  const auto fullTick = processor.tick(133334);
  assert(halfTick.sourceSequence == 1);
  assert(fullTick.sourceSequence == 1);
  assert(!halfTick.spankStarted);
  assert(!fullTick.spankStarted);
  assert(halfTick.sourceSequence == first.sourceSequence);

  auto right = processor.process(blob(2, 133334, 6, 6, 350), 133334);
  assert(right.sourceSequence == 2);
  assert(right.xNorm12 == fullTick.xNorm12);
  const auto interpolated = processor.tick(166667);
  assert(interpolated.xNorm12 >= right.xNorm12);
  assert(interpolated.xNorm12 <= processor.target().xNorm12);
  const auto jittered = processor.tick(181234);
  assert(jittered.xNorm12 >= interpolated.xNorm12);
  assert(jittered.xNorm12 <= processor.target().xNorm12);

  Processor gesture(BearaminV2());
  gesture.process(blob(1, 100000, 3, 3, 500), 100000);
  const auto onset = gesture.process(blob(2, 166667, 3, 3, 100), 166667);
  assert(onset.spankStarted);
  assert(onset.spankHeld);
  const auto heldTick = gesture.tick(200000);
  assert(!heldTick.spankStarted);
  assert(heldTick.spankHeld);
  assert(heldTick.sourceSequence == onset.sourceSequence);

  FeelProfile croppedProfile = BearaminV2();
  croppedProfile.centroidMinimumX = 2;
  croppedProfile.centroidMaximumX = 7;
  croppedProfile.maximumSourceDepthMm = 705;
  croppedProfile.depthOffsetMm = 45;
  Processor cropped(croppedProfile);
  const auto croppedEdge = cropped.process(blob(1, 66667, 2, 3, 400), 66667);
  assert(croppedEdge.present);
  assert(croppedEdge.xMilli == 100);
  assert(croppedEdge.zMm == 400);
  assert(croppedEdge.zNorm12 < first.zNorm12);

  Processor cv(BearCvV1());
  cv.process(blob(1, 66667, 3, 3, 300), 66667);
  DepthFrame8x8 empty = {};
  empty.sequence = 2;
  empty.capturedUs = 400000;
  const auto absent = cv.process(empty, 400000);
  assert(!absent.present);
  assert(cv.target().xNorm12 == 0);
  assert(cv.target().yNorm12 == 0);
  assert(cv.target().zNorm12 == 0);

  cv.reset();
  const auto resetState = cv.tick(500000);
  assert(resetState.xNorm12 == 0);
  assert(resetState.yNorm12 == 0);
  assert(resetState.zNorm12 == 0);
  assert(resetState.pitchNorm12 == kNormCenter);
  assert(resetState.rollNorm12 == kNormCenter);

  std::ifstream replay(argv[1]);
  assert(replay.good());
  Processor replayProcessor(BearaminV1Compat());
  std::string line;
  std::getline(replay, line);
  uint32_t replayCount = 0;
  while (std::getline(replay, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::stringstream row(line);
    uint32_t values[6] = {};
    char comma = 0;
    for (uint8_t field = 0; field < 6; ++field) {
      row >> values[field];
      if (field != 5) row >> comma;
    }
    const auto state = replayProcessor.process(
        blob(values[0], values[1], values[2], values[3], values[4]), values[1]);
    assert(state.sourceSequence == values[0]);
    assert(state.present == (values[5] != 0));
    ++replayCount;
  }
  assert(replayCount == 5);

  return 0;
}
