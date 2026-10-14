#pragma once

#include <stddef.h>
#include <string.h>

namespace bearamin_sidecar {

struct PresetResponseSpan {
  size_t start;
  size_t length;
};

inline bool presetResponseMarkerAt(const char *line, size_t lineLength, size_t offset) {
  static const char *const markers[] = {
      "BPERR,", "BPLS,", "BPRD,", "BPD,", "BPWR,", "BPAK,",
  };
  if (line == nullptr || offset >= lineLength) {
    return false;
  }
  for (const char *marker : markers) {
    const size_t markerLength = strlen(marker);
    if (offset + markerLength <= lineLength &&
        memcmp(line + offset, marker, markerLength) == 0) {
      return true;
    }
  }
  return false;
}

inline size_t findNextPresetResponseMarker(const char *line,
                                           size_t lineLength,
                                           size_t offset) {
  for (size_t index = offset; index < lineLength; ++index) {
    if (presetResponseMarkerAt(line, lineLength, index)) {
      return index;
    }
  }
  return lineLength;
}

// MatrixCore can deliver adjacent DPT response lines in one reconstructed frame.
// Return each response exactly once and in wire order so a following marker never
// contaminates the final CSV/hex token of the preceding response.
inline bool nextPresetResponseSpan(const char *line,
                                   size_t lineLength,
                                   size_t &cursor,
                                   PresetResponseSpan &span) {
  const size_t start = findNextPresetResponseMarker(line, lineLength, cursor);
  if (start == lineLength) {
    return false;
  }
  const size_t end = findNextPresetResponseMarker(line, lineLength, start + 1);
  span.start = start;
  span.length = end - start;
  cursor = end;
  return true;
}

}  // namespace bearamin_sidecar
