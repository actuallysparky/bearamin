#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace bearamin_preset {

// MatrixCore pauses regular bridge frames only long enough for the DPT to
// claim the half-duplex wire. Keep this below DPT's 250 ms freshness limit.
static constexpr uint32_t kQuietWatchdogMs = 200;

inline bool quietWindowActive(uint32_t quiet_until_ms, uint32_t now_ms) {
  return quiet_until_ms != 0 &&
         static_cast<int32_t>(now_ms - quiet_until_ms) < 0;
}

inline bool responseKindEquals(const char *line,
                               const char *prefix,
                               const char *kind) {
  if (line == nullptr || prefix == nullptr || kind == nullptr) {
    return false;
  }
  const size_t prefix_length = strlen(prefix);
  if (strncmp(line, prefix, prefix_length) != 0) {
    return false;
  }
  const char *sequence_end = strchr(line + prefix_length, ',');
  if (sequence_end == nullptr) {
    return false;
  }
  const char *response_kind = sequence_end + 1;
  const size_t kind_length = strlen(kind);
  return strncmp(response_kind, kind, kind_length) == 0 &&
         (response_kind[kind_length] == '\0' ||
          response_kind[kind_length] == ',');
}

inline bool terminalResponseAt(const char *line) {
  return strncmp(line, "BPERR,", 6) == 0 ||
         responseKindEquals(line, "BPLS,", "END") ||
         responseKindEquals(line, "BPRD,", "END") ||
         responseKindEquals(line, "BPWR,", "OK");
}

inline bool containsTerminalResponse(const char *line) {
  if (line == nullptr) {
    return false;
  }
  for (const char *cursor = line; *cursor != '\0'; ++cursor) {
    if (*cursor == 'B' && terminalResponseAt(cursor)) {
      return true;
    }
  }
  return false;
}

}  // namespace bearamin_preset
