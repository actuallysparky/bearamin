#pragma once

#include <stdint.h>

namespace bearamin_sensor_recovery {

inline bool streamNeedsRecovery(bool ready,
                                uint32_t lastReadMs,
                                uint32_t initializedMs,
                                uint32_t now,
                                uint32_t firstSampleDeadlineMs,
                                uint32_t staleDeadlineMs) {
  if (!ready) {
    return false;
  }
  if (lastReadMs == 0) {
    return now - initializedMs > firstSampleDeadlineMs;
  }
  return now - lastReadMs > staleDeadlineMs;
}

inline uint32_t nextRetryMs(uint32_t currentMs, uint32_t maximumMs) {
  if (currentMs >= maximumMs || currentMs > maximumMs / 2U) {
    return maximumMs;
  }
  return currentMs * 2U;
}

inline uint32_t sensorMonitoringStartMs(bool ready, uint32_t now) {
  return ready ? now : 0;
}

}  // namespace bearamin_sensor_recovery
