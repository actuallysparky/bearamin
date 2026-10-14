#pragma once

#include <stdint.h>

namespace dpt_multimode
{

// Single-producer/single-interrupt-consumer snapshot for a single-core target.
// The foreground writer fills the inactive slot before the release store. The
// audio interrupt loads one fully published slot and cannot be overlapped by a
// second foreground publication while it is using that slot.
template <typename T>
class RealtimeSnapshot
{
  public:
    RealtimeSnapshot() : active_(0), slots_{} {}

    const T& Read() const
    {
        const uint8_t active = __atomic_load_n(&active_, __ATOMIC_ACQUIRE);
        return slots_[active];
    }

    void Publish(const T& value)
    {
        const uint8_t active = __atomic_load_n(&active_, __ATOMIC_RELAXED);
        const uint8_t next   = static_cast<uint8_t>(active ^ 1u);
        slots_[next]         = value;
        __atomic_store_n(&active_, next, __ATOMIC_RELEASE);
    }

  private:
    uint8_t active_;
    T       slots_[2];
};

} // namespace dpt_multimode
