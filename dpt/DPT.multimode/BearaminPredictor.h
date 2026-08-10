#pragma once

#include "MultimodeEngine.h"

#include <stddef.h>
#include <stdint.h>

namespace dpt_multimode
{

class BearaminMotionPredictor
{
  public:
    BearaminMotionPredictor()
    {
        Reset();
    }

    void Reset()
    {
        initialized_ = false;
        state_time_ms_ = 0;
        last_source_time_ms_ = 0;
        latest_interval_ms_ = 50;
        for(size_t axis = 0; axis < kAxisCount; ++axis)
        {
            position_[axis] = kNeutral;
            velocity_[axis] = 0.0f;
        }
    }

    BearaminBridgePacket Process(const BearaminBridgePacket& input,
                                 uint32_t                    now_ms,
                                 bool                        fresh)
    {
        BearaminBridgePacket output = input;
        const bool playable
            = fresh && (input.flags & kBearaminStale) == 0
              && (input.flags & kBearaminMatrixValid) != 0
              && input.matrix_blob_count > 0;
        if(!playable)
        {
            Reset();
            return output;
        }

        const uint32_t bounded_age_ms
            = input.age_ms > kMaximumSourceAgeMs ? kMaximumSourceAgeMs
                                                  : input.age_ms;
        const uint32_t source_time_ms = now_ms - bounded_age_ms;
        if(!initialized_
           || static_cast<int32_t>(source_time_ms - last_source_time_ms_)
                  >= static_cast<int32_t>(kMinimumSourceAdvanceMs))
        {
            float measurement[kAxisCount];
            ReadAxes(input, measurement);
            Observe(measurement, source_time_ms);
        }

        float predicted[kAxisCount];
        Predict(now_ms, predicted);
        WriteAxes(output, predicted);
        return output;
    }

  private:
    static constexpr size_t kAxisCount = 5;
    static constexpr float kNeutral = 2048.0f;
    static constexpr uint32_t kMinimumSourceAdvanceMs = 5;
    static constexpr uint32_t kMaximumSourceAgeMs = 250;
    static constexpr float kAlpha = 0.85f;
    static constexpr float kBeta = 0.08f;

    static float ClampAxis(float value)
    {
        return Clamp(value, 0.0f, 4095.0f);
    }

    static uint16_t RoundAxis(float value)
    {
        return static_cast<uint16_t>(ClampAxis(value) + 0.5f);
    }

    static void ReadAxes(const BearaminBridgePacket& packet,
                         float                       axes[kAxisCount])
    {
        axes[0] = static_cast<float>(packet.matrix_x_norm);
        axes[1] = static_cast<float>(packet.matrix_y_norm);
        axes[2] = static_cast<float>(packet.matrix_z_norm);
        axes[3] = static_cast<float>(packet.pitch_norm);
        axes[4] = static_cast<float>(packet.roll_norm);
    }

    static void WriteAxes(BearaminBridgePacket& packet,
                          const float           axes[kAxisCount])
    {
        packet.matrix_x_norm = RoundAxis(axes[0]);
        packet.matrix_y_norm = RoundAxis(axes[1]);
        packet.matrix_z_norm = RoundAxis(axes[2]);
        packet.pitch_norm = RoundAxis(axes[3]);
        packet.roll_norm = RoundAxis(axes[4]);
    }

    void Observe(const float measurement[kAxisCount],
                 uint32_t    source_time_ms)
    {
        if(!initialized_)
        {
            for(size_t axis = 0; axis < kAxisCount; ++axis)
            {
                position_[axis] = measurement[axis];
                velocity_[axis] = 0.0f;
            }
            state_time_ms_ = source_time_ms;
            last_source_time_ms_ = source_time_ms;
            initialized_ = true;
            return;
        }

        const uint32_t elapsed_ms = source_time_ms - state_time_ms_;
        if(elapsed_ms < kMinimumSourceAdvanceMs)
            return;
        latest_interval_ms_
            = elapsed_ms < 15 ? 15 : (elapsed_ms > 100 ? 100 : elapsed_ms);
        const float elapsed = static_cast<float>(elapsed_ms);
        for(size_t axis = 0; axis < kAxisCount; ++axis)
        {
            const float predicted
                = position_[axis] + velocity_[axis] * elapsed;
            const float residual = measurement[axis] - predicted;
            position_[axis] = predicted + kAlpha * residual;
            velocity_[axis] += (kBeta / elapsed) * residual;
        }
        state_time_ms_ = source_time_ms;
        last_source_time_ms_ = source_time_ms;
    }

    void Predict(uint32_t now_ms, float output[kAxisCount]) const
    {
        if(!initialized_)
        {
            for(size_t axis = 0; axis < kAxisCount; ++axis)
                output[axis] = kNeutral;
            return;
        }
        const uint32_t elapsed_ms = now_ms - state_time_ms_;
        const float bounded_elapsed
            = static_cast<float>(elapsed_ms > latest_interval_ms_
                                     ? latest_interval_ms_
                                     : elapsed_ms);
        for(size_t axis = 0; axis < kAxisCount; ++axis)
            output[axis] = position_[axis] + velocity_[axis] * bounded_elapsed;
    }

    bool initialized_;
    float position_[kAxisCount];
    float velocity_[kAxisCount];
    uint32_t state_time_ms_;
    uint32_t last_source_time_ms_;
    uint32_t latest_interval_ms_;
};

} // namespace dpt_multimode
