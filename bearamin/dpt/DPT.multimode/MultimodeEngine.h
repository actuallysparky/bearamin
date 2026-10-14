#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace dpt_multimode
{
static constexpr int   kModeCount     = 5;
static constexpr int   kSelectableModeCount = 5;
static constexpr int   kCvOutCount    = 6;
static constexpr int   kPerformanceControlCount = 7;
static constexpr int   kDptControlCount = 8;
static constexpr uint8_t kDptStatusProtocolVersion = 3;
static constexpr uint16_t kDptStatusSettingsFresh     = 1u << 0;
static constexpr uint16_t kDptStatusBearaminFresh     = 1u << 1;
static constexpr uint16_t kDptStatusRxLineHigh        = 1u << 2;
static constexpr uint16_t kDptStatusRxLowSeen         = 1u << 3;
static constexpr uint16_t kDptStatusRxTransitionSeen  = 1u << 4;
static constexpr uint16_t kDptStatusRxTransitionRecent = 1u << 5;
static constexpr uint16_t kDptStatusPacketSeen        = 1u << 6;
static constexpr float kTwoPi         = 6.28318530717958647692f;
static constexpr float kAudioAmp      = 0.26f;
static constexpr float kCvMinVolts    = -5.0f;
static constexpr float kCvMaxVolts    = 5.0f;
static constexpr float kCvGateVolts   = 5.0f;

enum class Mode : uint8_t
{
    RingsModal = 0,
    RingsSympatheticStrings,
    RingsKarplusStrong,
    StackedFmDrone,
    BearaminBridge,
};

enum class RingsAlgorithm : uint8_t
{
    None = 0,
    Modal,
    SympatheticStrings,
    KarplusStrong,
};

inline RingsAlgorithm RingsAlgorithmForMode(Mode mode)
{
    switch(mode)
    {
        case Mode::RingsModal: return RingsAlgorithm::Modal;
        case Mode::RingsSympatheticStrings:
            return RingsAlgorithm::SympatheticStrings;
        case Mode::RingsKarplusStrong: return RingsAlgorithm::KarplusStrong;
        default: return RingsAlgorithm::None;
    }
}

inline bool IsRingsMode(Mode mode)
{
    return RingsAlgorithmForMode(mode) != RingsAlgorithm::None;
}

inline const char* ModeAnnouncementPath(Mode mode)
{
    switch(mode)
    {
        case Mode::RingsModal: return "dpt-modes/modal.wav";
        case Mode::RingsSympatheticStrings:
            return "dpt-modes/bearamin_resonator.wav";
        case Mode::RingsKarplusStrong:
            return "dpt-modes/karplus_strong.wav";
        case Mode::StackedFmDrone: return "dpt-modes/bearamin_fm_drone.wav";
        case Mode::BearaminBridge: return "dpt-modes/bearamin.wav";
        default: return "dpt-modes/unknown.wav";
    }
}

inline float Clamp(float value, float lo, float hi)
{
    if(value < lo)
        return lo;
    if(value > hi)
        return hi;
    return value;
}

inline float UnityAudioBlend(float input, float generated)
{
    return Clamp(input + generated, -1.0f, 1.0f);
}

inline float Abs(float value)
{
    return value < 0.0f ? -value : value;
}

inline float NormalizeBipolar(float value)
{
    return Clamp((value + 1.0f) * 0.5f, 0.0f, 1.0f);
}

inline float ControlToUnit(float bipolar_cv)
{
    return NormalizeBipolar(bipolar_cv);
}

inline float PanelControlToUnit(float panel_control)
{
    return Clamp(panel_control, 0.0f, 1.0f);
}

inline float Magnitude(float value)
{
    return Clamp(Abs(value), 0.0f, 1.0f);
}

inline int ModeIndexFromRaw(float raw_0_to_1)
{
    const float clamped = Clamp(raw_0_to_1, 0.0f, 0.999999f);
    return static_cast<int>(clamped * static_cast<float>(kModeCount));
}

inline int ModeIndexFromControl(float bipolar_control)
{
    const float panel = Clamp((NormalizeBipolar(bipolar_control) - 0.5f) * 2.0f,
                              0.0f,
                              0.999999f);
    return static_cast<int>(panel * static_cast<float>(kSelectableModeCount));
}

inline Mode SelectModeFromRaw(float raw_0_to_1)
{
    return static_cast<Mode>(ModeIndexFromRaw(raw_0_to_1));
}

inline Mode SelectModeFromControl(float bipolar_control)
{
    return static_cast<Mode>(ModeIndexFromControl(bipolar_control));
}

inline uint8_t QuantizeDptControl(float unipolar_control)
{
    const float clamped = Clamp(unipolar_control, 0.0f, 1.0f);
    return static_cast<uint8_t>(Clamp(clamped * 255.0f + 0.5f,
                                      0.0f,
                                      255.0f));
}

inline uint8_t QuantizeDptPanelPosition(float panel_control)
{
    return QuantizeDptControl(panel_control);
}

inline char HexNibble(uint8_t value)
{
    value &= 0x0Fu;
    return value < 10 ? static_cast<char>('0' + value)
                      : static_cast<char>('A' + (value - 10));
}

inline void WriteHexByte(char* out, uint8_t value)
{
    out[0] = HexNibble(static_cast<uint8_t>(value >> 4));
    out[1] = HexNibble(value);
}

inline void WriteHexU16(char* out, uint16_t value)
{
    WriteHexByte(out, static_cast<uint8_t>(value >> 8));
    WriteHexByte(out + 2, static_cast<uint8_t>(value & 0xFFu));
}

inline void WriteHexU32(char* out, uint32_t value)
{
    WriteHexU16(out, static_cast<uint16_t>(value >> 16));
    WriteHexU16(out + 4, static_cast<uint16_t>(value & 0xFFFFu));
}

struct DptPerformanceSnapshot
{
    uint32_t callback_count;
    uint32_t max_callback_us;
    uint32_t deadline_misses;
    uint32_t late_starts;
    uint32_t missed_periods;
    uint32_t max_start_gap_us;
    uint32_t tx_call_count;
    uint32_t max_irq_off_us;
    uint32_t max_tx_call_us;
};

struct BearaminTrajectoryDiagnostics
{
    uint8_t  protocol_version;
    uint16_t slew_tau_ms;
    uint32_t source_epoch_ms;
    uint16_t source_age_ms;
    uint32_t v10_packet_count;
    uint32_t v11_packet_count;
    uint32_t correction_count;
    uint32_t max_correction_norm;
    uint32_t invalid_keyframe_count;
    uint32_t stale_transitions;
    uint32_t deadline_misses;
};

inline size_t EncodeBearaminTrajectoryDiagnosticsLine(
    const BearaminTrajectoryDiagnostics& diagnostics,
    char*                                out,
    size_t                               capacity)
{
    // DPTD,1,PP,TTTT,EEEEEEEE,AAAA,CCCCCCCC,CCCCCCCC,CCCCCCCC,
    //        CCCCCCCC,CCCCCCCC,CCCCCCCC,CCCCCCCC\n
    static constexpr size_t kFieldCount = 7;
    static constexpr size_t kLineLength = 7 + 2 + 1 + 4 + 1 + 8 + 1 + 4
                                          + kFieldCount * 9 + 1;
    if(out == nullptr || capacity <= kLineLength)
        return 0;
    size_t index = 0;
    out[index++] = 'D';
    out[index++] = 'P';
    out[index++] = 'T';
    out[index++] = 'D';
    out[index++] = ',';
    out[index++] = '1';
    out[index++] = ',';
    WriteHexByte(out + index, diagnostics.protocol_version);
    index += 2;
    out[index++] = ',';
    WriteHexU16(out + index, diagnostics.slew_tau_ms);
    index += 4;
    out[index++] = ',';
    WriteHexU32(out + index, diagnostics.source_epoch_ms);
    index += 8;
    out[index++] = ',';
    WriteHexU16(out + index, diagnostics.source_age_ms);
    index += 4;
    const uint32_t fields[kFieldCount] = {
        diagnostics.v10_packet_count,
        diagnostics.v11_packet_count,
        diagnostics.correction_count,
        diagnostics.max_correction_norm,
        diagnostics.invalid_keyframe_count,
        diagnostics.stale_transitions,
        diagnostics.deadline_misses,
    };
    for(size_t field = 0; field < kFieldCount; ++field)
    {
        out[index++] = ',';
        WriteHexU32(out + index, fields[field]);
        index += 8;
    }
    out[index++] = '\n';
    out[index] = '\0';
    return index;
}

inline size_t EncodeDptPerformanceLine(const DptPerformanceSnapshot& snapshot,
                                       char*                         out,
                                       size_t                        capacity)
{
    static constexpr size_t kFieldCount = 9;
    static constexpr size_t kLineLength
        = 7 + kFieldCount * 8 + (kFieldCount - 1) + 1;
    if(out == nullptr || capacity <= kLineLength)
        return 0;

    size_t index = 0;
    out[index++] = 'D';
    out[index++] = 'P';
    out[index++] = 'T';
    out[index++] = 'P';
    out[index++] = ',';
    out[index++] = '1';
    out[index++] = ',';
    const uint32_t fields[kFieldCount] = {
        snapshot.callback_count,
        snapshot.max_callback_us,
        snapshot.deadline_misses,
        snapshot.late_starts,
        snapshot.missed_periods,
        snapshot.max_start_gap_us,
        snapshot.tx_call_count,
        snapshot.max_irq_off_us,
        snapshot.max_tx_call_us,
    };
    for(size_t field = 0; field < kFieldCount; field++)
    {
        WriteHexU32(out + index, fields[field]);
        index += 8;
        if(field + 1 < kFieldCount)
            out[index++] = ',';
    }
    out[index++] = '\n';
    out[index]   = '\0';
    return index;
}

inline size_t EncodeDptStatusLine(uint8_t        mode_index,
                                  const uint8_t controls[kDptControlCount],
                                  uint16_t       flags,
                                  uint8_t        resonator_controls_revision,
                                  char*          out,
                                  size_t         capacity)
{
    static constexpr size_t kStatusLineLength = 5 + 1 + 1 + 1 + 1 + 1
                                                + kDptControlCount * 2 + 1
                                                + 4 + 1 + 2 + 1;
    if(capacity <= kStatusLineLength)
        return 0;

    size_t index = 0;
    out[index++] = 'D';
    out[index++] = 'P';
    out[index++] = 'T';
    out[index++] = 'S';
    out[index++] = ',';
    out[index++] = static_cast<char>('0' + kDptStatusProtocolVersion);
    out[index++] = ',';
    WriteHexByte(out + index, mode_index);
    index += 2;
    out[index++] = ',';
    for(int i = 0; i < kDptControlCount; i++)
    {
        WriteHexByte(out + index, controls[i]);
        index += 2;
    }
    out[index++] = ',';
    WriteHexU16(out + index, flags);
    index += 4;
    out[index++] = ',';
    WriteHexByte(out + index, resonator_controls_revision);
    index += 2;
    out[index++] = '\n';
    out[index]   = '\0';
    return index;
}

inline float ControlToCarrierHz(float bipolar_cv)
{
    const float n = Magnitude(bipolar_cv);
    return 0.20f + n * n * 95.0f;
}

inline float ControlToCarrierTuneRatio(float bipolar_cv)
{
    const float n = Magnitude(bipolar_cv);
    return 1.0f + n * n * 31.0f;
}

inline float ControlToTunedCarrierHz(float base_cv, float tune_cv)
{
    return ControlToCarrierHz(base_cv) * ControlToCarrierTuneRatio(tune_cv);
}

inline float ControlToModHz(float bipolar_cv)
{
    const float n = Magnitude(bipolar_cv);
    return 0.05f + n * n * 65.0f;
}

inline float ControlToFmDepthHz(float bipolar_cv)
{
    const float n = Magnitude(bipolar_cv);
    return n * n * 360.0f;
}

inline float LfoToCv(float value)
{
    return Clamp(value, -1.0f, 1.0f) * 5.0f;
}

inline float Norm12ToVolts(uint16_t value)
{
    return Clamp(static_cast<float>(value) / 4095.0f, 0.0f, 1.0f) * 5.0f;
}

inline float Norm12ToBipolarVolts(uint16_t value)
{
    return Clamp(static_cast<float>(value) / 4095.0f * 2.0f - 1.0f,
                 -1.0f,
                 1.0f)
           * 5.0f;
}

inline float BearaminAxisToBipolarVolts(uint16_t value)
{
    const uint16_t clamped = value > 4095 ? 4095 : value;
    if(clamped <= 2048)
        return (static_cast<float>(clamped) - 2048.0f) * (5.0f / 2048.0f);
    return (static_cast<float>(clamped) - 2048.0f) * (5.0f / 2047.0f);
}

inline float BearaminCentroidToBipolarVolts(uint16_t value_milli,
                                             bool     invert)
{
    const float clamped = Clamp(static_cast<float>(value_milli), 0.0f, 1000.0f);
    const float position = invert ? 1000.0f - clamped : clamped;
    return (position - 500.0f) * 0.01f;
}

inline float PercentToVolts(uint8_t value)
{
    return Clamp(static_cast<float>(value) / 100.0f, 0.0f, 1.0f) * 5.0f;
}

inline float CountToVolts(uint8_t value, uint8_t max_value)
{
    if(max_value == 0)
        return 0.0f;
    return Clamp(static_cast<float>(value) / static_cast<float>(max_value),
                 0.0f,
                 1.0f)
           * 5.0f;
}

inline float TriangleFromPhase(float phase)
{
    const float p = phase - floorf(phase);
    if(p < 0.25f)
        return p * 4.0f;
    if(p < 0.75f)
        return 2.0f - p * 4.0f;
    return p * 4.0f - 4.0f;
}

class PhaseOscillator
{
  public:
    PhaseOscillator() : phase_(0.0f) {}
    explicit PhaseOscillator(float phase) : phase_(phase - floorf(phase)) {}

    void Reset(float phase = 0.0f) { phase_ = phase - floorf(phase); }

    float Sine(float hz, float sample_rate)
    {
        const float value = sinf(phase_ * kTwoPi);
        Advance(hz, sample_rate);
        return value;
    }

    float Triangle(float hz, float sample_rate)
    {
        const float value = TriangleFromPhase(phase_);
        Advance(hz, sample_rate);
        return value;
    }

    float Phase() const { return phase_; }

  private:
    void Advance(float hz, float sample_rate)
    {
        if(sample_rate <= 0.0f)
            return;
        phase_ += Clamp(hz, 0.0f, sample_rate * 0.45f) / sample_rate;
        phase_ -= floorf(phase_);
    }

    float phase_;
};

static constexpr uint8_t  kBearaminStart0      = 0xA5;
static constexpr uint8_t  kBearaminStart1      = 0x5A;
static constexpr uint8_t  kBearaminVersionV10 = 10;
static constexpr uint8_t  kBearaminVersionV11 = 11;
static constexpr uint8_t  kBearaminVersion     = kBearaminVersionV10;
static constexpr uint8_t  kBearaminPayloadSizeV10 = 64;
static constexpr uint8_t  kBearaminPayloadSizeV11 = 76;
static constexpr uint8_t  kBearaminPayloadSize = kBearaminPayloadSizeV10;
static constexpr size_t   kBearaminFrameSizeV10 = 2 + 1 + 1 + kBearaminPayloadSizeV10 + 2;
static constexpr size_t   kBearaminFrameSizeV11 = 2 + 1 + 1 + kBearaminPayloadSizeV11 + 2;
static constexpr size_t   kBearaminFrameSize   = kBearaminFrameSizeV10;
static constexpr size_t   kBearaminMaxFrameSize = kBearaminFrameSizeV11;
static_assert(kBearaminFrameSizeV10 == 70, "Bridge v10 size drift");
static_assert(kBearaminFrameSizeV11 == 82, "Bridge v11 size drift");
static constexpr uint8_t  kBearaminDptProbe[]  = {'D', 'P', 'T', '\n'};
static constexpr size_t   kBearaminDptProbeSize = sizeof(kBearaminDptProbe);
static constexpr uint16_t kBearaminOldValid    = 1u << 0;
static constexpr uint16_t kBearaminChaosActive = 1u << 0;
static constexpr uint16_t kBearaminMatrixValid = 1u << 1;
static constexpr uint16_t kBearaminImpactGate   = 1u << 2;
static constexpr uint16_t kBearaminModeAnnouncementsEnabled = 1u << 8;
static constexpr uint16_t kBearaminSpankGate    = 1u << 9;
static constexpr uint16_t kBearaminTofGate      = 1u << 4;
static constexpr uint16_t kBearaminStale        = 1u << 5;
static constexpr uint16_t kBearaminControlFmDroneRanger = 1u << 0;
static constexpr uint16_t kBearaminFmMappingMask        = 0x000E | 0x0070 | 0x0380;
static constexpr uint8_t  kBearaminFmXShift             = 1;
static constexpr uint8_t  kBearaminFmYShift             = 4;
static constexpr uint8_t  kBearaminFmZShift             = 7;
static constexpr uint16_t kBearaminCvXUnipolar          = 1u << 0;
static constexpr uint16_t kBearaminCvYUnipolar          = 1u << 1;
static constexpr uint16_t kBearaminCvZUnipolar          = 1u << 2;
static constexpr uint8_t  kBearaminResonatorBrightnessShift = 0;
static constexpr uint8_t  kBearaminResonatorDampingShift    = 2;
static constexpr uint8_t  kBearaminResonatorStructureShift  = 4;
static constexpr uint8_t  kBearaminResonatorPositionShift   = 6;
static constexpr uint8_t  kBearaminResonatorReverbShift     = 8;
static constexpr uint8_t  kBearaminResonatorFrequencyShift  = 10;
static constexpr uint16_t kBearaminResonatorAxisMask        = 0x0003;
static constexpr uint8_t  kBearaminResonatorMappingMagic       = 0x53;
static constexpr uint8_t  kResonatorWebControlSize             = 16;
static constexpr uint8_t  kResonatorWebDroneEnabled            = 1u << 0;
static constexpr uint8_t  kResonatorWebDroneWaveformShift      = 1;
static constexpr uint8_t  kResonatorWebDroneWaveformMask       = 0x02;
static constexpr uint8_t  kResonatorWebFrequencyQuantizationShift = 5;
static constexpr uint8_t  kResonatorWebFrequencyQuantizationMask  = 0x60;
static constexpr float    kResonatorNormalFeedbackMax          = 0.95f;
static constexpr float    kResonatorDroneMaxGain               = 0.25f;
static constexpr float    kResonatorBaseMidiNote            = 12.0f;
static constexpr float    kResonatorFrequencySpanSemitones  = 48.0f;
static constexpr float    kResonatorGestureOffsetGain       = 1.0f / 3.0f;
static constexpr float    kGestureAmountLowDeadZone         = 0.025f;
static constexpr float    kBearaminFmDroneModOffsetHz   = 100.0f;
static constexpr float    kBearaminFmDroneDepthOffsetHz = 240.0f;
static constexpr float    kBearaminFmDronePitchOffsetHz = 160.0f;
static constexpr float    kBearaminFmDroneStereoOffset  = 0.10f;

enum class ResonatorWebSource : uint8_t
{
    None = 0,
    XBipolar,
    YBipolar,
    ZBipolar,
    XUnipolar,
    YUnipolar,
    ZUnipolar,
    Cv5Wet,
    Pitch,
    Roll,
    Spank,
};

enum class ResonatorDroneWaveform : uint8_t
{
    Sine = 0,
    Saw,
};

enum class ResonatorFrequencyQuantization : uint8_t
{
    Off = 0,
    Chromatic,
    Major,
    MajorPentatonic,
};

enum ResonatorWebOffset : uint8_t
{
    ResonatorWebFlags = 0,
    ResonatorWebFeedbackBase = 1,
    ResonatorWebFeedbackSource = 2,
    ResonatorWebFeedbackDepth = 3,
    ResonatorWebDroneLevel = 4,
    ResonatorWebDroneSource = 5,
    ResonatorWebDroneDepth = 6,
    ResonatorWebReverbTime = 7,
    ResonatorWebReverbTimeSource = 8,
    ResonatorWebReverbTimeDepth = 9,
    ResonatorWebReverbDiffusion = 10,
    ResonatorWebReverbDiffusionSource = 11,
    ResonatorWebReverbDiffusionDepth = 12,
    ResonatorWebReverbLp = 13,
    ResonatorWebReverbLpSource = 14,
    ResonatorWebReverbLpDepth = 15,
};

inline void DefaultResonatorWebControls(uint8_t controls[kResonatorWebControlSize])
{
    for(uint8_t i = 0; i < kResonatorWebControlSize; i++)
        controls[i] = 0;
    controls[ResonatorWebDroneLevel]       = 20;
    controls[ResonatorWebReverbTime]       = 67;
    controls[ResonatorWebReverbDiffusion]  = 63;
    controls[ResonatorWebReverbLp]         = 60;
}

enum class BearaminFmParam : uint8_t
{
    Pitch = 0,
    Fm1Amount,
    Fm1Frequency,
    Fm2Amount,
    Fm2Frequency,
    Stereo,
    CvSlew,
};

enum class BearaminAxis : uint8_t
{
    None = 0,
    X,
    Y,
    Z,
    Pitch,
    Roll,
    Spank,
};

enum class ResonatorLane : uint8_t
{
    Brightness = 0,
    Damping,
    Structure,
    Position,
    Reverb,
    Frequency,
};

struct BearaminResonatorMapping
{
    BearaminAxis axis;
    bool         unipolar;
    float        multiplier;
};

inline uint16_t EncodeBearaminFmDroneControls(bool             enabled,
                                              BearaminFmParam x,
                                              BearaminFmParam y,
                                              BearaminFmParam z)
{
    uint16_t flags = enabled ? kBearaminControlFmDroneRanger : 0;
    flags |= (static_cast<uint16_t>(x) & 0x7u) << kBearaminFmXShift;
    flags |= (static_cast<uint16_t>(y) & 0x7u) << kBearaminFmYShift;
    flags |= (static_cast<uint16_t>(z) & 0x7u) << kBearaminFmZShift;
    return flags;
}

inline BearaminFmParam DecodeBearaminFmParam(uint16_t flags, uint8_t shift)
{
    const uint8_t raw = static_cast<uint8_t>((flags >> shift) & 0x7u);
    return raw <= static_cast<uint8_t>(BearaminFmParam::CvSlew)
               ? static_cast<BearaminFmParam>(raw)
               : BearaminFmParam::Pitch;
}

inline BearaminFmParam BearaminFmAxisParam(uint16_t flags, uint8_t shift)
{
    if((flags & kBearaminFmMappingMask) == 0)
    {
        if(shift == kBearaminFmXShift)
            return BearaminFmParam::Fm2Frequency;
        if(shift == kBearaminFmYShift)
            return BearaminFmParam::Fm1Frequency;
        return BearaminFmParam::Fm1Amount;
    }
    return DecodeBearaminFmParam(flags, shift);
}

inline uint16_t EncodeBearaminResonatorControls(BearaminAxis brightness,
                                                BearaminAxis damping,
                                                BearaminAxis structure,
                                                BearaminAxis position,
                                                BearaminAxis reverb
                                                = BearaminAxis::None,
                                                BearaminAxis frequency
                                                = BearaminAxis::None)
{
    uint16_t flags = 0;
    flags |= (static_cast<uint16_t>(brightness) & kBearaminResonatorAxisMask)
             << kBearaminResonatorBrightnessShift;
    flags |= (static_cast<uint16_t>(damping) & kBearaminResonatorAxisMask)
             << kBearaminResonatorDampingShift;
    flags |= (static_cast<uint16_t>(structure) & kBearaminResonatorAxisMask)
             << kBearaminResonatorStructureShift;
    flags |= (static_cast<uint16_t>(position) & kBearaminResonatorAxisMask)
             << kBearaminResonatorPositionShift;
    flags |= (static_cast<uint16_t>(reverb) & kBearaminResonatorAxisMask)
             << kBearaminResonatorReverbShift;
    flags |= (static_cast<uint16_t>(frequency) & kBearaminResonatorAxisMask)
             << kBearaminResonatorFrequencyShift;
    return flags;
}

inline BearaminAxis DecodeBearaminAxis(uint16_t flags, uint8_t shift)
{
    const uint8_t raw
        = static_cast<uint8_t>((flags >> shift) & kBearaminResonatorAxisMask);
    return raw <= static_cast<uint8_t>(BearaminAxis::Z)
               ? static_cast<BearaminAxis>(raw)
               : BearaminAxis::None;
}

inline BearaminAxis BearaminResonatorLaneAxis(uint16_t flags,
                                              ResonatorLane lane)
{
    switch(lane)
    {
        case ResonatorLane::Brightness:
            return DecodeBearaminAxis(flags, kBearaminResonatorBrightnessShift);
        case ResonatorLane::Damping:
            return DecodeBearaminAxis(flags, kBearaminResonatorDampingShift);
        case ResonatorLane::Structure:
            return DecodeBearaminAxis(flags, kBearaminResonatorStructureShift);
        case ResonatorLane::Position:
            return DecodeBearaminAxis(flags, kBearaminResonatorPositionShift);
        case ResonatorLane::Reverb:
            return DecodeBearaminAxis(flags, kBearaminResonatorReverbShift);
        case ResonatorLane::Frequency:
            return DecodeBearaminAxis(flags, kBearaminResonatorFrequencyShift);
        default: return BearaminAxis::None;
    }
}

inline uint8_t BearaminResonatorStrengthIndex(float multiplier)
{
    if(multiplier < -2.25f)
        return 0;
    if(multiplier < 0.0f)
        return 1;
    if(multiplier < 2.25f)
        return 2;
    return 3;
}

inline float BearaminResonatorStrengthForIndex(uint8_t index)
{
    switch(index)
    {
        case 0: return -3.0f;
        case 1: return -1.5f;
        case 3: return 3.0f;
        case 2:
        default: return 1.5f;
    }
}

inline uint8_t EncodeBearaminResonatorMapping(BearaminAxis axis,
                                              bool         unipolar,
                                              float        multiplier)
{
    if(axis == BearaminAxis::None || multiplier == 0.0f)
        return 0;
    if(axis == BearaminAxis::Pitch)
        return 25;
    if(axis == BearaminAxis::Roll)
        return 26;
    if(axis == BearaminAxis::Spank)
        return 27;
    if(axis > BearaminAxis::Z)
        return 0;
    return static_cast<uint8_t>(
        1
        + (static_cast<uint8_t>(axis) - 1 + (unipolar ? 3 : 0)) * 4
        + BearaminResonatorStrengthIndex(multiplier));
}

inline uint8_t EncodeBearaminResonatorMapping(BearaminAxis axis,
                                              float        multiplier)
{
    return EncodeBearaminResonatorMapping(axis, false, multiplier);
}

inline BearaminResonatorMapping DecodeBearaminResonatorMapping(uint8_t code)
{
    if(code == 25)
        return {BearaminAxis::Pitch, false, 1.5f};
    if(code == 26)
        return {BearaminAxis::Roll, false, 1.5f};
    if(code == 27)
        return {BearaminAxis::Spank, false, 1.5f};
    if(code == 0 || code > 24)
        return {BearaminAxis::None, false, 0.0f};
    const uint8_t zero_based = static_cast<uint8_t>(code - 1);
    const uint8_t range_axis_index = static_cast<uint8_t>(zero_based / 4);
    const uint8_t axis_index = static_cast<uint8_t>(range_axis_index % 3);
    const uint8_t strength_index = static_cast<uint8_t>(zero_based % 4);
    return {static_cast<BearaminAxis>(axis_index + 1),
            range_axis_index >= 3,
            BearaminResonatorStrengthForIndex(strength_index)};
}

inline uint32_t PackBearaminResonatorMappings(
    BearaminResonatorMapping brightness,
    BearaminResonatorMapping damping,
    BearaminResonatorMapping structure,
    BearaminResonatorMapping position,
    BearaminResonatorMapping reverb,
    BearaminResonatorMapping frequency)
{
    uint32_t packed = 0;
    const uint8_t codes[6] = {
        EncodeBearaminResonatorMapping(
            brightness.axis, brightness.unipolar, brightness.multiplier),
        EncodeBearaminResonatorMapping(
            damping.axis, damping.unipolar, damping.multiplier),
        EncodeBearaminResonatorMapping(
            structure.axis, structure.unipolar, structure.multiplier),
        EncodeBearaminResonatorMapping(
            position.axis, position.unipolar, position.multiplier),
        EncodeBearaminResonatorMapping(
            reverb.axis, reverb.unipolar, reverb.multiplier),
        EncodeBearaminResonatorMapping(
            frequency.axis, frequency.unipolar, frequency.multiplier),
    };
    for(int i = 0; i < 6; i++)
        packed |= static_cast<uint32_t>(codes[i] & 0x1Fu) << (i * 5);
    return packed;
}

struct BearaminBridgePacket
{
    uint8_t  protocol_version = kBearaminVersionV10;
    uint16_t sequence;
    uint16_t flags;
    uint16_t old_z_mm;
    uint16_t old_z_norm;
    uint16_t matrix_z_mm;
    uint16_t matrix_x_milli;
    uint16_t matrix_y_milli;
    uint16_t matrix_z_norm;
    uint16_t matrix_x_norm;
    uint16_t matrix_y_norm;
    uint16_t reserved0;
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  matrix_valid_count;
    uint8_t  matrix_blob_count;
    uint16_t age_ms;
    uint16_t control_flags;
    uint16_t impact_velocity_norm;
    uint16_t impact_age_ms;
    uint8_t  impact_flags;
    uint8_t  source_flags2;
    uint8_t  resonator_controls_revision;
    uint8_t  resonator_web_controls[kResonatorWebControlSize];
    int8_t   lane_depths[6];
    uint16_t pitch_norm;
    uint16_t roll_norm;
    uint8_t  orientation_confidence;
    uint32_t source_epoch_ms = 0;
    uint16_t trajectory_start_norm[5] = {};
    uint16_t slew_tau_ms = 43;
    float    trajectory_receipt_norm[5] = {};
    float    trajectory_decay_per_callback = 1.0f;
    bool     trajectory_prepared = false;
};

inline bool BearaminImpactGateActive(const BearaminBridgePacket& packet)
{
    return (packet.flags & kBearaminImpactGate) != 0;
}

inline float BearaminImpactVelocityUnit(const BearaminBridgePacket& packet)
{
    return Clamp(static_cast<float>(packet.impact_velocity_norm) / 4095.0f,
                 0.0f,
                 1.0f);
}

inline BearaminResonatorMapping BearaminResonatorLaneMapping(
    const BearaminBridgePacket& packet,
    ResonatorLane               lane)
{
    if(packet.reserved2 == kBearaminResonatorMappingMagic)
    {
        const uint32_t packed = static_cast<uint32_t>(packet.reserved0)
                                | (static_cast<uint32_t>(packet.reserved1)
                                   << 16)
                                | (static_cast<uint32_t>(packet.source_flags2)
                                   << 24);
        const uint8_t lane_index = static_cast<uint8_t>(lane);
        BearaminResonatorMapping mapping = DecodeBearaminResonatorMapping(
            static_cast<uint8_t>((packed >> (lane_index * 5)) & 0x1Fu));
        mapping.multiplier = static_cast<float>(packet.lane_depths[lane_index]) * 0.03f;
        return mapping;
    }
    return {BearaminAxis::None, false, 0.0f};
}

inline uint16_t ReadLe16(const uint8_t* buffer, size_t offset)
{
    return static_cast<uint16_t>(buffer[offset])
           | (static_cast<uint16_t>(buffer[offset + 1]) << 8);
}

inline void WriteLe16(uint8_t* buffer, size_t offset, uint16_t value)
{
    buffer[offset]     = static_cast<uint8_t>(value & 0xFF);
    buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void WriteLe32(uint8_t* buffer, size_t offset, uint32_t value)
{
    WriteLe16(buffer, offset, static_cast<uint16_t>(value & 0xFFFFu));
    WriteLe16(buffer, offset + 2, static_cast<uint16_t>(value >> 16));
}

inline uint32_t ReadLe32(const uint8_t* buffer, size_t offset)
{
    return static_cast<uint32_t>(ReadLe16(buffer, offset))
           | (static_cast<uint32_t>(ReadLe16(buffer, offset + 2)) << 16);
}

inline uint16_t Crc16Ccitt(const uint8_t* data, size_t size)
{
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < size; i++)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for(uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

inline void EncodeBearaminPayload(const BearaminBridgePacket& packet,
                                  uint8_t*                    payload)
{
    WriteLe16(payload, 0, packet.sequence);
    WriteLe16(payload, 2, packet.flags);
    WriteLe16(payload, 4, packet.old_z_mm);
    WriteLe16(payload, 6, packet.old_z_norm);
    WriteLe16(payload, 8, packet.matrix_z_mm);
    WriteLe16(payload, 10, packet.matrix_x_milli);
    WriteLe16(payload, 12, packet.matrix_y_milli);
    WriteLe16(payload, 14, packet.matrix_z_norm);
    WriteLe16(payload, 16, packet.matrix_x_norm);
    WriteLe16(payload, 18, packet.matrix_y_norm);
    WriteLe16(payload, 20, packet.reserved0);
    payload[22] = packet.reserved1;
    payload[23] = packet.reserved2;
    payload[24] = packet.matrix_valid_count;
    payload[25] = packet.matrix_blob_count;
    WriteLe16(payload, 26, packet.age_ms);
    WriteLe16(payload, 28, packet.control_flags);
    WriteLe16(payload, 30, packet.impact_velocity_norm);
    WriteLe16(payload, 32, packet.impact_age_ms);
    payload[34] = packet.impact_flags;
    payload[35] = packet.source_flags2;
    payload[36] = packet.resonator_controls_revision;
    for(uint8_t i = 0; i < kResonatorWebControlSize; i++)
        payload[37 + i] = packet.resonator_web_controls[i];
    for(uint8_t i = 0; i < 6; i++)
        payload[53 + i] = static_cast<uint8_t>(packet.lane_depths[i]);
    WriteLe16(payload, 59, packet.pitch_norm);
    WriteLe16(payload, 61, packet.roll_norm);
    payload[63] = packet.orientation_confidence;
}

inline BearaminBridgePacket DecodeBearaminPayload(const uint8_t* payload)
{
    BearaminBridgePacket packet = {};
    packet.pitch_norm = 2048;
    packet.roll_norm = 2048;
    packet.sequence            = ReadLe16(payload, 0);
    packet.flags               = ReadLe16(payload, 2);
    packet.old_z_mm            = ReadLe16(payload, 4);
    packet.old_z_norm          = ReadLe16(payload, 6);
    packet.matrix_z_mm         = ReadLe16(payload, 8);
    packet.matrix_x_milli      = ReadLe16(payload, 10);
    packet.matrix_y_milli      = ReadLe16(payload, 12);
    packet.matrix_z_norm       = ReadLe16(payload, 14);
    packet.matrix_x_norm       = ReadLe16(payload, 16);
    packet.matrix_y_norm       = ReadLe16(payload, 18);
    packet.reserved0           = ReadLe16(payload, 20);
    packet.reserved1           = payload[22];
    packet.reserved2           = payload[23];
    packet.matrix_valid_count  = payload[24];
    packet.matrix_blob_count   = payload[25];
    packet.age_ms              = ReadLe16(payload, 26);
    packet.control_flags       = ReadLe16(payload, 28);
    packet.impact_velocity_norm = ReadLe16(payload, 30);
    packet.impact_age_ms        = ReadLe16(payload, 32);
    packet.impact_flags         = payload[34];
    packet.source_flags2        = payload[35];
    packet.resonator_controls_revision = payload[36];
    for(uint8_t i = 0; i < kResonatorWebControlSize; i++)
        packet.resonator_web_controls[i] = payload[37 + i];
    for(uint8_t i = 0; i < 6; i++)
        packet.lane_depths[i] = static_cast<int8_t>(payload[53 + i]);
    packet.pitch_norm = ReadLe16(payload, 59);
    packet.roll_norm = ReadLe16(payload, 61);
    packet.orientation_confidence = payload[63];
    packet.protocol_version = kBearaminVersionV10;
    return packet;
}

inline bool ValidBearaminTrajectoryPacket(const BearaminBridgePacket& packet)
{
    if(packet.slew_tau_ms < 43 || packet.slew_tau_ms > 420)
        return false;
    const uint16_t targets[5] = {
        packet.matrix_x_norm,
        packet.matrix_y_norm,
        packet.matrix_z_norm,
        packet.pitch_norm,
        packet.roll_norm,
    };
    for(uint8_t axis = 0; axis < 5; ++axis)
    {
        if(packet.trajectory_start_norm[axis] > 4095 || targets[axis] > 4095)
            return false;
    }
    return true;
}

inline size_t EncodeBearaminFrame(const BearaminBridgePacket& packet,
                                  uint8_t*                    frame,
                                  size_t                      capacity)
{
    if(capacity < kBearaminFrameSize)
        return 0;

    frame[0] = kBearaminStart0;
    frame[1] = kBearaminStart1;
    frame[2] = kBearaminVersion;
    frame[3] = kBearaminPayloadSize;
    EncodeBearaminPayload(packet, frame + 4);
    const uint16_t crc = Crc16Ccitt(frame, 4 + kBearaminPayloadSize);
    WriteLe16(frame, 4 + kBearaminPayloadSize, crc);
    return kBearaminFrameSize;
}

inline size_t EncodeBearaminFrameV11(const BearaminBridgePacket& packet,
                                     uint8_t*                    frame,
                                     size_t                      capacity)
{
    if(capacity < kBearaminFrameSizeV11 || !ValidBearaminTrajectoryPacket(packet))
        return 0;
    frame[0] = kBearaminStart0;
    frame[1] = kBearaminStart1;
    frame[2] = kBearaminVersionV11;
    frame[3] = kBearaminPayloadSizeV11;
    EncodeBearaminPayload(packet, frame + 4);
    WriteLe32(frame + 4, 4, packet.source_epoch_ms);
    for(uint8_t axis = 0; axis < 5; ++axis)
        WriteLe16(frame + 4, 64 + axis * 2u, packet.trajectory_start_norm[axis]);
    WriteLe16(frame + 4, 74, packet.slew_tau_ms);
    const uint16_t crc = Crc16Ccitt(frame, 4 + kBearaminPayloadSizeV11);
    WriteLe16(frame, 4 + kBearaminPayloadSizeV11, crc);
    return kBearaminFrameSizeV11;
}

inline bool DecodeBearaminFrame(const uint8_t*         frame,
                                size_t                 size,
                                BearaminBridgePacket& packet)
{
    if(frame == nullptr || size < 6 || frame[0] != kBearaminStart0
       || frame[1] != kBearaminStart1)
    {
        return false;
    }
    const uint8_t version      = frame[2];
    const uint8_t payload_size = frame[3];
    const size_t expected_size = version == kBearaminVersionV10
                                 ? kBearaminFrameSizeV10
                                 : version == kBearaminVersionV11
                                       ? kBearaminFrameSizeV11
                                       : 0;
    const uint8_t expected_payload = version == kBearaminVersionV10
                                     ? kBearaminPayloadSizeV10
                                     : version == kBearaminVersionV11
                                           ? kBearaminPayloadSizeV11
                                           : 0;
    if(expected_size == 0 || size != expected_size
       || payload_size != expected_payload)
        return false;

    const uint16_t expected = ReadLe16(frame, 4 + payload_size);
    const uint16_t actual   = Crc16Ccitt(frame, 4 + payload_size);
    if(expected != actual)
        return false;

    packet = DecodeBearaminPayload(frame + 4);
    if(version == kBearaminVersionV11)
    {
        packet.protocol_version = kBearaminVersionV11;
        packet.source_epoch_ms = ReadLe32(frame + 4, 4);
        for(uint8_t axis = 0; axis < 5; ++axis)
            packet.trajectory_start_norm[axis]
                = ReadLe16(frame + 4, 64 + axis * 2u);
        packet.slew_tau_ms = ReadLe16(frame + 4, 74);
        if(!ValidBearaminTrajectoryPacket(packet))
            return false;
    }
    return true;
}

class BearaminBridgeParser
{
  public:
    BearaminBridgeParser() : index_(0), expected_size_(0) {}

    bool Push(uint8_t byte, BearaminBridgePacket& packet)
    {
        if(index_ == 0 && byte != kBearaminStart0)
            return false;
        if(index_ == 1 && byte != kBearaminStart1)
        {
            index_ = byte == kBearaminStart0 ? 1 : 0;
            return false;
        }

        buffer_[index_++] = byte;
        if(index_ == 4)
        {
            const uint8_t version      = buffer_[2];
            const uint8_t payload_size = buffer_[3];
            if(version == kBearaminVersionV10
               && payload_size == kBearaminPayloadSizeV10)
                expected_size_ = kBearaminFrameSizeV10;
            else if(version == kBearaminVersionV11
                    && payload_size == kBearaminPayloadSizeV11)
                expected_size_ = kBearaminFrameSizeV11;
            else
            {
                Reset();
                return false;
            }
        }

        if(expected_size_ == 0 || index_ < expected_size_)
            return false;

        const bool ok = DecodeBearaminFrame(buffer_, expected_size_, packet);
        Reset();
        return ok;
    }

    void Reset()
    {
        index_         = 0;
        expected_size_ = 0;
    }

  private:
    uint8_t buffer_[kBearaminMaxFrameSize];
    size_t  index_;
    size_t  expected_size_;
};

inline float Norm12ToUnit(uint16_t value)
{
    return Clamp(static_cast<float>(value) / 4095.0f, 0.0f, 1.0f);
}

inline float Norm12ToBipolarCentered(uint16_t value)
{
    return Clamp(Norm12ToUnit(value) * 2.0f - 1.0f, -1.0f, 1.0f);
}

inline float ControlToGestureAmount(float panel_control)
{
    const float value = Clamp(panel_control, 0.0f, 1.0f);
    if(value <= kGestureAmountLowDeadZone)
        return 0.0f;
    return (value - kGestureAmountLowDeadZone)
           / (1.0f - kGestureAmountLowDeadZone);
}

inline float ControlToResonatorMidiNote(float bipolar_cv)
{
    return kResonatorBaseMidiNote
           + ControlToUnit(bipolar_cv) * kResonatorFrequencySpanSemitones;
}

inline float UnitToResonatorMidiNote(float unit)
{
    return kResonatorBaseMidiNote
           + Clamp(unit, 0.0f, 1.0f) * kResonatorFrequencySpanSemitones;
}

inline float QuantizedRootMidiNoteFromPanelControl(float panel_control)
{
    const uint8_t snapshot = QuantizeDptPanelPosition(panel_control);
    const float snapshot_unit = static_cast<float>(snapshot) / 255.0f;
    return floorf(UnitToResonatorMidiNote(snapshot_unit) + 0.5f);
}

inline float QuantizeResonatorMidiNote(
    float note,
    float root_note,
    ResonatorFrequencyQuantization quantization)
{
    if(quantization == ResonatorFrequencyQuantization::Off)
        return note;

    static constexpr int8_t kChromatic[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    static constexpr int8_t kMajor[] = {0, 2, 4, 5, 7, 9, 11};
    static constexpr int8_t kMajorPentatonic[] = {0, 2, 4, 7, 9};
    const int8_t* intervals = kChromatic;
    size_t interval_count = sizeof(kChromatic) / sizeof(kChromatic[0]);
    if(quantization == ResonatorFrequencyQuantization::Major)
    {
        intervals = kMajor;
        interval_count = sizeof(kMajor) / sizeof(kMajor[0]);
    }
    else if(quantization
            == ResonatorFrequencyQuantization::MajorPentatonic)
    {
        intervals = kMajorPentatonic;
        interval_count
            = sizeof(kMajorPentatonic) / sizeof(kMajorPentatonic[0]);
    }

    float best_note = root_note;
    float best_distance = 1000.0f;
    for(int octave = -6; octave <= 6; octave++)
    {
        for(size_t i = 0; i < interval_count; i++)
        {
            const float candidate
                = root_note + static_cast<float>(octave * 12 + intervals[i]);
            const float distance = Abs(note - candidate);
            if(distance < best_distance)
            {
                best_note = candidate;
                best_distance = distance;
            }
        }
    }
    return best_note;
}

inline float ApplyUnitOffset(float base_unit, float offset_bipolar, float amount)
{
    return Clamp(base_unit
                     + offset_bipolar * Clamp(amount, 0.0f, 1.0f)
                           * kResonatorGestureOffsetGain,
                 0.0f,
                 1.0f);
}

inline float BearaminOffsetToCv(float offset_bipolar, float amount)
{
    return Clamp(offset_bipolar * Clamp(amount, 0.0f, 1.0f)
                     * kResonatorGestureOffsetGain,
                 -1.0f,
                 1.0f)
           * 5.0f;
}

inline float BearaminAxisValue(const BearaminBridgePacket& packet,
                               BearaminAxis                axis)
{
    switch(axis)
    {
        case BearaminAxis::X: return Norm12ToBipolarCentered(packet.matrix_x_norm);
        case BearaminAxis::Y: return Norm12ToBipolarCentered(packet.matrix_y_norm);
        case BearaminAxis::Z: return Norm12ToBipolarCentered(packet.matrix_z_norm);
        case BearaminAxis::Pitch: return Norm12ToBipolarCentered(packet.pitch_norm);
        case BearaminAxis::Roll: return Norm12ToBipolarCentered(packet.roll_norm);
        case BearaminAxis::Spank:
            return (packet.flags & kBearaminSpankGate) != 0 ? 1.0f : 0.0f;
        case BearaminAxis::None:
        default: return 0.0f;
    }
}

inline float BearaminMappingAxisValue(const BearaminBridgePacket& packet,
                                      BearaminAxis                axis,
                                      bool                        unipolar)
{
    if(!unipolar)
        return BearaminAxisValue(packet, axis);
    switch(axis)
    {
        case BearaminAxis::X: return Norm12ToUnit(packet.matrix_x_norm);
        case BearaminAxis::Y: return Norm12ToUnit(packet.matrix_y_norm);
        case BearaminAxis::Z: return Norm12ToUnit(packet.matrix_z_norm);
        case BearaminAxis::Pitch:
        case BearaminAxis::Roll:
        case BearaminAxis::Spank:
            return BearaminAxisValue(packet, axis);
        case BearaminAxis::None:
        default: return 0.0f;
    }
}

inline ResonatorWebSource DecodeResonatorWebSource(uint8_t value,
                                                    bool allow_cv5)
{
    if(value > static_cast<uint8_t>(ResonatorWebSource::Spank))
        return ResonatorWebSource::None;
    if(!allow_cv5 && value == static_cast<uint8_t>(ResonatorWebSource::Cv5Wet))
        return ResonatorWebSource::None;
    return static_cast<ResonatorWebSource>(value);
}

inline float ResonatorWebSourceValue(const BearaminBridgePacket& packet,
                                     ResonatorWebSource          source,
                                     float                       cv5_wet,
                                     float                       gesture_amount,
                                     bool                        bearamin_fresh)
{
    if(source == ResonatorWebSource::Cv5Wet)
        return Clamp(cv5_wet, 0.0f, 1.0f);
    if(!bearamin_fresh)
        return 0.0f;

    BearaminAxis axis = BearaminAxis::None;
    bool unipolar = false;
    switch(source)
    {
        case ResonatorWebSource::XBipolar: axis = BearaminAxis::X; break;
        case ResonatorWebSource::YBipolar: axis = BearaminAxis::Y; break;
        case ResonatorWebSource::ZBipolar: axis = BearaminAxis::Z; break;
        case ResonatorWebSource::XUnipolar:
            axis = BearaminAxis::X;
            unipolar = true;
            break;
        case ResonatorWebSource::YUnipolar:
            axis = BearaminAxis::Y;
            unipolar = true;
            break;
        case ResonatorWebSource::ZUnipolar:
            axis = BearaminAxis::Z;
            unipolar = true;
            break;
        case ResonatorWebSource::Pitch: axis = BearaminAxis::Pitch; break;
        case ResonatorWebSource::Roll: axis = BearaminAxis::Roll; break;
        case ResonatorWebSource::Spank: axis = BearaminAxis::Spank; break;
        case ResonatorWebSource::None:
        case ResonatorWebSource::Cv5Wet:
        default: return 0.0f;
    }
    return BearaminMappingAxisValue(packet, axis, unipolar)
           * Clamp(gesture_amount, 0.0f, 1.0f);
}

inline float ResolveResonatorWebPercent(const BearaminBridgePacket& packet,
                                        uint8_t base_percent,
                                        uint8_t source_value,
                                        uint8_t depth_value,
                                        bool allow_cv5,
                                        float cv5_wet,
                                        float gesture_amount,
                                        bool bearamin_fresh,
                                        float max_percent)
{
    const ResonatorWebSource source
        = DecodeResonatorWebSource(source_value, allow_cv5);
    const float depth = static_cast<float>(static_cast<int8_t>(depth_value));
    const float modulation = ResonatorWebSourceValue(
        packet, source, cv5_wet, gesture_amount, bearamin_fresh);
    return Clamp(static_cast<float>(base_percent) + modulation * depth,
                 0.0f,
                 max_percent)
           / 100.0f;
}

struct ResonatorWebParams
{
    float feedback;
    bool drone_enabled;
    ResonatorDroneWaveform drone_waveform;
    float drone_level;
    ResonatorFrequencyQuantization frequency_quantization;
    float reverb_time;
    float reverb_diffusion;
    float reverb_lp;
};

inline ResonatorWebParams MakeResonatorWebParams(
    const BearaminBridgePacket& packet,
    float cv5_wet,
    float gesture_amount,
    bool bearamin_fresh)
{
    const uint8_t* controls = packet.resonator_web_controls;
    const uint8_t flags = controls[ResonatorWebFlags];
    uint8_t waveform = static_cast<uint8_t>(
        (flags & kResonatorWebDroneWaveformMask)
        >> kResonatorWebDroneWaveformShift);
    if(waveform > static_cast<uint8_t>(ResonatorDroneWaveform::Saw))
        waveform = static_cast<uint8_t>(ResonatorDroneWaveform::Sine);

    ResonatorWebParams params;
    params.feedback = ResolveResonatorWebPercent(
        packet,
        controls[ResonatorWebFeedbackBase],
        controls[ResonatorWebFeedbackSource],
        controls[ResonatorWebFeedbackDepth],
        true,
        cv5_wet,
        gesture_amount,
        bearamin_fresh,
        95.0f);
    params.drone_enabled = (flags & kResonatorWebDroneEnabled) != 0;
    params.drone_waveform = static_cast<ResonatorDroneWaveform>(waveform);
    uint8_t frequency_quantization = static_cast<uint8_t>(
        (flags & kResonatorWebFrequencyQuantizationMask)
        >> kResonatorWebFrequencyQuantizationShift);
    if(frequency_quantization
       > static_cast<uint8_t>(
           ResonatorFrequencyQuantization::MajorPentatonic))
    {
        frequency_quantization
            = static_cast<uint8_t>(ResonatorFrequencyQuantization::Off);
    }
    params.frequency_quantization
        = static_cast<ResonatorFrequencyQuantization>(
            frequency_quantization);
    params.drone_level = ResolveResonatorWebPercent(
        packet,
        controls[ResonatorWebDroneLevel],
        controls[ResonatorWebDroneSource],
        controls[ResonatorWebDroneDepth],
        false,
        cv5_wet,
        gesture_amount,
        bearamin_fresh,
        100.0f);
    params.reverb_time = ResolveResonatorWebPercent(
        packet,
        controls[ResonatorWebReverbTime],
        controls[ResonatorWebReverbTimeSource],
        controls[ResonatorWebReverbTimeDepth],
        true,
        cv5_wet,
        gesture_amount,
        bearamin_fresh,
        100.0f);
    params.reverb_diffusion = ResolveResonatorWebPercent(
        packet,
        controls[ResonatorWebReverbDiffusion],
        controls[ResonatorWebReverbDiffusionSource],
        controls[ResonatorWebReverbDiffusionDepth],
        true,
        cv5_wet,
        gesture_amount,
        bearamin_fresh,
        100.0f);
    params.reverb_lp = ResolveResonatorWebPercent(
        packet,
        controls[ResonatorWebReverbLp],
        controls[ResonatorWebReverbLpSource],
        controls[ResonatorWebReverbLpDepth],
        true,
        cv5_wet,
        gesture_amount,
        bearamin_fresh,
        100.0f);
    return params;
}

inline float SoftLimit(float value)
{
    const float x = Clamp(value, -3.0f, 3.0f);
    const float x2 = x * x;
    return Clamp(x * (27.0f + x2) / (27.0f + 9.0f * x2), -1.0f, 1.0f);
}

struct BearaminFmDroneOffsets
{
    bool  active;
    float pitch_hz;
    float fm1_amount_hz;
    float fm1_frequency_hz;
    float fm2_amount_hz;
    float fm2_frequency_hz;
    float stereo;
    float cv_slew;
};

inline void ApplyBearaminFmAxis(BearaminFmDroneOffsets& offsets,
                                BearaminFmParam         param,
                                float                   value)
{
    switch(param)
    {
        case BearaminFmParam::Pitch:
            offsets.pitch_hz += value * kBearaminFmDronePitchOffsetHz;
            break;
        case BearaminFmParam::Fm1Amount:
            offsets.fm1_amount_hz += value * kBearaminFmDroneDepthOffsetHz;
            break;
        case BearaminFmParam::Fm1Frequency:
            offsets.fm1_frequency_hz += value * kBearaminFmDroneModOffsetHz;
            break;
        case BearaminFmParam::Fm2Amount:
            offsets.fm2_amount_hz += value * kBearaminFmDroneDepthOffsetHz;
            break;
        case BearaminFmParam::Fm2Frequency:
            offsets.fm2_frequency_hz += value * kBearaminFmDroneModOffsetHz;
            break;
        case BearaminFmParam::Stereo:
            offsets.stereo += value * kBearaminFmDroneStereoOffset;
            break;
        case BearaminFmParam::CvSlew: offsets.cv_slew += value; break;
    }
}

inline BearaminFmDroneOffsets BearaminFmDroneRangerOffsets(
    const BearaminBridgePacket& packet,
    bool                        fresh,
    float                       gesture_amount = 1.0f)
{
    BearaminFmDroneOffsets offsets
        = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const bool enabled = (packet.control_flags & kBearaminControlFmDroneRanger)
                         != 0;
    const bool matrix_valid = (packet.flags & kBearaminMatrixValid) != 0;
    const bool stale        = (packet.flags & kBearaminStale) != 0;
    if(!fresh || !enabled || !matrix_valid || stale)
        return offsets;

    offsets.active = true;
    const float amount = Clamp(gesture_amount, 0.0f, 1.0f);
    ApplyBearaminFmAxis(offsets,
                        BearaminFmAxisParam(packet.control_flags,
                                            kBearaminFmXShift),
                        Norm12ToBipolarCentered(packet.matrix_x_norm) * amount);
    ApplyBearaminFmAxis(offsets,
                        BearaminFmAxisParam(packet.control_flags,
                                            kBearaminFmYShift),
                        Norm12ToBipolarCentered(packet.matrix_y_norm) * amount);
    ApplyBearaminFmAxis(offsets,
                        BearaminFmAxisParam(packet.control_flags,
                                            kBearaminFmZShift),
                        Norm12ToBipolarCentered(packet.matrix_z_norm) * amount);
    return offsets;
}

inline float Norm12ToUnipolar10Volts(uint16_t value)
{
    return BearaminAxisToBipolarVolts(value) + 5.0f;
}

inline float Dac7554EquivalentVolts(float logical_volts)
{
    return Clamp(logical_volts + 2.0f, -7.0f, 7.0f);
}

inline float BearaminXyzDacVolts(float logical_volts)
{
    return Clamp(logical_volts, -5.0f, 5.0f);
}

inline void BearaminPacketToCv(const BearaminBridgePacket& packet,
                               bool                        fresh,
                               float                       cv[kCvOutCount])
{
    const bool stale = (packet.flags & kBearaminStale) != 0;
    if(!fresh || stale)
    {
        for(size_t i = 0; i < kCvOutCount; i++)
            cv[i] = 0.0f;
        return;
    }

    cv[0] = Norm12ToUnipolar10Volts(packet.matrix_x_norm);
    cv[1] = Norm12ToUnipolar10Volts(packet.matrix_y_norm);
    cv[2] = Norm12ToUnipolar10Volts(packet.matrix_z_norm);
    cv[3] = BearaminAxisToBipolarVolts(packet.matrix_x_norm);
    cv[4] = BearaminAxisToBipolarVolts(packet.matrix_y_norm);
    cv[5] = BearaminAxisToBipolarVolts(packet.matrix_z_norm);
}

class BearaminCvRouter
{
  public:
    BearaminCvRouter()
    : x_(0.0f), y_(0.0f), z_(0.0f), last_relax_ms_(0), relax_clock_set_(false)
    {
    }

    void Route(Mode                        mode,
               const BearaminBridgePacket& packet,
               bool                        fresh,
               uint32_t                    now_ms,
               const float                 requested[kCvOutCount],
               float                       routed[kCvOutCount])
    {
        for(size_t i = 0; i < kCvOutCount; i++)
            routed[i] = requested[i];

        if(static_cast<uint8_t>(mode) >= kSelectableModeCount)
            return;

        const bool stale = (packet.flags & kBearaminStale) != 0;
        const bool matrix_valid = (packet.flags & kBearaminMatrixValid) != 0;
        const bool chaos_active = (packet.flags & kBearaminChaosActive) != 0;
        const bool playable_blob
            = fresh && !stale
              && (chaos_active || (matrix_valid && packet.matrix_blob_count > 0));
        const bool presence_hold
            = fresh && !stale && matrix_valid && packet.matrix_blob_count == 0;

        if(playable_blob)
        {
            x_ = BearaminAxisToBipolarVolts(packet.matrix_x_norm);
            y_ = BearaminAxisToBipolarVolts(packet.matrix_y_norm);
            z_ = BearaminAxisToBipolarVolts(packet.matrix_z_norm);
            ResetRelaxClock(now_ms);
        }
        else if(presence_hold)
        {
            ResetRelaxClock(now_ms);
        }
        else if(fresh && !stale)
        {
            x_ = BearaminCentroidToBipolarVolts(packet.matrix_x_milli, false);
            y_ = BearaminCentroidToBipolarVolts(packet.matrix_y_milli, true);
            RelaxAtCadence(now_ms, false);
        }
        else
        {
            RelaxAtCadence(now_ms, true);
        }

        routed[3] = x_;
        routed[4] = y_;
        routed[5] = z_;
        if(mode == Mode::BearaminBridge)
        {
            routed[0] = Clamp(x_ + 5.0f, 0.0f, 10.0f);
            routed[1] = Clamp(y_ + 5.0f, 0.0f, 10.0f);
            routed[2] = Clamp(z_ + 5.0f, 0.0f, 10.0f);
        }
    }

  private:
    static constexpr uint32_t kRelaxCadenceMs = 67;
    static constexpr float    kMinimumRelaxStepVolts = 0.01f;

    static float RelaxOneStep(float value)
    {
        const float magnitude = Abs(value);
        if(magnitude <= kMinimumRelaxStepVolts)
            return 0.0f;
        const float step
            = magnitude / 16.0f > kMinimumRelaxStepVolts
                  ? magnitude / 16.0f
                  : kMinimumRelaxStepVolts;
        return value > 0.0f ? value - step : value + step;
    }

    void ResetRelaxClock(uint32_t now_ms)
    {
        last_relax_ms_ = now_ms;
        relax_clock_set_ = true;
    }

    void RelaxAtCadence(uint32_t now_ms, bool all_axes)
    {
        if(!relax_clock_set_)
        {
            ResetRelaxClock(now_ms);
            return;
        }
        const uint32_t steps = (now_ms - last_relax_ms_) / kRelaxCadenceMs;
        if(steps == 0)
            return;
        last_relax_ms_ += steps * kRelaxCadenceMs;
        const uint32_t applied_steps = steps > 128 ? 128 : steps;
        for(uint32_t i = 0; i < applied_steps; i++)
        {
            if(all_axes)
            {
                x_ = RelaxOneStep(x_);
                y_ = RelaxOneStep(y_);
            }
            z_ = RelaxOneStep(z_);
        }
    }

    float    x_;
    float    y_;
    float    z_;
    uint32_t last_relax_ms_;
    bool     relax_clock_set_;
};

inline uint16_t BearaminDiagnosticStatusFlags(bool settings_fresh,
                                               bool bearamin_fresh,
                                               bool rx_line_high,
                                               bool rx_low_seen,
                                               bool rx_transition_seen,
                                               bool rx_transition_recent,
                                               bool packet_seen)
{
    uint16_t flags = 0;
    if(settings_fresh)
        flags |= kDptStatusSettingsFresh;
    if(bearamin_fresh)
        flags |= kDptStatusBearaminFresh;
    if(rx_line_high)
        flags |= kDptStatusRxLineHigh;
    if(rx_low_seen)
        flags |= kDptStatusRxLowSeen;
    if(rx_transition_seen)
        flags |= kDptStatusRxTransitionSeen;
    if(rx_transition_recent)
        flags |= kDptStatusRxTransitionRecent;
    if(packet_seen)
        flags |= kDptStatusPacketSeen;
    return flags;
}

inline void BearaminBridgeOutputsToJacks(const float source[kCvOutCount],
                                         float       cv[kCvOutCount])
{
    for(size_t i = 0; i < kCvOutCount; i++)
        cv[i] = source[i];
}

inline void BearaminPacketToFmDronePatchCv(const BearaminBridgePacket& packet,
                                           bool                        fresh,
                                           float cv[kCvOutCount])
{
    const bool matrix_valid = (packet.flags & kBearaminMatrixValid) != 0;
    const bool stale        = (packet.flags & kBearaminStale) != 0;
    if(!fresh || !matrix_valid || stale)
    {
        cv[3] = 0.0f;
        cv[4] = 0.0f;
        cv[5] = 0.0f;
        return;
    }

    cv[3] = Norm12ToBipolarVolts(packet.matrix_x_norm);
    cv[4] = Norm12ToBipolarVolts(packet.matrix_y_norm);
    cv[5] = Norm12ToBipolarVolts(packet.matrix_z_norm);
}

inline void BearaminNoPacketDiagnosticToCv(bool heartbeat,
                                           bool rx_line_high,
                                           bool rx_low_seen,
                                           bool rx_transition_seen,
                                           bool rx_transition_recent,
                                           bool packet_seen,
                                           float cv[kCvOutCount])
{
    cv[0] = heartbeat ? 5.0f : 0.0f;
    cv[1] = rx_line_high ? 5.0f : 0.0f;
    cv[2] = rx_low_seen ? 5.0f : 0.0f;
    cv[3] = rx_transition_seen ? 5.0f : 0.0f;
    cv[4] = rx_transition_recent ? 5.0f : 0.0f;
    cv[5] = packet_seen ? 5.0f : 0.0f;
}

} // namespace dpt_multimode
