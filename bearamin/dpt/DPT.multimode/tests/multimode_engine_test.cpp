#include "../MultimodeEngine.h"
#include "../BearaminPredictor.h"
#include "../PresetProtocol.h"
#include "../RealtimeSnapshot.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace mm = dpt_multimode;

void AssertClose(float actual, float expected, float tolerance)
{
    if(fabsf(actual - expected) > tolerance)
    {
        printf("expected %.6f, got %.6f\n", expected, actual);
        assert(false);
    }
}

void TestModeSelection()
{
    assert(mm::ModeIndexFromRaw(-1.0f) == 0);
    assert(mm::ModeIndexFromRaw(0.0f) == 0);
    assert(mm::ModeIndexFromRaw(0.199f) == 0);
    assert(mm::ModeIndexFromRaw(0.20f) == 1);
    assert(mm::ModeIndexFromRaw(0.50f) == 2);
    assert(mm::ModeIndexFromRaw(0.999f) == 4);
    assert(mm::ModeIndexFromRaw(1.0f) == 4);

    assert(mm::ModeIndexFromControl(-1.0f) == 0);
    assert(mm::ModeIndexFromControl(-0.1f) == 0);
    assert(mm::ModeIndexFromControl(0.0f) == 0);
    assert(mm::ModeIndexFromControl(0.19f) == 0);
    assert(mm::ModeIndexFromControl(0.21f) == 1);
    assert(mm::ModeIndexFromControl(0.41f) == 2);
    assert(mm::ModeIndexFromControl(0.61f) == 3);
    assert(mm::ModeIndexFromControl(0.81f) == 4);
    assert(mm::ModeIndexFromControl(1.0f) == 4);

    for(int index = 0; index < mm::kSelectableModeCount; ++index)
    {
        const float midpoint
            = (static_cast<float>(index) + 0.5f)
              / static_cast<float>(mm::kSelectableModeCount);
        assert(static_cast<int>(mm::SelectModeFromControl(midpoint)) == index);
    }

    assert(static_cast<int>(mm::Mode::RingsModal) == 0);
    assert(static_cast<int>(mm::Mode::RingsSympatheticStrings) == 1);
    assert(static_cast<int>(mm::Mode::RingsKarplusStrong) == 2);
    assert(static_cast<int>(mm::Mode::StackedFmDrone) == 3);
    assert(static_cast<int>(mm::Mode::BearaminBridge) == 4);
    assert(mm::RingsAlgorithmForMode(mm::Mode::RingsModal)
           == mm::RingsAlgorithm::Modal);
    assert(mm::RingsAlgorithmForMode(mm::Mode::RingsSympatheticStrings)
           == mm::RingsAlgorithm::SympatheticStrings);
    assert(mm::RingsAlgorithmForMode(mm::Mode::RingsKarplusStrong)
           == mm::RingsAlgorithm::KarplusStrong);
    assert(mm::RingsAlgorithmForMode(mm::Mode::StackedFmDrone)
           == mm::RingsAlgorithm::None);
}

void TestModeAnnouncementPaths()
{
    assert(strcmp(mm::ModeAnnouncementPath(mm::Mode::RingsModal),
                  "dpt-modes/modal.wav")
           == 0);
    assert(strcmp(mm::ModeAnnouncementPath(mm::Mode::RingsSympatheticStrings),
                  "dpt-modes/bearamin_resonator.wav")
           == 0);
    assert(strcmp(mm::ModeAnnouncementPath(mm::Mode::RingsKarplusStrong),
                  "dpt-modes/karplus_strong.wav")
           == 0);
    assert(strcmp(mm::ModeAnnouncementPath(mm::Mode::StackedFmDrone),
                  "dpt-modes/bearamin_fm_drone.wav")
           == 0);
    assert(strcmp(mm::ModeAnnouncementPath(mm::Mode::BearaminBridge),
                  "dpt-modes/bearamin.wav")
           == 0);
}

void TestUnityAudioBlend()
{
    AssertClose(mm::UnityAudioBlend(0.25f, 0.50f), 0.75f, 0.0001f);
    AssertClose(mm::UnityAudioBlend(-0.25f, 0.50f), 0.25f, 0.0001f);
    AssertClose(mm::UnityAudioBlend(0.75f, 0.75f), 1.0f, 0.0001f);
    AssertClose(mm::UnityAudioBlend(-0.75f, -0.75f), -1.0f, 0.0001f);
}

void TestDptStatusEncoding()
{
    assert(mm::QuantizeDptControl(-1.0f) == 0);
    assert(mm::QuantizeDptControl(0.0f) == 0);
    assert(mm::QuantizeDptControl(0.5f) == 128);
    assert(mm::QuantizeDptControl(1.0f) == 255);
    assert(mm::QuantizeDptPanelPosition(-1.0f) == 0);
    assert(mm::QuantizeDptPanelPosition(0.0f) == 0);
    assert(mm::QuantizeDptPanelPosition(0.5f) == 128);
    assert(mm::QuantizeDptPanelPosition(1.0f) == 255);

    const uint8_t controls[mm::kDptControlCount] = {
        0x00, 0x11, 0x22, 0x33, 0x80, 0xAA, 0xFE, 0xFF};
    char line[40] = {};
    const size_t length = mm::EncodeDptStatusLine(
        static_cast<uint8_t>(mm::Mode::BearaminBridge),
        controls,
        0x0003,
        0x2A,
        line,
        sizeof(line));
    assert(length == strlen("DPTS,3,04,0011223380AAFEFF,0003,2A\n"));
    assert(strcmp(line, "DPTS,3,04,0011223380AAFEFF,0003,2A\n") == 0);

    char too_small[8] = {};
    assert(mm::EncodeDptStatusLine(0, controls, 0, 0, too_small, sizeof(too_small))
           == 0);
}

void TestDptPerformanceEncoding()
{
    mm::DptPerformanceSnapshot snapshot = {};
    snapshot.callback_count   = 0x01234567;
    snapshot.max_callback_us  = 0x00000321;
    snapshot.deadline_misses  = 2;
    snapshot.late_starts      = 3;
    snapshot.missed_periods   = 4;
    snapshot.max_start_gap_us = 0x00000456;
    snapshot.tx_call_count    = 0x89ABCDEF;
    snapshot.max_irq_off_us   = 0x000000F0;
    snapshot.max_tx_call_us   = 0x00005208;
    char line[96] = {};
    const size_t length
        = mm::EncodeDptPerformanceLine(snapshot, line, sizeof(line));
    assert(length
           == strlen("DPTP,1,01234567,00000321,00000002,00000003,"
                     "00000004,00000456,89ABCDEF,000000F0,00005208\n"));
    assert(strcmp(line,
                  "DPTP,1,01234567,00000321,00000002,00000003,"
                  "00000004,00000456,89ABCDEF,000000F0,00005208\n")
           == 0);

    char too_small[88] = {};
    assert(mm::EncodeDptPerformanceLine(
               snapshot, too_small, sizeof(too_small))
           == 0);
}

void TestBearaminTrajectoryDiagnosticsEncoding()
{
    const mm::BearaminTrajectoryDiagnostics diagnostics = {
        mm::kBearaminVersionV11,
        232,
        0x11223344,
        17,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
    };
    char line[96] = {};
    const size_t length = mm::EncodeBearaminTrajectoryDiagnosticsLine(
        diagnostics, line, sizeof(line));
    assert(length == 92);
    assert(strcmp(line,
                  "DPTD,1,0B,00E8,11223344,0011,00000001,00000002,"
                  "00000003,00000004,00000005,00000006,00000007\n")
           == 0);
    assert(mm::EncodeBearaminTrajectoryDiagnosticsLine(
               diagnostics, line, 92)
           == 0);
}

mm::BearaminBridgePacket PredictorPacket(uint16_t value, uint16_t age_ms = 0)
{
    mm::BearaminBridgePacket packet = {};
    packet.flags = mm::kBearaminMatrixValid;
    packet.matrix_blob_count = 1;
    packet.matrix_x_norm = value;
    packet.matrix_y_norm = value;
    packet.matrix_z_norm = value;
    packet.pitch_norm = value;
    packet.roll_norm = value;
    packet.age_ms = age_ms;
    return packet;
}

void TestBearaminPredictor()
{
    mm::BearaminMotionPredictor predictor;
    predictor.Process(PredictorPacket(1000), 0, true);
    predictor.Process(PredictorPacket(2000), 50, true);
    const uint16_t alpha_beta_value
        = predictor.Process(PredictorPacket(2000, 25), 75, true).matrix_x_norm;
    assert(alpha_beta_value > 1850);
    assert(alpha_beta_value < 2000);

    mm::BearaminMotionPredictor repeated_source;
    repeated_source.Process(PredictorPacket(1000), 100, true);
    mm::BearaminBridgePacket repeated = PredictorPacket(2000);
    repeated.age_ms = 50;
    assert(repeated_source.Process(repeated, 150, true).matrix_x_norm
           == 1000);

    mm::BearaminBridgePacket stale = PredictorPacket(123);
    stale.flags |= mm::kBearaminStale;
    assert(predictor.Process(stale, 200, true).matrix_x_norm == 123);
    assert(predictor.Process(PredictorPacket(3000), 250, true).matrix_x_norm
           == 3000);
}

void TestBearaminV11Trajectory()
{
    mm::BearaminBridgePacket packet = PredictorPacket(4095);
    packet.protocol_version = mm::kBearaminVersionV11;
    packet.sequence = 7;
    packet.source_epoch_ms = 1000;
    packet.slew_tau_ms = 100;
    for(size_t axis = 0; axis < 5; ++axis)
        packet.trajectory_start_norm[axis] = 0;
    mm::BearaminTrajectoryReconstructor reconstructor;
    assert(reconstructor.Prepare(packet, 1.0f));
    const mm::BearaminBridgePacket first = reconstructor.Process(packet, true);
    assert(first.matrix_x_norm == 0);
    const mm::BearaminBridgePacket second = reconstructor.Process(packet, true);
    assert(second.matrix_x_norm > 0 && second.matrix_x_norm < 4095);
    packet.sequence++;
    assert(reconstructor.Prepare(packet, 1.0f));
    const mm::BearaminBridgePacket heartbeat = reconstructor.Process(packet, true);
    assert(heartbeat.matrix_x_norm > second.matrix_x_norm);
    packet.slew_tau_ms = 42;
    assert(!reconstructor.Prepare(packet, 1.0f));
    packet.slew_tau_ms = 100;
    packet.flags |= mm::kBearaminStale;
    assert(reconstructor.Process(packet, true).matrix_x_norm == 4095);

    mm::BearaminBridgePacket chaos = PredictorPacket(3072);
    chaos.flags = mm::kBearaminChaosActive;
    chaos.matrix_blob_count = 0;
    chaos.protocol_version = mm::kBearaminVersionV11;
    chaos.source_epoch_ms = 2000;
    chaos.slew_tau_ms = 43;
    for(size_t axis = 0; axis < 5; ++axis)
        chaos.trajectory_start_norm[axis] = 3072;
    mm::BearaminTrajectoryReconstructor chaos_reconstructor;
    assert(chaos_reconstructor.Prepare(chaos, 1.0f));
    assert(chaos_reconstructor.Process(chaos, true).matrix_x_norm == 3072);

    mm::BearaminBridgePacket wire = PredictorPacket(3000, 25);
    wire.protocol_version = mm::kBearaminVersionV11;
    wire.sequence = 12;
    wire.source_epoch_ms = 0x11223344;
    wire.slew_tau_ms = 232;
    for(size_t axis = 0; axis < 5; ++axis)
        wire.trajectory_start_norm[axis] = 1000 + axis * 100;
    uint8_t frame[mm::kBearaminFrameSizeV11] = {};
    assert(mm::EncodeBearaminFrameV11(wire, frame, sizeof(frame))
           == mm::kBearaminFrameSizeV11);
    mm::BearaminBridgePacket decoded = {};
    assert(mm::DecodeBearaminFrame(frame, sizeof(frame), decoded));
    assert(decoded.protocol_version == mm::kBearaminVersionV11);
    assert(decoded.source_epoch_ms == wire.source_epoch_ms);
    assert(decoded.trajectory_start_norm[4] == 1400);
    mm::BearaminBridgeParser parser;
    bool complete = false;
    for(size_t index = 0; index < sizeof(frame); ++index)
        complete = parser.Push(frame[index], decoded);
    assert(complete && decoded.protocol_version == mm::kBearaminVersionV11);
}

void TestRealtimeSnapshot()
{
    struct Snapshot
    {
        uint32_t sequence;
        uint32_t values[16];
    };

    mm::RealtimeSnapshot<Snapshot> snapshots;
    Snapshot first = {};
    first.sequence = 1;
    for(size_t i = 0; i < 16; i++)
        first.values[i] = 0x11110000u + static_cast<uint32_t>(i);
    snapshots.Publish(first);
    const Snapshot& held = snapshots.Read();
    assert(held.sequence == 1);
    assert(held.values[15] == 0x1111000Fu);

    Snapshot second = {};
    second.sequence = 2;
    for(size_t i = 0; i < 16; i++)
        second.values[i] = 0x22220000u + static_cast<uint32_t>(i);
    snapshots.Publish(second);
    assert(held.sequence == 1);
    assert(held.values[15] == 0x1111000Fu);
    assert(snapshots.Read().sequence == 2);
    assert(snapshots.Read().values[15] == 0x2222000Fu);
}

void TestPresetProtocolHelpers()
{
    assert(mm::IsValidPresetName("default"));
    assert(mm::IsValidPresetName("Lead_01-A"));
    assert(!mm::IsValidPresetName(""));
    assert(!mm::IsValidPresetName("lead.json"));
    assert(!mm::IsValidPresetName("../lead"));

    const uint8_t bytes[] = {'{', '"', 'x', '"', ':', '1', '}'};
    char          hex[32] = {};
    mm::PresetHexEncode(bytes, sizeof(bytes), hex, sizeof(hex));
    assert(strcmp(hex, "7B2278223A317D") == 0);

    uint8_t decoded[16] = {};
    size_t  decoded_size = 0;
    assert(mm::PresetHexDecode(hex, decoded, sizeof(decoded), decoded_size));
    assert(decoded_size == sizeof(bytes));
    assert(memcmp(decoded, bytes, sizeof(bytes)) == 0);
    assert(mm::PresetCrc16(bytes, sizeof(bytes))
           == mm::PresetCrc16(decoded, decoded_size));
    assert(!mm::PresetHexDecode("7B2", decoded, sizeof(decoded), decoded_size));
    assert(!mm::PresetHexDecode("7Z", decoded, sizeof(decoded), decoded_size));
}

void TestCarrierTune()
{
    AssertClose(mm::ControlToCarrierHz(0.0f), 0.20f, 0.0001f);
    AssertClose(mm::ControlToCarrierTuneRatio(0.0f), 1.0f, 0.0001f);
    AssertClose(mm::ControlToCarrierTuneRatio(1.0f), 32.0f, 0.0001f);
    AssertClose(mm::ControlToTunedCarrierHz(0.0f, 0.0f), 0.20f, 0.0001f);
    assert(mm::ControlToTunedCarrierHz(1.0f, 1.0f)
           > mm::ControlToTunedCarrierHz(0.0f, 0.0f));
}

void TestResonatorControls()
{
    AssertClose(mm::ControlToUnit(-1.0f), 0.0f, 0.0001f);
    AssertClose(mm::ControlToUnit(0.0f), 0.5f, 0.0001f);
    AssertClose(mm::ControlToUnit(1.0f), 1.0f, 0.0001f);
    AssertClose(mm::PanelControlToUnit(-1.0f), 0.0f, 0.0001f);
    AssertClose(mm::PanelControlToUnit(0.0f), 0.0f, 0.0001f);
    AssertClose(mm::PanelControlToUnit(0.5f), 0.5f, 0.0001f);
    AssertClose(mm::PanelControlToUnit(1.0f), 1.0f, 0.0001f);
    AssertClose(mm::PanelControlToUnit(2.0f), 1.0f, 0.0001f);
    AssertClose(mm::ControlToGestureAmount(-1.0f), 0.0f, 0.0001f);
    AssertClose(mm::ControlToGestureAmount(0.0f), 0.0f, 0.0001f);
    AssertClose(mm::ControlToGestureAmount(4.0f / 255.0f), 0.0f, 0.0001f);
    AssertClose(mm::ControlToGestureAmount(0.025f), 0.0f, 0.0001f);
    AssertClose(mm::ControlToGestureAmount(0.5125f), 0.5f, 0.0001f);
    AssertClose(mm::ControlToGestureAmount(1.0f), 1.0f, 0.0001f);
    AssertClose(mm::ControlToResonatorMidiNote(-1.0f), 12.0f, 0.0001f);
    AssertClose(mm::ControlToResonatorMidiNote(1.0f), 60.0f, 0.0001f);
    AssertClose(mm::UnitToResonatorMidiNote(0.0f), 12.0f, 0.0001f);
    AssertClose(mm::UnitToResonatorMidiNote(1.0f), 60.0f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.5f, 3.0f, 1.0f), 1.0f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.5f, -3.0f, 1.0f), 0.0f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(1.0f, -3.0f, 1.0f), 0.0f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(1.0f, -1.5f, 1.0f), 0.5f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.0f, 1.5f, 1.0f), 0.5f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(1.0f, -3.0f, 0.5f), 0.5f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.0f, 3.0f, 0.5f), 0.5f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.5f, 3.0f, 0.5f), 1.0f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.5f, -3.0f, 0.5f), 0.0f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.5f, 1.0f, 1.0f), 0.8333f, 0.0001f);
    AssertClose(mm::ApplyUnitOffset(0.5f, -1.0f, 1.0f), 0.1667f, 0.0001f);

    const uint16_t flags = mm::EncodeBearaminResonatorControls(
        mm::BearaminAxis::None,
        mm::BearaminAxis::Y,
        mm::BearaminAxis::X,
        mm::BearaminAxis::Z,
        mm::BearaminAxis::None,
        mm::BearaminAxis::X);
    assert(mm::BearaminResonatorLaneAxis(flags,
                                         mm::ResonatorLane::Brightness)
           == mm::BearaminAxis::None);
    assert(mm::BearaminResonatorLaneAxis(flags, mm::ResonatorLane::Damping)
           == mm::BearaminAxis::Y);
    assert(mm::BearaminResonatorLaneAxis(flags, mm::ResonatorLane::Structure)
           == mm::BearaminAxis::X);
    assert(mm::BearaminResonatorLaneAxis(flags, mm::ResonatorLane::Position)
           == mm::BearaminAxis::Z);
    assert(mm::BearaminResonatorLaneAxis(flags, mm::ResonatorLane::Reverb)
           == mm::BearaminAxis::None);
    assert(mm::BearaminResonatorLaneAxis(flags, mm::ResonatorLane::Frequency)
           == mm::BearaminAxis::X);

    mm::BearaminBridgePacket packet = {};
    packet.matrix_x_norm = 4095;
    packet.matrix_y_norm = 2048;
    packet.matrix_z_norm = 0;
    packet.pitch_norm = 3072;
    packet.roll_norm = 1024;
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::X),
                1.0f,
                0.0001f);
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::Y),
                0.0f,
                0.01f);
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::Z),
                -1.0f,
                0.0001f);
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::Pitch),
                0.5f,
                0.01f);
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::Roll),
                -0.5f,
                0.01f);
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::Spank),
                0.0f,
                0.0001f);
    packet.flags |= mm::kBearaminSpankGate;
    AssertClose(mm::BearaminAxisValue(packet, mm::BearaminAxis::Spank),
                1.0f,
                0.0001f);
    AssertClose(mm::BearaminMappingAxisValue(
                    packet, mm::BearaminAxis::X, true),
                1.0f,
                0.0001f);
    AssertClose(mm::BearaminMappingAxisValue(
                    packet, mm::BearaminAxis::Y, true),
                0.5f,
                0.01f);
    AssertClose(mm::BearaminMappingAxisValue(
                    packet, mm::BearaminAxis::Z, true),
                0.0f,
                0.0001f);
    AssertClose(
        mm::ApplyUnitOffset(
            0.0f,
            mm::BearaminMappingAxisValue(packet, mm::BearaminAxis::X, true)
                * 3.0f,
            1.0f),
        1.0f,
        0.0001f);
    AssertClose(
        mm::ApplyUnitOffset(
            0.0f,
            mm::BearaminMappingAxisValue(packet, mm::BearaminAxis::X, true)
                * -3.0f,
            1.0f),
        0.0f,
        0.0001f);
    AssertClose(
        mm::ApplyUnitOffset(
            0.5f,
            mm::BearaminMappingAxisValue(packet, mm::BearaminAxis::X, true)
                * -1.5f,
            1.0f),
        0.0f,
        0.0001f);
    AssertClose(
        mm::ApplyUnitOffset(
            0.5f,
            mm::BearaminMappingAxisValue(packet, mm::BearaminAxis::X, true)
                * 1.5f,
            1.0f),
        1.0f,
        0.0001f);

    packet.control_flags = flags;
    auto incompatible = mm::BearaminResonatorLaneMapping(
        packet, mm::ResonatorLane::Frequency);
    assert(incompatible.axis == mm::BearaminAxis::None);
    assert(!incompatible.unipolar);
    AssertClose(incompatible.multiplier, 0.0f, 0.0001f);

    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::X, false, -3.0f)
           == 1);
    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::Z, false, 3.0f)
           == 12);
    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::X, true, -3.0f)
           == 13);
    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::Z, true, 3.0f)
           == 24);
    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::Pitch, false, 1.5f) == 25);
    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::Roll, false, -3.0f) == 26);
    assert(mm::EncodeBearaminResonatorMapping(
               mm::BearaminAxis::Spank, false, 3.0f) == 27);
    assert(mm::DecodeBearaminResonatorMapping(25).axis
           == mm::BearaminAxis::Pitch);
    assert(mm::DecodeBearaminResonatorMapping(26).axis
           == mm::BearaminAxis::Roll);
    assert(mm::DecodeBearaminResonatorMapping(27).axis
           == mm::BearaminAxis::Spank);
    auto invalid_mapping = mm::DecodeBearaminResonatorMapping(31);
    assert(invalid_mapping.axis == mm::BearaminAxis::None);
    assert(!invalid_mapping.unipolar);
    AssertClose(invalid_mapping.multiplier, 0.0f, 0.0001f);

    const uint32_t packed = mm::PackBearaminResonatorMappings(
        {mm::BearaminAxis::None, false, 0.0f},
        {mm::BearaminAxis::Y, true, -3.0f},
        {mm::BearaminAxis::X, false, 3.0f},
        {mm::BearaminAxis::Z, true, -1.5f},
        {mm::BearaminAxis::Y, false, 1.5f},
        {mm::BearaminAxis::Z, true, 3.0f});
    packet.reserved0 = static_cast<uint16_t>(packed & 0xFFFFu);
    packet.reserved1 = static_cast<uint8_t>((packed >> 16) & 0xFFu);
    packet.source_flags2 = static_cast<uint8_t>((packed >> 24) & 0xFFu);
    packet.reserved2 = mm::kBearaminResonatorMappingMagic;
    packet.lane_depths[0] = 0;
    packet.lane_depths[1] = -100;
    packet.lane_depths[2] = 100;
    packet.lane_depths[3] = -50;
    packet.lane_depths[4] = 50;
    packet.lane_depths[5] = 100;

    auto damping = mm::BearaminResonatorLaneMapping(
        packet, mm::ResonatorLane::Damping);
    assert(damping.axis == mm::BearaminAxis::Y);
    assert(damping.unipolar);
    AssertClose(damping.multiplier, -3.0f, 0.0001f);
    auto structure = mm::BearaminResonatorLaneMapping(
        packet, mm::ResonatorLane::Structure);
    assert(structure.axis == mm::BearaminAxis::X);
    assert(!structure.unipolar);
    AssertClose(structure.multiplier, 3.0f, 0.0001f);
    auto position = mm::BearaminResonatorLaneMapping(
        packet, mm::ResonatorLane::Position);
    assert(position.axis == mm::BearaminAxis::Z);
    assert(position.unipolar);
    AssertClose(position.multiplier, -1.5f, 0.0001f);
    auto reverb = mm::BearaminResonatorLaneMapping(
        packet, mm::ResonatorLane::Reverb);
    assert(reverb.axis == mm::BearaminAxis::Y);
    assert(!reverb.unipolar);
    AssertClose(reverb.multiplier, 1.5f, 0.0001f);
    auto frequency = mm::BearaminResonatorLaneMapping(
        packet, mm::ResonatorLane::Frequency);
    assert(frequency.axis == mm::BearaminAxis::Z);
    assert(frequency.unipolar);
    AssertClose(frequency.multiplier, 3.0f, 0.0001f);
}

void TestNormalizedCvMappings()
{
    AssertClose(mm::Norm12ToVolts(0), 0.0f, 0.0001f);
    AssertClose(mm::Norm12ToVolts(4095), 5.0f, 0.0001f);
    AssertClose(mm::Norm12ToBipolarVolts(0), -5.0f, 0.0001f);
    AssertClose(mm::Norm12ToBipolarVolts(2048), 0.0012f, 0.001f);
    AssertClose(mm::Norm12ToBipolarVolts(4095), 5.0f, 0.0001f);
    AssertClose(mm::Norm12ToUnipolar10Volts(0), 0.0f, 0.0001f);
    AssertClose(mm::Norm12ToUnipolar10Volts(2048), 5.0f, 0.0001f);
    AssertClose(mm::Norm12ToUnipolar10Volts(4095), 10.0f, 0.0001f);
    AssertClose(mm::Dac7554EquivalentVolts(-5.0f), -3.0f, 0.0001f);
    AssertClose(mm::Dac7554EquivalentVolts(0.0f), 2.0f, 0.0001f);
    AssertClose(mm::Dac7554EquivalentVolts(5.0f), 7.0f, 0.0001f);
    AssertClose(mm::Dac7554EquivalentVolts(8.0f), 7.0f, 0.0001f);
    AssertClose(mm::BearaminXyzDacVolts(-7.0f), -5.0f, 0.0001f);
    AssertClose(mm::BearaminXyzDacVolts(-5.0f), -5.0f, 0.0001f);
    AssertClose(mm::BearaminXyzDacVolts(0.0f), 0.0f, 0.0001f);
    AssertClose(mm::BearaminXyzDacVolts(5.0f), 5.0f, 0.0001f);
    AssertClose(mm::BearaminXyzDacVolts(7.0f), 5.0f, 0.0001f);
    AssertClose(mm::BearaminOffsetToCv(0.0f, 1.0f), 0.0f, 0.0001f);
    AssertClose(mm::BearaminOffsetToCv(1.5f, 1.0f), 2.5f, 0.0001f);
    AssertClose(mm::BearaminOffsetToCv(3.0f, 1.0f), 5.0f, 0.0001f);
    AssertClose(mm::BearaminOffsetToCv(-3.0f, 1.0f), -5.0f, 0.0001f);
    AssertClose(mm::BearaminOffsetToCv(3.0f, 0.5f), 2.5f, 0.0001f);
}

void TestResonatorWebControls()
{
    mm::BearaminBridgePacket packet = {};
    mm::DefaultResonatorWebControls(packet.resonator_web_controls);
    packet.matrix_x_norm = 4095;
    packet.matrix_y_norm = 2048;
    packet.matrix_z_norm = 4095;

    mm::ResonatorWebParams params
        = mm::MakeResonatorWebParams(packet, 0.75f, 1.0f, true);
    AssertClose(params.feedback, 0.0f, 0.0001f);
    assert(!params.drone_enabled);
    assert(params.drone_waveform == mm::ResonatorDroneWaveform::Sine);
    AssertClose(params.drone_level, 0.20f, 0.0001f);
    AssertClose(params.reverb_time, 0.67f, 0.0001f);
    AssertClose(params.reverb_diffusion, 0.63f, 0.0001f);
    AssertClose(params.reverb_lp, 0.60f, 0.0001f);
    assert(params.frequency_quantization
           == mm::ResonatorFrequencyQuantization::Off);

    AssertClose(mm::QuantizedRootMidiNoteFromPanelControl(0.0f),
                12.0f,
                0.0001f);
    AssertClose(mm::QuantizedRootMidiNoteFromPanelControl(0.5f),
                36.0f,
                0.0001f);
    AssertClose(mm::QuantizeResonatorMidiNote(
                    39.4f,
                    36.0f,
                    mm::ResonatorFrequencyQuantization::Off),
                39.4f,
                0.0001f);
    AssertClose(mm::QuantizeResonatorMidiNote(
                    39.4f,
                    36.0f,
                    mm::ResonatorFrequencyQuantization::Chromatic),
                39.0f,
                0.0001f);
    AssertClose(mm::QuantizeResonatorMidiNote(
                    39.4f,
                    36.0f,
                    mm::ResonatorFrequencyQuantization::Major),
                40.0f,
                0.0001f);
    AssertClose(mm::QuantizeResonatorMidiNote(
                    41.8f,
                    36.0f,
                    mm::ResonatorFrequencyQuantization::MajorPentatonic),
                43.0f,
                0.0001f);

    packet.resonator_web_controls[mm::ResonatorWebFeedbackBase] = 90;
    packet.resonator_web_controls[mm::ResonatorWebFeedbackSource]
        = static_cast<uint8_t>(mm::ResonatorWebSource::Cv5Wet);
    packet.resonator_web_controls[mm::ResonatorWebFeedbackDepth] = 20;
    params = mm::MakeResonatorWebParams(packet, 0.75f, 1.0f, true);
    AssertClose(params.feedback, 0.95f, 0.0001f);

    packet.resonator_web_controls[mm::ResonatorWebFlags]
        = mm::kResonatorWebDroneEnabled
          | (static_cast<uint8_t>(mm::ResonatorDroneWaveform::Saw)
             << mm::kResonatorWebDroneWaveformShift)
          | (static_cast<uint8_t>(
                 mm::ResonatorFrequencyQuantization::Major)
             << mm::kResonatorWebFrequencyQuantizationShift);
    packet.resonator_web_controls[mm::ResonatorWebFeedbackBase] = 120;
    params = mm::MakeResonatorWebParams(packet, 0.75f, 1.0f, true);
    AssertClose(params.feedback, 0.95f, 0.0001f);
    assert(params.drone_enabled);
    assert(params.drone_waveform == mm::ResonatorDroneWaveform::Saw);
    assert(params.frequency_quantization
           == mm::ResonatorFrequencyQuantization::Major);
    packet.resonator_web_controls[mm::ResonatorWebDroneLevel] = 10;
    packet.resonator_web_controls[mm::ResonatorWebDroneSource]
        = static_cast<uint8_t>(mm::ResonatorWebSource::ZUnipolar);
    packet.resonator_web_controls[mm::ResonatorWebDroneDepth] = 40;
    params = mm::MakeResonatorWebParams(packet, 0.0f, 0.5f, true);
    AssertClose(params.drone_level, 0.30f, 0.0001f);
    params = mm::MakeResonatorWebParams(packet, 0.0f, 0.5f, false);
    AssertClose(params.drone_level, 0.10f, 0.0001f);

    packet.resonator_web_controls[mm::ResonatorWebDroneSource]
        = static_cast<uint8_t>(mm::ResonatorWebSource::Cv5Wet);
    params = mm::MakeResonatorWebParams(packet, 1.0f, 1.0f, true);
    AssertClose(params.drone_level, 0.10f, 0.0001f);

    assert(mm::Abs(mm::SoftLimit(0.0f)) < 0.0001f);
    assert(mm::SoftLimit(100.0f) <= 1.0f);
    assert(mm::SoftLimit(-100.0f) >= -1.0f);

    float feedback_state = 0.0f;
    for(size_t i = 0; i < 100000; ++i)
    {
        feedback_state = mm::SoftLimit(0.1f + feedback_state * 0.95f);
        assert(isfinite(feedback_state));
        assert(mm::Abs(feedback_state) <= 1.0f);
    }
}

void TestPhaseOscillator()
{
    mm::PhaseOscillator sine;
    AssertClose(sine.Sine(1.0f, 4.0f), 0.0f, 0.0001f);
    AssertClose(sine.Sine(1.0f, 4.0f), 1.0f, 0.0001f);
    AssertClose(sine.Sine(1.0f, 4.0f), 0.0f, 0.0001f);

    mm::PhaseOscillator triangle;
    AssertClose(triangle.Triangle(1.0f, 4.0f), 0.0f, 0.0001f);
    AssertClose(triangle.Triangle(1.0f, 4.0f), 1.0f, 0.0001f);
}

void TestBearaminBridgeProtocol()
{
    mm::BearaminBridgePacket packet = {};
    packet.sequence                 = 42;
    packet.flags = mm::kBearaminOldValid | mm::kBearaminMatrixValid
                   | mm::kBearaminImpactGate;
    packet.old_z_mm           = 250;
    packet.old_z_norm         = 1024;
    packet.matrix_z_mm        = 420;
    packet.matrix_x_milli     = 250;
    packet.matrix_y_milli     = 750;
    packet.matrix_z_norm      = 2048;
    packet.matrix_x_norm      = 3072;
    packet.matrix_y_norm      = 4095;
    packet.reserved0          = 0x1234;
    packet.reserved1          = 0x56;
    packet.reserved2          = 0x78;
    packet.matrix_valid_count = 32;
    packet.matrix_blob_count  = 4;
    packet.age_ms             = 17;
    packet.control_flags      = mm::EncodeBearaminFmDroneControls(
        true,
        mm::BearaminFmParam::Fm2Frequency,
        mm::BearaminFmParam::Fm1Frequency,
        mm::BearaminFmParam::Fm1Amount);
    packet.impact_velocity_norm = 3072;
    packet.impact_age_ms        = 19;
    packet.impact_flags         = 0x01;
    packet.source_flags2        = 0xA0;
    packet.resonator_controls_revision = 0x2A;
    mm::DefaultResonatorWebControls(packet.resonator_web_controls);
    packet.resonator_web_controls[mm::ResonatorWebFeedbackBase] = 88;
    packet.resonator_web_controls[mm::ResonatorWebFlags]
        = static_cast<uint8_t>(
            static_cast<uint8_t>(
                mm::ResonatorFrequencyQuantization::MajorPentatonic)
            << mm::kResonatorWebFrequencyQuantizationShift);
    packet.lane_depths[0] = -37;
    packet.lane_depths[1] = 83;
    packet.pitch_norm = 1200;
    packet.roll_norm = 3300;
    packet.orientation_confidence = 207;

    uint8_t frame[mm::kBearaminFrameSize];
    assert(mm::EncodeBearaminFrame(packet, frame, sizeof(frame))
           == mm::kBearaminFrameSize);

    mm::BearaminBridgePacket decoded;
    assert(mm::DecodeBearaminFrame(frame, sizeof(frame), decoded));
    assert(decoded.sequence == packet.sequence);
    assert(decoded.flags == packet.flags);
    assert(decoded.matrix_y_norm == 4095);
    assert(decoded.reserved0 == 0x1234);
    assert(decoded.reserved1 == 0x56);
    assert(decoded.reserved2 == 0x78);
    assert(decoded.matrix_blob_count == 4);
    assert(decoded.control_flags == packet.control_flags);
    assert(decoded.impact_velocity_norm == 3072);
    assert(decoded.impact_age_ms == 19);
    assert(decoded.impact_flags == 0x01);
    assert(decoded.source_flags2 == 0xA0);
    assert(decoded.resonator_controls_revision == 0x2A);
    assert(decoded.resonator_web_controls[mm::ResonatorWebFeedbackBase] == 88);
    assert((decoded.resonator_web_controls[mm::ResonatorWebFlags]
            & mm::kResonatorWebFrequencyQuantizationMask)
           == (static_cast<uint8_t>(
                   mm::ResonatorFrequencyQuantization::MajorPentatonic)
               << mm::kResonatorWebFrequencyQuantizationShift));
    assert(decoded.lane_depths[0] == -37);
    assert(decoded.lane_depths[1] == 83);
    assert(decoded.pitch_norm == 1200);
    assert(decoded.roll_norm == 3300);
    assert(decoded.orientation_confidence == 207);
    assert(mm::BearaminImpactGateActive(decoded));
    AssertClose(mm::BearaminImpactVelocityUnit(decoded), 0.7502f, 0.001f);

    mm::BearaminBridgeParser parser;
    mm::BearaminBridgePacket parsed;
    bool                     complete = false;
    for(size_t i = 0; i < sizeof(frame); i++)
        complete = parser.Push(frame[i], parsed);
    assert(complete);
    assert(parsed.sequence == 42);

    uint8_t incompatible_frame[mm::kBearaminFrameSize] = {};
    memcpy(incompatible_frame, frame, sizeof(frame));
    incompatible_frame[2] = 9;
    assert(!mm::DecodeBearaminFrame(
        incompatible_frame, sizeof(incompatible_frame), decoded));

    frame[8] ^= 0x01;
    assert(!mm::DecodeBearaminFrame(frame, sizeof(frame), decoded));

    assert(mm::kBearaminDptProbeSize == 4);
    assert(mm::kBearaminDptProbe[0] == 'D');
    assert(mm::kBearaminDptProbe[1] == 'P');
    assert(mm::kBearaminDptProbe[2] == 'T');
    assert(mm::kBearaminDptProbe[3] == '\n');
}

void TestBearaminFmDroneRangerOffsets()
{
    mm::BearaminBridgePacket packet = {};
    packet.flags = mm::kBearaminMatrixValid;
    packet.control_flags = mm::EncodeBearaminFmDroneControls(
        true,
        mm::BearaminFmParam::Fm2Frequency,
        mm::BearaminFmParam::Fm1Frequency,
        mm::BearaminFmParam::Fm1Amount);
    packet.matrix_x_norm = 2048;
    packet.matrix_y_norm = 2048;
    packet.matrix_z_norm = 2048;

    mm::BearaminFmDroneOffsets offsets
        = mm::BearaminFmDroneRangerOffsets(packet, true);
    assert(offsets.active);
    AssertClose(offsets.fm1_frequency_hz, 0.0f, 0.03f);
    AssertClose(offsets.fm2_frequency_hz, 0.0f, 0.03f);
    AssertClose(offsets.fm1_amount_hz, 0.0f, 0.1f);

    packet.matrix_y_norm = 4095;
    packet.matrix_x_norm = 0;
    packet.matrix_z_norm = 0;
    offsets = mm::BearaminFmDroneRangerOffsets(packet, true);
    AssertClose(offsets.fm1_frequency_hz, mm::kBearaminFmDroneModOffsetHz, 0.0001f);
    AssertClose(offsets.fm2_frequency_hz, -mm::kBearaminFmDroneModOffsetHz, 0.0001f);
    AssertClose(offsets.fm1_amount_hz, -mm::kBearaminFmDroneDepthOffsetHz, 0.0001f);

    packet.matrix_z_norm = 4095;
    offsets = mm::BearaminFmDroneRangerOffsets(packet, true);
    AssertClose(offsets.fm1_amount_hz, mm::kBearaminFmDroneDepthOffsetHz, 0.0001f);

    packet.control_flags = mm::EncodeBearaminFmDroneControls(
        true,
        mm::BearaminFmParam::Pitch,
        mm::BearaminFmParam::Stereo,
        mm::BearaminFmParam::Fm2Amount);
    packet.matrix_x_norm = 4095;
    packet.matrix_y_norm = 0;
    packet.matrix_z_norm = 4095;
    offsets = mm::BearaminFmDroneRangerOffsets(packet, true);
    AssertClose(offsets.pitch_hz, mm::kBearaminFmDronePitchOffsetHz, 0.0001f);
    AssertClose(offsets.stereo, -mm::kBearaminFmDroneStereoOffset, 0.0001f);
    AssertClose(offsets.fm2_amount_hz, mm::kBearaminFmDroneDepthOffsetHz, 0.0001f);

    packet.control_flags = 0;
    offsets = mm::BearaminFmDroneRangerOffsets(packet, true);
    assert(!offsets.active);

    packet.control_flags = mm::EncodeBearaminFmDroneControls(
        true,
        mm::BearaminFmParam::Fm2Frequency,
        mm::BearaminFmParam::Fm1Frequency,
        mm::BearaminFmParam::Fm1Amount);
    packet.flags = mm::kBearaminMatrixValid | mm::kBearaminStale;
    offsets = mm::BearaminFmDroneRangerOffsets(packet, true);
    assert(!offsets.active);

    packet.flags = mm::kBearaminMatrixValid;
    offsets = mm::BearaminFmDroneRangerOffsets(packet, false);
    assert(!offsets.active);
}

void TestBearaminBridgeCvMapping()
{
    mm::BearaminBridgePacket packet = {};
    packet.flags = mm::kBearaminOldValid;
    packet.old_z_norm         = 4095;
    packet.matrix_z_norm      = 2048;
    packet.matrix_x_norm      = 0;
    packet.matrix_y_norm      = 4095;
    packet.matrix_valid_count = 32;

    float cv[mm::kCvOutCount];
    mm::BearaminPacketToCv(packet, true, cv);
    AssertClose(cv[0], 0.0f, 0.0001f);
    AssertClose(cv[1], 10.0f, 0.0001f);
    AssertClose(cv[2], 5.0f, 0.0001f);
    AssertClose(cv[3], -5.0f, 0.0001f);
    AssertClose(cv[4], 5.0f, 0.0001f);
    AssertClose(cv[5], 0.0f, 0.0001f);

    packet.control_flags = mm::kBearaminCvXUnipolar | mm::kBearaminCvZUnipolar;
    mm::BearaminPacketToCv(packet, true, cv);
    AssertClose(cv[0], 0.0f, 0.0001f);
    AssertClose(cv[1], 10.0f, 0.0001f);
    AssertClose(cv[2], 5.0f, 0.0001f);
    AssertClose(cv[3], -5.0f, 0.0001f);
    AssertClose(cv[4], 5.0f, 0.0001f);
    AssertClose(cv[5], 0.0f, 0.0001f);

    mm::BearaminBridgeOutputsToJacks(cv, cv);
    AssertClose(cv[0], 0.0f, 0.0001f);
    AssertClose(cv[1], 10.0f, 0.0001f);
    AssertClose(cv[2], 5.0f, 0.0001f);
    AssertClose(cv[3], -5.0f, 0.0001f);
    AssertClose(cv[4], 5.0f, 0.0001f);
    AssertClose(cv[5], 0.0f, 0.0001f);

    packet.flags = mm::kBearaminOldValid | mm::kBearaminStale;
    mm::BearaminPacketToCv(packet, true, cv);
    for(int i = 0; i < mm::kCvOutCount; i++)
        AssertClose(cv[i], 0.0f, 0.0001f);

    packet.flags = mm::kBearaminOldValid;
    mm::BearaminPacketToCv(packet, false, cv);
    for(int i = 0; i < mm::kCvOutCount; i++)
        AssertClose(cv[i], 0.0f, 0.0001f);
}

void TestBearaminCvRouter()
{
    const uint16_t axis_values[] = {0, 1024, 2048, 3071, 4095};
    const float expected[] = {-5.0f, -2.5f, 0.0f, 2.4988f, 5.0f};
    for(size_t i = 0; i < sizeof(axis_values) / sizeof(axis_values[0]); i++)
        AssertClose(mm::BearaminAxisToBipolarVolts(axis_values[i]),
                    expected[i],
                    0.001f);

    const float requested[mm::kCvOutCount]
        = {0.25f, -0.50f, 0.75f, 5.0f, -5.0f, 5.0f};
    mm::BearaminBridgePacket packet = {};
    packet.flags = mm::kBearaminMatrixValid;
    packet.matrix_blob_count = 1;
    packet.matrix_x_norm = 0;
    packet.matrix_y_norm = 2048;
    packet.matrix_z_norm = 4095;

    for(int mode_index = 0; mode_index < mm::kSelectableModeCount; mode_index++)
    {
        mm::BearaminCvRouter router;
        float routed[mm::kCvOutCount];
        const mm::Mode mode = static_cast<mm::Mode>(mode_index);
        router.Route(mode, packet, true, 100, requested, routed);
        AssertClose(routed[3], -5.0f, 0.0001f);
        AssertClose(routed[4], 0.0f, 0.0001f);
        AssertClose(routed[5], 5.0f, 0.0001f);
        if(mode == mm::Mode::BearaminBridge)
        {
            AssertClose(routed[0], 0.0f, 0.0001f);
            AssertClose(routed[1], 5.0f, 0.0001f);
            AssertClose(routed[2], 10.0f, 0.0001f);
        }
        else
        {
            AssertClose(routed[0], requested[0], 0.0001f);
            AssertClose(routed[1], requested[1], 0.0001f);
            AssertClose(routed[2], requested[2], 0.0001f);
        }
    }

    mm::BearaminCvRouter hold_router;
    float routed[mm::kCvOutCount];
    hold_router.Route(mm::Mode::StackedFmDrone, packet, true, 100, requested, routed);
    packet.matrix_blob_count = 0;
    packet.matrix_x_norm = packet.matrix_y_norm = packet.matrix_z_norm = 0;
    hold_router.Route(mm::Mode::StackedFmDrone, packet, true, 500, requested, routed);
    AssertClose(routed[3], -5.0f, 0.0001f);
    AssertClose(routed[4], 0.0f, 0.0001f);
    AssertClose(routed[5], 5.0f, 0.0001f);

    packet.flags = 0;
    packet.matrix_x_milli = 100;
    packet.matrix_y_milli = 900;
    hold_router.Route(mm::Mode::StackedFmDrone, packet, true, 567, requested, routed);
    AssertClose(routed[3], -4.0f, 0.0001f);
    AssertClose(routed[4], -4.0f, 0.0001f);
    assert(routed[5] > 0.0f && routed[5] < 5.0f);
    const float first_idle_z = routed[5];
    packet.matrix_x_milli = 150;
    packet.matrix_y_milli = 850;
    hold_router.Route(mm::Mode::StackedFmDrone, packet, true, 634, requested, routed);
    AssertClose(routed[3], -3.5f, 0.0001f);
    AssertClose(routed[4], -3.5f, 0.0001f);
    assert(routed[5] < first_idle_z);

    packet.flags = mm::kBearaminStale;
    const float stale_start_x = routed[3];
    hold_router.Route(mm::Mode::StackedFmDrone, packet, false, 701, requested, routed);
    assert(routed[3] > stale_start_x);
    hold_router.Route(mm::Mode::StackedFmDrone, packet, false, 67701, requested, routed);
    AssertClose(routed[3], 0.0f, 0.0001f);
    AssertClose(routed[4], 0.0f, 0.0001f);
    AssertClose(routed[5], 0.0f, 0.0001f);

    // Lorenz owns the XYZ axes even without a physical blob. Its presence
    // must not synthesize a ToF or impact gate, but it must reach the CV DAC.
    packet.flags = mm::kBearaminChaosActive;
    packet.matrix_blob_count = 0;
    packet.matrix_x_norm = 0;
    packet.matrix_y_norm = 2048;
    packet.matrix_z_norm = 4095;
    hold_router.Route(mm::Mode::BearaminBridge, packet, true, 768, requested, routed);
    AssertClose(routed[3], -5.0f, 0.0001f);
    AssertClose(routed[4], 0.0f, 0.0001f);
    AssertClose(routed[5], 5.0f, 0.0001f);

}

void TestBearaminDiagnosticStatusFlags()
{
    const uint16_t flags = mm::BearaminDiagnosticStatusFlags(
        true, false, true, true, true, false, true);
    assert((flags & mm::kDptStatusSettingsFresh) != 0);
    assert((flags & mm::kDptStatusBearaminFresh) == 0);
    assert((flags & mm::kDptStatusRxLineHigh) != 0);
    assert((flags & mm::kDptStatusRxLowSeen) != 0);
    assert((flags & mm::kDptStatusRxTransitionSeen) != 0);
    assert((flags & mm::kDptStatusRxTransitionRecent) == 0);
    assert((flags & mm::kDptStatusPacketSeen) != 0);
}

void TestBearaminFmDronePatchCvMapping()
{
    mm::BearaminBridgePacket packet = {};
    packet.flags = mm::kBearaminMatrixValid;
    packet.control_flags = 0;
    packet.matrix_x_norm = 0;
    packet.matrix_y_norm = 2048;
    packet.matrix_z_norm = 4095;

    float cv[mm::kCvOutCount] = {1.0f, 2.0f, 3.0f, -1.0f, -1.0f, -1.0f};
    mm::BearaminPacketToFmDronePatchCv(packet, true, cv);
    AssertClose(cv[0], 1.0f, 0.0001f);
    AssertClose(cv[1], 2.0f, 0.0001f);
    AssertClose(cv[2], 3.0f, 0.0001f);
    AssertClose(cv[3], -5.0f, 0.0001f);
    AssertClose(cv[4], 0.0f, 0.01f);
    AssertClose(cv[5], 5.0f, 0.0001f);

    packet.flags = mm::kBearaminMatrixValid | mm::kBearaminStale;
    mm::BearaminPacketToFmDronePatchCv(packet, true, cv);
    AssertClose(cv[3], 0.0f, 0.0001f);
    AssertClose(cv[4], 0.0f, 0.0001f);
    AssertClose(cv[5], 0.0f, 0.0001f);

    packet.flags = mm::kBearaminMatrixValid;
    cv[3] = cv[4] = cv[5] = -1.0f;
    mm::BearaminPacketToFmDronePatchCv(packet, false, cv);
    AssertClose(cv[3], 0.0f, 0.0001f);
    AssertClose(cv[4], 0.0f, 0.0001f);
    AssertClose(cv[5], 0.0f, 0.0001f);
}

void TestBearaminNoPacketDiagnosticCvMapping()
{
    float cv[mm::kCvOutCount];
    mm::BearaminNoPacketDiagnosticToCv(
        true, true, false, false, false, false, cv);
    AssertClose(cv[0], 5.0f, 0.0001f);
    AssertClose(cv[1], 5.0f, 0.0001f);
    AssertClose(cv[2], 0.0f, 0.0001f);
    AssertClose(cv[3], 0.0f, 0.0001f);
    AssertClose(cv[4], 0.0f, 0.0001f);
    AssertClose(cv[5], 0.0f, 0.0001f);

    mm::BearaminNoPacketDiagnosticToCv(
        false, false, true, true, true, true, cv);
    AssertClose(cv[0], 0.0f, 0.0001f);
    AssertClose(cv[1], 0.0f, 0.0001f);
    AssertClose(cv[2], 5.0f, 0.0001f);
    AssertClose(cv[3], 5.0f, 0.0001f);
    AssertClose(cv[4], 5.0f, 0.0001f);
    AssertClose(cv[5], 5.0f, 0.0001f);
}

int main()
{
    TestModeSelection();
    TestModeAnnouncementPaths();
    TestUnityAudioBlend();
    TestDptStatusEncoding();
    TestDptPerformanceEncoding();
    TestBearaminTrajectoryDiagnosticsEncoding();
    TestBearaminPredictor();
    TestBearaminV11Trajectory();
    TestRealtimeSnapshot();
    TestPresetProtocolHelpers();
    TestCarrierTune();
    TestResonatorControls();
    TestResonatorWebControls();
    TestNormalizedCvMappings();
    TestPhaseOscillator();
    TestBearaminBridgeProtocol();
    TestBearaminFmDroneRangerOffsets();
    TestBearaminBridgeCvMapping();
    TestBearaminCvRouter();
    TestBearaminDiagnosticStatusFlags();
    TestBearaminFmDronePatchCvMapping();
    TestBearaminNoPacketDiagnosticCvMapping();
    printf("multimode engine tests passed\n");
    return 0;
}
