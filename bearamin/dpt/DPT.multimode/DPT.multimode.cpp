#include "daisy.h"
#include "daisy_dpt.h"
#include "MultimodeEngine.h"
#include "BearaminPredictor.h"
#include "PresetProtocol.h"
#include "RealtimeSnapshot.h"
#include "rings/dsp/fx/reverb.h"
#include "rings/dsp/part.h"
#include "rings/dsp/patch.h"
#include "rings/dsp/performance_state.h"
#include "util/scopedirqblocker.h"
#include "util/hal_map.h"
#include "util/bsp_sd_diskio.h"
#include "util/sd_diskio.h"
#include "ff_gen_drv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace daisy;
using namespace daisy::dpt;

namespace mm = dpt_multimode;

extern SD_HandleTypeDef hsd1;
extern Disk_drvTypeDef disk;

namespace
{
DPT patch;
UartHandler bearamin_uart;
uint8_t bearamin_uart_rx_dma_buffer[256];
GPIO bearamin_cv_dac_sync;
GPIO bearamin_cv_dac_clock;
GPIO bearamin_cv_dac_data;

#ifndef USART_ISR_RXNE_RXFNE
#define USART_ISR_RXNE_RXFNE USART_ISR_RXNE
#endif
#ifndef USART_ISR_TXE_TXFNF
#define USART_ISR_TXE_TXFNF USART_ISR_TXE
#endif

static constexpr size_t kAnnouncementTransferSize = 8192;
static constexpr float  kAnnouncementDuck         = 0.55f;
static constexpr float  kAnnouncementGain         = 0.70f;

struct ExpSniffPin
{
    const char* name;
    Pin         pin;
};

constexpr ExpSniffPin kBearaminSniffPins[] = {
    {"AUX_1_RX_A2", DPT::A2},
    {"AUX_2_TX_A3", DPT::A3},
    {"MIDI_RX_A9", DPT::A9},
    {"MIDI_TX_A8", DPT::A8},
    {"CS_D1", DPT::D1},
    {"SCL_B7", DPT::B7},
    {"MOSI_D9", DPT::D9},
    {"SDA_B8", DPT::B8},
    {"MISO_D8", DPT::D8},
    {"CLK_D10", DPT::D10},
};
constexpr size_t kBearaminSniffPinCount
    = sizeof(kBearaminSniffPins) / sizeof(kBearaminSniffPins[0]);

constexpr bool kBearaminReturnTxSweepDiag = false;
constexpr uint32_t kBearaminReturnTxSweepPinMs = 1000;
constexpr uint32_t kBearaminReturnTxSweepToggleMs = 10;
constexpr ExpSniffPin kBearaminReturnTxSweepPins[] = {
    {"AUX_2_TX_A3", DPT::A3},
    {"AUX_1_RX_A2", DPT::A2},
    {"MIDI_TX_A8", DPT::A8},
    {"MIDI_RX_A9", DPT::A9},
};
constexpr size_t kBearaminReturnTxSweepPinCount
    = sizeof(kBearaminReturnTxSweepPins)
      / sizeof(kBearaminReturnTxSweepPins[0]);
constexpr bool kBearaminSingleWireAckEnabled = false;

SdmmcHandler sdmmc;
FatFSInterface fsi;
WavPlayer<kAnnouncementTransferSize> mode_announcement;

mm::BearaminBridgeParser bearamin_parser;
mm::RealtimeSnapshot<mm::BearaminBridgePacket> bearamin_raw_packets;
mm::BearaminBridgePacket bearamin_audio_packet = {};
mm::BearaminMotionPredictor bearamin_predictor;
mm::BearaminTrajectoryReconstructor bearamin_trajectory_reconstructor;
constexpr float kBearaminAudioCallbackMs = 1.0f;
uint32_t bearamin_v10_packet_count = 0;
uint32_t bearamin_v11_packet_count = 0;
uint32_t bearamin_invalid_trajectory_count = 0;
uint32_t bearamin_trajectory_correction_count = 0;
uint32_t bearamin_trajectory_max_correction_norm = 0;
uint32_t bearamin_stale_transition_count = 0;
uint32_t bearamin_last_trajectory_diagnostics_ms = 0;
mm::BearaminBridgePacket bearamin_previous_v11_packet = {};
bool bearamin_previous_v11_packet_valid = false;
bool bearamin_bridge_was_fresh = false;
uint32_t bearamin_last_packet_ms = 0;
uint32_t bearamin_last_probe_ms = 0;
uint32_t bearamin_single_wire_ack_start_ms = 0;
uint32_t bearamin_rx_byte_count = 0;
uint32_t bearamin_rx_last_byte_ms = 0;
uint32_t bearamin_rx_start_byte_count = 0;
uint32_t bearamin_rx_start_pair_count = 0;
uint32_t bearamin_rx_packet_count = 0;
uint32_t bearamin_rx_dpt_echo_count = 0;
uint8_t  bearamin_rx_previous_byte = 0;
size_t   bearamin_rx_dpt_echo_index = 0;
volatile uint8_t  bearamin_rx_debug_bytes[128] = {};
volatile uint32_t bearamin_rx_debug_write_index = 0;
volatile uint32_t bearamin_rx_debug_count = 0;
volatile uint32_t bearamin_uart_error_count = 0;
volatile uint32_t bearamin_uart_overrun_count = 0;
volatile uint32_t bearamin_uart_framing_count = 0;
volatile uint32_t bearamin_uart_noise_count = 0;
volatile uint32_t bearamin_uart_parity_count = 0;
volatile uint32_t bearamin_uart_last_error = 0;
volatile uint32_t bearamin_uart_tx_byte_count = 0;
volatile uint32_t bearamin_uart_tx_error_count = 0;
volatile uint32_t bearamin_single_wire_ack_count = 0;
volatile uint32_t bearamin_return_tx_sweep_index = 0;
volatile uint32_t bearamin_return_tx_sweep_toggle_count = 0;
uint32_t bearamin_return_tx_sweep_last_pin_ms = 0;
uint32_t bearamin_return_tx_sweep_last_toggle_ms = 0;
bool bearamin_return_tx_sweep_level = false;
bool     bearamin_single_wire_ack_pending = false;
bool     bearamin_single_wire_ack_active = false;
bool     bearamin_rx_gpio_level = true;
bool     bearamin_rx_gpio_low_seen = false;
uint32_t bearamin_rx_gpio_transition_count = 0;
uint32_t bearamin_rx_gpio_last_transition_ms = 0;
uint32_t bearamin_rx_gpio_sample_count = 0;
uint8_t  bearamin_sniff_pin_levels[kBearaminSniffPinCount] = {};
uint8_t  bearamin_sniff_pin_low_seen[kBearaminSniffPinCount] = {};
uint32_t bearamin_sniff_pin_transition_count[kBearaminSniffPinCount] = {};
uint32_t bearamin_sniff_pin_last_transition_ms[kBearaminSniffPinCount] = {};
uint32_t bearamin_sniff_pin_sample_count[kBearaminSniffPinCount] = {};
volatile uint32_t cv_output_write_count = 0;
volatile uint32_t cv_expander_update_count = 0;
volatile uint32_t bearamin_bridge_fresh_cv_count = 0;
volatile float    last_written_cv[mm::kCvOutCount] = {};
volatile float    desired_expander_cv[4] = {};
volatile uint32_t audio_callback_count = 0;
volatile float    audio_output_peak    = 0.0f;
volatile uint32_t audio_callback_max_cycles = 0;
volatile uint32_t audio_callback_deadline_miss_count = 0;
volatile uint32_t audio_callback_late_start_count = 0;
volatile uint32_t audio_callback_missed_period_count = 0;
volatile uint32_t audio_callback_max_start_gap_cycles = 0;
volatile uint32_t audio_callback_last_start_cycles = 0;
volatile uint32_t bearamin_uart_tx_call_count = 0;
volatile uint32_t bearamin_uart_tx_max_irq_off_cycles = 0;
volatile uint32_t bearamin_uart_tx_max_call_cycles = 0;
uint32_t          bearamin_last_performance_ms = 0;
bool              bearamin_impact_gate_previous = false;
uint32_t cv_expander_last_update_ms = 0;
volatile int  pending_announcement_mode     = -1;
volatile bool announcement_streaming_enabled = false;
volatile bool announcement_swap_requested    = false;
bool          announcements_mounted          = false;
bool          announcement_streaming_supported = false;
bool          preset_sd_init_ok              = false;
FRESULT       preset_sd_mount_result         = FR_NOT_READY;
FRESULT       preset_sd_directory_result     = FR_OK;
bool          preset_fatfs_linked            = false;
uint32_t      preset_sd_last_mount_attempt_ms = 0;
uint8_t       preset_sd_bsp_init_result      = MSD_OK;
uint32_t      preset_sd_card_state           = 0;
uint32_t      preset_sd_hal_error            = 0;
size_t        announcement_channels          = 2;
char          bearamin_preset_line[160]      = {};
size_t        bearamin_preset_line_index     = 0;
FIL           bearamin_preset_file;
DIR           bearamin_preset_dir;
alignas(32) uint8_t bearamin_preset_buffer[mm::kPresetMaxJsonBytes] = {};
uint16_t      bearamin_preset_write_seq      = 0;
char          bearamin_preset_write_name[mm::kPresetMaxNameLength + 1] = {};
size_t        bearamin_preset_write_size     = 0;
size_t        bearamin_preset_write_received = 0;
uint16_t      bearamin_preset_write_crc      = 0;
bool          bearamin_preset_write_active   = false;
bool          bearamin_preset_write_completed = false;
uint16_t      bearamin_preset_write_completed_seq = 0;
bool          bearamin_preset_list_completed = false;
uint16_t      bearamin_preset_list_completed_seq = 0;
size_t        bearamin_preset_list_count = 0;
bool          bearamin_preset_read_active    = false;
uint16_t      bearamin_preset_read_seq       = 0;
size_t        bearamin_preset_read_size      = 0;
uint16_t      bearamin_preset_read_crc       = 0;
size_t        bearamin_preset_io_offset      = 0;
uint16_t      bearamin_preset_storage_seq    = 0;
char          bearamin_preset_temp_path[80]  = {};
char          bearamin_preset_final_path[80] = {};
uint32_t      bearamin_preset_command_ms     = 0;
bool          bearamin_preset_immediate_tx_available = false;
uint32_t      bearamin_preset_tx_drop_count  = 0;

enum class PresetStorageJob : uint8_t
{
    None,
    List,
    Read,
    WriteData,
    WriteClose,
    WriteRemoveOld,
    WriteRename,
};

PresetStorageJob bearamin_preset_storage_job = PresetStorageJob::None;

static constexpr size_t kBearaminPresetTxQueueDepth = 4;
static constexpr size_t kBearaminPresetTxLineCapacity = 112;

struct BearaminPresetTxItem
{
    char     line[kBearaminPresetTxLineCapacity];
    uint32_t packet_count_at_queue;
    uint32_t command_ms;
    bool     immediate_allowed;
};

BearaminPresetTxItem bearamin_preset_tx_queue[kBearaminPresetTxQueueDepth] = {};
size_t               bearamin_preset_tx_head = 0;
size_t               bearamin_preset_tx_tail = 0;

bool BearaminSettingsFresh(uint32_t now)
{
    return bearamin_last_packet_ms != 0 && now - bearamin_last_packet_ms <= 1000;
}

uint32_t PerformanceCycleCount()
{
    return DWT->CYCCNT;
}

uint32_t PerformanceCyclesToUs(uint32_t cycles)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    return cycles_per_us == 0 ? 0 : cycles / cycles_per_us;
}

void RecordMaximum(volatile uint32_t& maximum, uint32_t value)
{
    if(value > maximum)
        maximum = value;
}

void InitDptPerformanceTelemetry()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

mm::DptPerformanceSnapshot CaptureDptPerformanceSnapshot()
{
    mm::DptPerformanceSnapshot snapshot;
    uint32_t callback_max_cycles = 0;
    uint32_t start_gap_max_cycles = 0;
    uint32_t irq_off_max_cycles = 0;
    uint32_t tx_call_max_cycles = 0;
    {
        ScopedIrqBlocker block;
        snapshot.callback_count = audio_callback_count;
        callback_max_cycles = audio_callback_max_cycles;
        snapshot.deadline_misses = audio_callback_deadline_miss_count;
        snapshot.late_starts = audio_callback_late_start_count;
        snapshot.missed_periods = audio_callback_missed_period_count;
        start_gap_max_cycles = audio_callback_max_start_gap_cycles;
        snapshot.tx_call_count = bearamin_uart_tx_call_count;
        irq_off_max_cycles = bearamin_uart_tx_max_irq_off_cycles;
        tx_call_max_cycles = bearamin_uart_tx_max_call_cycles;
    }
    snapshot.max_callback_us = PerformanceCyclesToUs(callback_max_cycles);
    snapshot.max_start_gap_us = PerformanceCyclesToUs(start_gap_max_cycles);
    snapshot.max_irq_off_us = PerformanceCyclesToUs(irq_off_max_cycles);
    snapshot.max_tx_call_us = PerformanceCyclesToUs(tx_call_max_cycles);
    return snapshot;
}

uint16_t BearaminTrajectoryTarget(const mm::BearaminBridgePacket& packet,
                                  size_t                          axis)
{
    switch(axis)
    {
        case 0: return packet.matrix_x_norm;
        case 1: return packet.matrix_y_norm;
        case 2: return packet.matrix_z_norm;
        case 3: return packet.pitch_norm;
        default: return packet.roll_norm;
    }
}

void RecordBearaminTrajectoryCorrection(const mm::BearaminBridgePacket& packet)
{
    if(packet.protocol_version != mm::kBearaminVersionV11)
        return;

    if(bearamin_previous_v11_packet_valid
       && packet.source_epoch_ms
              != bearamin_previous_v11_packet.source_epoch_ms)
    {
        const uint32_t elapsed_ms
            = packet.source_epoch_ms
              - bearamin_previous_v11_packet.source_epoch_ms;
        if(elapsed_ms <= 1000)
        {
            const float tau
                = static_cast<float>(bearamin_previous_v11_packet.slew_tau_ms);
            const float decay = expf(-static_cast<float>(elapsed_ms) / tau);
            for(size_t axis = 0; axis < 5; ++axis)
            {
                const float target
                    = static_cast<float>(
                        BearaminTrajectoryTarget(
                            bearamin_previous_v11_packet, axis));
                const float expected
                    = target
                      + (static_cast<float>(
                             bearamin_previous_v11_packet
                                 .trajectory_start_norm[axis])
                         - target)
                            * decay;
                const float received
                    = static_cast<float>(packet.trajectory_start_norm[axis]);
                const float difference
                    = expected >= received ? expected - received
                                           : received - expected;
                const uint32_t rounded_difference
                    = static_cast<uint32_t>(difference + 0.5f);
                if(rounded_difference > 1)
                {
                    bearamin_trajectory_correction_count++;
                    if(rounded_difference
                       > bearamin_trajectory_max_correction_norm)
                    {
                        bearamin_trajectory_max_correction_norm
                            = rounded_difference;
                    }
                }
            }
        }
    }
    bearamin_previous_v11_packet = packet;
    bearamin_previous_v11_packet_valid = true;
}

const mm::BearaminBridgePacket& RawBearaminPacket()
{
    return bearamin_raw_packets.Read();
}

const mm::BearaminBridgePacket& BearaminPacket()
{
    return bearamin_audio_packet;
}

mm::PhaseOscillator drone_left(0.00f);
mm::PhaseOscillator drone_right(0.25f);
mm::PhaseOscillator drone_mod_1(0.00f);
mm::PhaseOscillator drone_mod_2(0.50f);
float               resonator_output_env = 0.0f;

bool BearaminBridgeFresh(uint32_t now, uint32_t max_age_ms = 250)
{
    return bearamin_last_packet_ms != 0 && now - bearamin_last_packet_ms <= max_age_ms
           && !(RawBearaminPacket().flags & mm::kBearaminStale);
}

bool BearaminImpactGateLevel(bool fresh)
{
    return fresh && mm::BearaminImpactGateActive(BearaminPacket());
}

bool BearaminImpactGateTrig(bool fresh)
{
    const bool level = BearaminImpactGateLevel(fresh);
    const bool trig  = level && !bearamin_impact_gate_previous;
    bearamin_impact_gate_previous = level;
    return trig;
}

const int cv_inputs[8] = {
    CV_1,
    CV_2,
    CV_3,
    CV_4,
    CV_5,
    CV_6,
    CV_7,
    CV_8,
};

static constexpr uint32_t kBearaminDptProbeSendMs = 200;
static constexpr uint32_t kBearaminPerformanceSendMs = 5000;
static constexpr uint32_t kBearaminUartBaud = 38400;
static constexpr uint32_t kBearaminSingleWireAckPulseMs = 2;
static constexpr uint32_t kBearaminRxActivityHoldMs = 250;
static constexpr uint32_t kBearaminPresetImmediateTxMs = 150;
static constexpr uint32_t kBearaminPresetPostPacketDelayMs = 5;
static constexpr uint32_t kBearaminPresetPostPacketWindowMs = 8;
static constexpr uint32_t kBearaminSdIoTimeoutMs = 100;
static constexpr size_t kBearaminPresetIoChunkBytes = 512;
static constexpr uint32_t kCvExpanderUpdateMs = 2;
static constexpr bool kBearaminUartRxDmaEnabled = false;
static constexpr size_t kResonatorBlockSize = rings::kMaxBlockSize;
static_assert(kResonatorBlockSize <= 24, "Rings block size must fit stack buffers");
static constexpr float kResonatorInputGain = 2.0f;
static constexpr float kResonatorStrumImpulse = 0.75f;

rings::Part   resonator;
rings::Reverb resonator_reverb;
uint16_t resonator_reverb_buffer[32768];
float resonator_feedback_delay[kResonatorBlockSize] = {};
float resonator_drone_phase = 0.0f;
float resonator_feedback_smoothed = 0.0f;
float resonator_drone_level_smoothed = 0.0f;
float resonator_reverb_time_smoothed = 0.67f;
float resonator_reverb_diffusion_smoothed = 0.63f;
float resonator_reverb_lp_smoothed = 0.60f;

mm::Mode current_mode = mm::Mode::StackedFmDrone;
mm::BearaminCvRouter bearamin_cv_router;

rings::ResonatorModel RingsModelForMode(mm::Mode mode)
{
    switch(mm::RingsAlgorithmForMode(mode))
    {
        case mm::RingsAlgorithm::Modal:
            return rings::RESONATOR_MODEL_MODAL;
        case mm::RingsAlgorithm::SympatheticStrings:
            return rings::RESONATOR_MODEL_SYMPATHETIC_STRING;
        case mm::RingsAlgorithm::KarplusStrong:
            return rings::RESONATOR_MODEL_STRING;
        case mm::RingsAlgorithm::None:
        default: return rings::RESONATOR_MODEL_SYMPATHETIC_STRING;
    }
}

float ResonatorMidiNoteToHz(float midi_note)
{
    return 440.0f * powf(2.0f, (midi_note - 69.0f) / 12.0f);
}

float ResonatorPolyBlep(float phase, float phase_increment)
{
    if(phase_increment <= 0.0f)
        return 0.0f;
    if(phase < phase_increment)
    {
        const float t = phase / phase_increment;
        return t + t - t * t - 1.0f;
    }
    if(phase > 1.0f - phase_increment)
    {
        const float t = (phase - 1.0f) / phase_increment;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

void InitBearaminCvDac()
{
    // The DAC7554 on DPT requires SPI mode 1 and one uninterrupted 16-bit
    // SYNC frame. The platform SPI path did not latch independent channel
    // words on this hardware, so the Bearamin CV-only writer drives the same
    // pins explicitly from the foreground loop. Audio processing and upstream
    // CV values are untouched.
    bearamin_cv_dac_sync.Init(
        DPT::D1, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::VERY_HIGH);
    bearamin_cv_dac_clock.Init(
        DPT::D10, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::VERY_HIGH);
    bearamin_cv_dac_data.Init(
        DPT::D9, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::VERY_HIGH);
    bearamin_cv_dac_sync.Write(true);
    bearamin_cv_dac_clock.Write(false);
    bearamin_cv_dac_data.Write(false);
}

void WriteBearaminCvDacWord(uint16_t word)
{
    bearamin_cv_dac_clock.Write(false);
    System::DelayUs(2);
    bearamin_cv_dac_sync.Write(false);
    System::DelayUs(2);
    for(int bit = 15; bit >= 0; bit--)
    {
        bearamin_cv_dac_data.Write((word & (1u << bit)) != 0);
        System::DelayUs(2);
        bearamin_cv_dac_clock.Write(true);
        System::DelayUs(2);
        bearamin_cv_dac_clock.Write(false);
        System::DelayUs(2);
    }
    System::DelayUs(2);
    bearamin_cv_dac_sync.Write(true);
    System::DelayUs(2);
}

void WriteBearaminCvDacChannel(uint8_t channel, float volts)
{
    const uint16_t value = patch.VoltageToCodeExp(volts);
    const uint16_t word = static_cast<uint16_t>(
        0x8000u | ((static_cast<uint16_t>(channel) & 0x03u) << 12) | value);
    WriteBearaminCvDacWord(word);
}

void WriteCvOutputs(const float cv[mm::kCvOutCount])
{
    float routed_cv[mm::kCvOutCount];
    const uint32_t now = System::GetNow();
    bearamin_cv_router.Route(current_mode,
                             BearaminPacket(),
                             BearaminBridgeFresh(now),
                             now,
                             cv,
                             routed_cv);

    cv_output_write_count++;
    for(int i = 0; i < mm::kCvOutCount; i++)
        last_written_cv[i] = routed_cv[i];

    // Logical CV1-CV2 are driven by the Daisy native DAC. Logical CV3-CV6 are
    // driven by the DAC7554 expander. Preserve CV3's existing compensation,
    // but the Bearamin XYZ jacks use the directly verified DAC transfer so
    // logical -5/0/+5 V reaches CV4-CV6 without the legacy +2 V offset.
    desired_expander_cv[0] = mm::Dac7554EquivalentVolts(routed_cv[2]);
    for(int i = 1; i < 4; i++)
        desired_expander_cv[i] = mm::BearaminXyzDacVolts(routed_cv[i + 2]);

    const float native_max_volts
        = current_mode == mm::Mode::BearaminBridge ? 10.0f : 5.0f;
    patch.WriteCvOut(CV_OUT_1,
                     mm::Clamp(routed_cv[0], -5.0f, native_max_volts),
                     false);
    patch.WriteCvOut(CV_OUT_2,
                     mm::Clamp(routed_cv[1], -5.0f, native_max_volts),
                     false);
}

void ServiceCvExpanderOutputs()
{
    const uint32_t now = System::GetNow();
    if(now - cv_expander_last_update_ms < kCvExpanderUpdateMs)
        return;
    cv_expander_last_update_ms = now;

    float cv[4];
    {
        ScopedIrqBlocker block;
        for(int i = 0; i < 4; i++)
            cv[i] = desired_expander_cv[i];
    }

    for(uint8_t channel = 0; channel < 4; channel++)
        WriteBearaminCvDacChannel(channel, cv[channel]);
    cv_expander_update_count++;
}

void Silence(AudioHandle::OutputBuffer out, size_t size)
{
    for(size_t i = 0; i < size; i++)
    {
        out[0][i] = 0.0f;
        out[1][i] = 0.0f;
    }
}

void AddUnityPassThrough(AudioHandle::InputBuffer  in,
                         AudioHandle::OutputBuffer out,
                         size_t                    size)
{
    for(size_t i = 0; i < size; i++)
    {
        out[0][i] = mm::UnityAudioBlend(in[0][i], out[0][i]);
        out[1][i] = mm::UnityAudioBlend(in[1][i], out[1][i]);
    }
}

void ZeroCvAndGate()
{
    const float cv[mm::kCvOutCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    WriteCvOutputs(cv);
    patch.gate_out_1.Write(false);
    patch.gate_out_2.Write(false);
}

void OnModeChanged(mm::Mode next_mode)
{
    patch.gate_out_1.Write(false);
    if(next_mode != mm::Mode::BearaminBridge)
        patch.gate_out_2.Write(false);
    if(mm::IsRingsMode(next_mode))
        resonator.set_model(RingsModelForMode(next_mode));

    {
        ScopedIrqBlocker block;
        pending_announcement_mode
            = (BearaminPacket().flags
               & mm::kBearaminModeAnnouncementsEnabled)
                      != 0
                  ? static_cast<int>(next_mode)
                  : -1;
        announcement_streaming_enabled = false;
        announcement_swap_requested
            = pending_announcement_mode >= 0;
    }

    current_mode = next_mode;
}

void MixModeAnnouncement(AudioHandle::OutputBuffer out, size_t size)
{
    if(!announcement_streaming_enabled || announcement_swap_requested)
        return;

    for(size_t i = 0; i < size; i++)
    {
        float prompt[2] = {0.0f, 0.0f};
        mode_announcement.Stream(prompt, 2);
        if(announcement_channels == 1)
            prompt[1] = prompt[0];

        out[0][i] = mm::Clamp(out[0][i] * kAnnouncementDuck
                                  + prompt[0] * kAnnouncementGain,
                              -1.0f,
                              1.0f);
        out[1][i] = mm::Clamp(out[1][i] * kAnnouncementDuck
                                  + prompt[1] * kAnnouncementGain,
                              -1.0f,
                              1.0f);
    }

    if(!mode_announcement.GetPlaying())
        announcement_streaming_enabled = false;
}

float BearaminLaneAxisOffset(mm::ResonatorLane lane, bool bearamin_fresh)
{
    const mm::BearaminBridgePacket& packet = BearaminPacket();
    const bool stale = (packet.flags & mm::kBearaminStale) != 0;
    if(!bearamin_fresh || stale)
        return 0.0f;

    const mm::BearaminResonatorMapping mapping
        = mm::BearaminResonatorLaneMapping(packet, lane);
    return mm::BearaminMappingAxisValue(
               packet, mapping.axis, mapping.unipolar)
           * mapping.multiplier;
}

float ApplyBearaminLaneOffsetToControl(float             bipolar_control,
                                       mm::ResonatorLane lane,
                                       bool              bearamin_fresh,
                                       float             gesture_amount)
{
    const float unit = mm::ApplyUnitOffset(
        mm::ControlToUnit(bipolar_control),
        BearaminLaneAxisOffset(lane, bearamin_fresh),
        gesture_amount);
    return unit * 2.0f - 1.0f;
}

float BearaminLaneOffsetToCv(mm::ResonatorLane lane,
                             bool              bearamin_fresh,
                             float             gesture_amount)
{
    return mm::BearaminOffsetToCv(
        BearaminLaneAxisOffset(lane, bearamin_fresh), gesture_amount);
}

void RunBearaminResonator(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    const uint32_t now = System::GetNow();
    const bool bearamin_fresh = BearaminBridgeFresh(now);
    const float gesture_amount
        = mm::ControlToGestureAmount(patch.GetAdcValue(CV_7));

    const float brightness = mm::ApplyUnitOffset(
        mm::PanelControlToUnit(patch.GetAdcValue(CV_1)),
        BearaminLaneAxisOffset(mm::ResonatorLane::Brightness, bearamin_fresh),
        gesture_amount);
    const float damping = mm::ApplyUnitOffset(
        mm::PanelControlToUnit(patch.GetAdcValue(CV_2)),
        BearaminLaneAxisOffset(mm::ResonatorLane::Damping, bearamin_fresh),
        gesture_amount);
    const float structure = mm::ApplyUnitOffset(
        mm::PanelControlToUnit(patch.GetAdcValue(CV_3)),
        BearaminLaneAxisOffset(mm::ResonatorLane::Structure, bearamin_fresh),
        gesture_amount);
    const float position = mm::ApplyUnitOffset(
        mm::PanelControlToUnit(patch.GetAdcValue(CV_4)),
        BearaminLaneAxisOffset(mm::ResonatorLane::Position, bearamin_fresh),
        gesture_amount);
    const float reverb_mix = mm::ApplyUnitOffset(
        mm::PanelControlToUnit(patch.GetAdcValue(CV_5)),
        BearaminLaneAxisOffset(mm::ResonatorLane::Reverb, bearamin_fresh),
        gesture_amount);
    const float base_frequency
        = mm::PanelControlToUnit(patch.GetAdcValue(CV_6));
    const float frequency = mm::ApplyUnitOffset(
        base_frequency,
        BearaminLaneAxisOffset(mm::ResonatorLane::Frequency, bearamin_fresh),
        gesture_amount);
    const mm::BearaminBridgePacket& packet = BearaminPacket();
    const mm::ResonatorWebParams web_params = mm::MakeResonatorWebParams(
        packet,
        reverb_mix,
        gesture_amount,
        bearamin_fresh);
    const float root_note
        = mm::QuantizedRootMidiNoteFromPanelControl(base_frequency);
    const float midi_note = mm::QuantizeResonatorMidiNote(
        mm::UnitToResonatorMidiNote(frequency),
        root_note,
        web_params.frequency_quantization);
    const float resonator_hz = ResonatorMidiNoteToHz(midi_note);
    const float sample_rate = patch.AudioSampleRate();
    const float phase_increment
        = mm::Clamp(resonator_hz / sample_rate, 0.0f, 0.49f);
    const bool impact_strum = BearaminImpactGateTrig(bearamin_fresh);
    const bool strum = patch.gate_in_1.Trig() || impact_strum;
    const float strum_impulse = kResonatorStrumImpulse
                                * (impact_strum
                                       ? (0.35f + mm::BearaminImpactVelocityUnit(
                                                     packet)
                                                     * 0.90f)
                                       : 1.0f);

    rings::Patch rings_patch;
    rings_patch.structure  = structure;
    rings_patch.brightness = brightness;
    rings_patch.damping    = damping;
    rings_patch.position   = position;

    rings::PerformanceState state;
    state.strum            = strum;
    state.internal_exciter = false;
    state.internal_strum   = false;
    state.internal_note    = false;
    state.tonic            = 0.0f;
    state.note             = midi_note;
    state.fm               = 0.0f;
    state.chord            = 0;

    size_t offset = 0;
    while(offset < size)
    {
        const size_t chunk = size - offset < kResonatorBlockSize
                                 ? size - offset
                                 : kResonatorBlockSize;
        float mono[kResonatorBlockSize];
        float left[kResonatorBlockSize];
        float right[kResonatorBlockSize];
        float pre_reverb_feedback[kResonatorBlockSize];
        for(size_t i = 0; i < chunk; i++)
        {
            resonator_feedback_smoothed
                += (web_params.feedback - resonator_feedback_smoothed)
                   * 0.000416f;
            const float drone_target
                = web_params.drone_enabled ? web_params.drone_level : 0.0f;
            resonator_drone_level_smoothed
                += (drone_target - resonator_drone_level_smoothed) * 0.000416f;

            float drone = 0.0f;
            switch(web_params.drone_waveform)
            {
                case mm::ResonatorDroneWaveform::Saw:
                    drone = resonator_drone_phase * 2.0f - 1.0f
                            - ResonatorPolyBlep(
                                resonator_drone_phase, phase_increment);
                    break;
                case mm::ResonatorDroneWaveform::Sine:
                default:
                    drone = sinf(mm::kTwoPi * resonator_drone_phase);
                    break;
            }
            resonator_drone_phase += phase_increment;
            if(resonator_drone_phase >= 1.0f)
                resonator_drone_phase -= 1.0f;

            const float dry = (in[0][offset + i] + in[1][offset + i])
                              * 0.5f * kResonatorInputGain;
            mono[i] = mm::SoftLimit(
                dry
                + resonator_feedback_delay[i] * resonator_feedback_smoothed
                + drone * resonator_drone_level_smoothed
                      * mm::kResonatorDroneMaxGain);
        }
        if(strum && offset == 0)
            mono[0] = mm::SoftLimit(mono[0] + strum_impulse);

        resonator.Process(state, rings_patch, mono, left, right, chunk);
        for(size_t i = 0; i < chunk; i++)
            pre_reverb_feedback[i] = (left[i] + right[i]) * 0.5f;

        resonator_reverb_time_smoothed
            += (web_params.reverb_time - resonator_reverb_time_smoothed) * 0.01f;
        resonator_reverb_diffusion_smoothed
            += (web_params.reverb_diffusion - resonator_reverb_diffusion_smoothed)
               * 0.01f;
        resonator_reverb_lp_smoothed
            += (web_params.reverb_lp - resonator_reverb_lp_smoothed) * 0.01f;
        if(reverb_mix > 0.001f)
        {
            resonator_reverb.set_amount(reverb_mix);
            resonator_reverb.set_diffusion(resonator_reverb_diffusion_smoothed);
            resonator_reverb.set_time(resonator_reverb_time_smoothed);
            resonator_reverb.set_input_gain(0.2f);
            resonator_reverb.set_lp(resonator_reverb_lp_smoothed);
            resonator_reverb.Process(left, right, chunk);
        }

        for(size_t i = 0; i < chunk; i++)
        {
            out[0][offset + i] = mm::Clamp(left[i], -1.0f, 1.0f);
            out[1][offset + i] = mm::Clamp(right[i], -1.0f, 1.0f);
            resonator_feedback_delay[i] = pre_reverb_feedback[i];
            const float peak = mm::Clamp((mm::Abs(out[0][offset + i])
                                          + mm::Abs(out[1][offset + i]))
                                             * 0.5f,
                                         0.0f,
                                         1.0f);
            resonator_output_env = peak > resonator_output_env
                                       ? peak
                                       : resonator_output_env * 0.9992f;
        }
        state.strum = false;
        offset += chunk;
    }

    const float env_volts = mm::Clamp(resonator_output_env * 5.0f, 0.0f, 5.0f);
    const float cv[mm::kCvOutCount] = {
        env_volts,
        -env_volts,
        (midi_note - mm::kResonatorBaseMidiNote) / 12.0f,
        BearaminLaneOffsetToCv(
            mm::ResonatorLane::Position, bearamin_fresh, gesture_amount),
        BearaminLaneOffsetToCv(
            mm::ResonatorLane::Reverb, bearamin_fresh, gesture_amount),
        BearaminLaneOffsetToCv(
            mm::ResonatorLane::Frequency, bearamin_fresh, gesture_amount),
    };
    WriteCvOutputs(cv);
    patch.gate_out_1.Write(strum);
    patch.gate_out_2.Write(BearaminImpactGateLevel(bearamin_fresh));
}

void RunStackedFmDrone(AudioHandle::OutputBuffer out, size_t size)
{
    const float sample_rate  = patch.AudioSampleRate();
    const uint32_t now       = System::GetNow();
    const bool bearamin_fresh = bearamin_last_packet_ms != 0
                                && now - bearamin_last_packet_ms <= 250;
    const float cv7          = patch.GetAdcValue(CV_7);
    const float gesture_amount = mm::ControlToGestureAmount(cv7);
    const float cv1 = ApplyBearaminLaneOffsetToControl(
        patch.GetAdcValue(CV_1),
        mm::ResonatorLane::Brightness,
        bearamin_fresh,
        gesture_amount);
    const float cv2 = ApplyBearaminLaneOffsetToControl(
        patch.GetAdcValue(CV_2),
        mm::ResonatorLane::Damping,
        bearamin_fresh,
        gesture_amount);
    const float cv3 = ApplyBearaminLaneOffsetToControl(
        patch.GetAdcValue(CV_3),
        mm::ResonatorLane::Structure,
        bearamin_fresh,
        gesture_amount);
    const float cv4 = ApplyBearaminLaneOffsetToControl(
        patch.GetAdcValue(CV_4),
        mm::ResonatorLane::Position,
        bearamin_fresh,
        gesture_amount);
    const float cv5 = ApplyBearaminLaneOffsetToControl(
        patch.GetAdcValue(CV_5),
        mm::ResonatorLane::Reverb,
        bearamin_fresh,
        gesture_amount);
    const float cv6 = ApplyBearaminLaneOffsetToControl(
        patch.GetAdcValue(CV_6),
        mm::ResonatorLane::Frequency,
        bearamin_fresh,
        gesture_amount);
    const float mod_2_depth
        = mm::Clamp(mm::ControlToFmDepthHz(cv4), 0.0f, 720.0f);
    const float carrier_base = mm::Clamp(mm::ControlToCarrierHz(cv5),
                                         0.1f,
                                         sample_rate * 0.25f);
    const float spread       = mm::Clamp(cv6 * 0.10f, -0.25f, 0.25f);
    const float carrier_depth
        = mm::Clamp(mm::ControlToFmDepthHz(cv2), 0.0f, 720.0f);
    const float mod_1_base_hz = mm::Clamp(mm::ControlToModHz(cv1),
                                          0.01f,
                                          sample_rate * 0.25f);
    const float mod_2_base_hz = mm::Clamp(mm::ControlToModHz(cv3),
                                          0.01f,
                                          sample_rate * 0.25f);

    float last_mod_1 = 0.0f;
    float last_mod_2 = 0.0f;
    float last_left  = 0.0f;
    float last_right = 0.0f;
    float last_depth = 0.0f;

    for(size_t i = 0; i < size; i++)
    {
        last_mod_2 = drone_mod_2.Sine(mod_2_base_hz, sample_rate);
        const float mod_1_hz
            = mm::Clamp(mod_1_base_hz + last_mod_2 * mod_2_depth,
                        0.01f,
                        sample_rate * 0.25f);
        last_mod_1 = drone_mod_1.Sine(mod_1_hz, sample_rate);

        last_depth = last_mod_1 * carrier_depth;
        const float left_hz
            = mm::Clamp(carrier_base + last_depth, 0.1f, sample_rate * 0.25f);
        const float right_hz
            = mm::Clamp(carrier_base * (1.0f + spread) + last_depth * 0.72f,
                        0.1f,
                        sample_rate * 0.25f);

        last_left    = drone_left.Sine(left_hz, sample_rate);
        last_right   = drone_right.Triangle(right_hz, sample_rate);
        out[0][i]    = mm::kAudioAmp * last_left;
        out[1][i]    = mm::kAudioAmp * last_right;
    }

    float cv[mm::kCvOutCount] = {
        mm::LfoToCv(last_mod_1),
        mm::LfoToCv(last_mod_2),
        mm::Clamp(last_depth / 720.0f, -1.0f, 1.0f) * 5.0f,
        0.0f,
        0.0f,
        0.0f,
    };
    WriteCvOutputs(cv);
}

void RunBearaminBridge(AudioHandle::OutputBuffer out, size_t size)
{
    Silence(out, size);

    float cv[mm::kCvOutCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const uint32_t now = System::GetNow();
    const bool mode_heartbeat = (now % 500) < 250;
    const bool fresh = BearaminBridgeFresh(now);

    if(fresh)
        bearamin_bridge_fresh_cv_count++;

    WriteCvOutputs(cv);
    patch.gate_out_1.Write(fresh ? (BearaminPacket().flags
                                    & mm::kBearaminTofGate)
                                  : mode_heartbeat);
    patch.gate_out_2.Write(BearaminImpactGateLevel(fresh));
    patch.SetLed(fresh || mode_heartbeat);
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    const uint32_t callback_start = PerformanceCycleCount();
    const uint32_t callback_period_cycles = SystemCoreClock / 1000u;
    const uint32_t prior_start = audio_callback_last_start_cycles;
    if(prior_start != 0)
    {
        const uint32_t start_gap = callback_start - prior_start;
        RecordMaximum(audio_callback_max_start_gap_cycles, start_gap);
        if(start_gap > callback_period_cycles
                           + callback_period_cycles / 4u)
            audio_callback_late_start_count++;
        if(start_gap >= callback_period_cycles
                            + callback_period_cycles / 2u)
        {
            const uint32_t elapsed_periods
                = (start_gap + callback_period_cycles / 2u)
                  / callback_period_cycles;
            if(elapsed_periods > 1)
                audio_callback_missed_period_count += elapsed_periods - 1;
        }
    }
    audio_callback_last_start_cycles = callback_start;
    audio_callback_count++;

    const uint32_t predictor_now = System::GetNow();
    const mm::BearaminBridgePacket& raw_packet = RawBearaminPacket();
    const bool bearamin_fresh = BearaminBridgeFresh(predictor_now);
    if(raw_packet.protocol_version == mm::kBearaminVersionV11)
    {
        bearamin_audio_packet = bearamin_trajectory_reconstructor.Process(
            raw_packet, bearamin_fresh);
    }
    else
    {
        bearamin_audio_packet = bearamin_predictor.Process(
            raw_packet, predictor_now, bearamin_fresh);
    }

    patch.ProcessAllControls();

    const mm::Mode next_mode
        = mm::SelectModeFromControl(patch.GetAdcValue(CV_8));
    if(next_mode != current_mode)
        OnModeChanged(next_mode);

    switch(current_mode)
    {
        case mm::Mode::RingsModal:
        case mm::Mode::RingsSympatheticStrings:
        case mm::Mode::RingsKarplusStrong:
            RunBearaminResonator(in, out, size);
            break;
        case mm::Mode::StackedFmDrone: RunStackedFmDrone(out, size); break;
        case mm::Mode::BearaminBridge: RunBearaminBridge(out, size); break;
        default:
            Silence(out, size);
            ZeroCvAndGate();
            break;
    }

    if(current_mode != mm::Mode::BearaminBridge
       && !mm::IsRingsMode(current_mode))
    {
        const bool bearamin_fresh = BearaminBridgeFresh(System::GetNow());
        BearaminImpactGateTrig(bearamin_fresh);
        patch.gate_out_2.Write(BearaminImpactGateLevel(bearamin_fresh));
    }

    if(!mm::IsRingsMode(current_mode))
        AddUnityPassThrough(in, out, size);

    MixModeAnnouncement(out, size);

    float peak = 0.0f;
    for(size_t i = 0; i < size; i++)
    {
        const float left  = mm::Abs(out[0][i]);
        const float right = mm::Abs(out[1][i]);
        if(left > peak)
            peak = left;
        if(right > peak)
            peak = right;
    }
    audio_output_peak = peak;

    const uint32_t callback_cycles
        = PerformanceCycleCount() - callback_start;
    RecordMaximum(audio_callback_max_cycles, callback_cycles);
    if(callback_cycles >= callback_period_cycles)
        audio_callback_deadline_miss_count++;
}

void BearaminUartRxDmaCallback(uint8_t*            data,
                               size_t              size,
                               void*               context,
                               UartHandler::Result result);

void InitBearaminBridgeUart()
{
    UartHandler::Config cfg;
    cfg.periph           = UartHandler::Config::Peripheral::UART_4;
    cfg.mode             = UartHandler::Config::Mode::TX_RX;
    cfg.pin_config.rx    = DPT::A2;
    cfg.pin_config.tx    = DPT::A3;
    cfg.baudrate         = kBearaminUartBaud;
    cfg.stopbits         = UartHandler::Config::StopBits::BITS_1;
    cfg.parity           = UartHandler::Config::Parity::NONE;
    cfg.wordlength       = UartHandler::Config::WordLength::BITS_8;
    bearamin_uart.Init(cfg);
    UART4->CR1 &= ~USART_CR1_UE;
    UART4->CR1 |= USART_CR1_FIFOEN;
    UART4->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    if(kBearaminUartRxDmaEnabled)
        bearamin_uart.DmaListenStart(bearamin_uart_rx_dma_buffer,
                                     sizeof(bearamin_uart_rx_dma_buffer),
                                     BearaminUartRxDmaCallback,
                                     nullptr);
}

void EnableExpSniffPortClocks()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

bool ReadExpPin(Pin pin)
{
    return HAL_GPIO_ReadPin(GetHALPort(pin), GetHALPin(pin)) == GPIO_PIN_SET;
}

void WriteExpPin(Pin pin, bool level)
{
    HAL_GPIO_WritePin(GetHALPort(pin),
                      GetHALPin(pin),
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ConfigureExpOutput(Pin pin, bool level)
{
    GPIO_InitTypeDef init;
    init.Pin   = GetHALPin(pin);
    init.Mode  = GPIO_MODE_OUTPUT_PP;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    WriteExpPin(pin, level);
    HAL_GPIO_Init(GetHALPort(pin), &init);
}

void ConfigureBearaminRxUartPin()
{
    GPIO_InitTypeDef init;
    init.Pin       = GetHALPin(DPT::A2);
    init.Mode      = GPIO_MODE_AF_PP;
    init.Pull      = GPIO_PULLUP;
    init.Speed     = GPIO_SPEED_FREQ_HIGH;
    init.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GetHALPort(DPT::A2), &init);
}

void ConfigureBearaminRxAckPin()
{
    GPIO_InitTypeDef init;
    init.Pin   = GetHALPin(DPT::A2);
    init.Mode  = GPIO_MODE_OUTPUT_OD;
    init.Pull  = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    WriteExpPin(DPT::A2, false);
    HAL_GPIO_Init(GetHALPort(DPT::A2), &init);
}

bool TryReceiveBearaminByte(uint8_t& byte)
{
    USART_TypeDef* uart = UART4;
    const uint32_t isr  = uart->ISR;
    const bool     have_byte = (isr & USART_ISR_RXNE_RXFNE) != 0;

    if(have_byte)
        byte = static_cast<uint8_t>(uart->RDR & 0xffu);

    const uint32_t errors = isr
                            & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE
                               | USART_ISR_PE);
    if(errors != 0)
    {
        uint32_t clear_flags = 0;
        bearamin_uart_error_count++;
        bearamin_uart_last_error = errors;
        if(errors & USART_ISR_ORE)
        {
            bearamin_uart_overrun_count++;
            clear_flags |= USART_ICR_ORECF;
        }
        if(errors & USART_ISR_FE)
        {
            bearamin_uart_framing_count++;
            clear_flags |= USART_ICR_FECF;
        }
        if(errors & USART_ISR_NE)
        {
            bearamin_uart_noise_count++;
            clear_flags |= USART_ICR_NECF;
        }
        if(errors & USART_ISR_PE)
        {
            bearamin_uart_parity_count++;
            clear_flags |= USART_ICR_PECF;
        }
        uart->ICR = clear_flags;
    }

    return have_byte;
}

void ScheduleBearaminSingleWireAck();
void ProcessBearaminPresetLine(const char* line);

void AcceptBearaminPresetByte(uint8_t byte)
{
    if(byte == '\n' || byte == '\r')
    {
        bearamin_preset_line[bearamin_preset_line_index] = '\0';
        if(bearamin_preset_line_index > 0
           && mm::PresetLineHasPrefix(bearamin_preset_line, "BP"))
            ProcessBearaminPresetLine(bearamin_preset_line);
        bearamin_preset_line_index = 0;
        return;
    }

    if(byte < 32 || byte > 126)
    {
        bearamin_preset_line_index = 0;
        return;
    }

    if(bearamin_preset_line_index + 1 < sizeof(bearamin_preset_line))
    {
        bearamin_preset_line[bearamin_preset_line_index++]
            = static_cast<char>(byte);
    }
    else
    {
        bearamin_preset_line_index = 0;
    }
}

void AcceptBearaminBridgeByte(uint8_t byte)
{
    bearamin_rx_byte_count++;
    bearamin_rx_last_byte_ms = System::GetNow();
    bearamin_rx_debug_bytes[bearamin_rx_debug_write_index
                            % sizeof(bearamin_rx_debug_bytes)]
        = byte;
    bearamin_rx_debug_write_index++;
    if(bearamin_rx_debug_count < sizeof(bearamin_rx_debug_bytes))
        bearamin_rx_debug_count++;
    if(byte == mm::kBearaminStart0)
        bearamin_rx_start_byte_count++;
    if(bearamin_rx_previous_byte == mm::kBearaminStart0
       && byte == mm::kBearaminStart1)
        bearamin_rx_start_pair_count++;

    if(byte == mm::kBearaminDptProbe[bearamin_rx_dpt_echo_index])
    {
        bearamin_rx_dpt_echo_index++;
        if(bearamin_rx_dpt_echo_index == mm::kBearaminDptProbeSize)
        {
            bearamin_rx_dpt_echo_count++;
            bearamin_rx_dpt_echo_index = 0;
        }
    }
    else
    {
        bearamin_rx_dpt_echo_index = byte == mm::kBearaminDptProbe[0] ? 1 : 0;
    }
    bearamin_rx_previous_byte = byte;
    AcceptBearaminPresetByte(byte);

    mm::BearaminBridgePacket packet;
    if(bearamin_parser.Push(byte, packet))
    {
        if(packet.protocol_version == mm::kBearaminVersionV11)
        {
            if(!bearamin_trajectory_reconstructor.Prepare(
                   packet, kBearaminAudioCallbackMs))
            {
                bearamin_invalid_trajectory_count++;
                return;
            }
            RecordBearaminTrajectoryCorrection(packet);
            bearamin_v11_packet_count++;
        }
        else
        {
            bearamin_v10_packet_count++;
        }
        bearamin_raw_packets.Publish(packet);
        bearamin_last_packet_ms = System::GetNow();
        bearamin_rx_packet_count++;
        if(current_mode == mm::Mode::BearaminBridge)
            ScheduleBearaminSingleWireAck();
    }
}

void BearaminUartRxDmaCallback(uint8_t*            data,
                               size_t              size,
                               void*               context,
                               UartHandler::Result result)
{
    (void)context;
    if(result != UartHandler::Result::OK)
        return;
    for(size_t i = 0; i < size; i++)
        AcceptBearaminBridgeByte(data[i]);
}

void PumpBearaminBridgeRx(size_t byte_limit)
{
    for(size_t i = 0; i < byte_limit; i++)
    {
        uint8_t byte = 0;
        if(!TryReceiveBearaminByte(byte))
            return;
        AcceptBearaminBridgeByte(byte);
    }
}

bool DirectTransmitBearaminBytes(const uint8_t* data,
                                 size_t         size,
                                 uint32_t       timeout_ms)
{
    if(data == nullptr || size == 0)
        return true;

    const uint32_t call_start = PerformanceCycleCount();
    bearamin_uart_tx_call_count++;
    PumpBearaminBridgeRx(32);
    const uint32_t wire_time_ms = static_cast<uint32_t>(
        (size * 10u * 1000u + kBearaminUartBaud - 1u) / kBearaminUartBaud);
    const uint32_t effective_timeout_ms
        = timeout_ms > wire_time_ms + 5u ? timeout_ms : wire_time_ms + 5u;
    const UartHandler::Result result = bearamin_uart.BlockingTransmit(
        const_cast<uint8_t*>(data), size, effective_timeout_ms);
    if(result == UartHandler::Result::OK)
        bearamin_uart_tx_byte_count += static_cast<uint32_t>(size);
    else
        bearamin_uart_tx_error_count++;
    PumpBearaminBridgeRx(64);
    RecordMaximum(bearamin_uart_tx_max_call_cycles,
                  PerformanceCycleCount() - call_start);
    return result == UartHandler::Result::OK;
}

bool BearaminPresetTxQueueEmpty()
{
    return bearamin_preset_tx_head == bearamin_preset_tx_tail;
}

bool BearaminPresetTxQueueFull()
{
    return (bearamin_preset_tx_head + 1) % kBearaminPresetTxQueueDepth
           == bearamin_preset_tx_tail;
}

void SendBearaminPresetLine(const char* line)
{
    if(line == nullptr || BearaminPresetTxQueueFull())
    {
        bearamin_preset_tx_drop_count++;
        return;
    }

    BearaminPresetTxItem& item
        = bearamin_preset_tx_queue[bearamin_preset_tx_head];
    strncpy(item.line, line, sizeof(item.line) - 1);
    item.line[sizeof(item.line) - 1] = '\0';
    item.packet_count_at_queue = bearamin_rx_packet_count;
    item.command_ms            = bearamin_preset_command_ms;
    item.immediate_allowed     = bearamin_preset_immediate_tx_available;
    bearamin_preset_immediate_tx_available = false;
    bearamin_preset_tx_head
        = (bearamin_preset_tx_head + 1) % kBearaminPresetTxQueueDepth;
}

void ServiceBearaminPresetTx()
{
    if(BearaminPresetTxQueueEmpty() || bearamin_single_wire_ack_active)
        return;

    const uint32_t now = System::GetNow();
    BearaminPresetTxItem& item
        = bearamin_preset_tx_queue[bearamin_preset_tx_tail];
    const bool rx_idle = bearamin_rx_last_byte_ms == 0
                         || now - bearamin_rx_last_byte_ms
                                >= kBearaminPresetPostPacketDelayMs;
    const bool inside_command_guard
        = item.immediate_allowed && item.command_ms != 0
          && now - item.command_ms <= kBearaminPresetImmediateTxMs;
    const bool fresh_packet_gap
        = bearamin_rx_packet_count > item.packet_count_at_queue
          && bearamin_last_packet_ms != 0
          && now - bearamin_last_packet_ms
                 <= kBearaminPresetPostPacketWindowMs;
    if(!rx_idle || (!inside_command_guard && !fresh_packet_gap))
        return;

    DirectTransmitBearaminBytes(
        reinterpret_cast<const uint8_t*>(item.line), strlen(item.line), 40);
    item.line[0] = '\0';
    bearamin_preset_tx_tail
        = (bearamin_preset_tx_tail + 1) % kBearaminPresetTxQueueDepth;
}

void SendBearaminPresetError(uint16_t seq, const char* code)
{
    char line[112];
    snprintf(line, sizeof(line), "BPERR,%u,%s\n", seq, code);
    SendBearaminPresetLine(line);
}

void SendBearaminPresetChunkAck(uint16_t seq, size_t offset)
{
    char line[40];
    snprintf(line, sizeof(line), "BPAK,%u,%u\n", seq, static_cast<unsigned>(offset));
    SendBearaminPresetLine(line);
}

void SendBearaminPresetWriteError(uint16_t seq,
                                  FRESULT write_result,
                                  FRESULT close_result,
                                  UINT    written,
                                  size_t  expected)
{
    preset_sd_card_state = static_cast<uint32_t>(HAL_SD_GetCardState(&hsd1));
    preset_sd_hal_error  = HAL_SD_GetError(&hsd1);
    char code[80];
    snprintf(code,
             sizeof(code),
             "write_%u_%u_%u_%u_s%lu_e%lX",
             static_cast<unsigned>(write_result),
             static_cast<unsigned>(close_result),
             static_cast<unsigned>(written),
             static_cast<unsigned>(expected),
             static_cast<unsigned long>(preset_sd_card_state),
             static_cast<unsigned long>(preset_sd_hal_error));
    SendBearaminPresetError(seq, code);
}

void SendBearaminPresetReadError(uint16_t seq,
                                 FRESULT read_result,
                                 FRESULT close_result,
                                 UINT    last_read,
                                 size_t  total_read,
                                 size_t  expected)
{
    preset_sd_card_state = static_cast<uint32_t>(HAL_SD_GetCardState(&hsd1));
    preset_sd_hal_error  = HAL_SD_GetError(&hsd1);
    char code[96];
    snprintf(code,
             sizeof(code),
             "read_%u_%u_%u_%u_%u_s%lu_e%lX",
             static_cast<unsigned>(read_result),
             static_cast<unsigned>(close_result),
             static_cast<unsigned>(last_read),
             static_cast<unsigned>(total_read),
             static_cast<unsigned>(expected),
             static_cast<unsigned long>(preset_sd_card_state),
             static_cast<unsigned long>(preset_sd_hal_error));
    SendBearaminPresetError(seq, code);
}

void SendBearaminPresetStorageError(uint16_t seq)
{
    char code[32];
    if(!preset_sd_init_ok)
    {
        snprintf(code, sizeof(code), "sd_init");
    }
    else if(!announcements_mounted)
    {
        snprintf(code,
                 sizeof(code),
                 "sd_mount_%u_b%u_s%lu_e%lX",
                 static_cast<unsigned>(preset_sd_mount_result),
                 static_cast<unsigned>(preset_sd_bsp_init_result),
                 static_cast<unsigned long>(preset_sd_card_state),
                 static_cast<unsigned long>(preset_sd_hal_error));
    }
    else
    {
        snprintf(code,
                 sizeof(code),
                 "sd_dir_%u",
                 static_cast<unsigned>(preset_sd_directory_result));
    }
    SendBearaminPresetError(seq, code);
}

bool BuildPresetDirectoryPath(char* out, size_t capacity)
{
    if(!preset_fatfs_linked)
        return false;
    const int written
        = snprintf(out, capacity, "%spresets", fsi.GetSDPath());
    return written > 0 && static_cast<size_t>(written) < capacity;
}

bool BuildPresetPath(const char* name, const char* suffix, char* out, size_t capacity)
{
    if(!mm::IsValidPresetName(name))
        return false;
    if(!preset_fatfs_linked)
        return false;
    const int written
        = snprintf(out, capacity, "%spresets/%s.%s", fsi.GetSDPath(), name, suffix);
    return written > 0 && static_cast<size_t>(written) < capacity;
}

bool TryMountPresetStorage(bool force_retry = false)
{
    if(announcements_mounted)
        return true;

    const uint32_t now = System::GetNow();
    if(!force_retry && preset_sd_last_mount_attempt_ms != 0
       && now - preset_sd_last_mount_attempt_ms < 250)
    {
        return false;
    }
    preset_sd_last_mount_attempt_ms = now;

    auto unlink_fatfs = []() {
        if(!preset_fatfs_linked)
            return;
        f_mount(nullptr, fsi.GetSDPath(), 0);
        fsi.DeInit();
        preset_fatfs_linked = false;
        announcement_streaming_supported = false;
    };

    auto attempt_mount = [](SdmmcHandler::Speed speed,
                            SdmmcHandler::BusWidth width) {
        HAL_SD_DeInit(&hsd1);
        System::Delay(10);

        SdmmcHandler::Config sd_config;
        sd_config.Defaults();
        sd_config.speed = speed;
        sd_config.width = width;
        preset_sd_init_ok
            = sdmmc.Init(sd_config) == SdmmcHandler::Result::OK;
        if(!preset_sd_init_ok)
        {
            preset_sd_mount_result = FR_NOT_READY;
            return false;
        }
        preset_sd_bsp_init_result = BSP_SD_Init();
        preset_sd_card_state
            = static_cast<uint32_t>(HAL_SD_GetCardState(&hsd1));
        preset_sd_hal_error = HAL_SD_GetError(&hsd1);
        const bool only_speed_mode_unsupported
            = preset_sd_bsp_init_result != MSD_OK
              && preset_sd_card_state == HAL_SD_CARD_TRANSFER
              && (preset_sd_hal_error & HAL_SD_ERROR_UNSUPPORTED_FEATURE) != 0;
        if(preset_sd_bsp_init_result != MSD_OK && !only_speed_mode_unsupported)
        {
            preset_sd_mount_result = FR_NOT_READY;
            return false;
        }

        preset_fatfs_linked
            = fsi.Init(FatFSInterface::Config::MEDIA_SD)
              == FatFSInterface::Result::OK;
        if(!preset_fatfs_linked)
        {
            preset_sd_mount_result = FR_INVALID_DRIVE;
            return false;
        }
        if(only_speed_mode_unsupported)
        {
            const char* sd_path = fsi.GetSDPath();
            if(sd_path != nullptr && sd_path[0] >= '0' && sd_path[0] <= '9')
                disk.is_initialized[sd_path[0] - '0'] = 1;
        }

        preset_sd_mount_result
            = f_mount(&fsi.GetSDFileSystem(), fsi.GetSDPath(), 1);
        preset_sd_card_state
            = static_cast<uint32_t>(HAL_SD_GetCardState(&hsd1));
        preset_sd_hal_error = HAL_SD_GetError(&hsd1);
        announcements_mounted = preset_sd_mount_result == FR_OK;
        announcement_streaming_supported
            = announcements_mounted && speed != SdmmcHandler::Speed::SLOW;
        return announcements_mounted;
    };

    unlink_fatfs();

    if(attempt_mount(SdmmcHandler::Speed::STANDARD, SdmmcHandler::BusWidth::BITS_1))
        return true;
    unlink_fatfs();
    if(attempt_mount(SdmmcHandler::Speed::STANDARD, SdmmcHandler::BusWidth::BITS_4))
        return true;
    unlink_fatfs();
    if(attempt_mount(SdmmcHandler::Speed::FAST, SdmmcHandler::BusWidth::BITS_4))
        return true;
    unlink_fatfs();
    if(attempt_mount(SdmmcHandler::Speed::SLOW, SdmmcHandler::BusWidth::BITS_1))
        return true;

    if(preset_fatfs_linked)
    {
        f_mount(nullptr, fsi.GetSDPath(), 0);
        fsi.DeInit();
        preset_fatfs_linked = false;
    }
    announcements_mounted = false;
    announcement_streaming_supported = false;
    return false;
}

void InvalidatePresetStorage()
{
    announcements_mounted          = false;
    announcement_streaming_supported = false;
    if(preset_fatfs_linked)
    {
        f_mount(nullptr, fsi.GetSDPath(), 0);
        fsi.DeInit();
        preset_fatfs_linked = false;
    }
}

bool EnsurePresetDirectory()
{
    if(!TryMountPresetStorage(true))
        return false;
    char preset_dir[32];
    if(!BuildPresetDirectoryPath(preset_dir, sizeof(preset_dir)))
        return false;
    preset_sd_directory_result = f_mkdir(preset_dir);
    return preset_sd_directory_result == FR_OK
           || preset_sd_directory_result == FR_EXIST;
}

bool EndsWithJson(const char* name)
{
    const size_t length = strlen(name);
    return length > 5 && strcmp(name + length - 5, ".json") == 0;
}

void HandleBearaminPresetRead(uint16_t seq, const char* name);
void SetAnnouncementPlaybackEnabled(bool enabled);

bool PresetNameFromFileInfo(const FILINFO& file_info,
                            char*          name,
                            size_t         capacity)
{
    if(name == nullptr || capacity == 0)
        return false;
    name[0] = '\0';
    if((file_info.fattrib & AM_DIR) || !EndsWithJson(file_info.fname))
        return false;
    const size_t filename_length = strlen(file_info.fname);
    const size_t name_length     = filename_length - 5;
    if(name_length == 0 || name_length > mm::kPresetMaxNameLength
       || name_length + 1 > capacity)
        return false;
    memcpy(name, file_info.fname, name_length);
    name[name_length] = '\0';
    return mm::IsValidPresetName(name);
}

void HandleBearaminPresetList(uint16_t seq, bool restart = false)
{
    if(bearamin_preset_storage_job == PresetStorageJob::List
       && seq == bearamin_preset_storage_seq)
    {
        return;
    }
    if(bearamin_preset_storage_job == PresetStorageJob::None
       && bearamin_preset_list_completed
       && seq == bearamin_preset_list_completed_seq && !restart)
    {
        if(!BearaminPresetTxQueueEmpty())
            return;
        char line[32];
        snprintf(line,
                 sizeof(line),
                 "BPLS,%u,END,%u\n",
                 seq,
                 static_cast<unsigned>(bearamin_preset_list_count));
        SendBearaminPresetLine(line);
        return;
    }
    if(bearamin_preset_storage_job != PresetStorageJob::None)
    {
        SendBearaminPresetError(seq, "busy");
        return;
    }
    if(!EnsurePresetDirectory())
    {
        SendBearaminPresetStorageError(seq);
        return;
    }

    char preset_dir[32];
    if(!BuildPresetDirectoryPath(preset_dir, sizeof(preset_dir)))
    {
        SendBearaminPresetError(seq, "path");
        return;
    }
    memset(&bearamin_preset_dir, 0, sizeof(bearamin_preset_dir));
    FRESULT result = f_opendir(&bearamin_preset_dir, preset_dir);
    if(result != FR_OK)
    {
        SendBearaminPresetError(seq, "opendir");
        return;
    }
    bearamin_preset_read_active = false;
    bearamin_preset_list_completed = false;
    bearamin_preset_list_count = 0;
    bearamin_preset_storage_seq = seq;
    bearamin_preset_storage_job = PresetStorageJob::List;
    SetAnnouncementPlaybackEnabled(false);
}

void SendBearaminPresetReadBegin(uint16_t seq)
{
    char line[112];
    snprintf(line,
             sizeof(line),
             "BPRD,%u,BEGIN,%u,%04X\n",
             seq,
             static_cast<unsigned>(bearamin_preset_read_size),
             bearamin_preset_read_crc);
    SendBearaminPresetLine(line);
}

void HandleBearaminPresetRead(uint16_t seq, const char* name)
{
    if(bearamin_preset_storage_job == PresetStorageJob::Read
       && seq == bearamin_preset_storage_seq)
    {
        return;
    }
    if(bearamin_preset_storage_job == PresetStorageJob::None
       && bearamin_preset_read_active && seq == bearamin_preset_read_seq)
    {
        SendBearaminPresetReadBegin(seq);
        return;
    }
    if(bearamin_preset_storage_job != PresetStorageJob::None)
    {
        SendBearaminPresetError(seq, "busy");
        return;
    }
    if(!EnsurePresetDirectory())
    {
        SendBearaminPresetStorageError(seq);
        return;
    }

    char path[80];
    if(!BuildPresetPath(name, "json", path, sizeof(path)))
    {
        SendBearaminPresetError(seq, "name");
        return;
    }

    memset(&bearamin_preset_file, 0, sizeof(bearamin_preset_file));
    FRESULT result = f_open(&bearamin_preset_file, path, FA_READ);
    if(result != FR_OK)
    {
        SendBearaminPresetError(seq, "missing");
        return;
    }

    const FSIZE_t size = f_size(&bearamin_preset_file);
    if(size > mm::kPresetMaxJsonBytes)
    {
        f_close(&bearamin_preset_file);
        SendBearaminPresetError(seq, "large");
        return;
    }
    bearamin_preset_read_active = false;
    bearamin_preset_read_seq    = seq;
    bearamin_preset_read_size   = static_cast<size_t>(size);
    bearamin_preset_read_crc    = 0xFFFFu;
    bearamin_preset_io_offset   = 0;
    bearamin_preset_storage_seq = seq;
    bearamin_preset_storage_job = PresetStorageJob::Read;
    SetAnnouncementPlaybackEnabled(false);
}

void HandleBearaminPresetReadChunkRequest(uint16_t seq, size_t offset)
{
    if(!bearamin_preset_read_active || seq != bearamin_preset_read_seq)
    {
        SendBearaminPresetError(seq, "state");
        return;
    }
    if(offset > bearamin_preset_read_size
       || (offset < bearamin_preset_read_size
           && offset % mm::kPresetHexChunkBytes != 0))
    {
        SendBearaminPresetError(seq, "chunk");
        return;
    }

    char line[112];
    if(offset == bearamin_preset_read_size)
    {
        snprintf(line,
                 sizeof(line),
                 "BPRD,%u,END,%04X\n",
                 seq,
                 bearamin_preset_read_crc);
        SendBearaminPresetLine(line);
        return;
    }

    const size_t remaining = bearamin_preset_read_size - offset;
    const size_t chunk = remaining > mm::kPresetHexChunkBytes
                             ? mm::kPresetHexChunkBytes
                             : remaining;
    char hex[mm::kPresetHexChunkBytes * 2 + 1];
    mm::PresetHexEncode(
        bearamin_preset_buffer + offset, chunk, hex, sizeof(hex));
    snprintf(line,
             sizeof(line),
             "BPD,%u,%u,%s,%04X\n",
             seq,
             static_cast<unsigned>(offset),
             hex,
             mm::PresetCrc16(bearamin_preset_buffer + offset, chunk));
    SendBearaminPresetLine(line);
}

void HandleBearaminPresetWriteBegin(uint16_t seq,
                                    const char* name,
                                    size_t      size,
                                    uint16_t    crc)
{
    if(bearamin_preset_write_active)
    {
        if(seq == bearamin_preset_write_seq)
        {
            char line[32];
            snprintf(line, sizeof(line), "BPWR,%u,READY\n", seq);
            SendBearaminPresetLine(line);
        }
        else
        {
            SendBearaminPresetError(seq, "busy");
        }
        return;
    }
    if(bearamin_preset_storage_job != PresetStorageJob::None)
    {
        SendBearaminPresetError(seq, "busy");
        return;
    }
    if(!EnsurePresetDirectory())
    {
        SendBearaminPresetStorageError(seq);
        return;
    }
    if(!mm::IsValidPresetName(name))
    {
        SendBearaminPresetError(seq, "name");
        return;
    }
    if(size > mm::kPresetMaxJsonBytes)
    {
        SendBearaminPresetError(seq, "large");
        return;
    }
    bearamin_preset_write_seq = seq;
    bearamin_preset_write_size = size;
    bearamin_preset_write_received = 0;
    bearamin_preset_write_crc = crc;
    bearamin_preset_write_active = true;
    bearamin_preset_read_active = false;
    strncpy(bearamin_preset_write_name, name, sizeof(bearamin_preset_write_name) - 1);
    bearamin_preset_write_name[sizeof(bearamin_preset_write_name) - 1] = '\0';
    memset(bearamin_preset_buffer, 0, size);
    char line[32];
    snprintf(line, sizeof(line), "BPWR,%u,READY\n", seq);
    SendBearaminPresetLine(line);
}

void HandleBearaminPresetData(uint16_t seq,
                              size_t offset,
                              const char* hex,
                              uint16_t chunk_crc)
{
    if(!bearamin_preset_write_active || seq != bearamin_preset_write_seq)
    {
        SendBearaminPresetError(seq, "state");
        return;
    }
    uint8_t decoded[mm::kPresetHexChunkBytes];
    size_t decoded_size = 0;
    if(!mm::PresetHexDecode(hex, decoded, sizeof(decoded), decoded_size)
       || offset + decoded_size > bearamin_preset_write_size
       || mm::PresetCrc16(decoded, decoded_size) != chunk_crc)
    {
        return;
    }
    memcpy(bearamin_preset_buffer + offset, decoded, decoded_size);
    const size_t received_end = offset + decoded_size;
    if(received_end > bearamin_preset_write_received)
        bearamin_preset_write_received = received_end;
    SendBearaminPresetChunkAck(seq, offset);
}

void HandleBearaminPresetWriteEnd(uint16_t seq)
{
    if(!bearamin_preset_write_active || seq != bearamin_preset_write_seq)
    {
        if(bearamin_preset_storage_job != PresetStorageJob::None
           && seq == bearamin_preset_storage_seq)
            return;
        if(bearamin_preset_write_completed
           && seq == bearamin_preset_write_completed_seq)
        {
            char line[32];
            snprintf(line, sizeof(line), "BPWR,%u,OK\n", seq);
            SendBearaminPresetLine(line);
            return;
        }
        SendBearaminPresetError(seq, "state");
        return;
    }
    bearamin_preset_write_active = false;
    if(bearamin_preset_write_received != bearamin_preset_write_size
       || mm::PresetCrc16(bearamin_preset_buffer, bearamin_preset_write_size)
              != bearamin_preset_write_crc)
    {
        SendBearaminPresetError(seq, "crc");
        return;
    }

    if(!BuildPresetPath(bearamin_preset_write_name,
                        "tmp",
                        bearamin_preset_temp_path,
                        sizeof(bearamin_preset_temp_path))
       || !BuildPresetPath(
           bearamin_preset_write_name,
           "json",
           bearamin_preset_final_path,
           sizeof(bearamin_preset_final_path)))
    {
        SendBearaminPresetError(seq, "name");
        return;
    }

    memset(&bearamin_preset_file, 0, sizeof(bearamin_preset_file));
    FRESULT result
        = f_open(&bearamin_preset_file,
                 bearamin_preset_temp_path,
                 FA_WRITE | FA_CREATE_ALWAYS);
    if(result != FR_OK)
    {
        SendBearaminPresetError(seq, "open");
        return;
    }
    bearamin_preset_io_offset   = 0;
    bearamin_preset_storage_seq = seq;
    bearamin_preset_storage_job = PresetStorageJob::WriteData;
    SetAnnouncementPlaybackEnabled(false);
}

bool BearaminPresetStorageBusy()
{
    return bearamin_preset_storage_job != PresetStorageJob::None;
}

void ServiceBearaminPresetStorage()
{
    if(!BearaminPresetStorageBusy() || BearaminPresetTxQueueFull())
        return;

    if(bearamin_preset_storage_job == PresetStorageJob::List)
    {
        FILINFO file_info = {};
        const FRESULT result = f_readdir(&bearamin_preset_dir, &file_info);
        if(result != FR_OK)
        {
            f_closedir(&bearamin_preset_dir);
            bearamin_preset_storage_job = PresetStorageJob::None;
            SendBearaminPresetError(bearamin_preset_storage_seq, "readdir");
            InvalidatePresetStorage();
            return;
        }
        if(file_info.fname[0] == '\0')
        {
            const FRESULT close_result = f_closedir(&bearamin_preset_dir);
            bearamin_preset_storage_job = PresetStorageJob::None;
            if(close_result != FR_OK)
            {
                SendBearaminPresetError(
                    bearamin_preset_storage_seq, "closedir");
                InvalidatePresetStorage();
                return;
            }
            char line[32];
            snprintf(line,
                     sizeof(line),
                     "BPLS,%u,END,%u\n",
                     bearamin_preset_storage_seq,
                     static_cast<unsigned>(bearamin_preset_list_count));
            bearamin_preset_list_completed = true;
            bearamin_preset_list_completed_seq = bearamin_preset_storage_seq;
            SendBearaminPresetLine(line);
            return;
        }

        char name[mm::kPresetMaxNameLength + 1] = {};
        if(PresetNameFromFileInfo(file_info, name, sizeof(name)))
        {
            char line[96];
            snprintf(line,
                     sizeof(line),
                     "BPLS,%u,ITEM,%u,%s,%04X\n",
                     bearamin_preset_storage_seq,
                     static_cast<unsigned>(bearamin_preset_list_count),
                     name,
                     mm::PresetCrc16(
                         reinterpret_cast<const uint8_t*>(name),
                         strlen(name)));
            SendBearaminPresetLine(line);
            ++bearamin_preset_list_count;
        }
        return;
    }

    if(bearamin_preset_storage_job == PresetStorageJob::Read)
    {
        if(bearamin_preset_io_offset < bearamin_preset_read_size)
        {
            const size_t remaining
                = bearamin_preset_read_size - bearamin_preset_io_offset;
            const UINT request = static_cast<UINT>(
                remaining > kBearaminPresetIoChunkBytes
                    ? kBearaminPresetIoChunkBytes
                    : remaining);
            UINT transferred = 0;
            const FRESULT result
                = f_read(&bearamin_preset_file,
                         bearamin_preset_buffer + bearamin_preset_io_offset,
                         request,
                         &transferred);
            if(result != FR_OK || transferred == 0)
            {
                const FRESULT close_result = f_close(&bearamin_preset_file);
                bearamin_preset_storage_job = PresetStorageJob::None;
                SendBearaminPresetReadError(bearamin_preset_storage_seq,
                                            result,
                                            close_result,
                                            transferred,
                                            bearamin_preset_io_offset,
                                            bearamin_preset_read_size);
                InvalidatePresetStorage();
                return;
            }
            bearamin_preset_read_crc = mm::PresetCrc16Update(
                bearamin_preset_read_crc,
                bearamin_preset_buffer + bearamin_preset_io_offset,
                transferred);
            bearamin_preset_io_offset += transferred;
            return;
        }

        const FRESULT close_result = f_close(&bearamin_preset_file);
        bearamin_preset_storage_job = PresetStorageJob::None;
        if(close_result != FR_OK)
        {
            SendBearaminPresetReadError(bearamin_preset_storage_seq,
                                        FR_OK,
                                        close_result,
                                        0,
                                        bearamin_preset_io_offset,
                                        bearamin_preset_read_size);
            InvalidatePresetStorage();
            return;
        }
        bearamin_preset_read_active = true;
        SendBearaminPresetReadBegin(bearamin_preset_storage_seq);
        return;
    }

    if(bearamin_preset_storage_job == PresetStorageJob::WriteData)
    {
        if(bearamin_preset_io_offset >= bearamin_preset_write_size)
        {
            bearamin_preset_storage_job = PresetStorageJob::WriteClose;
            return;
        }
        const size_t remaining
            = bearamin_preset_write_size - bearamin_preset_io_offset;
        const UINT request = static_cast<UINT>(
            remaining > kBearaminPresetIoChunkBytes
                ? kBearaminPresetIoChunkBytes
                : remaining);
        UINT transferred = 0;
        const FRESULT result
            = f_write(&bearamin_preset_file,
                      bearamin_preset_buffer + bearamin_preset_io_offset,
                      request,
                      &transferred);
        if(result != FR_OK || transferred != request)
        {
            const FRESULT close_result = f_close(&bearamin_preset_file);
            f_unlink(bearamin_preset_temp_path);
            bearamin_preset_storage_job = PresetStorageJob::None;
            SendBearaminPresetWriteError(bearamin_preset_storage_seq,
                                         result,
                                         close_result,
                                         transferred,
                                         request);
            InvalidatePresetStorage();
            return;
        }
        bearamin_preset_io_offset += transferred;
        return;
    }

    if(bearamin_preset_storage_job == PresetStorageJob::WriteClose)
    {
        const FRESULT result = f_close(&bearamin_preset_file);
        if(result != FR_OK)
        {
            f_unlink(bearamin_preset_temp_path);
            bearamin_preset_storage_job = PresetStorageJob::None;
            SendBearaminPresetWriteError(bearamin_preset_storage_seq,
                                         FR_OK,
                                         result,
                                         0,
                                         bearamin_preset_write_size);
            InvalidatePresetStorage();
            return;
        }
        bearamin_preset_storage_job = PresetStorageJob::WriteRemoveOld;
        return;
    }

    if(bearamin_preset_storage_job == PresetStorageJob::WriteRemoveOld)
    {
        const FRESULT result = f_unlink(bearamin_preset_final_path);
        if(result != FR_OK && result != FR_NO_FILE)
        {
            f_unlink(bearamin_preset_temp_path);
            bearamin_preset_storage_job = PresetStorageJob::None;
            SendBearaminPresetError(bearamin_preset_storage_seq, "unlink");
            return;
        }
        bearamin_preset_storage_job = PresetStorageJob::WriteRename;
        return;
    }

    if(bearamin_preset_storage_job == PresetStorageJob::WriteRename)
    {
        const FRESULT result = f_rename(
            bearamin_preset_temp_path, bearamin_preset_final_path);
        bearamin_preset_storage_job = PresetStorageJob::None;
        if(result != FR_OK)
        {
            f_unlink(bearamin_preset_temp_path);
            SendBearaminPresetError(bearamin_preset_storage_seq, "rename");
            return;
        }
        char line[32];
        snprintf(line,
                 sizeof(line),
                 "BPWR,%u,OK\n",
                 bearamin_preset_storage_seq);
        bearamin_preset_write_completed = true;
        bearamin_preset_write_completed_seq = bearamin_preset_storage_seq;
        SendBearaminPresetLine(line);
    }
}

void ProcessBearaminPresetLine(const char* line)
{
    bearamin_preset_command_ms = System::GetNow();
    bearamin_preset_immediate_tx_available = true;
    char copy[sizeof(bearamin_preset_line)];
    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* command = strtok(copy, ",");
    char* seq_text = strtok(nullptr, ",");
    if(command == nullptr || seq_text == nullptr)
        return;
    const uint16_t seq = static_cast<uint16_t>(strtoul(seq_text, nullptr, 10));

    if(strcmp(command, "BPRQ") == 0)
    {
        char* offset_text = strtok(nullptr, ",");
        if(offset_text != nullptr)
            HandleBearaminPresetReadChunkRequest(
                seq, static_cast<size_t>(strtoul(offset_text, nullptr, 10)));
        return;
    }

    if(strcmp(command, "BPLS") == 0)
    {
        char* kind = strtok(nullptr, ",");
        if(kind != nullptr && strcmp(kind, "READ") == 0)
        {
            HandleBearaminPresetRead(seq, strtok(nullptr, ","));
            return;
        }
        HandleBearaminPresetList(
            seq, kind != nullptr && strcmp(kind, "RETRY") == 0);
        return;
    }
    if(strcmp(command, "BPRD") == 0)
    {
        HandleBearaminPresetRead(seq, strtok(nullptr, ","));
        return;
    }
    if(strcmp(command, "BPWR") == 0)
    {
        char* name = strtok(nullptr, ",");
        char* size_text = strtok(nullptr, ",");
        char* crc_text = strtok(nullptr, ",");
        if(name == nullptr || size_text == nullptr || crc_text == nullptr)
        {
            SendBearaminPresetError(seq, "args");
            return;
        }
        HandleBearaminPresetWriteBegin(
            seq,
            name,
            strtoul(size_text, nullptr, 10),
            static_cast<uint16_t>(strtoul(crc_text, nullptr, 16)));
        return;
    }
    if(strcmp(command, "BPD") == 0)
    {
        char* offset_text = strtok(nullptr, ",");
        char* hex = strtok(nullptr, ",");
        char* chunk_crc_text = strtok(nullptr, ",");
        if(offset_text == nullptr || hex == nullptr || chunk_crc_text == nullptr)
        {
            return;
        }
        HandleBearaminPresetData(
            seq,
            strtoul(offset_text, nullptr, 10),
            hex,
            static_cast<uint16_t>(strtoul(chunk_crc_text, nullptr, 16)));
        return;
    }
    if(strcmp(command, "BPWE") == 0)
    {
        HandleBearaminPresetWriteEnd(seq);
    }
}

void InitBearaminReturnTxSweep()
{
    if(!kBearaminReturnTxSweepDiag)
        return;

    EnableExpSniffPortClocks();
    for(size_t i = 0; i < kBearaminReturnTxSweepPinCount; i++)
        ConfigureExpOutput(kBearaminReturnTxSweepPins[i].pin, false);
    bearamin_return_tx_sweep_index = 0;
    bearamin_return_tx_sweep_level = false;
    bearamin_return_tx_sweep_last_pin_ms = System::GetNow();
    bearamin_return_tx_sweep_last_toggle_ms = bearamin_return_tx_sweep_last_pin_ms;
}

void ServiceBearaminReturnTxSweep()
{
    if(!kBearaminReturnTxSweepDiag)
        return;

    const uint32_t now = System::GetNow();
    if(now - bearamin_return_tx_sweep_last_pin_ms
       >= kBearaminReturnTxSweepPinMs)
    {
        WriteExpPin(kBearaminReturnTxSweepPins[bearamin_return_tx_sweep_index]
                        .pin,
                    false);
        bearamin_return_tx_sweep_index
            = (bearamin_return_tx_sweep_index + 1)
              % kBearaminReturnTxSweepPinCount;
        bearamin_return_tx_sweep_level = false;
        bearamin_return_tx_sweep_last_pin_ms = now;
        bearamin_return_tx_sweep_last_toggle_ms = now;
    }

    if(now - bearamin_return_tx_sweep_last_toggle_ms
       >= kBearaminReturnTxSweepToggleMs)
    {
        bearamin_return_tx_sweep_level = !bearamin_return_tx_sweep_level;
        WriteExpPin(kBearaminReturnTxSweepPins[bearamin_return_tx_sweep_index]
                        .pin,
                    bearamin_return_tx_sweep_level);
        bearamin_return_tx_sweep_last_toggle_ms = now;
        bearamin_return_tx_sweep_toggle_count++;
    }
}

void ScheduleBearaminSingleWireAck()
{
    if(kBearaminReturnTxSweepDiag)
        return;
    if(!kBearaminSingleWireAckEnabled)
        return;
    bearamin_single_wire_ack_pending = true;
}

void ServiceBearaminSingleWireAck()
{
    if(kBearaminReturnTxSweepDiag)
        return;
    if(!kBearaminSingleWireAckEnabled)
        return;

    const uint32_t now = System::GetNow();
    if(bearamin_single_wire_ack_active)
    {
        if(now - bearamin_single_wire_ack_start_ms
           >= kBearaminSingleWireAckPulseMs)
        {
            ConfigureBearaminRxUartPin();
            bearamin_single_wire_ack_active = false;
        }
        return;
    }

    if(!bearamin_single_wire_ack_pending)
        return;

    bearamin_single_wire_ack_pending  = false;
    bearamin_single_wire_ack_active   = true;
    bearamin_single_wire_ack_start_ms = now;
    bearamin_single_wire_ack_count++;
    ConfigureBearaminRxAckPin();
}

void InitBearaminRxSense()
{
    EnableExpSniffPortClocks();
    bearamin_rx_gpio_level = ReadExpPin(DPT::A2);
    bearamin_rx_gpio_low_seen = !bearamin_rx_gpio_level;

    for(size_t i = 0; i < kBearaminSniffPinCount; i++)
    {
        const bool level = ReadExpPin(kBearaminSniffPins[i].pin);
        bearamin_sniff_pin_levels[i]   = level ? 1 : 0;
        bearamin_sniff_pin_low_seen[i] = level ? 0 : 1;
    }
}

void SampleBearaminRxSenseBurst()
{
    for(int i = 0; i < 8; i++)
    {
        const bool level = ReadExpPin(DPT::A2);
        bearamin_rx_gpio_sample_count++;
        if(!level)
            bearamin_rx_gpio_low_seen = true;
        if(level != bearamin_rx_gpio_level)
        {
            bearamin_rx_gpio_level = level;
            bearamin_rx_gpio_transition_count++;
            bearamin_rx_gpio_last_transition_ms = System::GetNow();
        }
        for(size_t pin_index = 0; pin_index < kBearaminSniffPinCount;
            pin_index++)
        {
            const bool sniff_level
                = ReadExpPin(kBearaminSniffPins[pin_index].pin);
            bearamin_sniff_pin_sample_count[pin_index]++;
            if(!sniff_level)
                bearamin_sniff_pin_low_seen[pin_index] = 1;
            if(static_cast<uint8_t>(sniff_level ? 1 : 0)
               != bearamin_sniff_pin_levels[pin_index])
            {
                bearamin_sniff_pin_levels[pin_index]
                    = sniff_level ? 1 : 0;
                bearamin_sniff_pin_transition_count[pin_index]++;
                bearamin_sniff_pin_last_transition_ms[pin_index]
                    = System::GetNow();
            }
        }
    }
}

void InitModeAnnouncements()
{
    TryMountPresetStorage(true);
}

int ConsumePendingAnnouncementMode()
{
    ScopedIrqBlocker block;
    const int        mode = pending_announcement_mode;
    pending_announcement_mode = -1;
    return mode;
}

void SetAnnouncementPlaybackEnabled(bool enabled)
{
    ScopedIrqBlocker block;
    announcement_streaming_enabled = enabled;
    announcement_swap_requested    = false;
}

void PlayModeAnnouncement(int mode_index)
{
    if(!announcements_mounted || !announcement_streaming_supported
       || mode_index < 0 || mode_index >= mm::kModeCount)
    {
        SetAnnouncementPlaybackEnabled(false);
        return;
    }

    const char* path = mm::ModeAnnouncementPath(static_cast<mm::Mode>(mode_index));
    if(mode_announcement.Init(path)
       != WavPlayer<kAnnouncementTransferSize>::Result::Ok)
    {
        SetAnnouncementPlaybackEnabled(false);
        return;
    }

    announcement_channels = mode_announcement.GetChannels();
    mode_announcement.SetLooping(false);
    mode_announcement.Restart();
    if(mode_announcement.Prepare()
       != WavPlayer<kAnnouncementTransferSize>::Result::Ok)
    {
        SetAnnouncementPlaybackEnabled(false);
        return;
    }

    SetAnnouncementPlaybackEnabled(true);
}

void ServiceModeAnnouncements()
{
    if(BearaminPresetStorageBusy())
        return;
    if((RawBearaminPacket().flags & mm::kBearaminModeAnnouncementsEnabled) == 0)
    {
        ConsumePendingAnnouncementMode();
        SetAnnouncementPlaybackEnabled(false);
        return;
    }

    const int pending_mode = ConsumePendingAnnouncementMode();
    if(pending_mode >= 0)
        PlayModeAnnouncement(pending_mode);

    if(announcement_streaming_enabled)
    {
        const auto result = mode_announcement.Prepare();
        if(result == WavPlayer<kAnnouncementTransferSize>::Result::DiskError
           || result
                  == WavPlayer<kAnnouncementTransferSize>::Result::PrepareOverrun)
        {
            SetAnnouncementPlaybackEnabled(false);
        }
    }
}

void PollBearaminBridgeUart()
{
    if(kBearaminUartRxDmaEnabled)
        return;

    if(bearamin_single_wire_ack_active)
        return;

    for(int i = 0; i < 256; i++)
    {
        uint8_t byte = 0;
        if(!TryReceiveBearaminByte(byte))
            return;
        AcceptBearaminBridgeByte(byte);
    }
}

void SendBearaminBridgeStatus()
{
    if(kBearaminReturnTxSweepDiag)
        return;
    if(bearamin_single_wire_ack_active || !BearaminPresetTxQueueEmpty())
        return;

    const uint32_t now = System::GetNow();
    if(bearamin_last_probe_ms != 0
       && now - bearamin_last_probe_ms < kBearaminDptProbeSendMs)
    {
        return;
    }
    if(bearamin_rx_last_byte_ms != 0 && now - bearamin_rx_last_byte_ms < 15)
        return;
    bearamin_last_probe_ms = now;

    uint8_t probe[mm::kBearaminDptProbeSize];
    for(size_t i = 0; i < mm::kBearaminDptProbeSize; i++)
        probe[i] = mm::kBearaminDptProbe[i];
    DirectTransmitBearaminBytes(probe, sizeof(probe), 5);

    const uint8_t mode_index = static_cast<uint8_t>(current_mode);
    uint8_t controls[mm::kDptControlCount];
    for(int i = 0; i < mm::kDptControlCount; i++)
        controls[i]
            = mm::QuantizeDptPanelPosition(patch.GetAdcValue(cv_inputs[i]));

    const bool bearamin_fresh
        = current_mode == mm::Mode::BearaminBridge
          && BearaminBridgeFresh(now);
    if(bearamin_bridge_was_fresh && !bearamin_fresh)
        bearamin_stale_transition_count++;
    bearamin_bridge_was_fresh = bearamin_fresh;
    const bool rx_recent = bearamin_rx_last_byte_ms != 0
                           && now - bearamin_rx_last_byte_ms
                                  <= kBearaminRxActivityHoldMs;
    const bool rx_gpio_recent
        = bearamin_rx_gpio_last_transition_ms != 0
          && now - bearamin_rx_gpio_last_transition_ms
                 <= kBearaminRxActivityHoldMs;
    const uint16_t status_flags = mm::BearaminDiagnosticStatusFlags(
        BearaminSettingsFresh(now),
        bearamin_fresh,
        bearamin_rx_gpio_level,
        bearamin_rx_gpio_low_seen,
        bearamin_rx_gpio_transition_count > 0,
        rx_gpio_recent || rx_recent,
        bearamin_last_packet_ms != 0);

    char status[40];
    const size_t status_length = mm::EncodeDptStatusLine(
        mode_index,
        controls,
        status_flags,
        RawBearaminPacket().resonator_controls_revision,
        status,
        sizeof(status));
    if(status_length > 0)
    {
        DirectTransmitBearaminBytes(
            reinterpret_cast<uint8_t*>(status), status_length, 5);
    }

    if(bearamin_last_performance_ms == 0
       || now - bearamin_last_performance_ms >= kBearaminPerformanceSendMs)
    {
        bearamin_last_performance_ms = now;
        const mm::DptPerformanceSnapshot snapshot
            = CaptureDptPerformanceSnapshot();
        char performance[96];
        const size_t performance_length = mm::EncodeDptPerformanceLine(
            snapshot, performance, sizeof(performance));
        if(performance_length > 0)
        {
            DirectTransmitBearaminBytes(
                reinterpret_cast<uint8_t*>(performance),
                performance_length,
                25);
        }
    }

    if(bearamin_last_trajectory_diagnostics_ms == 0
       || now - bearamin_last_trajectory_diagnostics_ms
              >= kBearaminPerformanceSendMs)
    {
        bearamin_last_trajectory_diagnostics_ms = now;
        const mm::BearaminBridgePacket& raw = RawBearaminPacket();
        const mm::DptPerformanceSnapshot snapshot
            = CaptureDptPerformanceSnapshot();
        mm::BearaminTrajectoryDiagnostics diagnostics = {};
        diagnostics.protocol_version = raw.protocol_version;
        diagnostics.slew_tau_ms
            = raw.protocol_version == mm::kBearaminVersionV11
                  ? raw.slew_tau_ms
                  : 0;
        diagnostics.source_epoch_ms = raw.source_epoch_ms;
        diagnostics.source_age_ms = raw.age_ms;
        diagnostics.v10_packet_count = bearamin_v10_packet_count;
        diagnostics.v11_packet_count = bearamin_v11_packet_count;
        diagnostics.correction_count = bearamin_trajectory_correction_count;
        diagnostics.max_correction_norm
            = bearamin_trajectory_max_correction_norm;
        diagnostics.invalid_keyframe_count = bearamin_invalid_trajectory_count;
        diagnostics.stale_transitions = bearamin_stale_transition_count;
        diagnostics.deadline_misses = snapshot.deadline_misses;
        char trajectory_diagnostics[96] = {};
        const size_t trajectory_diagnostics_length
            = mm::EncodeBearaminTrajectoryDiagnosticsLine(
                diagnostics,
                trajectory_diagnostics,
                sizeof(trajectory_diagnostics));
        if(trajectory_diagnostics_length > 0)
        {
            DirectTransmitBearaminBytes(
                reinterpret_cast<uint8_t*>(trajectory_diagnostics),
                trajectory_diagnostics_length,
                25);
        }
    }
}

} // namespace

int main(void)
{
    patch.Init();
    patch.SetAudioSampleRate(48000);
    patch.SetAudioBlockSize(48);
    SD_SetIoTimeout(kBearaminSdIoTimeoutMs);
    InitBearaminCvDac();

    InitBearaminBridgeUart();
    InitBearaminRxSense();
    InitModeAnnouncements();
    InitBearaminReturnTxSweep();
    resonator.Init(resonator_reverb_buffer);
    resonator.set_model(rings::RESONATOR_MODEL_SYMPATHETIC_STRING);
    resonator.set_polyphony(1);
    resonator_reverb.Init(resonator_reverb_buffer);
    InitDptPerformanceTelemetry();
    patch.StartAudio(AudioCallback);

    while(1)
    {
        SampleBearaminRxSenseBurst();
        ServiceCvExpanderOutputs();
        ServiceBearaminSingleWireAck();
        PollBearaminBridgeUart();
        ServiceBearaminPresetTx();
        ServiceBearaminPresetStorage();
        ServiceBearaminPresetTx();
        ServiceBearaminReturnTxSweep();
        SendBearaminBridgeStatus();
        ServiceModeAnnouncements();
    }
}
