#if defined(BEARAMIN_TARGET_MATRIX_VL53L8CX)
#include "targets/vl53l8cx_s3.h"
#elif defined(BEARAMIN_TARGET_MATRIX_VL53L7CX_REVC)
#include "targets/vl53l7cx_revc_s3.h"
#else
#include "targets/vl53l7cx_s3.h"
#endif

#include <Arduino.h>
#include <ArduinoOTA.h>
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
#include <vl53l8cx.h>
#else
#include <Adafruit_VL53L7CX.h>
#endif
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <esp_timer.h>
#include <nvs.h>
#include "../../../common/default_preset_store.h"

#include "../../../../third_party/bearfield-core/include/bearfield/bearfield.h"
#include "../../../common/chaos_lorenz.h"

#include "../../../common/capture_protocol.h"
#include "../../../common/direction_logic.h"
#include "../../../common/bridge_protocol.h"
#include "../../../common/cv_logic.h"
#include "../../../common/orientation_logic.h"
#include "../../../common/preset_quiet.h"
#include "../../../common/sensor_recovery.h"
#include "../../../common/sidecar_protocol.h"
#include "../../../common/trajectory_keyframe.h"
#include "../../../common/wifi_band_logic.h"
#include "esp32-hal-rmt.h"

#if __has_include("../../../common/wifi_config.h")
#include "../../../common/wifi_config.h"
#endif

uint16_t capAgeMs(uint32_t timestampMs);

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

#ifndef MATRIXCORE_MDNS_HOSTNAME
#define MATRIXCORE_MDNS_HOSTNAME "bearamin-matrixcore"
#endif

#ifndef BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS
#define BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS 1
#endif

#ifndef BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X
#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X 0
#endif

#ifndef BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y
#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y 1
#endif

#ifndef BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK
#define BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK UINT64_MAX
#endif

#ifndef BEARAMIN_TRUE_GRID_CENTROID_MIN_X
#define BEARAMIN_TRUE_GRID_CENTROID_MIN_X 0
#endif

#ifndef BEARAMIN_TRUE_GRID_CENTROID_MAX_X
#define BEARAMIN_TRUE_GRID_CENTROID_MAX_X 7
#endif

#ifndef BEARAMIN_TRUE_GRID_CENTROID_MIN_Y
#define BEARAMIN_TRUE_GRID_CENTROID_MIN_Y 0
#endif

#ifndef BEARAMIN_TRUE_GRID_CENTROID_MAX_Y
#define BEARAMIN_TRUE_GRID_CENTROID_MAX_Y 7
#endif

#ifndef BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS 1
#endif

#ifndef BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X 0
#endif

#ifndef BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y 0
#endif

#ifndef BEARAMIN_STATUS_USES_DISPLAY_PROFILE
#define BEARAMIN_STATUS_USES_DISPLAY_PROFILE 0
#endif

#ifndef BEARAMIN_OPTICAL_CLOCK_HZ
#define BEARAMIN_OPTICAL_CLOCK_HZ 400000
#endif

#ifndef BEARAMIN_MERGED_MATRIX_SENSOR_STATUS
#define BEARAMIN_MERGED_MATRIX_SENSOR_STATUS 0
#endif

using espmatrix::AccelSample;
using espmatrix::CentroidMilli;
using espmatrix::ImpactDetector;
using espmatrix::ImpactState;
using espmatrix::LiquidColor;
using espmatrix::blendColorMilli;
using espmatrix::clampByte;
using espmatrix::depthBrightnessForDistance;
using espmatrix::logicalColumnForPhysicalVerticalFlip;
using espmatrix::midpointMarkerBlendMilli;
using espmatrix::nearestLedCoordinateForMilli;
using espmatrix::liquidRainbowColor;
using espmatrix::rotatedClockwisePixelIndex;
using espmatrix::rotatedClockwisePixelIndex180;
using espmatrix::scaleChannel;
using espmatrix::transposeCentroidMilli;
using espmatrix::unavailableImpactState;
using bearamin_sensor_recovery::nextRetryMs;
using bearamin_sensor_recovery::sensorMonitoringStartMs;
using bearamin_sensor_recovery::streamNeedsRecovery;

constexpr uint8_t kLedPin = 14;
constexpr uint8_t kImuSdaPin = 11;
constexpr uint8_t kImuSclPin = 12;
constexpr uint8_t kOpticalSdaPin = 1;
constexpr uint8_t kOpticalSclPin = 2;
#ifndef BEARAMIN_MATRIX_LPR_PIN
#define BEARAMIN_MATRIX_LPR_PIN 5
#endif
constexpr uint8_t kMatrixLprPin = BEARAMIN_MATRIX_LPR_PIN;
constexpr uint8_t kMatrixAuxPin = 4;
constexpr uint8_t kDptTxPin = 33;
constexpr uint8_t kDptRxPin = 34;
constexpr uint8_t kSidecarTxPin = 35;
constexpr uint8_t kSidecarRxPin = 36;

constexpr uint8_t kMatrixSensorAddress = 0x29;
constexpr uint8_t kQmiAddress = 0x6B;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kDptBaud = 38400;
constexpr uint32_t kSidecarBaud = 460800;
constexpr uint32_t kOpticalClockHz = BEARAMIN_OPTICAL_CLOCK_HZ;
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
constexpr bool kMatrixUsesVl53l8cx = true;
constexpr const char *kMatrixSensorName = "VL53L8CX";
#else
constexpr bool kMatrixUsesVl53l8cx = false;
constexpr const char *kMatrixSensorName = "VL53L7CX";
#endif
constexpr uint8_t kMatrixSourceHz = 15;
constexpr uint8_t kProcessedOutputHz = 30;
constexpr uint32_t kDptSendMs = 1000 / kMatrixSourceHz;
constexpr uint32_t kDptProbeMs = 250;
constexpr uint32_t kDptRxTxGuardMs = 2;
constexpr bool kDptContinuousTxEnabled = true;
constexpr uint32_t kSidecarStateMs = 1000 / kMatrixSourceHz;
constexpr uint32_t kSidecarCellsMs = 100;
constexpr uint32_t kSidecarSensorDiagnosticsMs = 1000;
constexpr size_t kCaptureRingBytes = 16 * 1024;
constexpr size_t kCaptureDrainMaxBytes = 512;
constexpr size_t kCapturePriorityReserveBytes = 256;
constexpr size_t kSidecarTxBufferBytes = 1024;
constexpr uint32_t kSidecarStatusStaleMs = 1500;
constexpr uint32_t kMatrixOtaTimeoutMs = 90UL * 1000UL;
constexpr uint32_t kMatrixOtaBlinkMs = 180;
constexpr uint32_t kDiagnosticLogMs = 1000;
constexpr uint32_t kDptRxBlinkMs = 140;
constexpr uint32_t kMatrixReadyRetryUs = 500;
constexpr uint32_t kMatrixReadyLeadUs = 4000;
constexpr uint32_t kImuPollMs = 2;
constexpr uint32_t kImuRetryMinMs = 250;
constexpr uint32_t kImuRetryMaxMs = 10000;
constexpr uint32_t kSensorRecoveryMs = 1000;
constexpr uint32_t kSensorFirstSampleMs = 1500;
constexpr uint32_t kSensorRetryMinMs = 5000;
constexpr uint32_t kSensorRetryMaxMs = 300000;
constexpr uint16_t kPlayableMaxMm = 600;
constexpr uint16_t kFadeMaxMm = 750;
constexpr uint16_t kMatrixMaxValidMm = 2000;
constexpr uint16_t kMatrixMinMusicMm = 60;
constexpr uint16_t kMatrixDepthOffsetMm = 45;
constexpr uint16_t kMatrixMaxMusicMm = kFadeMaxMm - kMatrixDepthOffsetMm;
constexpr uint16_t kMatrixBlobWindowMm = 250;
constexpr uint8_t kMatrixMinBlobCells = 4;
constexpr uint8_t kMatrixStatusRangingOk = 5;
constexpr uint8_t kMatrixStatusRangingOkLargePulse = 9;
constexpr uint16_t kMaxBridgeValue = 4095;
constexpr uint16_t kCvSlewMaxStep = 32;
constexpr uint32_t kTrajectoryStaleMs = 250;
constexpr uint8_t kLedCount = 64;
constexpr uint8_t kMatrixWidth = 8;
constexpr uint8_t kBrightnessPercent = 12;
constexpr uint8_t kStatusVectorDotBrightness = 220;
constexpr uint8_t kStatusVectorTailMinBrightness = 48;
constexpr uint8_t kStatusVectorIdleDotBrightness = 72;
constexpr uint8_t kStatusVectorIdleTailMinBrightness = 18;
constexpr uint8_t kStatusBackgroundBrightness = 6;
constexpr bool kMergedMatrixSensorStatus =
    BEARAMIN_MERGED_MATRIX_SENSOR_STATUS != 0;
constexpr uint16_t kStatusBackgroundMaxMm = 500;
constexpr uint16_t kStatusVectorYellowMm = 75;
constexpr uint16_t kRmtSymbols = kLedCount * 24;
constexpr float kAccelScale16g = 16.0f / 32768.0f;
constexpr float kGyroScale128dps = 128.0f / 32768.0f;

constexpr uint8_t kRegWhoAmI = 0x00;
constexpr uint8_t kRegRevision = 0x01;
constexpr uint8_t kRegCtrl1 = 0x02;
constexpr uint8_t kRegCtrl2 = 0x03;
constexpr uint8_t kRegCtrl3 = 0x04;
constexpr uint8_t kRegCtrl5 = 0x06;
constexpr uint8_t kRegCtrl7 = 0x08;
constexpr uint8_t kRegAccelXL = 0x35;
constexpr uint8_t kRegGyroXL = 0x3B;
constexpr uint8_t kRegReset = 0x60;
constexpr uint16_t kVl53l7cxIdReg = 0x010F;

struct Color {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

struct ImuSample {
  AccelSample accel;
  float gyroX;
  float gyroY;
  float gyroZ;
  int16_t rawAccelX;
  int16_t rawAccelY;
  int16_t rawAccelZ;
  int16_t rawGyroX;
  int16_t rawGyroY;
  int16_t rawGyroZ;
};

struct OpticalState {
  bool busReady = false;
  bool matrixReady = false;
  bool matrixPresent = false;
  bool syntheticGate = false;
  uint16_t matrixZMm = 0;
  uint16_t matrixXMilli = 500;
  uint16_t matrixYMilli = 500;
  uint16_t matrixZNorm = cvBridgeCenterValue(kMaxBridgeValue);
  uint16_t matrixXNorm = cvBridgeCenterValue(kMaxBridgeValue);
  uint16_t matrixYNorm = cvBridgeCenterValue(kMaxBridgeValue);
  uint16_t pitchNorm = 2048;
  uint16_t rollNorm = 2048;
  uint16_t orientationMedianDepthMm = 0;
  uint8_t orientationConfidence = 0;
  bool spankGate = false;
  uint16_t orientationFitUs = 0;
  uint16_t orientationFitMaxUs = 0;
  uint8_t matrixValidCount = 0;
  uint8_t matrixBlobCount = 0;
  uint16_t cellsMm[64] = {};
  uint8_t statuses[64] = {};
  uint32_t lastMatrixReadMs = 0;
  uint32_t matrixMonitoringStartedMs = 0;
  uint32_t lastRecoveryMs = 0;
  uint16_t i2cRecoveries = 0;
};

enum class OpticalRecoveryJob : uint8_t {
  None,
  MatrixOnly,
  FullBus,
};

enum class OpticalRecoveryPhase : uint8_t {
  Idle,
  Queued,
  Running,
  Complete,
};

Color ledFrame[kLedCount];
rmt_data_t ledData[kRmtSymbols];
TwoWire opticalWire = TwoWire(1);
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
VL53L8CX matrixSensor(&opticalWire, kMatrixLprPin, kMatrixAuxPin);
VL53L8CX_ResultsData matrixResults;
#else
Adafruit_VL53L7CX matrixSensor;
VL53L7CX_ResultsData matrixResults;
#endif
bool matrixRangingStarted = false;
HardwareSerial DptSerial(1);
HardwareSerial SidecarSerial(2);
WebServer matrixIdentityServer(80);
bool matrixIdentityRoutesConfigured = false;
bool matrixIdentityServerReady = false;
bearamin_bridge::DptProbeParser dptProbeParser;
bearamin_bridge::DptStatus dptStatus = {};
bearamin_bridge::DptTrajectoryDiagnostics dptTrajectoryDiagnostics = {};
bearamin_sidecar::Parser sidecarParser;
ImpactDetector impactDetector;
bearamin_chaos::StartupUnlock chaosUnlock;
bearamin_chaos::Lorenz chaosLorenz;
bearamin_chaos::Axes chaosAxes = {};
float chaosTofInfluence = 0.0f;
uint64_t chaosLastUs = 0;
uint32_t chaosRainbowUntilMs = 0;
bool previousImpactGate = false;
bearamin_orientation::OrientationTracker orientationTracker;
bearamin_orientation::SpankDetector spankDetector;
bearfield::FeelProfile matrixFeelProfile() {
  bearfield::FeelProfile profile = bearfield::BearaminV2();
  // The LED renderer maps unipolar X/Y 0 to its lower-right corner. A real
  // no-presence frame therefore returns X/Y to its visual midpoint, while
  // unipolar depth returns to zero.
  profile.idleAxisNorm = bearfield::kNormCenter;
  profile.idleDepthNorm = 0;
  profile.maximumSourceDepthMm = kMatrixMaxMusicMm;
  profile.depthOffsetMm = kMatrixDepthOffsetMm;
  profile.centroidMinimumX = BEARAMIN_TRUE_GRID_CENTROID_MIN_X;
  profile.centroidMaximumX = BEARAMIN_TRUE_GRID_CENTROID_MAX_X;
  profile.centroidMinimumY = BEARAMIN_TRUE_GRID_CENTROID_MIN_Y;
  profile.centroidMaximumY = BEARAMIN_TRUE_GRID_CENTROID_MAX_Y;
  return profile;
}
bearfield::Processor fieldProcessor(matrixFeelProfile());
bearfield::ProcessedFieldState processedField = {};
bearamin_trajectory::Authority trajectoryAuthority;
bearamin_trajectory::EvaluatedAxes trajectoryOutputAxes = {};
bool trajectoryOutputValid = false;
OpticalState optical;
OpticalState opticalRecoveryWorking;
volatile OpticalRecoveryPhase opticalRecoveryPhase = OpticalRecoveryPhase::Idle;
OpticalRecoveryJob opticalRecoveryJob = OpticalRecoveryJob::None;
bool opticalRecoveryAnyReady = false;
bool opticalRecoveryAllReady = false;
uint8_t opticalRecoveryBusFlags = 0;
uint32_t opticalRecoveryLastDurationMs = 0;
uint32_t opticalRecoveryMaxDurationMs = 0;
StaticTask_t opticalRecoveryTaskBuffer;
StackType_t opticalRecoveryTaskStack[8192];
TaskHandle_t opticalRecoveryTaskHandle = nullptr;
portMUX_TYPE opticalRecoveryMux = portMUX_INITIALIZER_UNLOCKED;
ImuSample latestImu = {};
ImpactState latestImpact = {};
uint16_t bridgeSequence = 0;
uint16_t sidecarSequence = 0;
uint16_t sidecarCommandSequence = 0;
uint16_t sidecarPresetSequence = 0;
uint16_t sidecarFrameErrors = 0;
uint16_t cvBridgeX = cvBridgeCenterValue(kMaxBridgeValue);
uint16_t cvBridgeY = cvBridgeCenterValue(kMaxBridgeValue);
uint16_t cvBridgeZ = cvBridgeCenterValue(kMaxBridgeValue);
uint16_t cvSlewStep = 16;
uint16_t dptControlFlags = 0;
uint16_t dptReserved0 = 0;
uint8_t dptReserved1 = 0;
uint8_t dptReserved2 = 0;
uint8_t dptSourceFlags2 = 0;
int8_t dptLaneDepths[bearamin_bridge::kLaneDepthCount] = {50, 50, 50, 50, 50, 50};
uint8_t resonatorControlsRevision = 0;
uint8_t resonatorWebControls[bearamin_bridge::kResonatorWebControlSize] = {};
bearamin_sidecar::ResonatorWebPageAssembler resonatorWebPageAssembler;
bool modeAnnouncementsEnabled = false;
bool sidecarControlsInitialized = false;
uint16_t sidecarStatusFlags = 0;
uint32_t lastDptSendMs = 0;
uint32_t lastDptSourceSequenceSent = 0;
uint32_t lastDptProbeMs = 0;
uint32_t lastDptRxMs = 0;
uint32_t lastDptWireRxByteMs = 0;
uint32_t lastSidecarStateMs = 0;
uint32_t lastSidecarCellsMs = 0;
uint32_t lastSidecarSensorDiagnosticsMs = 0;
uint32_t lastSidecarMatrixPerformanceMs = 0;
uint32_t lastSidecarStatusMs = 0;
uint32_t sidecarRxByteCount = 0;
uint32_t sidecarCommandFrameCount = 0;
uint32_t matrixOtaModeStartedMs = 0;
uint32_t matrixOtaLastProgressMs = 0;
uint32_t lastDiagnosticLogMs = 0;
uint32_t lastSidecarConsoleLogMs = 0;
uint64_t nextMatrixReadyCheckUs = 0;
uint64_t lastMatrixFrameUs = 0;
uint64_t lastProcessedOutputUs = 0;
uint32_t matrixSourceSequence = 0;
uint64_t matrixRateWindowStartUs = 0;
uint32_t matrixRateWindowFrames = 0;
uint32_t matrixEffectiveRateMilliHz = 0;
uint32_t matrixIntervalMaxUs = 0;
uint32_t matrixReadMaxUs = 0;
uint32_t matrixProcessMaxUs = 0;
uint32_t matrixLoopGapMaxUs = 0;
uint64_t lastLoopUs = 0;
uint32_t matrixReadErrors = 0;
uint8_t matrixReadyPollStatus = 0;
uint8_t matrixReadyPollValue = 0;
uint16_t matrixRecoveryStackMinimumWords = 0xFFFFu;
uint32_t lastOpticalRecoveryAttemptMs = 0;
uint32_t opticalRecoveryRetryMs = kSensorRetryMinMs;
uint16_t opticalRecoveryFailures = 0;
uint8_t lastOpticalRecoveryReason = bearamin_sidecar::SENSOR_RECOVERY_BOOT;
uint8_t lastRecoveryBusFlags = 0;
uint32_t lastImuPollMs = 0;
uint32_t nextImuRetryMs = 0;
uint32_t imuRetryMs = kImuRetryMinMs;
uint32_t imuReadFailureCount = 0;
uint32_t imuRecoveryCount = 0;
bool ledReady = false;
bool imuReady = false;
bool matrixOtaMode = false;
bool matrixOtaWifiReady = false;
bool matrixOtaServicesReady = false;
bool matrixOtaStarted = false;
bool matrixOtaActive = false;
bool engineeringStreamEnabled = false;
uint32_t dptProbeTxCount = 0;
uint32_t dptFrameTxCount = 0;
uint32_t dptPresetTxCount = 0;
uint32_t dptRxByteCount = 0;
uint32_t dptStatusLineCount = 0;
uint32_t dptTrajectoryDiagnosticsLineCount = 0;
uint32_t dptPresetLineCount = 0;
uint32_t lastDptRxDiagMs = 0;
uint32_t lastDptConsoleLogMs = 0;
uint32_t dptPresetQuietUntilMs = 0;
uint8_t dptRxDebugBytes[32] = {};
uint8_t dptRxDebugWriteIndex = 0;
uint8_t dptRxDebugCount = 0;
bearamin_capture::ByteRing<kCaptureRingBytes> captureRing;
bool captureActive = false;
bool captureStopping = false;
uint32_t captureSessionToken = 0;
uint32_t captureSequence = 0;
uint32_t captureRecordCounts[bearamin_capture::kRecordTypeCount] = {};
uint32_t captureDropCounts[bearamin_capture::kRecordTypeCount] = {};
uint32_t captureTotalDrops = 0;
uint32_t capturePendingDropCount = 0;
uint32_t capturePendingDropFirstSequence = 0;
uint32_t capturePendingDropLastSequence = 0;
uint8_t capturePendingDropType = 0;
uint32_t captureLastFinalSequence = 0;
uint8_t captureImuPayload[bearamin_capture::kMaxImuBatchPayloadSize] = {};
uint8_t captureImuCount = 0;
uint64_t captureImuBatchStartUs = 0;

uint64_t captureCoreUs() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

void captureNoteDrop(uint8_t type, uint32_t sequence) {
  if (type < bearamin_capture::kRecordTypeCount) {
    ++captureDropCounts[type];
  }
  ++captureTotalDrops;
  if (capturePendingDropCount == 0) {
    capturePendingDropFirstSequence = sequence;
    capturePendingDropType = type;
  } else if (capturePendingDropType != type) {
    capturePendingDropType = 0;
  }
  capturePendingDropLastSequence = sequence;
  ++capturePendingDropCount;
}

bool capturePushEncoded(uint8_t type,
                        uint16_t flags,
                        uint32_t sequence,
                        uint64_t coreUs,
                        const uint8_t *payload,
                        uint16_t payloadSize) {
  uint8_t record[bearamin_capture::kMaxRecordSize] = {};
  const size_t size = bearamin_capture::encodeRecord(
      type,
      flags,
      sequence,
      coreUs,
      payload,
      payloadSize,
      record,
      sizeof(record));
  if (size == 0 || !captureRing.TryPush(record, size)) {
    return false;
  }
  if (type < bearamin_capture::kRecordTypeCount) {
    ++captureRecordCounts[type];
  }
  return true;
}

void captureFlushPendingGap() {
  if (!captureActive || capturePendingDropCount == 0) {
    return;
  }
  uint8_t payload[bearamin_capture::kGapPayloadSize] = {};
  payload[0] = capturePendingDropType;
  payload[1] = bearamin_capture::GAP_MATRIXCORE_RING;
  bearamin_capture::writeLe32(payload, 4, capturePendingDropFirstSequence);
  bearamin_capture::writeLe32(payload, 8, capturePendingDropLastSequence);
  bearamin_capture::writeLe32(payload, 12, capturePendingDropCount);
  const uint32_t sequence = captureSequence;
  if (!capturePushEncoded(
          bearamin_capture::RECORD_GAP,
          bearamin_capture::FLAG_NONE,
          sequence,
          captureCoreUs(),
          payload,
          sizeof(payload))) {
    return;
  }
  ++captureSequence;
  capturePendingDropCount = 0;
  capturePendingDropFirstSequence = 0;
  capturePendingDropLastSequence = 0;
  capturePendingDropType = 0;
}

bool captureRecord(uint8_t type,
                   const uint8_t *payload,
                   uint16_t payloadSize,
                   uint16_t flags = bearamin_capture::FLAG_NONE) {
  if (!captureActive || captureStopping) {
    return false;
  }
  captureFlushPendingGap();
  const uint32_t sequence = captureSequence++;
  if (capturePushEncoded(
          type, flags, sequence, captureCoreUs(), payload, payloadSize)) {
    return true;
  }
  captureNoteDrop(type, sequence);
  return false;
}

void captureControlRecord(uint8_t kind,
                          const uint8_t *data,
                          size_t length) {
  if (data == nullptr || length > bearamin_capture::kMaxPayloadSize - 2) {
    return;
  }
  uint8_t payload[bearamin_capture::kMaxPayloadSize] = {};
  payload[0] = kind;
  payload[1] = static_cast<uint8_t>(length);
  for (size_t i = 0; i < length; ++i) {
    payload[2 + i] = data[i];
  }
  captureRecord(
      bearamin_capture::RECORD_DPT_CONTROL,
      payload,
      static_cast<uint16_t>(length + 2));
}

void captureStart(uint32_t sessionToken) {
  captureRing.Clear();
  captureActive = true;
  captureStopping = false;
  captureSessionToken = sessionToken;
  captureSequence = 0;
  captureTotalDrops = 0;
  capturePendingDropCount = 0;
  capturePendingDropFirstSequence = 0;
  capturePendingDropLastSequence = 0;
  capturePendingDropType = 0;
  captureImuCount = 0;
  captureImuBatchStartUs = 0;
  for (uint8_t i = 0; i < bearamin_capture::kRecordTypeCount; ++i) {
    captureRecordCounts[i] = 0;
    captureDropCounts[i] = 0;
  }

  uint8_t payload[bearamin_capture::kSessionStartPayloadSize] = {};
  bearamin_capture::writeLe16(payload, 0, bearamin_capture::kSchemaVersion);
  bearamin_capture::writeLe16(payload, 2, bearamin_sidecar::kVersion);
  bearamin_capture::writeLe32(payload, 4, sessionToken);
  bearamin_capture::writeLe32(payload, 8, static_cast<uint32_t>(kCaptureRingBytes));
  bearamin_capture::writeLe32(payload, 12, kSidecarBaud);
  bearamin_capture::writeLe16(payload, 16, bearamin_bridge::kVersion);
  bearamin_capture::writeLe16(payload, 18, kMaxBridgeValue);
  bearamin_capture::writeLe32(payload, 20, millis());
  const uint32_t sequence = captureSequence++;
  if (!capturePushEncoded(
          bearamin_capture::RECORD_SESSION_START,
          bearamin_capture::FLAG_NONE,
          sequence,
          captureCoreUs(),
          payload,
          sizeof(payload))) {
    captureNoteDrop(bearamin_capture::RECORD_SESSION_START, sequence);
  }

  uint8_t metadata[32] = {};
  bearamin_capture::writeLe16(metadata, 0, bearamin_capture::kSchemaVersion);
  bearamin_capture::writeLe16(metadata, 2, bearamin_sidecar::kVersion);
  bearamin_capture::writeLe16(metadata, 4, bearamin_bridge::kVersion);
  metadata[6] = static_cast<uint8_t>(sizeof(bearamin_sidecar::StateFrame));
  metadata[7] =
      static_cast<uint8_t>(sizeof(bearamin_capture::ProcessedSensorSnapshotV2));
  const char buildStamp[] = __DATE__ " " __TIME__;
  for (size_t i = 0; i < sizeof(buildStamp) - 1 && i < 24; ++i) {
    metadata[8 + i] = static_cast<uint8_t>(buildStamp[i]);
  }
  captureRecord(
      bearamin_capture::RECORD_CAPTURE_METADATA,
      metadata,
      sizeof(metadata));
}

void captureRequestStop() {
  if (captureActive) {
    if (captureImuCount > 0) {
      captureImuPayload[0] = captureImuCount;
      captureImuPayload[1] = imuReady ? 1 : 0;
      bearamin_capture::writeLe16(
          captureImuPayload, 2, latestImpact.velocityNorm);
      bearamin_capture::writeLe16(
          captureImuPayload, 4, latestImpact.ageMs);
      captureImuPayload[6] = latestImpact.gate ? 1 : 0;
      captureImuPayload[7] = 0;
      captureRecord(
          bearamin_capture::RECORD_IMU_BATCH,
          captureImuPayload,
          static_cast<uint16_t>(
              bearamin_capture::kImuBatchHeaderSize +
              captureImuCount * bearamin_capture::kImuSampleSize),
          bearamin_capture::FLAG_PARTIAL);
      captureImuCount = 0;
      captureImuBatchStartUs = 0;
    }
    captureStopping = true;
  }
}

void captureFinishStopIfPossible() {
  if (!captureActive || !captureStopping) {
    return;
  }
  captureStopping = false;
  captureFlushPendingGap();
  captureStopping = true;
  if (capturePendingDropCount != 0 ||
      captureRing.Free() <
          bearamin_capture::kRecordOverhead +
              bearamin_capture::kSessionEndPayloadSize) {
    return;
  }

  uint8_t payload[bearamin_capture::kSessionEndPayloadSize] = {};
  bearamin_capture::writeLe16(payload, 0, bearamin_capture::kSchemaVersion);
  bearamin_capture::writeLe16(payload, 2, 10);
  bearamin_capture::writeLe32(payload, 4, captureSequence);
  uint32_t totalRecords = 1;
  for (uint8_t i = 1; i < bearamin_capture::kRecordTypeCount; ++i) {
    totalRecords += captureRecordCounts[i];
    if (i <= 10) {
      bearamin_capture::writeLe32(
          payload, 12 + (i - 1) * 4, captureDropCounts[i]);
    }
  }
  bearamin_capture::writeLe32(payload, 8, totalRecords);
  bearamin_capture::writeLe32(payload, 52, captureTotalDrops);
  const uint32_t sequence = captureSequence;
  if (!capturePushEncoded(
          bearamin_capture::RECORD_SESSION_END,
          bearamin_capture::FLAG_NONE,
          sequence,
          captureCoreUs(),
          payload,
          sizeof(payload))) {
    return;
  }
  ++captureSequence;
  captureLastFinalSequence = sequence;
  captureActive = false;
  captureStopping = false;
  captureImuCount = 0;
}

void captureDrainToSidecar() {
  size_t drained = 0;
  while (captureRing.Used() >= bearamin_capture::kRecordOverhead &&
         drained < kCaptureDrainMaxBytes) {
    uint8_t header[bearamin_capture::kHeaderSize] = {};
    if (captureRing.Peek(header, sizeof(header)) != sizeof(header) ||
        !bearamin_capture::hasMagic(header, sizeof(header)) ||
        header[4] != bearamin_capture::kVersion) {
      captureRing.Clear();
      return;
    }
    const size_t recordSize = bearamin_capture::recordSizeForPayload(
        bearamin_capture::readLe16(header, 8));
    if (recordSize == 0 ||
        captureRing.Used() < recordSize ||
        drained + recordSize > kCaptureDrainMaxBytes) {
      return;
    }
    const int available = SidecarSerial.availableForWrite();
    if (available <= static_cast<int>(kCapturePriorityReserveBytes) ||
        static_cast<size_t>(available) -
                kCapturePriorityReserveBytes <
            recordSize) {
      return;
    }
    uint8_t record[bearamin_capture::kMaxRecordSize] = {};
    if (captureRing.Peek(record, recordSize) != recordSize) {
      return;
    }
    const size_t written = SidecarSerial.write(record, recordSize);
    if (written != recordSize) {
      return;
    }
    captureRing.Consume(recordSize);
    drained += recordSize;
  }
}

uint16_t captureSensorFlags() {
  uint16_t flags = 0;
  if (optical.matrixReady) flags |= bearamin_capture::SENSOR_MATRIX_READY;
  if (optical.matrixPresent) flags |= bearamin_capture::SENSOR_MATRIX_PRESENT;
  if (imuReady) flags |= bearamin_capture::SENSOR_IMU_READY;
  if (latestImpact.gate) flags |= bearamin_capture::SENSOR_IMPACT_GATE;
  if (optical.spankGate) flags |= bearamin_capture::SENSOR_SPANK_GATE;
  return flags;
}

void captureMatrixFrame() {
  if (!captureActive || captureStopping) {
    return;
  }
  uint8_t payload[bearamin_capture::kMatrixPayloadSize] = {};
  payload[0] = optical.matrixValidCount;
  payload[1] = optical.matrixBlobCount;
  payload[2] = (optical.matrixReady ? 1u : 0u) |
               (optical.matrixPresent ? 2u : 0u);
  bearamin_capture::writeLe16(payload, 4, optical.matrixZMm);
  bearamin_capture::writeLe16(payload, 6, optical.matrixXMilli);
  bearamin_capture::writeLe16(payload, 8, optical.matrixYMilli);
  bearamin_capture::writeLe16(payload, 10, optical.matrixZNorm);
  bearamin_capture::writeLe16(payload, 12, optical.matrixXNorm);
  bearamin_capture::writeLe16(payload, 14, optical.matrixYNorm);
  // MatrixCore has already converted this frame from raw sensor order into
  // bearamin_true_grid_v1. Capture and every downstream subscriber retain
  // that row-major order; display transforms never enter this path.
  for (uint8_t logicalIndex = 0; logicalIndex < 64; ++logicalIndex) {
    bearamin_capture::writeLe16(
        payload, 16 + logicalIndex * 2, optical.cellsMm[logicalIndex]);
    payload[144 + logicalIndex] = optical.statuses[logicalIndex];
  }
  captureRecord(
      bearamin_capture::RECORD_MATRIX_FRAME, payload, sizeof(payload));
}

void captureImuSample(const ImuSample &sample, uint64_t coreUs) {
  if (!captureActive || captureStopping) {
    return;
  }
  if (captureImuCount == 0) {
    captureImuBatchStartUs = coreUs;
  }
  const uint8_t index = captureImuCount;
  const size_t offset =
      bearamin_capture::kImuBatchHeaderSize +
      static_cast<size_t>(index) * bearamin_capture::kImuSampleSize;
  const uint64_t delta = coreUs - captureImuBatchStartUs;
  bearamin_capture::writeLe16(
      captureImuPayload,
      offset,
      static_cast<uint16_t>(delta > 0xFFFFu ? 0xFFFFu : delta));
  bearamin_capture::writeLe16(
      captureImuPayload, offset + 2, static_cast<uint16_t>(sample.rawAccelX));
  bearamin_capture::writeLe16(
      captureImuPayload, offset + 4, static_cast<uint16_t>(sample.rawAccelY));
  bearamin_capture::writeLe16(
      captureImuPayload, offset + 6, static_cast<uint16_t>(sample.rawAccelZ));
  bearamin_capture::writeLe16(
      captureImuPayload, offset + 8, static_cast<uint16_t>(sample.rawGyroX));
  bearamin_capture::writeLe16(
      captureImuPayload, offset + 10, static_cast<uint16_t>(sample.rawGyroY));
  bearamin_capture::writeLe16(
      captureImuPayload, offset + 12, static_cast<uint16_t>(sample.rawGyroZ));
  ++captureImuCount;
  if (captureImuCount < bearamin_capture::kMaxImuBatchSamples) {
    return;
  }
  captureImuPayload[0] = captureImuCount;
  captureImuPayload[1] = imuReady ? 1 : 0;
  bearamin_capture::writeLe16(
      captureImuPayload, 2, latestImpact.velocityNorm);
  bearamin_capture::writeLe16(captureImuPayload, 4, latestImpact.ageMs);
  captureImuPayload[6] = latestImpact.gate ? 1 : 0;
  captureImuPayload[7] = 0;
  captureRecord(
      bearamin_capture::RECORD_IMU_BATCH,
      captureImuPayload,
      static_cast<uint16_t>(
          bearamin_capture::kImuBatchHeaderSize +
          captureImuCount * bearamin_capture::kImuSampleSize));
  captureImuCount = 0;
  captureImuBatchStartUs = 0;
}

void captureProcessedSnapshot() {
  if (!captureActive || captureStopping) {
    return;
  }
  bearamin_capture::ProcessedSensorSnapshotV2 snapshot = {};
  snapshot.sensorFlags = captureSensorFlags();
  snapshot.matrixZMm = optical.matrixZMm;
  snapshot.matrixXMilli = optical.matrixXMilli;
  snapshot.matrixYMilli = optical.matrixYMilli;
  snapshot.matrixZNorm = optical.matrixZNorm;
  snapshot.matrixXNorm = optical.matrixXNorm;
  snapshot.matrixYNorm = optical.matrixYNorm;
  snapshot.matrixValidCount = optical.matrixValidCount;
  snapshot.matrixBlobCount = optical.matrixBlobCount;
  snapshot.impactVelocityNorm = latestImpact.velocityNorm;
  snapshot.impactAgeMs = latestImpact.ageMs;
  snapshot.accelXMilliG =
      static_cast<int16_t>(latestImu.accel.x * 1000.0f);
  snapshot.accelYMilliG =
      static_cast<int16_t>(latestImu.accel.y * 1000.0f);
  snapshot.accelZMilliG =
      static_cast<int16_t>(latestImu.accel.z * 1000.0f);
  snapshot.sensorAgeMs = capAgeMs(optical.lastMatrixReadMs);
  snapshot.pitchNorm = optical.pitchNorm;
  snapshot.rollNorm = optical.rollNorm;
  snapshot.orientationConfidence = optical.orientationConfidence;
  uint8_t payload[bearamin_capture::kProcessedSnapshotV2PayloadSize] = {};
  bearamin_capture::encodeProcessedSnapshotV2(snapshot, payload);
  captureRecord(
      bearamin_capture::RECORD_PROCESSED_SNAPSHOT_V2,
      payload,
      sizeof(payload));
}

void captureDptControl(const uint8_t *frame, size_t length) {
  if (!captureActive || captureStopping || frame == nullptr ||
      length > bearamin_bridge::kFrameSize) {
    return;
  }
  captureControlRecord(
      bearamin_capture::CONTROL_DPT_FRAME, frame, length);
}

void captureDiagnostic() {
  if (!captureActive || captureStopping) {
    return;
  }
  uint8_t payload[bearamin_capture::kDiagnosticPayloadSize] = {};
  bearamin_capture::writeLe16(payload, 4, opticalRecoveryFailures);
  bearamin_capture::writeLe16(payload, 6, optical.i2cRecoveries);
  bearamin_capture::writeLe16(
      payload, 10, capAgeMs(optical.lastMatrixReadMs));
  payload[12] = lastOpticalRecoveryReason;
  payload[15] = lastRecoveryBusFlags;
  bearamin_capture::writeLe32(payload, 16, imuReadFailureCount);
  bearamin_capture::writeLe32(payload, 20, imuRecoveryCount);
  captureRecord(
      bearamin_capture::RECORD_DIAGNOSTIC, payload, sizeof(payload));
}

void sendPresetLineToSidecar(const char *line) {
  if (line == nullptr) {
    return;
  }
  bearamin_sidecar::PresetFrame preset = {};
  preset.sequence = sidecarPresetSequence++;
  preset.op = bearamin_sidecar::PRESET_OP_LINE;
  while (preset.length < bearamin_sidecar::kPresetLinePayloadSize &&
         line[preset.length] != '\0') {
    preset.data[preset.length] = static_cast<uint8_t>(line[preset.length]);
    ++preset.length;
  }
  if (preset.length < bearamin_sidecar::kPresetLinePayloadSize) {
    preset.data[preset.length++] = '\n';
  }
  uint8_t frame[bearamin_sidecar::kPresetFrameSize] = {};
  const size_t length = bearamin_sidecar::encodePresetFrame(preset, frame, sizeof(frame));
  if (length > 0) {
    SidecarSerial.write(frame, length);
  }
}

void serialLogPrintf(const char *format, ...) {
  if (!Serial) {
    return;
  }

  char buffer[384];
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (written <= 0) {
    return;
  }

  const size_t length = static_cast<size_t>(
      std::min<int>(written, static_cast<int>(sizeof(buffer) - 1)));
  if (Serial.availableForWrite() < static_cast<int>(length)) {
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(buffer), length);
}

void serialLogPrintln(const char *text) {
  serialLogPrintf("%s\n", text);
}

void sendMatrixIdentityJson() {
  const String mac = WiFi.macAddress();
  char payload[256] = {};
  snprintf(payload,
           sizeof(payload),
           "{\"device\":\"Bearamin MatrixCore\",\"role\":\"matrix_core\","
           "\"mac\":\"%s\",\"hostname\":\"%s\",\"matrix_sensor\":\"%s\","
           "\"ota_ready\":%s}",
           mac.c_str(),
           MATRIXCORE_MDNS_HOSTNAME,
           kMatrixSensorName,
           matrixOtaServicesReady ? "true" : "false");
  matrixIdentityServer.send(200, "application/json", payload);
}

void startMatrixIdentityServer() {
  if (!matrixIdentityRoutesConfigured) {
    matrixIdentityServer.on("/device.json", HTTP_GET, sendMatrixIdentityJson);
    matrixIdentityServer.on("/status.json", HTTP_GET, sendMatrixIdentityJson);
    matrixIdentityRoutesConfigured = true;
  }
  matrixIdentityServer.begin();
  matrixIdentityServerReady = true;
}

void stopMatrixOtaMode() {
  if (!matrixOtaMode) {
    return;
  }
  ArduinoOTA.end();
  if (matrixIdentityServerReady) {
    matrixIdentityServer.stop();
    matrixIdentityServerReady = false;
  }
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  matrixOtaMode = false;
  matrixOtaWifiReady = false;
  matrixOtaServicesReady = false;
  matrixOtaStarted = false;
  matrixOtaActive = false;
  matrixOtaModeStartedMs = 0;
  matrixOtaLastProgressMs = 0;
  serialLogPrintln("matrix_ota=stopped");
}

void configureMatrixOtaCallbacks() {
  ArduinoOTA.onStart([]() {
    matrixOtaStarted = true;
    matrixOtaActive = true;
    matrixOtaLastProgressMs = millis();
    serialLogPrintln("matrix_ota=receive_start");
  });
  ArduinoOTA.onProgress([](unsigned int, unsigned int) {
    matrixOtaStarted = true;
    matrixOtaActive = true;
    matrixOtaLastProgressMs = millis();
  });
  ArduinoOTA.onEnd([]() {
    matrixOtaActive = false;
    serialLogPrintln("matrix_ota=receive_end rebooting=1");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    matrixOtaActive = false;
    serialLogPrintf("matrix_ota=error code=%u\n", static_cast<unsigned>(error));
    stopMatrixOtaMode();
  });
}

void startMatrixOtaMode(uint32_t now) {
  if (matrixOtaMode) {
    matrixOtaModeStartedMs = now;
    return;
  }
  if (strlen(WIFI_SSID) == 0) {
    serialLogPrintln("matrix_ota=failed reason=no_wifi_ssid");
    return;
  }
  matrixOtaMode = true;
  matrixOtaWifiReady = false;
  matrixOtaServicesReady = false;
  matrixOtaStarted = false;
  matrixOtaActive = false;
  matrixOtaModeStartedMs = now;
  matrixOtaLastProgressMs = 0;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(MATRIXCORE_MDNS_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  serialLogPrintf("matrix_ota=connecting hostname=%s\n", MATRIXCORE_MDNS_HOSTNAME);
}

void pollMatrixOta(uint32_t now) {
  if (!matrixOtaMode) {
    return;
  }
  if (!matrixOtaStarted &&
      matrixOtaModeStartedMs != 0 &&
      now >= matrixOtaModeStartedMs &&
      now - matrixOtaModeStartedMs >= kMatrixOtaTimeoutMs) {
    serialLogPrintln("matrix_ota=timeout");
    stopMatrixOtaMode();
    return;
  }

  const bool connected = WiFi.status() == WL_CONNECTED;
  matrixOtaWifiReady = connected;
  if (!connected) {
    return;
  }

  if (!matrixOtaServicesReady) {
    ArduinoOTA.setHostname(MATRIXCORE_MDNS_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) {
      ArduinoOTA.setPassword(OTA_PASSWORD);
    }
    configureMatrixOtaCallbacks();
    ArduinoOTA.begin();
    MDNS.addService("arduino", "tcp", 3232);
    startMatrixIdentityServer();
    MDNS.addService("http", "tcp", 80);
    matrixOtaServicesReady = true;
    serialLogPrintf("matrix_ota=ready hostname=%s ip=%s\n",
                    MATRIXCORE_MDNS_HOSTNAME,
                    WiFi.localIP().toString().c_str());
  }
  if (matrixIdentityServerReady) {
    matrixIdentityServer.handleClient();
  }
  ArduinoOTA.handle();
}

void encodeBit(bool set, uint16_t &cursor) {
  ledData[cursor].level0 = 1;
  ledData[cursor].duration0 = set ? 8 : 4;
  ledData[cursor].level1 = 0;
  ledData[cursor].duration1 = set ? 4 : 8;
  ++cursor;
}

void encodeByte(uint8_t value, uint16_t &cursor) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    encodeBit((value & (1 << (7 - bit))) != 0, cursor);
  }
}

void writeLeds() {
  uint16_t cursor = 0;
  for (uint8_t i = 0; i < kLedCount; ++i) {
    encodeByte(scaleChannel(ledFrame[i].red, kBrightnessPercent), cursor);
    encodeByte(scaleChannel(ledFrame[i].green, kBrightnessPercent), cursor);
    encodeByte(scaleChannel(ledFrame[i].blue, kBrightnessPercent), cursor);
  }
  if (ledReady) {
    rmtWrite(kLedPin, ledData, kRmtSymbols, RMT_WAIT_FOR_EVER);
  }
}

uint8_t pixelIndex(uint8_t column, uint8_t row) {
  static const bearfield::DisplayMountConfig mount = {
      {static_cast<bearfield::QuarterTurns>(
           BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS),
       BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X != 0,
       BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y != 0},
      bearfield::PixelOrder::RowMajor,
      nullptr,
  };
  return bearfield::displayPixelIndex(column, row, mount);
}

void clearLeds() {
  for (Color &pixel : ledFrame) {
    pixel = {0, 0, 0};
  }
}

void startChaosActivationFeedback(uint32_t now) {
  constexpr uint32_t kChaosRainbowDurationMs = 1000;
  chaosRainbowUntilMs = now + kChaosRainbowDurationMs;
}

bool renderChaosActivationFeedback(uint32_t now) {
  if (static_cast<int32_t>(chaosRainbowUntilMs - now) <= 0) {
    return false;
  }
  clearLeds();
  const float phase = static_cast<float>(now) / 200.0f;
  const uint8_t rank = static_cast<uint8_t>((now / 53u) & 0x0fu);
  for (uint8_t row = 0; row < kMatrixWidth; ++row) {
    for (uint8_t column = 0; column < kMatrixWidth; ++column) {
      const LiquidColor color = liquidRainbowColor(column, row, rank, phase);
      ledFrame[pixelIndex(column, row)] = {color.red, color.green, color.blue};
    }
  }
  writeLeds();
  return true;
}

void addPixel(uint8_t column, uint8_t row, Color color) {
  if (column >= kMatrixWidth || row >= kMatrixWidth) {
    return;
  }
  Color &pixel = ledFrame[pixelIndex(column, row)];
  pixel.red = clampByte(pixel.red + color.red);
  pixel.green = clampByte(pixel.green + color.green);
  pixel.blue = clampByte(pixel.blue + color.blue);
}

void setTopStatusPixel(uint8_t columnFromLeft, Color color) {
  if (columnFromLeft >= kMatrixWidth) {
    return;
  }
#if BEARAMIN_STATUS_USES_DISPLAY_PROFILE
  // Matrix Core v3's complete panel, including its board-status row, follows
  // the local display profile. This is display-only and leaves true-grid data
  // plus all downstream telemetry unchanged.
  Color &pixel = ledFrame[pixelIndex(columnFromLeft, kMatrixWidth - 1)];
#else
  // Matrix Core v2 status indicators are board hardware UI, not true-grid
  // data. Keep their installed physical top-row placement independent of the
  // occupancy/vector consumer profile.
  Color &pixel = ledFrame[(kMatrixWidth - 1) * kMatrixWidth +
                          (kMatrixWidth - 1 - columnFromLeft)];
#endif
  pixel = color;
}

Color scaleColor(Color color, uint8_t brightness, uint8_t maxBrightness = kStatusVectorDotBrightness) {
  if (maxBrightness == 0) {
    return {0, 0, 0};
  }
  return {
      static_cast<uint8_t>((static_cast<uint16_t>(color.red) * brightness) / maxBrightness),
      static_cast<uint8_t>((static_cast<uint16_t>(color.green) * brightness) / maxBrightness),
      static_cast<uint8_t>((static_cast<uint16_t>(color.blue) * brightness) / maxBrightness),
  };
}

uint16_t statusNearMilliForDistance(uint16_t distanceMm) {
  if (distanceMm == 0 || distanceMm >= kPlayableMaxMm) {
    return 0;
  }
  if (distanceMm <= kStatusVectorYellowMm) {
    return 1000;
  }
  const uint32_t span = kPlayableMaxMm - kStatusVectorYellowMm;
  const uint32_t fromFar = kPlayableMaxMm - distanceMm;
  const uint32_t linearMilli = (fromFar * 1000UL) / span;
  return static_cast<uint16_t>((linearMilli * linearMilli * (3000UL - (2UL * linearMilli)) + 500000UL) / 1000000UL);
}

Color vectorDepthColor(uint8_t brightness,
                       uint16_t distanceMm,
                       uint16_t zNorm,
                       bool showMidpointMarker = true) {
  const uint32_t nearMilli = statusNearMilliForDistance(distanceMm);
  const LiquidColor depthColor = {
      static_cast<uint8_t>((kStatusVectorDotBrightness * nearMilli + 500UL) / 1000UL),
      static_cast<uint8_t>((kStatusVectorDotBrightness * nearMilli + 500UL) / 1000UL),
      static_cast<uint8_t>((kStatusVectorDotBrightness * (1000UL - nearMilli) + 500UL) / 1000UL),
  };
  const LiquidColor color = showMidpointMarker
                                ? blendColorMilli(
                                      depthColor,
                                      LiquidColor{220, 0, 255},
                                      midpointMarkerBlendMilli(zNorm))
                                : depthColor;
  return scaleColor(Color{color.red, color.green, color.blue}, brightness);
}

uint16_t statusVectorDepthMm() {
  return optical.matrixZMm;
}

uint16_t expandVectorMilli(uint16_t value) {
  if (value > 1000) {
    value = 1000;
  }
  const int32_t centered = static_cast<int32_t>(value) - 500;
  const int32_t expanded = 500 + ((centered * 17) / 10);
  return static_cast<uint16_t>(std::max<int32_t>(0, std::min<int32_t>(1000, expanded)));
}

Color readyColor(bool ready) {
  return ready ? Color{0, 80, 0} : Color{80, 0, 0};
}

Color sidecarWifiStatusColor(uint32_t now) {
  const bool sidecarFresh = lastSidecarStatusMs != 0 && millis() - lastSidecarStatusMs <= kSidecarStatusStaleMs;
  const bool sidecarSoftApReady = (sidecarStatusFlags & bearamin_sidecar::SIDECAR_STATUS_SOFTAP_READY) != 0;
  const bool sidecarWifiReady = (sidecarStatusFlags & bearamin_sidecar::SIDECAR_STATUS_WIFI_READY) != 0;
  const bool sidecarWifiFiveGhz = (sidecarStatusFlags & bearamin_sidecar::SIDECAR_STATUS_WIFI_5GHZ) != 0;
  if (sidecarFresh && sidecarSoftApReady) {
    return Color{70, 0, 90};
  }
  if (sidecarFresh && sidecarWifiReady) {
    return Color{
        0,
        sidecarWifiFiveGhz
            ? bearamin_wifi::fiveGhzPulseBrightness(now)
            : static_cast<uint8_t>(80),
        0};
  }
  return Color{80, 40, 0};
}

Color wifiStatusColor(uint32_t now) {
  if (matrixOtaActive) {
    return ((now / kMatrixOtaBlinkMs) & 0x01u) == 0 ? Color{0, 0, 140} : Color{0, 0, 10};
  }
  if (matrixOtaMode) {
    return Color{0, 0, 110};
  }
  return sidecarWifiStatusColor(now);
}

Color dptStatusColor(uint32_t now) {
  if (lastDptRxMs != 0 && now - lastDptRxMs < kDptRxBlinkMs) {
    return Color{0, 150, 40};
  }
  return dptStatus.protocolVersion > 0 ? Color{0, 80, 0} : Color{80, 40, 0};
}

uint16_t capAgeMs(uint32_t lastMs) {
  if (lastMs == 0) {
    return 65535;
  }
  const uint32_t age = millis() - lastMs;
  return age > 65535U ? 65535U : static_cast<uint16_t>(age);
}

uint8_t scanI2cBus(TwoWire &bus, const char *label) {
  uint8_t count = 0;
  serialLogPrintf("%s_scan=", label);
  for (uint8_t address = 1; address < 127; ++address) {
    bus.beginTransmission(address);
    if (bus.endTransmission() == 0) {
      serialLogPrintf("%s0x%02x", count == 0 ? "" : ",", address);
      ++count;
    }
  }
  if (count == 0) {
    serialLogPrintf("none");
  }
  serialLogPrintf(" count=%u\n", count);
  return count;
}

bool i2cAddressPresent(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void logKnownOpticalAddresses(TwoWire &bus, const char *label) {
  serialLogPrintf("%s_known_addresses matrix_0x29=%u sda=%u scl=%u\n",
                label,
                i2cAddressPresent(bus, kMatrixSensorAddress) ? 1 : 0,
                digitalRead(kOpticalSdaPin),
                digitalRead(kOpticalSclPin));
}

bool readI2cRegister16Address16(TwoWire &bus, uint8_t address, uint16_t reg, uint16_t &value) {
  bus.beginTransmission(address);
  bus.write(static_cast<uint8_t>((reg >> 8) & 0xFF));
  bus.write(static_cast<uint8_t>(reg & 0xFF));
  if (bus.endTransmission(false) != 0) {
    return false;
  }
  if (bus.requestFrom(address, static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  const uint8_t high = bus.read();
  const uint8_t low = bus.read();
  value = (static_cast<uint16_t>(high) << 8) | low;
  return true;
}

void logVl53l7cxDefaultIdentity(TwoWire &bus, const char *label) {
  uint16_t id = 0;
  if (readI2cRegister16Address16(bus, kMatrixSensorAddress, kVl53l7cxIdReg, id)) {
    serialLogPrintf("%s_vl53l7cx_id=0x%04x\n", label, id);
  } else {
    serialLogPrintf("%s_vl53l7cx_id=err\n", label);
  }
}

const char *sensorRecoveryReasonName(uint8_t reason) {
  switch (reason) {
    case bearamin_sidecar::SENSOR_RECOVERY_MATRIX_NO_FIRST_SAMPLE:
      return "matrix_no_first_sample";
    case bearamin_sidecar::SENSOR_RECOVERY_MATRIX_STALE:
      return "matrix_stale";
    case bearamin_sidecar::SENSOR_RECOVERY_MATRIX_NOT_READY:
      return "matrix_not_ready";
    default:
      return "boot";
  }
}

uint8_t opticalBusFlags() {
  uint8_t flags = 0;
  if (digitalRead(kOpticalSdaPin) == HIGH) {
    flags |= bearamin_sidecar::SENSOR_BUS_SDA_HIGH;
  }
  if (digitalRead(kOpticalSclPin) == HIGH) {
    flags |= bearamin_sidecar::SENSOR_BUS_SCL_HIGH;
  }
  if (i2cAddressPresent(opticalWire, kMatrixSensorAddress)) {
    flags |= bearamin_sidecar::SENSOR_BUS_DEFAULT_ADDRESS_ACK;
  }
  return flags;
}

void clearI2cBus(uint8_t sdaPin, uint8_t sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(sdaPin) == LOW) {
    pinMode(sclPin, OUTPUT_OPEN_DRAIN);
    digitalWrite(sclPin, HIGH);
    for (uint8_t i = 0; i < 9 && digitalRead(sdaPin) == LOW; ++i) {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(5);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(5);
    }
  }
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  pinMode(sclPin, INPUT_PULLUP);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  pinMode(sdaPin, INPUT_PULLUP);
}

void stopMatrixRanging() {
  if (!matrixRangingStarted) {
    return;
  }
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  matrixSensor.stop_ranging();
#else
  matrixSensor.stopRanging();
#endif
  matrixRangingStarted = false;
}

bool configureMatrixSensor() {
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  uint8_t status = matrixSensor.set_resolution(64);
  serialLogPrintf("matrix_config stage=resolution status=%u\n", status);
  if (status != 0) return false;
  status = matrixSensor.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
  serialLogPrintf("matrix_config stage=mode status=%u\n", status);
  if (status != 0) return false;
  status = matrixSensor.set_target_order(VL53L8CX_TARGET_ORDER_STRONGEST);
  serialLogPrintf("matrix_config stage=target_order status=%u\n", status);
  if (status != 0) return false;
  status = matrixSensor.set_integration_time_ms(10);
  serialLogPrintf("matrix_config stage=integration status=%u\n", status);
  if (status != 0) return false;
  status = matrixSensor.set_ranging_frequency_hz(kMatrixSourceHz);
  serialLogPrintf("matrix_config stage=frequency status=%u\n", status);
  if (status != 0) return false;
  status = matrixSensor.start_ranging();
  serialLogPrintf("matrix_config stage=start status=%u\n", status);
  matrixRangingStarted = status == 0;
  return matrixRangingStarted;
#else
  matrixRangingStarted = matrixSensor.setResolution(64) &&
      matrixSensor.setRangingFrequency(kMatrixSourceHz) &&
      matrixSensor.setTargetOrder(VL53L7CX_TARGET_ORDER_STRONGEST) &&
      matrixSensor.setIntegrationTime(10) && matrixSensor.startRanging();
  return matrixRangingStarted;
#endif
}

bool beginMatrixSensor() {
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  const int beginStatus =
      matrixSensor.begin(static_cast<uint16_t>(kMatrixSensorAddress) << 1);
  serialLogPrintf("matrix_init stage=begin status=%d\n", beginStatus);
  if (beginStatus != 0) return false;
  const uint8_t initStatus = matrixSensor.init();
  serialLogPrintf("matrix_init stage=firmware status=%u\n", initStatus);
  return initStatus == 0;
#else
  return matrixSensor.begin(
      kMatrixSensorAddress, &opticalWire, kOpticalClockHz);
#endif
}

bool matrixDataReady() {
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  uint8_t ready = 0;
  matrixReadyPollStatus = matrixSensor.check_data_ready(&ready);
  matrixReadyPollValue = ready;
  if (matrixReadyPollStatus != 0) {
    ++matrixReadErrors;
    return false;
  }
  return ready != 0;
#else
  return matrixSensor.isDataReady();
#endif
}

bool readMatrixData() {
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  return matrixSensor.get_ranging_data(&matrixResults) == 0;
#else
  return matrixSensor.getRangingData(&matrixResults);
#endif
}

void resetMatrixPerformanceWindow() {
  const uint64_t nowUs = esp_timer_get_time();
  nextMatrixReadyCheckUs = nowUs;
  lastMatrixFrameUs = 0;
  matrixRateWindowStartUs = nowUs;
  matrixRateWindowFrames = 0;
  matrixEffectiveRateMilliHz = 0;
  matrixIntervalMaxUs = 0;
  matrixReadMaxUs = 0;
  matrixProcessMaxUs = 0;
  matrixLoopGapMaxUs = 0;
}

bool initMatrixAtDefaultAddress(OpticalState &state, bool diagnosticScan) {
  opticalWire.setClock(kOpticalClockHz);
  digitalWrite(kMatrixLprPin, HIGH);
  delay(500);
  if (diagnosticScan) {
    logKnownOpticalAddresses(opticalWire, "optical_matrix_released");
  }
#if !defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  logVl53l7cxDefaultIdentity(opticalWire, "optical_matrix_released");
#endif
  state.matrixReady = beginMatrixSensor();
  if (state.matrixReady) {
    state.matrixReady = configureMatrixSensor();
  }
  state.lastMatrixReadMs = 0;
  state.matrixMonitoringStartedMs =
      sensorMonitoringStartMs(state.matrixReady, millis());
  if (state.matrixReady) {
    resetMatrixPerformanceWindow();
  }
  return state.matrixReady;
}

bool recoverMatrixSensor(OpticalState &state) {
  stopMatrixRanging();
  state.matrixReady = false;
  state.matrixPresent = false;
  state.lastMatrixReadMs = 0;
  digitalWrite(kMatrixLprPin, LOW);
  delay(20);
  const bool ready = initMatrixAtDefaultAddress(state, false);
  serialLogPrintf("matrix_recovery=in_place ready=%u\n", ready ? 1 : 0);
  return ready;
}

#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
void prepareVl53l8cxOpticalBusForDeferredInit(OpticalState &state) {
  pinMode(kMatrixLprPin, OUTPUT);
  pinMode(kMatrixAuxPin, OUTPUT);
  digitalWrite(kMatrixLprPin, LOW);
  digitalWrite(kMatrixAuxPin, LOW);
  delay(10);

  clearI2cBus(kOpticalSdaPin, kOpticalSclPin);
  opticalWire.begin(kOpticalSdaPin, kOpticalSclPin);
  opticalWire.setClock(kOpticalClockHz);
  opticalWire.setTimeOut(30);
  state = OpticalState{};
  state.busReady = true;
  serialLogPrintln(
      "optical_bus=ready matrix_sensor=VL53L8CX matrix=deferred");
}
#endif

bool initSharedOpticalBus(OpticalState &state, bool diagnosticScan = true) {
  pinMode(kMatrixLprPin, OUTPUT);
  pinMode(kMatrixAuxPin,
          kMatrixUsesVl53l8cx ? OUTPUT : INPUT_PULLUP);
  digitalWrite(kMatrixLprPin, LOW);
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  digitalWrite(kMatrixAuxPin, LOW);
#endif
  delay(10);

  clearI2cBus(kOpticalSdaPin, kOpticalSclPin);
  opticalWire.begin(kOpticalSdaPin, kOpticalSclPin);
  opticalWire.setClock(kOpticalClockHz);
  opticalWire.setTimeOut(30);
  const uint16_t preservedRecoveries = state.i2cRecoveries;
  state = OpticalState{};
  state.i2cRecoveries = preservedRecoveries;
  state.busReady = true;
  if (diagnosticScan) {
    logKnownOpticalAddresses(opticalWire, "optical_matrix_shutdown");
  }

  initMatrixAtDefaultAddress(state, diagnosticScan);
  const uint32_t monitoringStartMs = millis();
  state.matrixMonitoringStartedMs =
      sensorMonitoringStartMs(state.matrixReady, monitoringStartMs);

  serialLogPrintf("optical_bus=%s matrix_sensor=%s matrix=%s addr=0x%02x lpr=GPIO%u\n",
                state.busReady ? "ready" : "failed",
                kMatrixSensorName,
                state.matrixReady ? "ready" : "missing",
                kMatrixSensorAddress,
                kMatrixLprPin);
  return state.matrixReady;
}

bool opticalRecoveryActive() {
  return opticalRecoveryPhase != OpticalRecoveryPhase::Idle;
}

void opticalRecoveryWorker(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    OpticalRecoveryJob job = OpticalRecoveryJob::None;
    portENTER_CRITICAL(&opticalRecoveryMux);
    job = opticalRecoveryJob;
    opticalRecoveryPhase = OpticalRecoveryPhase::Running;
    portEXIT_CRITICAL(&opticalRecoveryMux);

    const uint32_t startedMs = millis();
    const uint8_t busFlags = opticalBusFlags();
    bool anyReady = false;
    if (job == OpticalRecoveryJob::MatrixOnly) {
      anyReady = recoverMatrixSensor(opticalRecoveryWorking);
    } else if (job == OpticalRecoveryJob::FullBus) {
      opticalWire.end();
      anyReady = initSharedOpticalBus(opticalRecoveryWorking, false);
    }
    const uint32_t durationMs = millis() - startedMs;
    const bool allReady = opticalRecoveryWorking.matrixReady;

    portENTER_CRITICAL(&opticalRecoveryMux);
    opticalRecoveryBusFlags = busFlags;
    opticalRecoveryAnyReady = anyReady;
    opticalRecoveryAllReady = allReady;
    opticalRecoveryLastDurationMs = durationMs;
    if (durationMs > opticalRecoveryMaxDurationMs) {
      opticalRecoveryMaxDurationMs = durationMs;
    }
    opticalRecoveryPhase = OpticalRecoveryPhase::Complete;
    portEXIT_CRITICAL(&opticalRecoveryMux);
  }
}

void startOpticalRecoveryWorker() {
  opticalRecoveryTaskHandle = xTaskCreateStaticPinnedToCore(
      opticalRecoveryWorker,
      "optical_recovery",
      sizeof(opticalRecoveryTaskStack) / sizeof(opticalRecoveryTaskStack[0]),
      nullptr,
      0,
      opticalRecoveryTaskStack,
      &opticalRecoveryTaskBuffer,
      0);
}

bool writeQmi(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kQmiAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readQmi(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(kQmiAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(kQmiAddress, length) != length) {
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

uint8_t readQmiByte(uint8_t reg) {
  uint8_t value = 0;
  readQmi(reg, &value, 1);
  return value;
}

int16_t readSigned16(const uint8_t *buffer, uint8_t offset) {
  return static_cast<int16_t>((static_cast<uint16_t>(buffer[offset + 1]) << 8) | buffer[offset]);
}

bool initImu() {
  Wire.begin(kImuSdaPin, kImuSclPin);
  Wire.setClock(400000);
  scanI2cBus(Wire, "imu");
  if (readQmiByte(kRegWhoAmI) != 0x05) {
    return false;
  }
  writeQmi(kRegReset, 0xB0);
  delay(20);
  if (readQmiByte(kRegWhoAmI) != 0x05) {
    return false;
  }
  return writeQmi(kRegCtrl1, 0x40) &&
         writeQmi(kRegCtrl2, 0x2C) &&
         writeQmi(kRegCtrl3, 0x36) &&
         writeQmi(kRegCtrl5, 0x00) &&
         writeQmi(kRegCtrl7, 0x03);
}

bool readImu(ImuSample &sample) {
  uint8_t accelBuffer[6] = {};
  uint8_t gyroBuffer[6] = {};
  if (!readQmi(kRegAccelXL, accelBuffer, sizeof(accelBuffer)) ||
      !readQmi(kRegGyroXL, gyroBuffer, sizeof(gyroBuffer))) {
    return false;
  }
  sample.rawAccelX = readSigned16(accelBuffer, 0);
  sample.rawAccelY = readSigned16(accelBuffer, 2);
  sample.rawAccelZ = readSigned16(accelBuffer, 4);
  sample.rawGyroX = readSigned16(gyroBuffer, 0);
  sample.rawGyroY = readSigned16(gyroBuffer, 2);
  sample.rawGyroZ = readSigned16(gyroBuffer, 4);
  sample.accel.x = sample.rawAccelX * kAccelScale16g;
  sample.accel.y = sample.rawAccelY * kAccelScale16g;
  sample.accel.z = sample.rawAccelZ * kAccelScale16g;
  sample.gyroX = sample.rawGyroX * kGyroScale128dps;
  sample.gyroY = sample.rawGyroY * kGyroScale128dps;
  sample.gyroZ = sample.rawGyroZ * kGyroScale128dps;
  return true;
}

void pollImu(uint32_t now) {
  if (!imuReady) {
    if (nextImuRetryMs != 0 &&
        static_cast<int32_t>(now - nextImuRetryMs) < 0) {
      return;
    }
    if (!initImu()) {
      nextImuRetryMs = now + imuRetryMs;
      imuRetryMs = nextRetryMs(imuRetryMs, kImuRetryMaxMs);
      return;
    }
    imuReady = true;
    nextImuRetryMs = 0;
    imuRetryMs = kImuRetryMinMs;
    impactDetector.reset();
    latestImpact = unavailableImpactState();
    ++imuRecoveryCount;
    serialLogPrintf("imu=recovered recoveries=%lu read_failures=%lu\n",
                    static_cast<unsigned long>(imuRecoveryCount),
                    static_cast<unsigned long>(imuReadFailureCount));
  }
  if (now - lastImuPollMs < kImuPollMs) {
    return;
  }
  lastImuPollMs = now;
  if (!readImu(latestImu)) {
    imuReady = false;
    latestImpact = unavailableImpactState();
    impactDetector.reset();
    ++imuReadFailureCount;
    const uint32_t scheduledRetryMs = imuRetryMs;
    nextImuRetryMs = now + scheduledRetryMs;
    imuRetryMs = nextRetryMs(imuRetryMs, kImuRetryMaxMs);
    serialLogPrintf("imu=read_failed failures=%lu retry_ms=%lu\n",
                    static_cast<unsigned long>(imuReadFailureCount),
                    static_cast<unsigned long>(scheduledRetryMs));
    return;
  }
  latestImpact = impactDetector.process(latestImu.accel, now);
  if (latestImpact.gate && !previousImpactGate && chaosUnlock.event(now)) {
    chaosLorenz.seed(static_cast<uint32_t>(esp_timer_get_time()) ^ latestImpact.velocityNorm);
    startChaosActivationFeedback(now);
    serialLogPrintln("chaos=enabled");
  }
  previousImpactGate = latestImpact.gate;
  captureImuSample(latestImu, captureCoreUs());
}

bool matrixValueValid(uint16_t valueMm) {
  return cvOpticalDistanceUsableMm(valueMm, kMatrixMaxValidMm);
}

bool matrixMusicValueValid(uint16_t valueMm) {
  return valueMm >= kMatrixMinMusicMm && valueMm <= kMatrixMaxMusicMm;
}

bool matrixStatusValid(uint8_t status) {
  return status == kMatrixStatusRangingOk || status == kMatrixStatusRangingOkLargePulse;
}

uint16_t adjustedMatrixNearestMm() {
  if (!optical.matrixPresent || optical.matrixZMm == 0) {
    return 0;
  }
  const uint16_t playableDistanceMm = cvPlayableDistanceWithCloseClampMm(
      optical.matrixZMm, true, kMatrixMinMusicMm, kMatrixMaxMusicMm);
  if (playableDistanceMm == 0) {
    return 0;
  }
  return bearamin_bridge::clampU16(
      static_cast<uint32_t>(playableDistanceMm) + kMatrixDepthOffsetMm,
      65535);
}

void updateOrientation(
    const bearamin_orientation::SurfaceEstimate &estimate,
    uint32_t now) {
  const bearamin_orientation::OrientationOutput output =
      orientationTracker.update(estimate, now);
  optical.pitchNorm = output.pitchNorm;
  optical.rollNorm = output.rollNorm;
  optical.orientationConfidence = output.confidence;
  optical.orientationMedianDepthMm =
      estimate.confident ? estimate.medianDepthMm : 0;
  optical.spankGate =
      spankDetector.update(estimate.confident, estimate.medianDepthMm, now);
}

void analyzeMatrix(uint64_t capturedUs) {
  bearfield::DepthFrame8x8 rawFrame = {};
  rawFrame.sequence = ++matrixSourceSequence;
  rawFrame.capturedUs = capturedUs;
  for (uint8_t index = 0; index < 64; ++index) {
    rawFrame.distanceMm[index] = optical.cellsMm[index];
    if (matrixValueValid(optical.cellsMm[index]) &&
        matrixStatusValid(optical.statuses[index])) {
      rawFrame.validMask |= UINT64_C(1) << index;
    }
  }
  static const bearfield::SensorMountConfig sensorMount = {
      {static_cast<bearfield::QuarterTurns>(
           BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS),
       BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X != 0,
       BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y != 0},
      BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK,
  };
  const bearfield::DepthFrame8x8 canonical =
      bearfield::orientFrame(rawFrame, sensorMount);
  for (uint8_t index = 0; index < 64; ++index) {
    optical.cellsMm[index] = canonical.distanceMm[index];
    optical.statuses[index] =
        ((canonical.validMask >> index) & 1ULL) != 0
            ? kMatrixStatusRangingOk
            : 0;
  }
  fieldProcessor.process(canonical, capturedUs);
  processedField = fieldProcessor.target();
  optical.matrixPresent = processedField.present;
  optical.spankGate = processedField.spankHeld;
  // x/y are continuous values. Keep the 15 Hz cooked target here, while the
  // local panel uses trajectoryOutputAxes at its native render cadence.
  optical.matrixXMilli = processedField.present
      ? processedField.xMilli
      : static_cast<uint16_t>(
            (static_cast<uint32_t>(processedField.xNorm12) * 1000UL +
             bearamin_trajectory::kNormMaximum / 2UL) /
            bearamin_trajectory::kNormMaximum);
  optical.matrixYMilli = processedField.present
      ? processedField.yMilli
      : static_cast<uint16_t>(
            (static_cast<uint32_t>(processedField.yNorm12) * 1000UL +
             bearamin_trajectory::kNormMaximum / 2UL) /
            bearamin_trajectory::kNormMaximum);
  optical.matrixZMm = processedField.zMm;
  optical.matrixXNorm = processedField.xNorm12;
  optical.matrixYNorm = processedField.yNorm12;
  optical.matrixZNorm = processedField.zNorm12;
  optical.pitchNorm = processedField.pitchNorm12;
  optical.rollNorm = processedField.rollNorm12;
  optical.orientationConfidence = processedField.orientationConfidence;
  optical.matrixValidCount = processedField.validCount;
  optical.matrixBlobCount = processedField.blobCount;
  // This is deliberately not an output mapping: it only biases attractor dwell.
  chaosTofInfluence = (static_cast<float>(processedField.xNorm12) - 2048.0f) / 4095.0f +
      (static_cast<float>(processedField.yNorm12) - 2048.0f) / 8190.0f +
      (static_cast<float>(processedField.zNorm12) - 2048.0f) / 12285.0f;
  const uint32_t sourceEpochMs = millis();
  const uint16_t targetNorm[bearamin_trajectory::kAxisCount] = {
      processedField.xNorm12, processedField.yNorm12, processedField.zNorm12,
      processedField.pitchNorm12, processedField.rollNorm12};
  trajectoryAuthority.publishTarget(
      targetNorm, sourceEpochMs,
      bearamin_trajectory::tauForSlewStep(cvSlewStep));
}

void pollMatrix(uint32_t now) {
  const uint64_t nowUs = esp_timer_get_time();
  if (opticalRecoveryActive() || !optical.matrixReady ||
      nowUs < nextMatrixReadyCheckUs) {
    return;
  }
  if (!matrixDataReady()) {
    nextMatrixReadyCheckUs = nowUs + kMatrixReadyRetryUs;
    return;
  }
  const uint64_t readStartedUs = esp_timer_get_time();
  if (!readMatrixData()) {
    optical.syntheticGate = false;
    ++matrixReadErrors;
    nextMatrixReadyCheckUs = esp_timer_get_time() + kMatrixReadyRetryUs;
    return;
  }
  const uint32_t readUs =
      static_cast<uint32_t>(esp_timer_get_time() - readStartedUs);
  matrixReadMaxUs = std::max(matrixReadMaxUs, readUs);
  for (uint8_t i = 0; i < 64; ++i) {
    const int16_t distanceMm = matrixResults.distance_mm[i];
    optical.cellsMm[i] = distanceMm > 0 ? static_cast<uint16_t>(distanceMm) : 0;
    optical.statuses[i] = matrixResults.target_status[i];
  }
  optical.lastMatrixReadMs = now;
  const uint64_t processStartedUs = esp_timer_get_time();
  analyzeMatrix(processStartedUs);
  optical.syntheticGate = false;
  for (uint8_t row = 2; row < 6 && !optical.syntheticGate; ++row) {
    for (uint8_t column = 2; column < 6; ++column) {
      const uint8_t index = static_cast<uint8_t>(row * 8 + column);
      if (matrixMusicValueValid(optical.cellsMm[index]) &&
          matrixStatusValid(optical.statuses[index])) {
        optical.syntheticGate = true;
        break;
      }
    }
  }
  captureMatrixFrame();
  const uint32_t processUs =
      static_cast<uint32_t>(esp_timer_get_time() - processStartedUs);
  matrixProcessMaxUs = std::max(matrixProcessMaxUs, processUs);

  const uint64_t completedUs = esp_timer_get_time();
  if (lastMatrixFrameUs != 0) {
    matrixIntervalMaxUs = std::max(
        matrixIntervalMaxUs,
        static_cast<uint32_t>(completedUs - lastMatrixFrameUs));
  }
  lastMatrixFrameUs = completedUs;
  ++matrixRateWindowFrames;
  const uint64_t rateElapsedUs = completedUs - matrixRateWindowStartUs;
  if (rateElapsedUs >= 1000000) {
    matrixEffectiveRateMilliHz = static_cast<uint32_t>(
        (static_cast<uint64_t>(matrixRateWindowFrames) * 1000000000ULL) /
        rateElapsedUs);
    matrixRateWindowStartUs = completedUs;
    matrixRateWindowFrames = 0;
  }
  const uint32_t periodUs = 1000000UL / kMatrixSourceHz;
  nextMatrixReadyCheckUs =
      readStartedUs + (periodUs > kMatrixReadyLeadUs
                           ? periodUs - kMatrixReadyLeadUs
                           : kMatrixReadyRetryUs);
}

void serviceProcessedOutput(uint64_t nowUs) {
  constexpr uint64_t kOutputPeriodUs = 1000000ULL / kProcessedOutputHz;
  if (lastProcessedOutputUs != 0 &&
      nowUs - lastProcessedOutputUs < kOutputPeriodUs) {
    return;
  }
  lastProcessedOutputUs = nowUs;
  const uint32_t nowMs = millis();
  if (chaosUnlock.latched()) {
    const float elapsed = chaosLastUs == 0 ? 1.0f / kProcessedOutputHz :
        static_cast<float>(nowUs - chaosLastUs) / 1000000.0f;
    chaosLastUs = nowUs;
    chaosAxes = chaosLorenz.tick(elapsed, static_cast<float>(cvSlewStep) / kCvSlewMaxStep,
                                 chaosTofInfluence);
    trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_X] = chaosAxes.x;
    trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_Y] = chaosAxes.y;
    trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_Z] = chaosAxes.z;
    trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_PITCH] = processedField.pitchNorm12;
    trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_ROLL] = processedField.rollNorm12;
    trajectoryOutputValid = true;
    cvBridgeX = chaosAxes.x;
    cvBridgeY = chaosAxes.y;
    cvBridgeZ = chaosAxes.z;
    return;
  }
  const bool sourceFresh = optical.lastMatrixReadMs != 0 &&
      nowMs - optical.lastMatrixReadMs <= kTrajectoryStaleMs;
  if (!sourceFresh || !trajectoryAuthority.hasKeyframe() ||
      !trajectoryAuthority.changeTau(
          bearamin_trajectory::tauForSlewStep(cvSlewStep), nowMs) ||
      !bearamin_trajectory::evaluateAtSourceTime(
          trajectoryAuthority.keyframe(), nowMs, trajectoryOutputAxes)) {
    trajectoryOutputValid = false;
    cvBridgeX = cvBridgeCenterValue(kMaxBridgeValue);
    cvBridgeY = cvBridgeCenterValue(kMaxBridgeValue);
    cvBridgeZ = cvBridgeCenterValue(kMaxBridgeValue);
    return;
  }
  trajectoryOutputValid = true;
  cvBridgeX = trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_X];
  cvBridgeY = trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_Y];
  cvBridgeZ = trajectoryOutputAxes.norm[bearamin_trajectory::AXIS_Z];
}

void recoverOpticalIfNeeded(uint32_t now) {
  OpticalRecoveryPhase phase = OpticalRecoveryPhase::Idle;
  portENTER_CRITICAL(&opticalRecoveryMux);
  phase = opticalRecoveryPhase;
  portEXIT_CRITICAL(&opticalRecoveryMux);

  if (phase == OpticalRecoveryPhase::Complete) {
    OpticalRecoveryJob completedJob = OpticalRecoveryJob::None;
    bool anyReady = false;
    bool allReady = false;
    uint8_t busFlags = 0;
    uint32_t durationMs = 0;
    portENTER_CRITICAL(&opticalRecoveryMux);
    completedJob = opticalRecoveryJob;
    anyReady = opticalRecoveryAnyReady;
    allReady = opticalRecoveryAllReady;
    busFlags = opticalRecoveryBusFlags;
    durationMs = opticalRecoveryLastDurationMs;
    portEXIT_CRITICAL(&opticalRecoveryMux);

    optical = opticalRecoveryWorking;
    lastRecoveryBusFlags = busFlags;
    const uint32_t monitoringStartMs = millis();
    if (optical.matrixReady) {
      optical.matrixPresent = false;
      optical.lastMatrixReadMs = 0;
      optical.matrixMonitoringStartedMs = monitoringStartMs;
    }
    const bool succeeded =
        completedJob == OpticalRecoveryJob::MatrixOnly
            ? optical.matrixReady :
        allReady;
    if (succeeded) {
      opticalRecoveryRetryMs = kSensorRetryMinMs;
      opticalRecoveryFailures = 0;
    } else {
      ++opticalRecoveryFailures;
      opticalRecoveryRetryMs =
          nextRetryMs(opticalRecoveryRetryMs, kSensorRetryMaxMs);
    }
    serialLogPrintf(
        "optical_recovery=end reason=%s job=%u success=%u any_ready=%u "
        "matrix_ready=%u failures=%u duration_ms=%lu "
        "max_duration_ms=%lu next_retry_ms=%lu\n",
        sensorRecoveryReasonName(lastOpticalRecoveryReason),
        static_cast<unsigned>(completedJob),
        succeeded ? 1 : 0,
        anyReady ? 1 : 0,
        optical.matrixReady ? 1 : 0,
        opticalRecoveryFailures,
        static_cast<unsigned long>(durationMs),
        static_cast<unsigned long>(opticalRecoveryMaxDurationMs),
        static_cast<unsigned long>(
            succeeded ? kSensorRecoveryMs : opticalRecoveryRetryMs));
    portENTER_CRITICAL(&opticalRecoveryMux);
    opticalRecoveryJob = OpticalRecoveryJob::None;
    opticalRecoveryPhase = OpticalRecoveryPhase::Idle;
    portEXIT_CRITICAL(&opticalRecoveryMux);
    return;
  }
  if (phase != OpticalRecoveryPhase::Idle || matrixOtaMode) {
    return;
  }

  const bool matrixFirstSampleMissing = optical.matrixReady && optical.lastMatrixReadMs == 0 &&
                                        streamNeedsRecovery(optical.matrixReady,
                                                            optical.lastMatrixReadMs,
                                                            optical.matrixMonitoringStartedMs,
                                                            now,
                                                            kSensorFirstSampleMs,
                                                            kSensorRecoveryMs);
  const bool matrixStale = optical.matrixReady && optical.lastMatrixReadMs > 0 &&
                           streamNeedsRecovery(optical.matrixReady,
                                               optical.lastMatrixReadMs,
                                               optical.matrixMonitoringStartedMs,
                                               now,
                                               kSensorFirstSampleMs,
                                               kSensorRecoveryMs);
  const bool missingSensor = !optical.matrixReady;
  const bool matrixOnlyIssue =
      matrixFirstSampleMissing || matrixStale || !optical.matrixReady;
  const uint32_t retryDelay = missingSensor ? opticalRecoveryRetryMs : kSensorRecoveryMs;
  if ((!matrixFirstSampleMissing && !matrixStale && !missingSensor) ||
      (lastOpticalRecoveryAttemptMs != 0 &&
       now - lastOpticalRecoveryAttemptMs < retryDelay)) {
    return;
  }
  if (matrixFirstSampleMissing) {
    lastOpticalRecoveryReason =
        bearamin_sidecar::SENSOR_RECOVERY_MATRIX_NO_FIRST_SAMPLE;
  } else if (matrixStale) {
    lastOpticalRecoveryReason = bearamin_sidecar::SENSOR_RECOVERY_MATRIX_STALE;
  } else {
    lastOpticalRecoveryReason = bearamin_sidecar::SENSOR_RECOVERY_MATRIX_NOT_READY;
  }
  lastOpticalRecoveryAttemptMs = now;
  optical.lastRecoveryMs = now;
  ++optical.i2cRecoveries;
  opticalRecoveryWorking = optical;
  OpticalRecoveryJob job = OpticalRecoveryJob::FullBus;
  if (matrixOnlyIssue) {
    job = OpticalRecoveryJob::MatrixOnly;
  }
  optical.matrixPresent = false;
  optical.syntheticGate = false;
  if (job == OpticalRecoveryJob::MatrixOnly) {
    optical.matrixReady = false;
  } else {
    optical.matrixReady = false;
  }
  serialLogPrintf(
      "optical_recovery=queued reason=%s job=%u attempt=%u "
      "matrix_ready=%u matrix_age_ms=%u sda=%u scl=%u "
      "retry_ms=%lu\n",
      sensorRecoveryReasonName(lastOpticalRecoveryReason),
      static_cast<unsigned>(job),
      optical.i2cRecoveries,
      opticalRecoveryWorking.matrixReady ? 1 : 0,
      capAgeMs(opticalRecoveryWorking.lastMatrixReadMs),
      digitalRead(kOpticalSdaPin),
      digitalRead(kOpticalSclPin),
      static_cast<unsigned long>(retryDelay));
  portENTER_CRITICAL(&opticalRecoveryMux);
  opticalRecoveryJob = job;
  opticalRecoveryPhase = OpticalRecoveryPhase::Queued;
  portEXIT_CRITICAL(&opticalRecoveryMux);
  if (opticalRecoveryTaskHandle != nullptr) {
    xTaskNotifyGive(opticalRecoveryTaskHandle);
  } else {
    portENTER_CRITICAL(&opticalRecoveryMux);
    opticalRecoveryPhase = OpticalRecoveryPhase::Idle;
    opticalRecoveryJob = OpticalRecoveryJob::None;
    portEXIT_CRITICAL(&opticalRecoveryMux);
    optical = opticalRecoveryWorking;
    ++opticalRecoveryFailures;
    opticalRecoveryRetryMs =
        nextRetryMs(opticalRecoveryRetryMs, kSensorRetryMaxMs);
    serialLogPrintln(
        "optical_recovery=not_queued reason=worker_unavailable");
  }
}

bearamin_bridge::Packet buildDptPacket() {
  bearamin_bridge::Packet packet = {};
  packet.sequence = bridgeSequence++;
  const bool chaosActive = chaosUnlock.latched();
  if (chaosActive) {
    packet.flags |= bearamin_bridge::FLAG_CHAOS_ACTIVE;
  }
  if (optical.syntheticGate) {
    packet.flags |= bearamin_bridge::FLAG_GESTURE_GATE;
  }
  if (optical.matrixPresent) {
    packet.flags |= bearamin_bridge::FLAG_MATRIX_VALID | bearamin_bridge::FLAG_MATRIX_DETECTED;
  }
  if (latestImpact.gate) {
    packet.flags |= bearamin_bridge::FLAG_IMPACT_GATE;
  }
  if (optical.spankGate) {
    packet.flags |= bearamin_bridge::FLAG_SPANK_GATE;
  }
  if (modeAnnouncementsEnabled) {
    packet.flags |= bearamin_bridge::FLAG_MODE_ANNOUNCEMENTS_ENABLED;
  }
  packet.reservedZMm = 0;
  packet.reservedZNorm = 0;
  packet.matrixZMm = optical.matrixZMm;
  packet.matrixXMilli = optical.matrixXMilli;
  packet.matrixYMilli = optical.matrixYMilli;
  packet.matrixZNorm = optical.matrixZNorm;
  packet.matrixXNorm = optical.matrixXNorm;
  packet.matrixYNorm = optical.matrixYNorm;
  packet.matrixValidCount = optical.matrixValidCount;
  packet.matrixBlobCount = optical.matrixBlobCount;
  const uint16_t matrixAge = capAgeMs(optical.lastMatrixReadMs);
  packet.ageMs = matrixAge;
  if (packet.ageMs > 250) {
    packet.flags |= bearamin_bridge::FLAG_STALE;
  }
  packet.controlFlags = dptControlFlags;
  packet.reserved0 = dptReserved0;
  packet.reserved1 = dptReserved1;
  packet.reserved2 = dptReserved2;
  packet.sourceFlags2 = dptSourceFlags2;
  packet.impactVelocityNorm = latestImpact.velocityNorm;
  packet.impactAgeMs = latestImpact.ageMs;
  packet.impactFlags = latestImpact.gate ? 1 : 0;
  packet.resonatorControlsRevision = resonatorControlsRevision;
  for (uint8_t i = 0; i < bearamin_bridge::kResonatorWebControlSize; ++i) {
    packet.resonatorWebControls[i] = resonatorWebControls[i];
  }
  for (uint8_t i = 0; i < bearamin_bridge::kLaneDepthCount; ++i) {
    packet.laneDepths[i] = dptLaneDepths[i];
  }
  packet.pitchNorm = optical.pitchNorm;
  packet.rollNorm = optical.rollNorm;
  packet.orientationConfidence = optical.orientationConfidence;
  if (chaosActive) {
    // Lorenz is already the cooked 30 Hz modulation source. Send its current
    // axes as a complete v11 keyframe so DPT never falls back to the physical
    // centroid merely because presence is absent. Gate flags remain sensor-led.
    packet.sourceEpochMs = millis();
    packet.matrixXNorm = chaosAxes.x;
    packet.matrixYNorm = chaosAxes.y;
    packet.matrixZNorm = chaosAxes.z;
    packet.trajectoryStartNorm[bearamin_trajectory::AXIS_X] = chaosAxes.x;
    packet.trajectoryStartNorm[bearamin_trajectory::AXIS_Y] = chaosAxes.y;
    packet.trajectoryStartNorm[bearamin_trajectory::AXIS_Z] = chaosAxes.z;
    packet.trajectoryStartNorm[bearamin_trajectory::AXIS_PITCH] = packet.pitchNorm;
    packet.trajectoryStartNorm[bearamin_trajectory::AXIS_ROLL] = packet.rollNorm;
    packet.slewTauMs = bearamin_trajectory::kMinimumTauMs;
  } else if (trajectoryAuthority.hasKeyframe()) {
    const bearamin_trajectory::KeyframeV1 &keyframe =
        trajectoryAuthority.keyframe();
    packet.sourceEpochMs = keyframe.sourceEpochMs;
    packet.matrixXNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_X];
    packet.matrixYNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_Y];
    packet.matrixZNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_Z];
    packet.pitchNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_PITCH];
    packet.rollNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_ROLL];
    for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
      packet.trajectoryStartNorm[axis] = keyframe.startNorm[axis];
    }
    packet.slewTauMs = keyframe.slewTauMs;
  }
  return packet;
}

void sendDpt(uint32_t now) {
  if (!kDptContinuousTxEnabled) {
    return;
  }
  if (bearamin_preset::quietWindowActive(dptPresetQuietUntilMs, now)) {
    return;
  }
  if (lastDptWireRxByteMs != 0 &&
      now - lastDptWireRxByteMs < kDptRxTxGuardMs) {
    return;
  }
  if (now - lastDptProbeMs >= kDptProbeMs) {
    DptSerial.write(bearamin_bridge::kDptProbe, bearamin_bridge::kDptProbeSize);
    ++dptProbeTxCount;
    lastDptProbeMs = now;
  }
  const bool sourceAdvanced =
      matrixSourceSequence != lastDptSourceSequenceSent;
  const bool heartbeatDue = now - lastDptSendMs >= kDptSendMs;
  if (!sourceAdvanced && !heartbeatDue) {
    return;
  }
  const bearamin_bridge::Packet packet = buildDptPacket();
  uint8_t frame[bearamin_bridge::kFrameSizeV11] = {};
  const size_t length = bearamin_bridge::encodeFrameV11(
      packet, frame, sizeof(frame));
  if (length > 0) {
    DptSerial.write(frame, length);
    ++dptFrameTxCount;
    lastDptSendMs = now;
    lastDptSourceSequenceSent = matrixSourceSequence;
    captureProcessedSnapshot();
    captureDptControl(frame, length);
  }
}

void rememberDptRxByte(uint8_t byte) {
  dptRxDebugBytes[dptRxDebugWriteIndex % sizeof(dptRxDebugBytes)] = byte;
  dptRxDebugWriteIndex = (dptRxDebugWriteIndex + 1) % sizeof(dptRxDebugBytes);
  if (dptRxDebugCount < sizeof(dptRxDebugBytes)) {
    ++dptRxDebugCount;
  }
}

void sendDptRxDiag(uint32_t now) {
  if (now - lastDptRxDiagMs < kDiagnosticLogMs || dptRxDebugCount == 0) {
    return;
  }
  lastDptRxDiagMs = now;
  char line[96] = {};
  int written = snprintf(line,
                         sizeof(line),
                         "BPDG,%lu,%lu,%lu,",
                         static_cast<unsigned long>(dptRxByteCount),
                         static_cast<unsigned long>(dptStatusLineCount),
                         static_cast<unsigned long>(dptPresetLineCount));
  if (written <= 0) {
    return;
  }
  size_t cursor = static_cast<size_t>(written);
  const char hex[] = "0123456789ABCDEF";
  const uint8_t start = dptRxDebugCount < sizeof(dptRxDebugBytes)
                            ? 0
                            : dptRxDebugWriteIndex;
  for (uint8_t i = 0; i < dptRxDebugCount && cursor + 2 < sizeof(line); ++i) {
    const uint8_t byte = dptRxDebugBytes[(start + i) % sizeof(dptRxDebugBytes)];
    line[cursor++] = hex[byte >> 4];
    line[cursor++] = hex[byte & 0x0F];
  }
  line[cursor] = '\0';
  sendPresetLineToSidecar(line);
}

void pollDptStatus() {
  static char line[128] = {};
  static uint8_t index = 0;
  while (DptSerial.available()) {
    const uint8_t byte = static_cast<uint8_t>(DptSerial.read());
    const char c = static_cast<char>(byte);
    lastDptWireRxByteMs = millis();
    ++dptRxByteCount;
    rememberDptRxByte(byte);
    if (c == '\n' || c == '\r') {
      line[index] = '\0';
      if (index > 0) {
        if (line[0] == 'B' && line[1] == 'P') {
          ++dptPresetLineCount;
          sendPresetLineToSidecar(line);
          if (bearamin_preset::containsTerminalResponse(line)) {
            dptPresetQuietUntilMs = 0;
          }
        } else {
          bearamin_bridge::DptStatus parsed = {};
          if (bearamin_bridge::decodeDptStatusLine(line, parsed)) {
            ++dptStatusLineCount;
            if (parsed.protocolVersion == 0 && dptStatus.protocolVersion > 0) {
              dptStatus.modeIndex = parsed.modeIndex;
            } else {
              dptStatus = parsed;
            }
            lastDptRxMs = millis();
            captureControlRecord(
                bearamin_capture::CONTROL_DPT_STATUS,
                reinterpret_cast<const uint8_t *>(line),
                index);
          } else {
            bearamin_bridge::DptPerformance performance = {};
            if (bearamin_bridge::decodeDptPerformanceLine(line, performance)) {
              sendPresetLineToSidecar(line);
              captureControlRecord(
                  bearamin_capture::CONTROL_DPT_STATUS,
                  reinterpret_cast<const uint8_t *>(line),
                  index);
            } else {
              bearamin_bridge::DptTrajectoryDiagnostics trajectoryDiagnostics = {};
              if (bearamin_bridge::decodeDptTrajectoryDiagnosticsLine(
                      line, trajectoryDiagnostics)) {
                dptTrajectoryDiagnostics = trajectoryDiagnostics;
                ++dptTrajectoryDiagnosticsLineCount;
                lastDptRxMs = millis();
                sendPresetLineToSidecar(line);
                captureControlRecord(
                    bearamin_capture::CONTROL_DPT_STATUS,
                    reinterpret_cast<const uint8_t *>(line),
                    index);
              }
            }
          }
        }
      }
      index = 0;
    } else if (static_cast<uint8_t>(c) < 32 || static_cast<uint8_t>(c) > 126) {
      index = 0;
    } else if (index == 0 && c != 'D' && c != 'B') {
      continue;
    } else if (index + 1 < sizeof(line)) {
      line[index++] = c;
    } else {
      index = 0;
    }
  }
}

void sendRuntimeSettings(uint16_t requestSequence) {
  bearamin_sidecar::RuntimeSettingsFrame settings = {};
  settings.sequence = requestSequence;
  settings.dptModeIndex = dptStatus.modeIndex;
  settings.modeAnnouncementsEnabled = modeAnnouncementsEnabled;
  settings.controlsInitializedBySidecar = sidecarControlsInitialized;
  settings.cvSlewStep = cvSlewStep;
  settings.dptControlFlags = dptControlFlags;
  settings.dptReserved0 = dptReserved0;
  settings.dptReserved1 = dptReserved1;
  settings.dptReserved2 = dptReserved2;
  settings.dptSourceFlags2 = dptSourceFlags2;
  settings.resonatorControlsRevision = resonatorControlsRevision;
  for (uint8_t i = 0; i < bearamin_bridge::kLaneDepthCount; ++i) {
    settings.laneDepths[i] = dptLaneDepths[i];
  }
  for (uint8_t i = 0; i < bearamin_bridge::kResonatorWebControlSize; ++i) {
    settings.resonatorWebControls[i] = resonatorWebControls[i];
  }
  uint8_t frame[bearamin_sidecar::kRuntimeSettingsFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeRuntimeSettingsFrame(
      settings, frame, sizeof(frame));
  if (length > 0) {
    SidecarSerial.write(frame, length);
  }
}

void captureSidecarCommand(
    const bearamin_sidecar::CommandFrame &command) {
  uint8_t frame[bearamin_sidecar::kCommandFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeCommandFrame(
      command, bearamin_sidecar::FRAME_COMMAND, frame, sizeof(frame));
  if (length > 0) {
    captureControlRecord(
        bearamin_capture::CONTROL_SIDECAR_COMMAND, frame, length);
  }
}

void applySidecarCommand(const bearamin_sidecar::CommandFrame &command) {
  sidecarCommandSequence = command.sequence;
  if (captureActive &&
      command.command != bearamin_sidecar::COMMAND_CAPTURE_START) {
    captureSidecarCommand(command);
  }
  bool initializesControls = false;
  if (command.command == bearamin_sidecar::COMMAND_SET_DPT_MODE) {
    dptStatus.modeIndex = static_cast<uint8_t>(command.arg0 & 0xFFu);
    initializesControls = true;
  } else if (command.command == bearamin_sidecar::COMMAND_SET_CONTROL_FLAGS) {
    dptControlFlags = command.arg0;
    dptReserved0 = command.arg1;
    dptReserved1 = static_cast<uint8_t>(command.arg2 & 0xFFu);
    dptSourceFlags2 = static_cast<uint8_t>((command.arg2 >> 8) & 0xFFu);
    dptReserved2 = static_cast<uint8_t>(command.arg3 & 0xFFu);
    initializesControls = true;
  } else if (command.command == bearamin_sidecar::COMMAND_SET_CV_SLEW) {
    cvSlewStep = command.arg0 > kCvSlewMaxStep ? kCvSlewMaxStep : command.arg0;
    initializesControls = true;
  } else if (command.command == bearamin_sidecar::COMMAND_SET_SIDECAR_STATUS) {
    sidecarStatusFlags = command.arg0;
    lastSidecarStatusMs = millis();
  } else if (command.command == bearamin_sidecar::COMMAND_ENTER_MATRIX_OTA) {
    startMatrixOtaMode(millis());
  } else if (command.command == bearamin_sidecar::COMMAND_SET_ENGINEERING_STREAM) {
    engineeringStreamEnabled = command.arg0 != 0;
  } else if (command.command == bearamin_sidecar::COMMAND_SET_RESONATOR_WEB_PAGE) {
    resonatorWebPageAssembler.Apply(
        command, resonatorControlsRevision, resonatorWebControls);
    initializesControls = true;
  } else if (command.command == bearamin_sidecar::COMMAND_SET_LANE_DEPTHS) {
    const uint16_t words[3] = {command.arg1, command.arg2, command.arg3};
    for (uint8_t i = 0; i < bearamin_bridge::kLaneDepthCount; ++i) {
      dptLaneDepths[i] = static_cast<int8_t>(
          (words[i / 2] >> ((i % 2) * 8)) & 0xFFu);
    }
    initializesControls = true;
  } else if (command.command == bearamin_sidecar::COMMAND_SET_MODE_ANNOUNCEMENTS) {
    modeAnnouncementsEnabled = command.arg0 != 0;
    initializesControls = true;
  } else if (command.command == bearamin_sidecar::COMMAND_REQUEST_RUNTIME_SETTINGS) {
    sendRuntimeSettings(command.sequence);
  } else if (command.command == bearamin_sidecar::COMMAND_CAPTURE_START) {
    const uint32_t token =
        static_cast<uint32_t>(command.arg0) |
        (static_cast<uint32_t>(command.arg1) << 16);
    captureStart(token);
    captureSidecarCommand(command);
  } else if (command.command == bearamin_sidecar::COMMAND_CAPTURE_STOP) {
    captureRequestStop();
  } else if (command.command == bearamin_sidecar::COMMAND_ENTER_LORENZ) {
    chaosUnlock.force();
    chaosLorenz.seed(
        static_cast<uint32_t>(esp_timer_get_time()) ^ command.sequence);
    startChaosActivationFeedback(millis());
    serialLogPrintln("chaos=enabled source=sidecar");
  }
  if (initializesControls) {
    sidecarControlsInitialized = true;
  }
}

// NVS commits a single blob atomically; retain the old default on failed saves.
// Open without erasing/reinitializing NVS: other system namespaces are unrelated.
struct DefaultPresetNvs {
  uint8_t Read(uint8_t *data, size_t &length) {
    nvs_handle_t handle;
    esp_err_t result = nvs_open("bearamin-boot", NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return bearamin_default::kMissing;
    if (result != ESP_OK) return bearamin_default::kStorage;
    result = nvs_get_blob(handle, "default", data, &length);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return bearamin_default::kMissing;
    if (result == ESP_ERR_NVS_INVALID_LENGTH) return bearamin_default::kInvalid;
    return result == ESP_OK ? bearamin_default::kOk : bearamin_default::kStorage;
  }
  bool Write(const uint8_t *data, size_t length) {
    nvs_handle_t handle;
    if (nvs_open("bearamin-boot", NVS_READWRITE, &handle) != ESP_OK) return false;
    const bool ok = nvs_set_blob(handle, "default", data, length) == ESP_OK &&
                    nvs_commit(handle) == ESP_OK;
    nvs_close(handle);
    return ok;
  }
};
DefaultPresetNvs defaultPresetNvs;
bearamin_default::Store<DefaultPresetNvs> defaultPresetStore(defaultPresetNvs);

void handleDefaultPreset(const bearamin_sidecar::PresetFrame &request) {
  const auto reply = defaultPresetStore.Handle(request);
  uint8_t frame[bearamin_sidecar::kPresetFrameSize] = {};
  const size_t length = bearamin_sidecar::encodePresetFrame(reply, frame, sizeof(frame));
  if (length) SidecarSerial.write(frame, length);
}

void pollSidecarCommands() {
  while (SidecarSerial.available()) {
    bearamin_sidecar::StateFrame unusedState = {};
    bearamin_sidecar::CommandFrame command = {};
    bearamin_sidecar::MatrixCellsFrame unusedCells = {};
    bearamin_sidecar::PresetFrame preset = {};
    const uint8_t sidecarByte = static_cast<uint8_t>(SidecarSerial.read());
    ++sidecarRxByteCount;
    const uint8_t type = sidecarParser.Push(
        sidecarByte, unusedState, command, unusedCells, preset);
    if (type == bearamin_sidecar::FRAME_COMMAND) {
      ++sidecarCommandFrameCount;
      applySidecarCommand(command);
      uint8_t ack[bearamin_sidecar::kCommandFrameSize] = {};
      const size_t length = bearamin_sidecar::encodeCommandFrame(
          command, bearamin_sidecar::FRAME_ACK, ack, sizeof(ack));
      if (length > 0) {
        SidecarSerial.write(ack, length);
      }
    } else if (type == bearamin_sidecar::FRAME_PRESET &&
               bearamin_default::isOperation(preset.op)) {
      handleDefaultPreset(preset);
    } else if (type == bearamin_sidecar::FRAME_PRESET &&
               preset.op == bearamin_sidecar::PRESET_OP_LINE &&
               preset.length > 0) {
      const uint32_t now = millis();
      if (!bearamin_preset::quietWindowActive(
              dptPresetQuietUntilMs, now)) {
        dptPresetQuietUntilMs =
            now + bearamin_preset::kQuietWatchdogMs;
      }
      DptSerial.write(preset.data, preset.length);
      DptSerial.flush();
      ++dptPresetTxCount;
    }
  }
}

bearamin_sidecar::StateFrame buildSidecarState(uint32_t now) {
  bearamin_sidecar::StateFrame state = {};
  state.sequence = sidecarSequence++;
  state.uptimeMs = now;
  if (lastDptRxMs != 0 && now - lastDptRxMs <= kSidecarStatusStaleMs) {
    state.stateFlags |= bearamin_sidecar::STATE_DPT_READY;
  }
  if (optical.matrixReady) {
    state.stateFlags |= bearamin_sidecar::STATE_MATRIX_READY;
  }
  if (optical.matrixPresent) {
    state.stateFlags |= bearamin_sidecar::STATE_MATRIX_PRESENT;
  }
  if (imuReady) {
    state.stateFlags |= bearamin_sidecar::STATE_IMU_READY;
  }
  if (latestImpact.gate) {
    state.stateFlags |= bearamin_sidecar::STATE_IMPACT_GATE;
  }
  if (optical.spankGate) {
    state.stateFlags |= bearamin_sidecar::STATE_SPANK_GATE;
  }
  if (matrixOtaMode) {
    state.stateFlags |= bearamin_sidecar::STATE_MATRIX_OTA_MODE;
  }
  if (matrixOtaActive) {
    state.stateFlags |= bearamin_sidecar::STATE_MATRIX_OTA_ACTIVE;
  }
  if (bearamin_preset::quietWindowActive(dptPresetQuietUntilMs, now)) {
    state.stateFlags |= bearamin_sidecar::STATE_DPT_CONTROL_TX_QUIET;
  }
  if (modeAnnouncementsEnabled) {
    state.stateFlags |= bearamin_sidecar::STATE_MODE_ANNOUNCEMENTS_ENABLED;
  }
  if (opticalRecoveryActive()) {
    state.stateFlags |= bearamin_sidecar::STATE_OPTICAL_RECOVERY_ACTIVE;
  }
  if (optical.syntheticGate) {
    state.stateFlags |= bearamin_sidecar::STATE_SYNTHETIC_GATE;
  }
  state.matrixZMm = optical.matrixZMm;
  state.matrixXMilli = optical.matrixXMilli;
  state.matrixYMilli = optical.matrixYMilli;
  state.matrixZNorm = optical.matrixZNorm;
  state.matrixXNorm = optical.matrixXNorm;
  state.matrixYNorm = optical.matrixYNorm;
  state.matrixValidCount = optical.matrixValidCount;
  state.matrixBlobCount = optical.matrixBlobCount;
  state.impactVelocityNorm = latestImpact.velocityNorm;
  state.impactAgeMs = latestImpact.ageMs;
  state.accelXMilliG = static_cast<int16_t>(latestImu.accel.x * 1000.0f);
  state.accelYMilliG = static_cast<int16_t>(latestImu.accel.y * 1000.0f);
  state.accelZMilliG = static_cast<int16_t>(latestImu.accel.z * 1000.0f);
  state.dptModeIndex = dptStatus.modeIndex;
  state.dptProtocolVersion = dptStatus.protocolVersion;
  state.dptFlags = dptStatus.flags;
  for (uint8_t i = 0; i < bearamin_bridge::kDptControlCount; ++i) {
    state.dptControls[i] = dptStatus.controls[i];
  }
  state.dptControlFlags = dptControlFlags;
  state.dptResonatorControlsRevision = dptStatus.resonatorControlsRevision;
  state.pitchNorm = optical.pitchNorm;
  state.rollNorm = optical.rollNorm;
  if (trajectoryAuthority.hasKeyframe()) {
    const bearamin_trajectory::KeyframeV1 &keyframe =
        trajectoryAuthority.keyframe();
    state.uptimeMs = keyframe.sourceEpochMs;
    state.matrixXNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_X];
    state.matrixYNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_Y];
    state.matrixZNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_Z];
    state.pitchNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_PITCH];
    state.rollNorm = keyframe.targetNorm[bearamin_trajectory::AXIS_ROLL];
    for (uint8_t axis = 0; axis < bearamin_trajectory::kAxisCount; ++axis) {
      state.trajectoryStartNorm[axis] = keyframe.startNorm[axis];
    }
    state.trajectoryTauMs = keyframe.slewTauMs;
    state.trajectoryValid = true;
  }
  state.orientationConfidence = optical.orientationConfidence;
  state.sensorAgeMs = capAgeMs(optical.lastMatrixReadMs);
  state.frameErrors = sidecarFrameErrors;
  state.i2cRecoveries = optical.i2cRecoveries;
  state.dptProbeTxCount = static_cast<uint16_t>(dptProbeTxCount);
  state.dptFrameTxCount = static_cast<uint16_t>(dptFrameTxCount);
  state.dptPresetTxCount = static_cast<uint16_t>(dptPresetTxCount);
  state.dptRxByteCount = static_cast<uint16_t>(dptRxByteCount);
  state.dptStatusLineCount = static_cast<uint16_t>(dptStatusLineCount);
  state.dptPresetLineCount = static_cast<uint16_t>(dptPresetLineCount);
  for (uint8_t i = 0; i < 8; ++i) {
    state.cellsMm[i] = optical.cellsMm[i];
  }
  return state;
}

void sendSidecarState(uint32_t now) {
  if (now - lastSidecarStateMs < kSidecarStateMs) {
    return;
  }
  lastSidecarStateMs = now;
  const bearamin_sidecar::StateFrame state = buildSidecarState(now);
  uint8_t frame[bearamin_sidecar::kStateFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeStateFrame(state, frame, sizeof(frame));
  if (length > 0) {
    SidecarSerial.write(frame, length);
  }
}

bearamin_sidecar::MatrixCellsFrame buildSidecarCells(uint32_t now) {
  bearamin_sidecar::MatrixCellsFrame cells = {};
  cells.sequence = sidecarSequence++;
  cells.validCount = optical.matrixValidCount;
  cells.blobCount = optical.matrixBlobCount;
  cells.sensorAgeMs = capAgeMs(optical.lastMatrixReadMs);
  for (uint8_t index = 0; index < 64; ++index) {
    cells.cellsMm[index] = optical.cellsMm[index];
  }
  return cells;
}

void sendSidecarCells(uint32_t now) {
  if (!engineeringStreamEnabled ||
      lastSidecarStatusMs == 0 ||
      now - lastSidecarStatusMs > kSidecarStatusStaleMs) {
    return;
  }
  if (now - lastSidecarCellsMs < kSidecarCellsMs) {
    return;
  }
  lastSidecarCellsMs = now;
  const bearamin_sidecar::MatrixCellsFrame cells = buildSidecarCells(now);
  uint8_t frame[bearamin_sidecar::kMatrixCellsFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeMatrixCellsFrame(cells, frame, sizeof(frame));
  if (length > 0) {
    SidecarSerial.write(frame, length);
  }
}

bearamin_sidecar::SensorDiagnosticsFrame buildSensorDiagnostics() {
  bearamin_sidecar::SensorDiagnosticsFrame diagnostics = {};
  diagnostics.sequence = sidecarSequence++;
  diagnostics.recoveryFailures = opticalRecoveryFailures;
  diagnostics.matrixAgeMs = capAgeMs(optical.lastMatrixReadMs);
  diagnostics.recoveryReason = lastOpticalRecoveryReason;
  diagnostics.busFlags =
      optical.i2cRecoveries > 0 ? lastRecoveryBusFlags : opticalBusFlags();
  return diagnostics;
}

void sendSensorDiagnostics(uint32_t now) {
  if (now - lastSidecarSensorDiagnosticsMs < kSidecarSensorDiagnosticsMs) {
    return;
  }
  lastSidecarSensorDiagnosticsMs = now;
  const bearamin_sidecar::SensorDiagnosticsFrame diagnostics =
      buildSensorDiagnostics();
  uint8_t frame[bearamin_sidecar::kSensorDiagnosticsFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeSensorDiagnosticsFrame(
      diagnostics, frame, sizeof(frame));
  if (length > 0) {
    SidecarSerial.write(frame, length);
    captureDiagnostic();
  }
}

void sendMatrixPerformance(uint32_t now) {
  if (now - lastSidecarMatrixPerformanceMs <
      kSidecarSensorDiagnosticsMs) {
    return;
  }
  lastSidecarMatrixPerformanceMs = now;
  if (opticalRecoveryTaskHandle != nullptr) {
    matrixRecoveryStackMinimumWords = std::min<uint16_t>(
        matrixRecoveryStackMinimumWords,
        static_cast<uint16_t>(uxTaskGetStackHighWaterMark(
            opticalRecoveryTaskHandle)));
  }
  bearamin_sidecar::MatrixPerformanceFrame performance = {};
  performance.sequence = sidecarSequence++;
  performance.effectiveRateMilliHz = matrixEffectiveRateMilliHz;
  performance.intervalMaxUs = matrixIntervalMaxUs;
  performance.readMaxUs = matrixReadMaxUs;
  performance.processMaxUs = matrixProcessMaxUs;
  performance.loopGapMaxUs = matrixLoopGapMaxUs;
  performance.freeHeapBytes = ESP.getFreeHeap();
  performance.minimumFreeHeapBytes = ESP.getMinFreeHeap();
  performance.readErrors = matrixReadErrors;
  performance.recoveryStackWords =
      matrixRecoveryStackMinimumWords == 0xFFFFu
          ? 0
          : matrixRecoveryStackMinimumWords;
  performance.sourceHz = kMatrixSourceHz;
  performance.outputHz = kProcessedOutputHz;
  performance.readyPollStatus = matrixReadyPollStatus;
  performance.readyPollValue = matrixReadyPollValue;
  uint8_t frame[bearamin_sidecar::kMatrixPerformanceFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeMatrixPerformanceFrame(
      performance, frame, sizeof(frame));
  if (length > 0) {
    SidecarSerial.write(frame, length);
  }
}

void renderLeds(uint32_t now) {
  static bool havePreviousVector = false;
  static uint16_t previousXMilli = 500;
  static uint16_t previousYMilli = 500;

  if (renderChaosActivationFeedback(now)) {
    return;
  }

  clearLeds();
  if (optical.matrixReady) {
    for (uint8_t displayRow = 0; displayRow < kMatrixWidth; ++displayRow) {
      for (uint8_t column = 0; column < kMatrixWidth; ++column) {
        const uint16_t distance =
            optical.cellsMm[static_cast<uint8_t>(displayRow * 8 + column)];
        if (distance == 0 || distance > kStatusBackgroundMaxMm) {
          continue;
        }
        addPixel(
            column,
            displayRow,
            Color{
                kStatusBackgroundBrightness,
                kStatusBackgroundBrightness,
                kStatusBackgroundBrightness});
      }
    }
  }
  if (optical.matrixReady) {
    const bool activeVector = trajectoryOutputValid && optical.matrixPresent;
    const uint16_t evaluatedXMilli = trajectoryOutputValid
        ? static_cast<uint16_t>(
              (static_cast<uint32_t>(trajectoryOutputAxes.norm[
                   bearamin_trajectory::AXIS_X]) * 1000UL +
               bearamin_trajectory::kNormMaximum / 2UL) /
              bearamin_trajectory::kNormMaximum)
        : 500;
    const uint16_t evaluatedYMilli = trajectoryOutputValid
        ? static_cast<uint16_t>(
              (static_cast<uint32_t>(trajectoryOutputAxes.norm[
                   bearamin_trajectory::AXIS_Y]) * 1000UL +
               bearamin_trajectory::kNormMaximum / 2UL) /
              bearamin_trajectory::kNormMaximum)
        : 500;
    const uint16_t xMilli = expandVectorMilli(evaluatedXMilli);
    const uint16_t yMilli = expandVectorMilli(evaluatedYMilli);
    const int16_t dxMilli = havePreviousVector ? static_cast<int16_t>(static_cast<int32_t>(xMilli) - previousXMilli) : 0;
    const int16_t dyMilli = havePreviousVector ? static_cast<int16_t>(static_cast<int32_t>(yMilli) - previousYMilli) : 0;
    previousXMilli = xMilli;
    previousYMilli = yMilli;
    havePreviousVector = true;

    const float centerX = static_cast<float>(xMilli) / 1000.0f;
    const float centerY = static_cast<float>(yMilli) / 1000.0f;
    const float dx = static_cast<float>(dxMilli) / 1000.0f;
    const float dy = static_cast<float>(dyMilli) / 1000.0f;
    const float speed = std::sqrt(dx * dx + dy * dy);
    const float speedAmount = speed > 0.050f ? 1.0f : speed / 0.050f;
    const float invSpeed = speed > 0.0001f ? 1.0f / speed : 0.0f;
    const float vx = dx * invSpeed;
    const float vy = dy * invSpeed;
    const float tailLength = 0.45f + speedAmount * 4.2f;
    const float tailWidth = 0.42f + speedAmount * 0.18f;
    const uint16_t depthMm = activeVector ? statusVectorDepthMm() : kPlayableMaxMm;
    const uint8_t dotBrightness = activeVector ? kStatusVectorDotBrightness : kStatusVectorIdleDotBrightness;
    const uint8_t tailBrightness = activeVector ? kStatusVectorTailMinBrightness : kStatusVectorIdleTailMinBrightness;

    for (uint8_t row = 0; row < 8; ++row) {
      for (uint8_t column = 0; column < 8; ++column) {
        const float cellX = static_cast<float>(column) / 7.0f;
        const float cellY = static_cast<float>(row) / 7.0f;
        const float relX = (cellX - centerX) * 7.0f;
        const float relY = (cellY - centerY) * 7.0f;
        const float distance = std::sqrt(relX * relX + relY * relY);
        float intensity = distance < 0.75f ? 1.0f - (distance / 0.75f) : 0.0f;
        if (speed > 0.0001f) {
          const float behind = -(relX * vx + relY * vy);
          const float side = std::fabs(relX * -vy + relY * vx);
          if (behind >= 0.0f && behind <= tailLength && side <= tailWidth) {
            const float along = 1.0f - (behind / tailLength);
            const float lateral = 1.0f - (side / tailWidth);
            const float tail = along * along * lateral * (0.30f + 0.70f * speedAmount);
            if (tail > intensity) {
              intensity = tail;
            }
          }
        }
        if (intensity <= 0.06f) {
          continue;
        }
        const uint8_t brightness = static_cast<uint8_t>(
            tailBrightness +
            (intensity * (dotBrightness - tailBrightness)));
        addPixel(
            column,
            row,
            vectorDepthColor(brightness, depthMm, cvBridgeZ));
      }
    }
    if (!activeVector) {
      // Exact center falls between four cells on an even-sized grid, where the
      // radial dot alone drops below its cutoff. Keep one anchor visible and
      // preserve the normal Z midpoint marker so bipolar neutral is purple.
      addPixel(
          nearestLedCoordinateForMilli(xMilli, kMatrixWidth),
          nearestLedCoordinateForMilli(yMilli, kMatrixWidth),
          vectorDepthColor(
              kStatusVectorIdleDotBrightness, kPlayableMaxMm, cvBridgeZ));
    }
  } else {
    havePreviousVector = false;
  }
  if (latestImpact.gate) {
    const uint8_t flash = latestImpact.velocityNorm > 3000 ? 190 : 110;
    for (uint8_t i = 0; i < 8; ++i) {
      addPixel(i, i, Color{flash, flash / 3, 0});
      addPixel(7 - i, i, Color{flash, flash / 3, 0});
    }
  }
  setTopStatusPixel(0, wifiStatusColor(now));
  const Color matrixSensorStatus = readyColor(optical.matrixReady);
  if (kMergedMatrixSensorStatus) {
    setTopStatusPixel(3, matrixSensorStatus);
    setTopStatusPixel(4, matrixSensorStatus);
  } else {
    setTopStatusPixel(
        3,
        optical.syntheticGate ? Color{0, 150, 0} : Color{45, 20, 0});
    setTopStatusPixel(4, matrixSensorStatus);
  }
  setTopStatusPixel(7, dptStatusColor(now));
  (void)now;
  writeLeds();
}

void logDiagnostics(uint32_t now) {
  if (now - lastDiagnosticLogMs < kDiagnosticLogMs) {
    return;
  }
  lastDiagnosticLogMs = now;
  serialLogPrintf(
      "status uptime_ms=%lu matrix_ready=%u matrix_present=%u matrix_age_ms=%u valid=%u blobs=%u "
      "matrix_z_mm=%u x_norm=%u y_norm=%u pitch_norm=%u roll_norm=%u "
      "orientation_confidence=%u orientation_fit_us=%u orientation_fit_max_us=%u "
      "spank_gate=%u imu_ready=%u impact_gate=%u "
      "impact_velocity=%u impact_age_ms=%u sidecar_status=0x%04x sidecar_age_ms=%u "
      "i2c_recoveries=%u recovery_failures=%u "
      "recovery_reason=%s\n",
      static_cast<unsigned long>(now),
      optical.matrixReady ? 1 : 0,
      optical.matrixPresent ? 1 : 0,
      capAgeMs(optical.lastMatrixReadMs),
      optical.matrixValidCount,
      optical.matrixBlobCount,
      optical.matrixZMm,
      optical.matrixXNorm,
      optical.matrixYNorm,
      optical.pitchNorm,
      optical.rollNorm,
      optical.orientationConfidence,
      optical.orientationFitUs,
      optical.orientationFitMaxUs,
      optical.spankGate ? 1 : 0,
      imuReady ? 1 : 0,
      latestImpact.gate ? 1 : 0,
      latestImpact.velocityNorm,
      latestImpact.ageMs,
      sidecarStatusFlags,
      capAgeMs(lastSidecarStatusMs),
      optical.i2cRecoveries,
      opticalRecoveryFailures,
      sensorRecoveryReasonName(lastOpticalRecoveryReason));
  // Keep the motion readout below the USB CDC write-buffer size. The full
  // status line is intentionally best-effort and can be dropped when the
  // transport cannot accept its complete formatted record atomically.
  serialLogPrintf(
      "matrix_motion ready=%u present=%u valid=%u blobs=%u z_mm=%u "
      "x_milli=%u y_milli=%u x_norm=%u y_norm=%u\n",
      optical.matrixReady ? 1 : 0,
      optical.matrixPresent ? 1 : 0,
      optical.matrixValidCount,
      optical.matrixBlobCount,
      optical.matrixZMm,
      optical.matrixXMilli,
      optical.matrixYMilli,
      optical.matrixXNorm,
      optical.matrixYNorm);
}

void logDptDiagnostics(uint32_t now) {
  if (now - lastDptConsoleLogMs < kDiagnosticLogMs) {
    return;
  }
  lastDptConsoleLogMs = now;
  serialLogPrintf(
      "dpt_uart ready=%u protocol=%u mode=%u age_ms=%u "
      "trajectory_protocol=%u tau_ms=%u v11=%lu corrections=%lu "
      "rx_bytes=%lu status_lines=%lu probes_tx=%lu frames_tx=%lu\n",
      lastDptRxMs != 0 && now - lastDptRxMs <= kSidecarStatusStaleMs ? 1 : 0,
      dptStatus.protocolVersion,
      dptStatus.modeIndex,
      capAgeMs(lastDptRxMs),
      dptTrajectoryDiagnostics.bridgeProtocolVersion,
      dptTrajectoryDiagnostics.slewTauMs,
      static_cast<unsigned long>(dptTrajectoryDiagnostics.v11PacketCount),
      static_cast<unsigned long>(dptTrajectoryDiagnostics.correctionCount),
      static_cast<unsigned long>(dptRxByteCount),
      static_cast<unsigned long>(dptStatusLineCount),
      static_cast<unsigned long>(dptProbeTxCount),
      static_cast<unsigned long>(dptFrameTxCount));
}

void logSidecarDiagnostics(uint32_t now) {
  if ((lastSidecarConsoleLogMs == 0 &&
       now < kDiagnosticLogMs + kDiagnosticLogMs / 2) ||
      (lastSidecarConsoleLogMs != 0 &&
       now - lastSidecarConsoleLogMs < kDiagnosticLogMs)) {
    return;
  }
  lastSidecarConsoleLogMs = now;
  serialLogPrintf(
      "sidecar_uart ready=%u age_ms=%u status=0x%04x "
      "rx_bytes=%lu command_frames=%lu\n",
      lastSidecarStatusMs != 0 &&
              now - lastSidecarStatusMs <= kSidecarStatusStaleMs
          ? 1
          : 0,
      capAgeMs(lastSidecarStatusMs),
      sidecarStatusFlags,
      static_cast<unsigned long>(sidecarRxByteCount),
      static_cast<unsigned long>(sidecarCommandFrameCount));
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(80);
  serialLogPrintln("Bearamin Matrix S3 core POC");
  bearamin_bridge::setDefaultResonatorWebControls(resonatorWebControls);
  serialLogPrintf(
      "optical_i2c sensor=%s sda=GPIO%u scl=GPIO%u lpr=GPIO%u aux=GPIO%u clock_hz=%lu\n",
      kMatrixSensorName, kOpticalSdaPin, kOpticalSclPin, kMatrixLprPin,
      kMatrixAuxPin,
      static_cast<unsigned long>(kOpticalClockHz));
  ledReady = rmtInit(kLedPin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  clearLeds();
  writeLeds();
  chaosUnlock.begin(millis());
  pinMode(kDptRxPin, INPUT_PULLUP);
  DptSerial.begin(kDptBaud, SERIAL_8N1, kDptRxPin, kDptTxPin);
  SidecarSerial.setTxBufferSize(kSidecarTxBufferBytes);
  SidecarSerial.begin(kSidecarBaud, SERIAL_8N1, kSidecarRxPin, kSidecarTxPin);
  WiFi.mode(WIFI_OFF);
  imuReady = initImu();
  if (!imuReady) {
    latestImpact = unavailableImpactState();
    nextImuRetryMs = millis() + kImuRetryMinMs;
  }
  serialLogPrintf("imu=%s sda=GPIO%u scl=GPIO%u\n", imuReady ? "ready" : "missing", kImuSdaPin, kImuSclPin);
#if defined(BEARAMIN_MATRIX_SENSOR_VL53L8CX)
  prepareVl53l8cxOpticalBusForDeferredInit(optical);
#else
  initSharedOpticalBus(optical);
#endif
  lastRecoveryBusFlags = opticalBusFlags();
  startOpticalRecoveryWorker();
  resetMatrixPerformanceWindow();
  lastLoopUs = esp_timer_get_time();
}

void loop() {
  const uint64_t loopUs = esp_timer_get_time();
  if (lastLoopUs != 0) {
    matrixLoopGapMaxUs = std::max(
        matrixLoopGapMaxUs, static_cast<uint32_t>(loopUs - lastLoopUs));
  }
  lastLoopUs = loopUs;
  const uint32_t now = millis();
  constexpr uint32_t matrixPeriodMs =
      (1000u + kMatrixSourceHz - 1u) / kMatrixSourceHz;
  if (optical.lastMatrixReadMs == 0 ||
      now - optical.lastMatrixReadMs > matrixPeriodMs * 3u ||
      opticalRecoveryActive()) {
    optical.syntheticGate = false;
  }
  pollImu(now);
  pollMatrix(now);
  serviceProcessedOutput(loopUs);
  recoverOpticalIfNeeded(now);
  pollDptStatus();
  sendDptRxDiag(now);
  pollMatrixOta(now);
  sendDpt(millis());
  pollSidecarCommands();
  sendSidecarState(now);
  sendSidecarCells(now);
  sendSensorDiagnostics(now);
  sendMatrixPerformance(now);
  renderLeds(now);
  logDiagnostics(now);
  logDptDiagnostics(now);
  logSidecarDiagnostics(now);
  captureFinishStopIfPossible();
  captureDrainToSidecar();
}
