#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <SD.h>
#include "../../../common/default_preset_store.h"
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ctype.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <stdarg.h>

#if defined(BEARAMIN_TARGET_SIDECAR_C5_REVC_S3_LAYOUT)
#include "targets/xiao_esp32c5_revc_s3layout.h"
#elif defined(BEARAMIN_TARGET_SIDECAR_C5)
#include "targets/xiao_esp32c5.h"
#else
#include "targets/xiao_esp32s3.h"
#endif

#include "../../../common/capture_protocol.h"
#include "../../../common/obs_overlay_protocol.h"
#include "../../../common/preset_response_split.h"
#include "../../../common/sidecar_protocol.h"
#include "../../../common/wifi_band_logic.h"
#include "../../../BearaminSidecar/capture_worker_js.h"

#if __has_include("../../../common/wifi_config.h")
#include "../../../common/wifi_config.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef WIFI_SOFTAP_PASSWORD
#define WIFI_SOFTAP_PASSWORD ""
#endif

#ifndef MDNS_HOSTNAME
#define MDNS_HOSTNAME "bearamin"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

#ifndef BEARAMIN_MDNS_HOSTNAME
#define BEARAMIN_MDNS_HOSTNAME MDNS_HOSTNAME
#endif

#ifndef BEARAMIN_SOFTAP_SSID
#define BEARAMIN_SOFTAP_SSID "bearamin"
#endif

#ifndef BEARAMIN_FIRMWARE_VERSION
#define BEARAMIN_FIRMWARE_VERSION "bearamin-sidecar-matrix-poc"
#endif

#ifndef BEARAMIN_DEVICE_ROLE
#define BEARAMIN_DEVICE_ROLE "xiao_sidecar"
#endif

#ifndef BEARAMIN_BOOT_LABEL
#define BEARAMIN_BOOT_LABEL "Bearamin XIAO sidecar POC"
#endif

#ifndef BEARAMIN_WIFI_PREFER_5GHZ
#define BEARAMIN_WIFI_PREFER_5GHZ 0
#endif

#ifndef BEARAMIN_WIFI_PREFER_2GHZ
#define BEARAMIN_WIFI_PREFER_2GHZ 0
#endif

#ifndef BEARAMIN_MATRIX_RX_PIN
#define BEARAMIN_MATRIX_RX_PIN 4
#endif

#ifndef BEARAMIN_MATRIX_TX_PIN
#define BEARAMIN_MATRIX_TX_PIN 5
#endif

#ifndef BEARAMIN_DEBUG_RX_PIN
#define BEARAMIN_DEBUG_RX_PIN -1
#endif

#ifndef BEARAMIN_DEBUG_TX_PIN
#define BEARAMIN_DEBUG_TX_PIN 7
#endif

#ifndef BEARAMIN_SD_CS_PIN
#define BEARAMIN_SD_CS_PIN 3
#endif

#ifndef BEARAMIN_SD_SCK_PIN
#define BEARAMIN_SD_SCK_PIN 7
#endif

#ifndef BEARAMIN_SD_MISO_PIN
#define BEARAMIN_SD_MISO_PIN 8
#endif

#ifndef BEARAMIN_SD_MOSI_PIN
#define BEARAMIN_SD_MOSI_PIN 9
#endif

#ifndef BEARAMIN_SD_SPI_HZ
#define BEARAMIN_SD_SPI_HZ 1000000
#endif

constexpr uint8_t kMatrixRxPin = BEARAMIN_MATRIX_RX_PIN;
constexpr uint8_t kMatrixTxPin = BEARAMIN_MATRIX_TX_PIN;
constexpr int8_t kDebugRxPin = BEARAMIN_DEBUG_RX_PIN;
constexpr int8_t kDebugTxPin = BEARAMIN_DEBUG_TX_PIN;
constexpr uint8_t kSdCsPin = BEARAMIN_SD_CS_PIN;
constexpr uint8_t kSdSckPin = BEARAMIN_SD_SCK_PIN;
constexpr uint8_t kSdMisoPin = BEARAMIN_SD_MISO_PIN;
constexpr uint8_t kSdMosiPin = BEARAMIN_SD_MOSI_PIN;
constexpr uint32_t kSdSpiHz = BEARAMIN_SD_SPI_HZ;
constexpr uint32_t kMatrixBaud = 460800;
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kDebugBaud = 115200;
constexpr uint32_t kWifiTimeoutMs = 30000;
constexpr uint32_t kWifiPreferredBandAttemptMs = 8000;
constexpr bool kWifiPrefer5Ghz = BEARAMIN_WIFI_PREFER_5GHZ != 0;
constexpr bool kWifiPrefer2Ghz = BEARAMIN_WIFI_PREFER_2GHZ != 0;
constexpr uint32_t kMatrixStaleMs = 500;
constexpr uint32_t kSidecarStatusMs = 500;
constexpr uint32_t kEngineeringStreamTimeoutMs = 2500;
constexpr uint32_t kLogCellsFreshMs = 300;
constexpr uint16_t kLogRingSize = 240;
constexpr uint8_t kLogMaxFramesPerResponse = 40;
constexpr size_t kCapturePsramBytes = 1024 * 1024;
constexpr size_t kCaptureFallbackBytes = 64 * 1024;
constexpr size_t kCaptureHttpMaxBytes = 8192;
constexpr size_t kMatrixRxBufferBytes = 4096;
constexpr uint32_t kCaptureStopWaitMs = 750;
constexpr uint8_t kWifiDiagRingSize = 48;
constexpr uint8_t kPresetDefaultMaxAttempts = 5;
constexpr uint16_t kPresetDefaultRetryMs = 1500;
constexpr uint8_t kPresetSdMountMaxAttempts = 4;
constexpr uint16_t kPresetSdMountRetryDelayMs = 40;
constexpr uint32_t kPresetSdMinimumSpiHz = 100000;
constexpr uint32_t kRuntimeSettingsRetryMs = 1000;
constexpr size_t kPresetMaxJsonBytes = 12000;
constexpr size_t kPresetMaxNameLength = 48;
constexpr size_t kPresetMaxListedPresets = 24;
constexpr const char *kPresetDirectory = "/presets";
constexpr uint32_t kSoftApRescanMs = 30000;
constexpr uint32_t kSoftApReconnectWindowMs = 8000;
constexpr const char *kSoftApSsid = BEARAMIN_SOFTAP_SSID;
constexpr const char *kSoftApPassword = WIFI_SOFTAP_PASSWORD;

HardwareSerial MatrixSerial(1);
HardwareSerial DebugSerial(2);
WebServer server(80);
WiFiUDP obsTelemetryUdp;
const IPAddress obsMulticastIp(
    bearamin_obs::kMulticastAddress[0],
    bearamin_obs::kMulticastAddress[1],
    bearamin_obs::kMulticastAddress[2],
    bearamin_obs::kMulticastAddress[3]);
bearamin_sidecar::Parser matrixParser;
bearamin_capture::StreamParser<> captureParser;
const char kSensorFaviconSvg[] PROGMEM = R"SVG(<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" role="img" aria-label="Bearamin soundfield"><defs><linearGradient id="fieldBg" x1="6" y1="58" x2="58" y2="6" gradientUnits="userSpaceOnUse"><stop offset="0" stop-color="#071114"/><stop offset=".52" stop-color="#182029"/><stop offset="1" stop-color="#311a3d"/></linearGradient><linearGradient id="fieldStroke" x1="14" y1="46" x2="52" y2="14" gradientUnits="userSpaceOnUse"><stop offset="0" stop-color="#4df4d1"/><stop offset=".55" stop-color="#f5dd6b"/><stop offset="1" stop-color="#ff5ca8"/></linearGradient><filter id="softGlow" x="-40%" y="-40%" width="180%" height="180%"><feGaussianBlur stdDeviation="1.4" result="blur"/><feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge></filter></defs><rect width="64" height="64" rx="13" fill="url(#fieldBg)"/><path d="M12 43 32 18l20 25-20 11Z" fill="#0b171a" opacity=".86"/><path d="M17 42h30M22 35h20M26 29h12M18 42l14 12 14-12M22 35l10 19 10-19M26 29l6 25 6-25" fill="none" stroke="#8da5ac" stroke-width="1.4" opacity=".55"/><path d="M16 45c7-8 14-12 21-12 5 0 9 2 12 5" fill="none" stroke="url(#fieldStroke)" stroke-width="4.2" stroke-linecap="round" filter="url(#softGlow)"/><circle cx="16" cy="45" r="4.8" fill="#4df4d1"/><circle cx="37" cy="33" r="4.1" fill="#f5dd6b"/><circle cx="49" cy="38" r="4.5" fill="#ff5ca8"/><circle cx="32" cy="18" r="3.2" fill="#e8fbff"/><path d="M32 18v36" stroke="#e8fbff" stroke-width="2.1" stroke-linecap="round" opacity=".85"/></svg>)SVG";
const char kSoundfieldLogoSvg[] PROGMEM = R"SVG(
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" role="img" aria-label="Bearamin soundfield logo">
  <defs>
    <linearGradient id="fieldStroke" x1="14" y1="46" x2="52" y2="14" gradientUnits="userSpaceOnUse"><stop offset="0" stop-color="#4df4d1"/><stop offset=".55" stop-color="#f5dd6b"/><stop offset="1" stop-color="#ff5ca8"/></linearGradient>
    <filter id="softGlow" x="-60%" y="-60%" width="220%" height="220%"><feGaussianBlur stdDeviation="1.8" result="blur"/><feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
    <filter id="lineGlow" x="-40%" y="-40%" width="180%" height="180%"><feGaussianBlur stdDeviation=".55" result="blur"/><feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
  </defs>
  <path d="M12 43 32 18l20 25-20 11Z" fill="#071114" opacity=".62"/>
  <path d="M17 42h30M22 35h20M26 29h12M18 42l14 12 14-12M22 35l10 19 10-19M26 29l6 25 6-25" fill="none" stroke="#8da5ac" stroke-width="1.5" opacity=".72" filter="url(#lineGlow)"/>
  <path d="M16 45c7-8 14-12 21-12 5 0 9 2 12 5" fill="none" stroke="url(#fieldStroke)" stroke-width="4.8" stroke-linecap="round" filter="url(#softGlow)"/>
  <circle cx="16" cy="45" r="5.3" fill="#4df4d1" filter="url(#softGlow)"/><circle cx="37" cy="33" r="4.7" fill="#f5dd6b" filter="url(#softGlow)"/><circle cx="49" cy="38" r="5.1" fill="#ff5ca8" filter="url(#softGlow)"/><circle cx="32" cy="18" r="3.7" fill="#e8fbff" filter="url(#softGlow)"/>
  <path d="M32 18v36" stroke="#e8fbff" stroke-width="2.4" stroke-linecap="round" opacity=".95" filter="url(#lineGlow)"/>
</svg>)SVG";
bearamin_sidecar::StateFrame latestState = {};
bearamin_sidecar::MatrixCellsFrame latestCells = {};
bearamin_sidecar::SensorDiagnosticsFrame latestSensorDiagnostics = {};
bearamin_sidecar::RuntimeSettingsFrame latestRuntimeSettings = {};
bearamin_sidecar::MatrixPerformanceFrame latestMatrixPerformance = {};
bearamin_bridge::DptPerformance latestDptPerformance = {};
uint32_t lastStateMs = 0;
uint32_t lastCellsMs = 0;
uint32_t lastSensorDiagnosticsMs = 0;
uint32_t lastMatrixPerformanceMs = 0;
uint32_t lastDptPerformanceMs = 0;
uint32_t matrixBytes = 0;
uint32_t matrixFrames = 0;
uint32_t matrixCellFrames = 0;
uint32_t matrixSensorDiagnosticsFrames = 0;
uint32_t matrixPerformanceFrames = 0;
uint32_t dptPerformanceFrames = 0;
uint32_t matrixAcks = 0;
uint32_t matrixParserEvents = 0;
uint32_t commandSequence = 0;
uint32_t lastSidecarStatusMs = 0;
uint32_t lastEngineeringRequestMs = 0;
uint32_t lastResonatorControlSendMs = 0;
uint32_t lastRuntimeSettingsRequestMs = 0;
uint32_t lastSoftApRescanMs = 0;
bearamin_obs::SubscriberLeaseTable obsSubscriberLeases;
uint32_t obsLastClientNonce = 0;
uint32_t obsSubscriptions = 0;
uint32_t obsLeaseExpirations = 0;
uint32_t obsTableFullRejections = 0;
uint32_t obsStatePackets = 0;
uint32_t obsCellPackets = 0;
uint32_t obsSendFailures = 0;
uint32_t obsMalformedPackets = 0;
uint32_t obsMaxSendUs = 0;
uint16_t obsStateSequence = 0;
uint16_t obsCellSequence = 0;
bool obsStatePending = false;
bool obsCellsPending = false;
bool obsUdpReady = false;
bool wifiReady = false;
bool otaReady = false;
bool webReady = false;
bool softApReady = false;
bool debugSerialReady = false;
bool engineeringStreamEnabled = false;
bool modeAnnouncementsEnabled = false;
uint16_t defaultRequestSequence = 0;
bearamin_sidecar::PresetFrame defaultReply = {};
bool defaultReplyReady = false;
bool presetDefaultAvailable = false;
bool bootControlSyncComplete = false;
bool bootControlSyncUsedRuntime = false;
bool runtimeSettingsReceived = false;
bool fmDroneRangerMode = true;
uint8_t fmDroneMapX = bearamin_bridge::FM_PARAM_FM2_FREQUENCY;
uint8_t fmDroneMapY = bearamin_bridge::FM_PARAM_FM1_FREQUENCY;
uint8_t fmDroneMapZ = bearamin_bridge::FM_PARAM_FM1_AMOUNT;
constexpr uint8_t kGestureConfigBearaminFm = 0;
constexpr uint8_t kGestureConfigRingsResonator = 1;
constexpr uint8_t kGestureConfigCount = 2;
uint8_t modeGestureMaps[kGestureConfigCount][6] = {
    {7, 11, 3, 0, 0, 0},
    {0, 7, 3, 11, 0, 0},
};
int8_t modeGestureDepths[kGestureConfigCount][6] = {
    {50, 50, 50, 0, 0, 0},
    {0, 50, 50, 50, 0, 0},
};
uint16_t cvSlewStep = 16;
uint8_t resonatorControlsRevision = 1;
uint8_t resonatorWebControls[bearamin_bridge::kResonatorWebControlSize] = {
    0, 0, 0, 0, 20, 0, 0, 67, 0, 0, 63, 0, 0, 60, 0, 0};

struct LogFrameEntry {
  uint32_t seq;
  uint32_t sidecarMs;
  uint32_t coreMs;
  uint16_t stateFlags;
  uint8_t dptMode;
  uint8_t dptProtocolVersion;
  uint16_t dptFlags;
  uint16_t matrixXMilli;
  uint16_t matrixYMilli;
  uint16_t matrixZMm;
  uint16_t matrixXNorm;
  uint16_t matrixYNorm;
  uint16_t matrixZNorm;
  uint16_t pitchNorm;
  uint16_t rollNorm;
  uint8_t orientationConfidence;
  uint8_t matrixValidCount;
  uint8_t matrixBlobCount;
  uint16_t impactVelocityNorm;
  uint16_t impactAgeMs;
  int16_t accelXMilliG;
  int16_t accelYMilliG;
  int16_t accelZMilliG;
  uint16_t sampleCellsMm[8];
  uint16_t cellsMm[64];
  bool cellsPresent;
};

enum WifiDiagPhase : uint8_t {
  WIFI_DIAG_BOOT_START,
  WIFI_DIAG_CONNECT_WINDOW_START,
  WIFI_DIAG_SCAN_FAILED,
  WIFI_DIAG_SCAN_NO_SSID,
  WIFI_DIAG_SCAN_SELECTED,
  WIFI_DIAG_CONNECT_BEGIN,
  WIFI_DIAG_CONNECT_READY,
  WIFI_DIAG_CONNECT_TIMEOUT,
  WIFI_DIAG_CONNECT_WINDOW_FAILED,
  WIFI_DIAG_SOFTAP_START,
  WIFI_DIAG_SOFTAP_STOP,
  WIFI_DIAG_SOFTAP_RESCAN,
  WIFI_DIAG_MDNS_READY,
  WIFI_DIAG_MDNS_FAILED,
  WIFI_DIAG_WEB_READY,
  WIFI_DIAG_OTA_READY,
  WIFI_DIAG_POLL_LOST,
  WIFI_DIAG_EVENT_STA_START,
  WIFI_DIAG_EVENT_STA_CONNECTED,
  WIFI_DIAG_EVENT_STA_GOT_IP,
  WIFI_DIAG_EVENT_STA_DISCONNECTED,
  WIFI_DIAG_EVENT_STA_LOST_IP,
  WIFI_DIAG_EVENT_AP_START,
  WIFI_DIAG_EVENT_AP_STOP,
  WIFI_DIAG_EVENT_AP_CLIENT_CONNECTED,
  WIFI_DIAG_EVENT_AP_CLIENT_DISCONNECTED,
  WIFI_DIAG_EVENT_SCAN_DONE,
  WIFI_DIAG_POLL_RECOVERED
};

struct WifiDiagEvent {
  uint32_t seq;
  uint32_t ms;
  uint32_t attempt;
  uint32_t elapsedMs;
  uint8_t phase;
  int16_t wifiStatus;
  int16_t wifiMode;
  int16_t scanCount;
  int16_t matchCount;
  int32_t rssi;
  int32_t channel;
  uint8_t bssid[6];
  uint8_t disconnectReason;
  int16_t disconnectRssi;
  uint32_t ip;
};

LogFrameEntry logFrames[kLogRingSize] = {};
uint16_t logWriteIndex = 0;
uint16_t logCount = 0;
uint32_t logSequence = 0;
bool telemetryLoggingEnabled = false;
String presetLastError = "";
String presetLastLine = "";
String dptRxDebugLine = "";
bool presetDefaultAttempted = false;
bool presetDefaultLoaded = false;
bool presetSdReady = false;
uint32_t presetSdActiveSpiHz = 0;
uint8_t presetDefaultAttempts = 0;
uint32_t presetDefaultNextAttemptMs = 0;
WifiDiagEvent wifiDiagEvents[kWifiDiagRingSize] = {};
uint8_t wifiDiagWriteIndex = 0;
uint8_t wifiDiagCount = 0;
uint32_t wifiDiagSequence = 0;
uint32_t wifiDiagConnectAttempts = 0;
uint32_t wifiDiagSuccessfulConnects = 0;
uint32_t wifiDiagFailedScans = 0;
uint32_t wifiDiagNoSsidScans = 0;
uint32_t wifiDiagDisconnects = 0;
uint32_t wifiDiagCurrentAttempt = 0;
uint8_t wifiDiagLastPhase = WIFI_DIAG_BOOT_START;
uint8_t wifiDiagLastDisconnectReason = 0;
int16_t wifiDiagLastDisconnectRssi = 0;
int32_t wifiDiagLastRssi = -1000;
int32_t wifiDiagLastChannel = 0;
uint8_t wifiDiagLastBssid[6] = {};
int16_t wifiDiagLastScanCount = -1;
int16_t wifiDiagLastMatchCount = -1;
bool wifiDiagEventsRegistered = false;
uint32_t wifiDiagLastLostMs = 0;
uint32_t wifiDiagLastRecoveredMs = 0;
uint32_t wifiDiagLastOutageMs = 0;
uint32_t wifiDiagLongestOutageMs = 0;
uint32_t wifiDiagMatrixFramesAtLoss = 0;
uint32_t wifiDiagMatrixFramesAtRecovery = 0;
uint32_t wifiDiagMatrixFramesDuringLastOutage = 0;
uint32_t wifiDiagCoreAgeAtLossMs = 0;
uint32_t wifiDiagCoreAgeAtRecoveryMs = 0;
bool wifiDiagOutageActive = false;
uint32_t sidecarBootResetReason = 0;
uint32_t sidecarLoopCount = 0;
uint32_t sidecarLastLoopMs = 0;
uint32_t sidecarMaxLoopGapMs = 0;
uint32_t sidecarMinFreeHeap = UINT32_MAX;
uint32_t webRequestCount = 0;
uint32_t lastWebRequestMs = 0;

class CaptureByteStore {
 public:
  CaptureByteStore()
      : bytes_(nullptr),
        capacity_(0),
        head_(0),
        tail_(0),
        used_(0),
        highWater_(0),
        corruptions_(0) {}

  void Begin(uint8_t *bytes, size_t capacity) {
    bytes_ = bytes;
    capacity_ = capacity;
    Clear();
  }

  void Clear() {
    head_ = 0;
    tail_ = 0;
    used_ = 0;
    highWater_ = 0;
    corruptions_ = 0;
  }

  size_t Capacity() const { return capacity_; }
  size_t Used() const { return used_; }
  size_t HighWater() const { return highWater_; }
  uint32_t Corruptions() const { return corruptions_; }

  bool Append(const uint8_t *record, size_t size) {
    if (bytes_ == nullptr || record == nullptr || size > capacity_ - used_) {
      return false;
    }
    for (size_t i = 0; i < size; ++i) {
      bytes_[head_] = record[i];
      head_ = (head_ + 1) % capacity_;
    }
    used_ += size;
    if (used_ > highWater_) {
      highWater_ = used_;
    }
    return true;
  }

  bool Append(const bearamin_capture::RecordView &record) {
    uint8_t encoded[bearamin_capture::kMaxRecordSize] = {};
    const size_t size = bearamin_capture::encodeRecord(
        record.type,
        record.flags,
        record.sequence,
        record.coreUs,
        record.payload,
        record.payloadSize,
        encoded,
        sizeof(encoded));
    return size > 0 && Append(encoded, size);
  }

  void AckThrough(uint32_t acknowledgedSequence) {
    while (used_ >= bearamin_capture::kRecordOverhead) {
      uint8_t record[bearamin_capture::kMaxRecordSize] = {};
      size_t size = 0;
      bearamin_capture::RecordView view = {};
      if (!ReadAt(0, record, sizeof(record), size, view)) {
        ++corruptions_;
        Clear();
        return;
      }
      if (static_cast<int32_t>(
              view.sequence - acknowledgedSequence) > 0) {
        return;
      }
      tail_ = (tail_ + size) % capacity_;
      used_ -= size;
    }
  }

  size_t CopyAfter(uint32_t afterSequence,
                   uint8_t *output,
                   size_t limit,
                   uint32_t &lastSequence) {
    if (output == nullptr || limit == 0) {
      return 0;
    }
    size_t relativeOffset = 0;
    size_t written = 0;
    while (relativeOffset < used_) {
      uint8_t record[bearamin_capture::kMaxRecordSize] = {};
      size_t size = 0;
      bearamin_capture::RecordView view = {};
      if (!ReadAt(relativeOffset, record, sizeof(record), size, view)) {
        ++corruptions_;
        break;
      }
      relativeOffset += size;
      if (static_cast<int32_t>(view.sequence - afterSequence) <= 0) {
        continue;
      }
      if (written + size > limit) {
        break;
      }
      for (size_t i = 0; i < size; ++i) {
        output[written + i] = record[i];
      }
      written += size;
      lastSequence = view.sequence;
    }
    return written;
  }

 private:
  bool ReadAt(size_t relativeOffset,
              uint8_t *record,
              size_t capacity,
              size_t &recordSize,
              bearamin_capture::RecordView &view) const {
    if (relativeOffset + bearamin_capture::kRecordOverhead > used_) {
      return false;
    }
    uint8_t header[bearamin_capture::kHeaderSize] = {};
    for (size_t i = 0; i < sizeof(header); ++i) {
      header[i] = ByteAt(relativeOffset + i);
    }
    if (!bearamin_capture::hasMagic(header, sizeof(header)) ||
        header[4] != bearamin_capture::kVersion) {
      return false;
    }
    const uint16_t payloadSize = bearamin_capture::readLe16(header, 8);
    recordSize = bearamin_capture::recordSizeForPayload(payloadSize);
    if (recordSize == 0 || recordSize > capacity ||
        relativeOffset + recordSize > used_) {
      return false;
    }
    for (size_t i = 0; i < recordSize; ++i) {
      record[i] = ByteAt(relativeOffset + i);
    }
    return bearamin_capture::decodeRecord(record, recordSize, view);
  }

  uint8_t ByteAt(size_t relativeOffset) const {
    return bytes_[(tail_ + relativeOffset) % capacity_];
  }

  uint8_t *bytes_;
  size_t capacity_;
  size_t head_;
  size_t tail_;
  size_t used_;
  size_t highWater_;
  mutable uint32_t corruptions_;
};

uint8_t captureFallbackBuffer[kCaptureFallbackBytes] = {};
uint8_t captureHttpBuffer[kCaptureHttpMaxBytes] = {};
CaptureByteStore captureStore;
String captureSessionId;
uint32_t captureSessionToken = 0;
bool captureSessionActive = false;
bool captureSessionStopping = false;
bool captureSessionFinalized = false;
bool captureUsingPsram = false;
uint32_t captureFirstSequence = 0;
uint32_t captureLatestSequence = 0;
uint32_t captureFinalSequence = 0;
uint32_t captureAcknowledgedSequence = 0;
uint32_t captureRecordCount = 0;
uint32_t captureDropCount = 0;
uint32_t captureSequenceErrors = 0;
uint32_t captureExpectedSequence = 0;
bool captureHaveSequence = false;
uint32_t capturePendingDropFirst = 0;
uint32_t capturePendingDropLast = 0;
uint32_t capturePendingDropCount = 0;
uint64_t capturePendingDropCoreUs = 0;
uint8_t capturePendingEnd[bearamin_capture::kMaxRecordSize] = {};
size_t capturePendingEndSize = 0;

uint32_t captureTokenForId(const String &id) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < id.length(); ++i) {
    hash ^= static_cast<uint8_t>(id[i]);
    hash *= 16777619u;
  }
  return hash;
}

void resetCaptureSession(const String &id) {
  captureStore.Clear();
  captureSessionId = id;
  captureSessionToken = captureTokenForId(id);
  captureSessionActive = true;
  captureSessionStopping = false;
  captureSessionFinalized = false;
  captureFirstSequence = 0;
  captureLatestSequence = 0;
  captureFinalSequence = 0;
  captureAcknowledgedSequence = 0;
  captureRecordCount = 0;
  captureDropCount = 0;
  captureSequenceErrors = 0;
  captureExpectedSequence = 0;
  captureHaveSequence = false;
  capturePendingDropFirst = 0;
  capturePendingDropLast = 0;
  capturePendingDropCount = 0;
  capturePendingDropCoreUs = 0;
  capturePendingEndSize = 0;
}

bool appendSidecarGapIfPending() {
  if (capturePendingDropCount == 0) {
    return true;
  }
  uint8_t payload[bearamin_capture::kGapPayloadSize] = {};
  payload[0] = 0;
  payload[1] = bearamin_capture::GAP_SIDECAR_RING;
  bearamin_capture::writeLe32(payload, 4, capturePendingDropFirst);
  bearamin_capture::writeLe32(payload, 8, capturePendingDropLast);
  bearamin_capture::writeLe32(payload, 12, capturePendingDropCount);
  uint8_t encoded[bearamin_capture::kMaxRecordSize] = {};
  const size_t size = bearamin_capture::encodeRecord(
      bearamin_capture::RECORD_GAP,
      bearamin_capture::FLAG_SIDECAR_GENERATED,
      capturePendingDropLast,
      capturePendingDropCoreUs,
      payload,
      sizeof(payload),
      encoded,
      sizeof(encoded));
  if (size == 0 || !captureStore.Append(encoded, size)) {
    return false;
  }
  ++captureRecordCount;
  capturePendingDropFirst = 0;
  capturePendingDropLast = 0;
  capturePendingDropCount = 0;
  capturePendingDropCoreUs = 0;
  return true;
}

void rememberSidecarDrop(const bearamin_capture::RecordView &record) {
  if (capturePendingDropCount == 0) {
    capturePendingDropFirst = record.sequence;
  }
  capturePendingDropLast = record.sequence;
  capturePendingDropCoreUs = record.coreUs;
  ++capturePendingDropCount;
  ++captureDropCount;
}

void flushPendingCaptureEnd() {
  if (capturePendingEndSize == 0 || !appendSidecarGapIfPending()) {
    return;
  }
  if (captureStore.Append(capturePendingEnd, capturePendingEndSize)) {
    ++captureRecordCount;
    capturePendingEndSize = 0;
    captureSessionFinalized = true;
    captureSessionStopping = false;
    captureSessionActive = false;
  }
}

void rememberTelemetrySnapshot(
    const bearamin_capture::ProcessedSensorSnapshotV1 &snapshot,
    uint64_t coreUs);
void rememberTelemetrySnapshot(
    const bearamin_capture::ProcessedSensorSnapshotV2 &snapshot,
    uint64_t coreUs);

void acceptCaptureRecord(const bearamin_capture::RecordView &record) {
  if (record.type == bearamin_capture::RECORD_SESSION_START) {
    if (record.payloadSize < 8 ||
        bearamin_capture::readLe32(record.payload, 4) != captureSessionToken) {
      return;
    }
  } else if (!captureSessionActive && !captureSessionStopping) {
    return;
  }

  if (captureHaveSequence &&
      record.sequence != captureExpectedSequence &&
      record.type != bearamin_capture::RECORD_GAP) {
    ++captureSequenceErrors;
  }
  if (!captureHaveSequence) {
    captureFirstSequence = record.sequence;
    captureHaveSequence = true;
  }
  captureExpectedSequence = record.sequence + 1;
  captureLatestSequence = record.sequence;

  if (record.type == bearamin_capture::RECORD_MATRIX_FRAME &&
      record.payloadSize == bearamin_capture::kMatrixPayloadSize) {
    latestCells.sequence = static_cast<uint16_t>(record.sequence);
    latestCells.validCount = record.payload[0];
    latestCells.blobCount = record.payload[1];
    latestCells.sensorAgeMs = 0;
    for (uint8_t i = 0; i < 64; ++i) {
      latestCells.cellsMm[i] =
          bearamin_capture::readLe16(record.payload, 16 + i * 2);
    }
    lastCellsMs = millis();
  } else if (
      record.type == bearamin_capture::RECORD_PROCESSED_SNAPSHOT_V1) {
    bearamin_capture::ProcessedSensorSnapshotV1 snapshot = {};
    if (bearamin_capture::decodeProcessedSnapshotV1(
            record.payload, record.payloadSize, snapshot)) {
      rememberTelemetrySnapshot(snapshot, record.coreUs);
    }
  } else if (
      record.type == bearamin_capture::RECORD_PROCESSED_SNAPSHOT_V2) {
    bearamin_capture::ProcessedSensorSnapshotV2 snapshot = {};
    if (bearamin_capture::decodeProcessedSnapshotV2(
            record.payload, record.payloadSize, snapshot)) {
      rememberTelemetrySnapshot(snapshot, record.coreUs);
    }
  }

  if (!appendSidecarGapIfPending() || !captureStore.Append(record)) {
    rememberSidecarDrop(record);
    if (record.type == bearamin_capture::RECORD_SESSION_END) {
      capturePendingEndSize = bearamin_capture::encodeRecord(
          record.type,
          record.flags,
          record.sequence,
          record.coreUs,
          record.payload,
          record.payloadSize,
          capturePendingEnd,
          sizeof(capturePendingEnd));
      captureFinalSequence = record.sequence;
    }
    return;
  }
  ++captureRecordCount;
  if (record.type == bearamin_capture::RECORD_SESSION_END) {
    captureFinalSequence = record.sequence;
    captureSessionFinalized = true;
    captureSessionStopping = false;
    captureSessionActive = false;
  }
}

void initializeCaptureStore() {
  uint8_t *psram = static_cast<uint8_t *>(heap_caps_malloc(
      kCapturePsramBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (psram != nullptr) {
    captureStore.Begin(psram, kCapturePsramBytes);
    captureUsingPsram = true;
  } else {
    captureStore.Begin(captureFallbackBuffer, sizeof(captureFallbackBuffer));
    captureUsingPsram = false;
  }
}

void setEngineeringStream(bool enabled);
bool obsSubscriptionActive(uint32_t now);
void reconcileEngineeringStream(uint32_t now);
void pollMatrixUart();
void sendCommand(uint8_t commandId,
                 uint16_t arg0,
                 uint16_t arg1 = 0,
                 uint16_t arg2 = 0,
                 uint16_t arg3 = 0);
void recordWifiDiag(WifiDiagPhase phase,
                    uint32_t attempt = 0,
                    int32_t scanCount = -1,
                    int32_t matchCount = -1,
                    int32_t channel = 0,
                    int32_t rssi = -1000,
                    const uint8_t *bssid = nullptr,
                    uint8_t disconnectReason = 0,
                    int32_t disconnectRssi = 0,
                    uint32_t elapsedMs = 0,
                    uint32_t ip = 0);


const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Bearamin</title><link rel="icon" href="/favicon.svg" type="image/svg+xml"><style>
:root{color-scheme:dark;--bg:#080706;--panel:#12100d;--raised:#191610;--line:#3b3224;--gold:#f0b84e;--ink:#f8f2e5;--muted:#b9ad97;--dim:#807561;--green:#a5db78;--red:#ff806d;--cyan:#70d6d8}*{box-sizing:border-box}[hidden]{display:none!important}body{margin:0;background:radial-gradient(circle at 8% 0%,rgba(112,214,216,.09),transparent 26rem),radial-gradient(circle at 88% 8%,rgba(215,156,255,.07),transparent 28rem),var(--bg);color:var(--ink);font:14px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}main{width:min(1440px,100%);margin:auto;padding:24px}h1,h2,h3,p{margin:0}h1{font-size:25px;line-height:1.1}h2{font-size:19px}h3{font-size:14px}.eyebrow,.slot-title{color:var(--dim);font-size:10px;font-weight:800;text-transform:uppercase;letter-spacing:.08em}.mono{color:var(--muted);font:12px SFMono-Regular,Consolas,monospace}.good{color:var(--green)!important}.bad{color:var(--red)!important}.warn{color:var(--gold)!important}.button{min-height:40px;padding:8px 12px;border:1px solid var(--line);border-radius:9px;background:#0d0c0a;color:var(--ink);font:700 14px inherit;cursor:pointer}.button:hover{border-color:var(--gold)}.button.primary{background:var(--gold);border-color:var(--gold);color:#211605}.button:disabled{opacity:.45;cursor:not-allowed}.surface{background:rgba(18,16,13,.92);border:1px solid var(--line);border-radius:11px;padding:14px}.surface-head{display:flex;justify-content:space-between;align-items:start;gap:12px;margin-bottom:11px}.surface-head p{color:var(--muted);margin-top:3px}.topbar{display:grid;grid-template-columns:minmax(240px,1fr) auto;align-items:center;gap:18px;padding-bottom:18px;border-bottom:1px solid var(--line)}.brand{display:flex;align-items:center;gap:12px}.brand-logo{width:52px;height:52px;object-fit:contain;filter:drop-shadow(0 0 14px rgba(112,214,216,.34))}.subline{display:flex;gap:7px;align-items:center;flex-wrap:wrap;color:var(--muted);font-size:12px;margin-top:5px}.dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 9px rgba(165,219,120,.5)}.dot.attention{background:var(--gold);box-shadow:0 0 9px rgba(240,184,78,.5)}.toolbar{display:flex;gap:8px;align-items:center;justify-content:flex-end;flex-wrap:wrap}.preset-button{display:grid;text-align:left;min-width:160px}.preset-button small{color:var(--dim);text-transform:uppercase;font-size:9px;letter-spacing:.08em}.health{display:grid;grid-template-columns:1.35fr repeat(4,minmax(0,1fr));gap:9px;margin:15px 0}.health>div{border:1px solid var(--line);border-radius:9px;padding:10px 12px;background:rgba(18,16,13,.86);min-width:0}.health strong{display:block;margin-top:2px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.readiness{display:flex;align-items:center;gap:10px}.readiness .symbol{font-size:22px;color:var(--gold)}.mode-block{display:grid;grid-template-columns:minmax(250px,.72fr) minmax(0,2fr);gap:18px;align-items:end;margin:16px 0}.mode-copy p{color:var(--muted);margin-top:4px}.mode-tabs{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:6px}.mode-tab{min-height:49px;display:grid;place-items:center;border:1px solid var(--line);border-radius:9px;padding:7px;color:var(--dim);font-weight:800;text-align:center}.mode-tab.active{color:var(--ink);background:#3b2b10;border-color:var(--gold);box-shadow:inset 0 -3px var(--gold)}.layout{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(270px,.8fr);gap:14px}.lane{display:grid;grid-template-columns:37px 26px minmax(0,1fr) auto;gap:9px;align-items:center;min-height:79px;padding:8px 0;border-top:1px solid var(--line)}.lane:first-child{border-top:0}.lane-number{width:27px;height:27px;border:2px solid var(--lane);border-radius:50%;display:grid;place-items:center;font-weight:800}.jack{width:23px;height:23px;border-radius:50%;border:4px solid #938e85;background:radial-gradient(circle,#050505 0 38%,#eee 41% 50%,#686258 53%)}.lane-name{font-weight:800}.lane small{display:block;color:var(--muted);margin-top:2px}.lane-value{color:var(--gold);font-weight:800;text-align:right}.map-controls{display:grid;grid-template-columns:minmax(130px,1fr) 110px;gap:7px;margin-top:6px}.select{width:100%;min-height:31px;background:#0b0906;border:1px solid var(--line);border-radius:6px;color:var(--ink);padding:5px;font:inherit}.strength{display:flex;align-items:center;gap:5px}.strength input{width:100%;accent-color:var(--gold)}.knob{width:31px;height:31px;border-radius:50%;border:3px solid #080706;background:conic-gradient(from -135deg,var(--lane) 0deg,var(--lane) var(--angle),#342c1a var(--angle),#342c1a 270deg,#080706 270deg,#080706 360deg)}.outputs{display:grid;gap:9px}.output-group{border:1px solid var(--line);border-radius:8px;padding:10px}.output-group h3{margin-bottom:6px}.output-row{display:flex;justify-content:space-between;gap:10px;padding:5px 0;border-top:1px solid #30291f}.output-row:first-of-type{border-top:0}.output-row span{color:var(--muted);font-size:12px}.response{margin-top:14px}.response input{width:100%;accent-color:var(--gold)}.notice,.fault{display:flex;gap:9px;margin-top:14px;border:1px solid #8f6c2a;background:#20170a;border-radius:9px;padding:11px;color:var(--ink)}.notice>span,.fault>span{color:var(--gold);font-weight:900}.fault{margin:0 0 14px;border-color:#883e32;background:#24100d}.fault>span{color:var(--red)}.modal-backdrop{position:fixed;inset:0;z-index:20;display:grid;place-items:center;padding:18px;background:rgba(0,0,0,.72)}.modal-panel{width:min(760px,100%);max-height:calc(100vh - 36px);overflow:auto;display:grid;gap:14px;margin:0}.dialog-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}.text-input{width:100%;min-height:38px;padding:8px;border:1px solid var(--line);border-radius:7px;background:#090806;color:var(--ink);font:inherit}.action-row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:12px}.default-card{border-color:var(--gold);background:linear-gradient(135deg,#261c0b,#15110b)}.badge{display:inline-block;padding:2px 7px;border-radius:999px;background:#3b2b10;color:var(--gold);font-size:10px;font-weight:800}.preset-list{display:grid;gap:7px}.preset-row{display:flex;align-items:center;justify-content:space-between;gap:9px;border:1px solid var(--line);border-radius:8px;padding:9px}.preset-row p{color:var(--muted);font-size:12px;margin-top:2px}.system-grid{display:grid;grid-template-columns:minmax(0,1.4fr) minmax(280px,.8fr);gap:14px}.status-row{display:grid;grid-template-columns:110px minmax(0,1fr) auto;gap:8px;padding:8px 0;border-top:1px solid var(--line)}.status-row:first-child{border-top:0}.status-row span{color:var(--muted)}.status-row strong:last-child{color:var(--green)}.status-row strong.bad{color:var(--red)}.service{display:grid;gap:12px}.service>section{border-top:1px solid var(--line);padding-top:11px}.service>section:first-child{border-top:0;padding-top:0}.service p{color:var(--muted);margin:4px 0 9px}.check{display:flex;align-items:center;gap:7px;color:var(--muted);font-size:12px}pre{margin:0;max-height:340px;overflow:auto;white-space:pre-wrap;background:#090806;border:1px solid var(--line);border-radius:7px;padding:10px;color:var(--muted)}.raw-grid{display:grid;grid-template-columns:repeat(8,1fr);gap:3px;margin-bottom:10px}.raw-cell{aspect-ratio:1;display:grid;place-items:center;border:1px solid #2a1f0c;border-radius:4px;color:#f8efd9;font:10px SFMono-Regular,Consolas,monospace}.raw-cell.near{background:#4b0f0b}.raw-cell.play{background:#3a2a08}.raw-cell.far{background:#071f25}.raw-cell.zero{color:#6d5f45}@media(max-width:900px){main{padding:12px}.topbar,.mode-block,.layout,.system-grid,.dialog-grid{grid-template-columns:1fr}.toolbar{justify-content:flex-start}.health{grid-template-columns:1fr 1fr}.mode-tabs{grid-template-columns:1fr 1fr}.lane{grid-template-columns:31px 23px minmax(0,1fr)}.lane-value{grid-column:3;text-align:left}.map-controls{grid-template-columns:1fr}.status-row{grid-template-columns:95px minmax(0,1fr)}.status-row strong:last-child{grid-column:2}.brand-logo{width:45px;height:45px}}
</style></head><body><main><style>
.capture-compact{max-width:220px}.capture-metrics{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:7px;margin:10px 0}.capture-metric{border:1px solid var(--line);border-radius:7px;padding:8px;background:#0c0a08}.capture-metric strong{display:block;color:var(--gold)}.capture-library{display:grid;gap:7px;margin-top:10px}.capture-item{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px;align-items:center;border-top:1px solid var(--line);padding-top:8px}.capture-item:first-child{border-top:0}.capture-item-actions{display:flex;gap:5px;flex-wrap:wrap}.capture-item-actions .button{min-height:32px;padding:5px 8px;font-size:12px}@media(max-width:700px){.capture-metrics{grid-template-columns:1fr 1fr}.capture-item{grid-template-columns:1fr}}
</style>
<header class="topbar"><div class="brand"><img class="brand-logo" src="/favicon.svg" alt="Bearamin Soundfield"><div><div class="eyebrow">DPT optical instrument</div><h1>Bearamin</h1><div class="subline"><span id="ready_dot" class="dot"></span><span id="readiness">Waiting for instrument</span></div></div></div><div class="toolbar"><button id="settings_load" class="button preset-button"><small>Boot preset</small><strong>★ default · Manage</strong></button><button id="settings_save" class="button">Save preset</button><span id="capture_compact" class="mono capture-compact" hidden></span><button id="log_start" class="button primary">Capture</button><button id="log_stop" class="button" disabled>Stop</button><button id="system_open" class="button">System</button></div></header>
<section id="instrument_view"><div class="health"><div class="readiness"><span class="symbol">⌁</span><div><div class="eyebrow">Signal path</div><strong id="path_status">Waiting for MatrixCore</strong></div></div><div><div class="eyebrow">Optical array</div><strong id="matrix">Waiting</strong></div><div><div class="eyebrow">Center gate</div><strong id="gate">Waiting</strong></div><div><div class="eyebrow">Impact</div><strong id="impact">Idle</strong></div><div><div class="eyebrow">Network</div><strong id="network">Waiting</strong></div></div>
<section class="mode-block"><div class="mode-copy"><div class="eyebrow">Active DPT mode</div><h2 id="mode_name">Waiting for DPT</h2><p>Selected by the physical DPT CV8 knob. The web UI follows it.</p></div><div id="mode_tabs" class="mode-tabs" role="list" aria-label="Read-only DPT mode positions"></div></section>
<div class="layout"><section class="surface"><div class="surface-head"><div><h2>Modulation lanes</h2><p>Map a Bearamin axis into each physical DPT control.</p></div><span id="settings_state" class="mono" aria-live="polite"></span></div><div id="cv_in"></div></section><aside><section class="surface"><div class="surface-head"><div><h2>Outputs &amp; patching</h2><p>Current mode signal reference.</p></div></div><div id="outputs" class="outputs"></div></section><section class="surface response"><div class="surface-head"><div><h2>Gesture response</h2><p>Applies to bridge CV movement in every mode.</p></div></div><label class="eyebrow" for="cv_slew">CV slew <strong id="cv_slew_value"></strong></label><input id="cv_slew" type="range" min="0" max="32" step="1" value="16"><div class="subline"><span>Immediate</span><span style="margin-left:auto">Smooth</span></div></section></aside></div><div id="resonator_anchor"></div></section>
<section id="system_view" hidden><div class="surface-head"><div><div class="eyebrow">System</div><h2>Instrument health and service tools</h2><p>Maintenance is separate from playing controls. Raw sensor traffic stays off until enabled.</p></div><button id="system_close" class="button">← Back to instrument</button></div><div class="system-grid"><div><section id="capture_system" class="surface"><div class="surface-head"><div><h2>Capture</h2><p>Loss-tolerant binary capture, browser persistence, verification, and JSON cooking.</p></div><span id="capture_stage" class="mono">idle</span></div><div id="capture_metrics" class="capture-metrics"></div><div id="capture_errors" class="mono"></div><h3 style="margin-top:12px">Capture library</h3><div id="capture_library" class="capture-library"><span class="mono">loading browser captures</span></div></section><section class="surface" style="margin-top:14px"><div class="surface-head"><div><h2>Signal path</h2><p>Live low-rate status; no engineering stream.</p></div></div><div id="status_rows"></div></section><section id="diag" class="surface" hidden style="margin-top:14px"><div class="surface-head"><div><h2>Live sensor diagnostics</h2><p id="diag_summary">Engineering stream active only while this view is open.</p></div><button id="diag_close" class="button">Close diagnostics</button></div><div id="raw_grid" class="raw-grid"></div><pre id="raw_meta">{}</pre><pre id="wifi_diag" style="margin-top:10px">{}</pre><pre id="json" style="margin-top:10px">{}</pre></section></div><aside class="surface service"><section><h2>Mode announcements</h2><p>Play the SD-card voice prompt when the physical mode dial changes. Leave this off for seamless switching.</p><label class="check"><input id="announce_modes" type="checkbox"> Announce modes</label><span id="announce_modes_state" class="mono" aria-live="polite"></span></section><section><h2>MatrixCore OTA</h2><p>Temporarily enables MatrixCore Wi-Fi and accepts an OTA upload for 90 seconds.</p><label class="check"><input id="ota_confirm" type="checkbox"> I stopped capture and confirmed the intended image.</label><div class="action-row"><button id="matrix_ota_btn" class="button primary">Arm OTA for 90 seconds</button><span id="matrix_ota" class="mono" aria-live="polite"></span></div></section><section><h2>Live sensor diagnostics</h2><p>Enables the 64-cell engineering stream only while this view is open.</p><button id="eng" class="button">Open live diagnostics</button></section><section><h2>Support snapshot</h2><p>Download the current status without raw sensor history.</p><button id="support_download" class="button">Download diagnostics</button></section><section><h2>Firmware</h2><div class="status-row"><strong>Sidecar</strong><span id="firmware">—</span></div><div class="status-row"><strong>DPT protocol</strong><span id="protocol">—</span></div></section></aside></div></section>
<div id="preset_modal" class="modal-backdrop" hidden><section id="save_panel" class="surface modal-panel" hidden><div class="surface-head"><div><div class="eyebrow">Presets / Save</div><h2>Save the current instrument setup</h2></div><button class="button preset_close">Close</button></div><div class="dialog-grid"><div><label class="eyebrow" for="preset_name">Save a named copy</label><input id="preset_name" class="text-input" maxlength="48" placeholder="preset name"><p class="mono" style="margin-top:7px">Stores mappings, CV slew, and reflected DPT state. CV8 mode ownership remains physical.</p><div class="action-row"><button id="settings_save_sd" class="button primary">Save preset</button><button id="settings_download" class="button">Download JSON</button></div></div><div class="surface default-card"><div class="eyebrow">Boot preset</div><h3>★ default <span class="badge">Loads on startup</span></h3><p class="mono" style="margin-top:6px">default is the one persistent preset Bearamin restores after boot. Named presets remain unchanged.</p><div class="action-row"><button id="preset_default_save" class="button primary">Overwrite default</button></div></div></div><span id="preset_save_state" class="mono" aria-live="polite"></span></section><section id="load_panel" class="surface modal-panel" hidden><div class="surface-head"><div><div class="eyebrow">Presets / Load</div><h2>Load a preset</h2><p>Loading applies mappings and settings now; it never changes CV8 ownership or boot behavior.</p></div><button class="button preset_close">Close</button></div><div id="preset_list" class="preset-list"></div><div class="action-row"><button id="settings_upload" class="button">Choose JSON</button><input id="settings_file" type="file" accept="application/json,.json" hidden><span id="preset_load_state" class="mono" aria-live="polite"></span></div></section></div>
<script>
const $=id=>document.getElementById(id),modes=[{name:'Modal',mapKey:'rings_resonator',cvIn:['Brightness','Damping','Structure','Position','Reverb mix','Frequency|C0-C4','Bearamin amount','Mode select'],cvOut:['Envelope +','Envelope -','Frequency|V/oct','Bearamin X|bipolar','Bearamin Y|bipolar','Bearamin Z|bipolar'],gate:['Strum pulse','IMU impact'],audio:['Resonator L','Resonator R']},{name:'Sympathetic Strings',mapKey:'rings_resonator',cvIn:['Brightness','Damping','Structure','Position','Reverb mix','Frequency|C0-C4','Bearamin amount','Mode select'],cvOut:['Envelope +','Envelope -','Frequency|V/oct','Bearamin X|bipolar','Bearamin Y|bipolar','Bearamin Z|bipolar'],gate:['Strum pulse','IMU impact'],audio:['Resonator L','Resonator R']},{name:'Karplus-Strong',mapKey:'rings_resonator',cvIn:['Brightness','Damping','Structure','Position','Reverb mix','Frequency|C0-C4','Bearamin amount','Mode select'],cvOut:['Envelope +','Envelope -','Frequency|V/oct','Bearamin X|bipolar','Bearamin Y|bipolar','Bearamin Z|bipolar'],gate:['Strum pulse','IMU impact'],audio:['Resonator L','Resonator R']},{name:'Bearamin FM drone',mapKey:'bearamin_fm',cvIn:['Mod 1 rate','FM1 depth|into carriers','Mod 2 rate','FM2 depth|into mod 1','Base pitch','Stereo spread','Bearamin amount','Mode select'],cvOut:['Mod 1|bipolar LFO','Mod 2|bipolar LFO','Cooked FM|bend CV','Bearamin X|bipolar','Bearamin Y|bipolar','Bearamin Z|bipolar'],gate:['Unused','Unused'],audio:['Input L + FM|unity blend','Input R + FM|unity blend']},{name:'Bearamin CV',cvIn:['Unused','Unused','Unused','Unused','Unused','Unused','Unused','Mode select'],cvOut:['X axis|unipolar','Y axis|unipolar','Z axis|unipolar','X axis|bipolar','Y axis|bipolar','Z axis|bipolar'],gate:['Gesture gate','IMU impact'],audio:['Input L|unity pass-through','Input R|unity pass-through']}],axes=[
['none','None'],['x_bipolar','X bipolar'],['y_bipolar','Y bipolar'],['z_bipolar','Z bipolar'],['x_unipolar','X unipolar'],['y_unipolar','Y unipolar'],['z_unipolar','Z unipolar'],['pitch','Pitch'],['roll','Roll'],['spank','Spank']],colors=['#ff806d','#f0b84e','#f5df61','#a5db78','#70d6d8','#d79cff'];let latestDevice=null,engineeringOn=false,wifiDiagLastFetch=0,captureActive=false,captureStatus=null,captureDownloads={};
modes[4].gate[0]='Center presence';
function esc(v){const e=document.createElement('div');e.textContent=v==null?'':String(v);return e.innerHTML}function lines(v){return String(v).split('|').map(x=>`<div>${esc(x)}</div>`).join('')}function axisValue(v){return axes.some(a=>a[0]===v)?v:'none'}function depthValue(v){if(v==='--')return -100;if(v==='-')return -50;if(v==='+')return 50;if(v==='++')return 100;return Math.max(-100,Math.min(100,Number.isFinite(+v)?+v:50))}function sendSettings(q){$('settings_state').textContent='applying';return fetch('/dpt/settings?'+q,{cache:'no-store'}).then(r=>r.json()).then(d=>{latestDevice=d;renderPanel(d);$('settings_state').textContent='applied'}).catch(()=>{$('settings_state').textContent='settings error'})}
function mapControls(mode,lane,d){const a=axisValue(d[`map_${mode}_${lane}_axis`]),v=depthValue(d[`map_${mode}_${lane}_depth`]),position=(v+100)/2;return `<div class="map-controls"><select class="select" aria-label="Modulation source" data-setting="map_${mode}_${lane}_axis">${axes.map(([x,t])=>`<option value="${x}"${a===x?' selected':''}>${t}</option>`).join('')}</select><div class="lane-depth"><input type="range" min="-100" max="100" step="1" value="${v}" aria-label="Modulation depth attenuverter ${v}%" data-setting="map_${mode}_${lane}_depth" data-lane-depth="1"><output style="--depth-position:${position}%">${v}%</output></div></div>`}
function webSourceSelect(key,value,cv5){const opts=cv5?axes.concat([['cv5_wet','CV5 wet control']]):axes;return `<select class="select" data-res-setting="${key}">${opts.map(([x,t])=>`<option value="${x}"${x===value?' selected':''}>${t}</option>`).join('')}</select>`}function webSlider(key,value,min,max,suffix='%'){return `<div><input type="range" min="${min}" max="${max}" step="1" value="${value}" data-res-setting="${key}" data-res-output="${key}_value"><span id="${key}_value" class="mono">${value}${suffix}</span></div>`}function reverbLpDisplay(value){const a=Math.max(0,Math.min(.999,+value/100)),hz=Math.min(24000,-48000*Math.log(1-a)/(2*Math.PI));return hz>=1000?(hz/1000).toFixed(hz<10000?1:0)+' kHz':Math.round(hz)+' Hz'}function webLpSlider(key,value){return `<div><input type="range" min="0" max="100" step="1" value="${value}" data-res-setting="${key}" data-res-output="${key}_value" data-res-display="lp"><span id="${key}_value" class="mono">${reverbLpDisplay(value)}</span></div>`}function webRadioGroup(key,value,options){return `<fieldset class="web-choice"><legend>Drone voice</legend><div class="choice-options">${options.map(([x,t])=>`<label><input type="radio" name="${key}" value="${x}" data-res-setting="${key}"${x===value?' checked':''}> ${t}</label>`).join('')}</div></fieldset>`}
function renderResonatorControls(d){let section=$('resonator_web');if(!section){section=document.createElement('section');section.id='resonator_web';section.className='surface web-controls';$('resonator_anchor').before(section)}const active=+d.dpt_mode>=0&&+d.dpt_mode<=2&&!$('instrument_view').hidden;section.hidden=!active;if(!active)return;const reverbOpen=!!document.querySelector('#resonator_reverb_detail[open]'),feedbackMax=95,applied=!!d.resonator_controls_applied,wet=Math.round((((d.dpt_knobs||[])[4]||0)/2.55));section.innerHTML=`<div class="surface-head"><div><h2>Resonator web controls</h2><p>Feedback and drone are mixed into the resonator input. Reverb details now live with CV5 wet/dry below.</p></div><span class="control-status ${applied?'good':''}">${applied?'Active on DPT':'Applying to DPT…'}</span></div><div class="web-control-grid"><article class="web-card feedback-card"><div class="eyebrow">Input loop</div><h3>Feedback</h3><p>Recirculate the resonator output before reverb, capped at 95%.</p><div class="web-row"><label>Amount</label>${webSlider('res_feedback_amount',Math.min(feedbackMax,+d.res_feedback_amount||0),0,feedbackMax)}</div><div class="web-modulation"><label><span>Modulation source</span>${webSourceSelect('res_feedback_source',d.res_feedback_source||'none',true)}</label><label><span>Modulation depth attenuverter</span>${webSlider('res_feedback_depth',+d.res_feedback_depth||0,-100,100)}</label></div></article><div class="web-divider"><span>Separate exciter</span></div><article class="web-card drone-card"><div class="eyebrow">Input generator</div><h3>Drone</h3><p>A continuous tuned exciter mixed with audio, strums, and feedback.</p><label class="web-check"><input type="checkbox" data-res-setting="res_drone_enabled"${d.res_drone_enabled?' checked':''}> Enable drone</label>${webRadioGroup('res_drone_waveform',d.res_drone_waveform||'sine',[['sine','Sine'],['saw','Saw']])}<div class="web-row"><label>Level</label>${webSlider('res_drone_level',+d.res_drone_level||0,0,100)}</div><div class="web-modulation"><label><span>Modulation source</span>${webSourceSelect('res_drone_source',d.res_drone_source||'none',false)}</label><label><span>Modulation depth attenuverter</span>${webSlider('res_drone_depth',+d.res_drone_depth||0,-100,100)}</label></div><p class="depth-help">Depth works as an attenuverter around Level: positive follows the source, negative inverts it, and 0% disables modulation.</p></article><details id="resonator_reverb_detail" class="reverb-detail"${reverbOpen?' open':''}><summary>Reverb detail</summary><div class="reverb-detail-body"><p>CV5 wet/dry remains always-on (${wet}% now). Each depth control is an attenuverter around its saved value: positive follows the modulation source, negative inverts it, and 0% means no modulation.</p>${[['time','Time'],['diffusion','Diffusion'],['lp','Low-pass']].map(([key,label])=>`<div class="web-row"><label>${label}</label>${key==='lp'?webLpSlider(`res_reverb_${key}`,+d[`res_reverb_${key}`]||0):webSlider(`res_reverb_${key}`,+d[`res_reverb_${key}`]||0,0,100)}</div><div class="web-modulation"><label><span>Modulation source</span>${webSourceSelect(`res_reverb_${key}_source`,d[`res_reverb_${key}_source`]||'none',true)}</label><label><span>Modulation depth attenuverter</span>${webSlider(`res_reverb_${key}_depth`,+d[`res_reverb_${key}_depth`]||0,-100,100)}</label></div>`).join('')}</div></details></div>`;const detail=$('resonator_reverb_detail'),reverbLane=document.querySelectorAll('#cv_in .lane')[4];if(detail&&reverbLane)reverbLane.appendChild(detail);document.querySelectorAll('[data-res-setting]').forEach(e=>{if(e.type==='range')e.oninput=()=>{$(e.dataset.resOutput).textContent=e.dataset.resDisplay==='lp'?reverbLpDisplay(e.value):e.value+'%'};e.onchange=()=>{const value=e.type==='checkbox'?(e.checked?1:0):e.value;e.blur();sendSettings(encodeURIComponent(e.dataset.resSetting)+'='+encodeURIComponent(value))}})}
function renderPanel(d){const index=Number.isFinite(+d.dpt_mode)&&+d.dpt_mode>=0&&+d.dpt_mode<modes.length?+d.dpt_mode:0,mode=modes[index],controlHead=mode.mapKey?'<div class="lane-control-head"><div class="map-control-head"><span></span><span>Modulation depth attenuverter</span></div></div>':'';$('mode_name').textContent=mode.name;$('mode_tabs').innerHTML=modes.map((m,i)=>`<div class="mode-tab ${i===index?'active':''}" role="listitem"${i===index?' aria-current="true"':''}>${esc(m.name)}</div>`).join('');$('cv_in').innerHTML=controlHead+mode.cvIn.map((p,i)=>{const k=Math.max(0,Math.min(255,(d.dpt_knobs||[])[i]||0)),ctl=mode.mapKey&&i<6?mapControls(mode.mapKey,i,d):'',name=lines(p);return `<div class="lane" style="--lane:${colors[i]||'#ddd'}"><div class="lane-number">${i+1}</div><div class="jack"></div><div><div class="slot-title">${i===6?'Global control':i===7?'Hardware-only control':'CV input '+(i+1)}</div><div class="lane-name">${name}</div>${ctl}</div><div>${i<6?`<div class="knob" style="--angle:${(k/255)*270}deg;--lane:${colors[i]}"></div><div class="lane-value">${Math.round(k/2.55)}%</div>`:i===6?'<div class="lane-value">Amount</div>':'<div class="lane-value">CV8</div>'}</div></div>`}).join('');$('outputs').innerHTML=group('CV outputs',mode.cvOut,'CV out')+group('Gate outputs',mode.gate,'Gate')+group('Audio',mode.audio,'Audio out');$('cv_slew').value=Number.isFinite(+d.cv_slew_step)?+d.cv_slew_step:16;$('cv_slew_value').textContent=$('cv_slew').value+' / 32';document.querySelectorAll('[data-setting]').forEach(e=>{if(e.dataset.laneDepth)e.oninput=()=>{const o=e.nextElementSibling,v=depthValue(e.value);o.textContent=v+'%';o.style.setProperty('--depth-position',(v+100)/2+'%')};e.onchange=()=>{e.blur();sendSettings(encodeURIComponent(e.dataset.setting)+'='+encodeURIComponent(e.value))}})}function group(title,items,label){return `<div class="output-group"><h3>${title}</h3>${items.map((v,i)=>`<div class="output-row"><span>${label} ${i+1}</span><strong>${lines(v)}</strong></div>`).join('')}</div>`}
const mappingConfigs=['bearamin_fm','rings_resonator'],settingsKeys=['fm_drone_ranger','fm_map_x','fm_map_y','fm_map_z','cv_slew_step','res_feedback_amount','res_feedback_source','res_feedback_depth','res_drone_enabled','res_drone_waveform','res_drone_level','res_drone_source','res_drone_depth','res_reverb_time','res_reverb_time_source','res_reverb_time_depth','res_reverb_diffusion','res_reverb_diffusion_source','res_reverb_diffusion_depth','res_reverb_lp','res_reverb_lp_source','res_reverb_lp_depth','res_frequency_quantization'];for(const key of mappingConfigs)for(let l=0;l<6;l++)settingsKeys.push(`map_${key}_${l}_axis`,`map_${key}_${l}_depth`);function p(k,v){return encodeURIComponent(k)+'='+encodeURIComponent(v)}function snapshotSettings(){const d=latestDevice||{},settings={};for(const k of settingsKeys)settings[k]=d[k];return{format:'bearamin-settings',version:10,firmware:d.firmware_version||'',saved_at:new Date().toISOString(),settings}}function settingsToParams(s){s=s||{};const out=[p('fm_enabled',s.fm_drone_ranger?1:0),p('fm_x',s.fm_map_x||'fm2_frequency'),p('fm_y',s.fm_map_y||'fm1_frequency'),p('fm_z',s.fm_map_z||'fm1_amount')];for(const key of mappingConfigs)for(let l=0;l<6;l++)out.push(p(`map_${key}_${l}_axis`,axisValue(s[`map_${key}_${l}_axis`])),p(`map_${key}_${l}_depth`,depthValue(s[`map_${key}_${l}_depth`])));out.push(p('cv_slew',s.cv_slew_step??16));return out.join('&')}function currentPresetJson(){return JSON.stringify(snapshotSettings(),null,2)+'\n'}function sanitizePresetName(v){return String(v||'').split('').filter(c=>/[A-Za-z0-9_-]/.test(c)).join('').slice(0,47)}
function closePresets(){$('preset_modal').hidden=true;$('save_panel').hidden=true;$('load_panel').hidden=true}function openPreset(kind){$('preset_modal').hidden=false;$('save_panel').hidden=kind!=='save';$('load_panel').hidden=kind!=='load';if(kind==='save')$('preset_name').focus()}function savePreset(name){name=sanitizePresetName(name||$('preset_name').value);if(!name){$('preset_save_state').textContent='name required';return}$('preset_save_state').textContent='saving';fetch('/presets/save?name='+encodeURIComponent(name),{method:'POST',cache:'no-store'}).then(r=>r.json()).then(d=>{if(!d.ok)throw Error(d.error);$('preset_save_state').textContent='saved '+d.filename;setTimeout(closePresets,500)}).catch(()=>{$('preset_save_state').textContent='save error'})}function renderPresetList(d){const list=(d.presets||[]).slice().sort((a,b)=>(a.name==='default'?-1:b.name==='default'?1:a.name.localeCompare(b.name)));$('preset_list').innerHTML=list.length?list.map(x=>`<div class="preset-row ${x.name==='default'?'default-card':''}"><div><strong>${x.name==='default'?'★ ':''}${esc(x.name)}</strong>${x.name==='default'?'<p><span class="badge">Boot preset</span> Loads on startup</p>':'<p>Applies settings now; boot behavior unchanged</p>'}</div><button class="button ${x.name==='default'?'primary':''}" data-preset="${esc(x.name)}">${x.name==='default'?'Reload default':'Load'}</button></div>`).join(''):'<div class="mono">no saved presets</div>';document.querySelectorAll('[data-preset]').forEach(b=>b.onclick=()=>loadPreset(b.dataset.preset))}function showLoadPanel(){openPreset('load');$('preset_load_state').textContent='loading';fetch('/presets.json',{cache:'no-store'}).then(r=>r.json()).then(d=>{renderPresetList(d);$('preset_load_state').textContent=d.ok?(d.presets||[]).length+' presets':d.error||'preset list error'}).catch(()=>{$('preset_load_state').textContent='preset list error'})}function loadPreset(name){$('preset_load_state').textContent='loading';fetch('/presets/load?name='+encodeURIComponent(name),{method:'POST',cache:'no-store'}).then(r=>r.json()).then(d=>{if(d.ok===false)throw Error();latestDevice=d;renderPanel(d);$('preset_load_state').textContent='loaded '+name;setTimeout(closePresets,500)}).catch(()=>{$('preset_load_state').textContent='load error'})}function downloadSettingsFile(){const a=document.createElement('a'),u=URL.createObjectURL(new Blob([currentPresetJson()],{type:'application/json'}));a.href=u;a.download='bearamin-settings-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';a.click();URL.revokeObjectURL(u)}function loadSettingsFile(file){if(!file)return;file.text().then(JSON.parse).then(data=>{if(data.format!=='bearamin-settings'||data.version!==10||!data.settings)throw Error('preset_version');return sendSettings(settingsToParams(data.settings))}).then(closePresets).catch(()=>{$('preset_load_state').textContent='upload requires preset v10'})}
function setView(system){$('instrument_view').hidden=system;$('system_view').hidden=!system;if(!system&&engineeringOn)setEngineering(false)}function renderAnnouncementSetting(d){$('announce_modes').checked=!!d.announce_modes;$('announce_modes_state').textContent=d.announce_modes_applied===d.announce_modes?'Applied on DPT':'Applying…'}function renderRawGrid(d){$('raw_grid').innerHTML=Array.from({length:64},(_,i)=>{const v=+(d.cells_mm||[])[i]||0,c=v===0?'zero':v<25?'near':v<900?'play':'far';return `<div class="raw-cell ${c}">${v}</div>`}).join('');$('raw_meta').textContent=JSON.stringify(d,null,2)}function setEngineering(on){engineeringOn=on;$('diag').hidden=!on;fetch('/engineering/stream?enabled='+(on?1:0),{cache:'no-store'}).catch(()=>{});if(on)pollEngineering()}function pollEngineering(){if(!engineeringOn)return;fetch('/engineering/matrix.json',{cache:'no-store'}).then(r=>r.json()).then(renderRawGrid).catch(()=>{$('raw_meta').textContent='engineering stream unavailable'})}function pollWifiDiag(){if(!engineeringOn||Date.now()-wifiDiagLastFetch<2500)return;wifiDiagLastFetch=Date.now();fetch('/wifi/diag.json',{cache:'no-store'}).then(r=>r.json()).then(d=>{$('wifi_diag').textContent=JSON.stringify(d,null,2)}).catch(()=>{$('wifi_diag').textContent='Wi-Fi diagnostics unavailable'})}function triggerMatrixOta(){if(!$('ota_confirm').checked){$('matrix_ota').textContent='confirm preflight first';return}$('matrix_ota').textContent='arming';$('matrix_ota_btn').disabled=true;fetch('/matrixcore/ota',{method:'POST',cache:'no-store'}).catch(()=>{$('matrix_ota').textContent='request failed'}).finally(()=>setTimeout(()=>{$('matrix_ota_btn').disabled=false},1200))}function downloadSupport(){const a=document.createElement('a'),u=URL.createObjectURL(new Blob([JSON.stringify(latestDevice||{},null,2)],{type:'application/json'}));a.href=u;a.download='bearamin-support-'+Date.now()+'.json';a.click();URL.revokeObjectURL(u)}
async function poll(){try{const d=await(await fetch('/status.json',{cache:'no-store'})).json();latestDevice=d;const playable=!d.core_link_stale&&d.dpt_ready&&d.matrix_ready;$('readiness').textContent=playable?'Ready to play':'Instrument needs attention';$('ready_dot').className='dot';$('path_status').textContent=d.core_link_stale?'MatrixCore link stale':d.dpt_ready?'Ready':'DPT offline';$('matrix').textContent=d.matrix_ready?`8×8 · ${Number(d.matrix_effective_hz||0).toFixed(1)} Hz source`:'Offline';$('gate').textContent=d.synthetic_gate?'Open':'Closed';$('gate').className=d.synthetic_gate?'good':'';$('impact').textContent=d.impact_gate?'Hit '+d.impact_velocity:'Idle';$('network').textContent=d.network_mode+' · '+(d.sta_ip||d.softap_ip||'offline');$('firmware').textContent=d.firmware_version||'bearamin-sidecar-matrix-poc';$('protocol').textContent='v'+(d.dpt_protocol_version||'—');const ota=d.matrixcore_ota_active?'receiving':d.matrixcore_ota_mode?'armed':'idle';$('matrix_ota').textContent=ota==='idle'?'':ota;$('status_rows').innerHTML=[['Sidecar','Wi-Fi web host',d.web_ready?'Ready':'Offline'],['MatrixCore',`VL53L7CX sensor and DPT bridge · frame age ${d.core_age_ms||0} ms`,d.core_link_stale?'Stale':'Ready'],['Center gate','Any valid center zone in musical range',d.synthetic_gate?'Open':'Closed'],['DPT','Physical CV8 reports '+(modes[d.dpt_mode]?.name||'mode'),d.dpt_ready?'Ready':'Offline'],['Boot preset','default loads on startup',d.preset_default_loaded?'Loaded':d.boot_control_sync_source==='matrix_runtime'?'Session retained':d.preset_default_attempted?'Not loaded':'Checking']].map(x=>`<div class="status-row"><strong>${x[0]}</strong><span>${x[1]}</span><strong class="${x[2]==='Offline'||x[2]==='Stale'?'bad':''}">${x[2]}</strong></div>`).join('');$('diag_summary').textContent=engineeringOn?'Canonical 64-cell 8×8 stream active.':'Engineering stream idle';renderAnnouncementSetting(d);if(engineeringOn){$('json').textContent=JSON.stringify(d,null,2);pollEngineering();pollWifiDiag()}renderPanel(d)}catch(e){$('readiness').textContent='Web connection error';$('ready_dot').className='dot attention'}}
const pollWithoutQuietStatus=poll;poll=async function(){await pollWithoutQuietStatus();const d=latestDevice;if(!d||!d.dpt_control_tx_quiet)return;$('readiness').textContent='Preset transfer';$('ready_dot').className='dot attention';$('path_status').textContent='Preset storage active · motion control remains live';const rows=$('status_rows').querySelectorAll('.status-row');if(rows[3])rows[3].querySelector('strong:last-child').textContent='Preset transfer'}
const captureWorker=new Worker('/capture-worker.js');
function captureStamp(){return new Date().toISOString().replace(/[-:]/g,'').replace(/\.\d{3}Z$/,'Z')}
function bytesLabel(n){if(n>=1048576)return(n/1048576).toFixed(1)+' MiB';if(n>=1024)return(n/1024).toFixed(1)+' KiB';return(n||0)+' B'}
function elapsedLabel(ms){const s=Math.floor((ms||0)/1000);return Math.floor(s/60)+':'+String(s%60).padStart(2,'0')}
function captureMetric(label,value){return `<div class="capture-metric"><span class="eyebrow">${label}</span><strong>${esc(value)}</strong></div>`}
function renderCaptureStatus(s){captureStatus=s;const stoppable=['starting','capturing','stopping','draining'].includes(s.stage),terminal=['idle','complete','incomplete','interrupted','error'].includes(s.stage);captureActive=!terminal;$('log_start').disabled=captureActive;$('log_stop').disabled=!stoppable;const dropped=+s.drops||0,compact=stoppable?`Capture ${elapsedLabel(s.elapsedMs)}`:s.stage==='complete'?(captureDownloads.raw&&captureDownloads.json?'Raw and JSON downloads requested':'Capture complete'):s.stage==='error'||s.stage==='incomplete'?'Capture incomplete':'';$('capture_stage').textContent=s.stage||'idle';$('capture_compact').hidden=!compact&&!dropped;$('capture_compact').className='mono capture-compact '+(dropped?'warn':'');$('capture_compact').textContent=[compact,dropped?`${dropped} dropped records`:null].filter(Boolean).join(' · ');const d=s.device||{},counts=s.counts||{};$('capture_metrics').innerHTML=[captureMetric('Duration',elapsedLabel(s.elapsedMs)),captureMetric('Stored',bytesLabel(s.bytes)),captureMetric('Records',s.records||0),captureMetric('Ring',`${bytesLabel(d.used_bytes||0)} / ${bytesLabel(d.capacity_bytes||0)}`),captureMetric('High water',bytesLabel(d.high_water_bytes||0)),captureMetric('Streams',Object.entries(counts).map(([k,v])=>`${k}:${v}`).join(' ')||'waiting')].join('');$('capture_errors').className='mono '+(dropped||s.crcErrors||s.sequenceErrors?'warn':'');$('capture_errors').textContent=`drops ${dropped} · CRC ${s.crcErrors||0} · sequence ${s.sequenceErrors||0} · store ${d.store_errors||0}`}
function requestBrowserDownload(m){const u=URL.createObjectURL(m.blob),a=document.createElement('a');a.href=u;a.download=m.filename;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(u),30000);captureDownloads[m.kind]=true;if(captureStatus)renderCaptureStatus({...captureStatus,stage:m.kind==='raw'?'raw download requested':'JSON download requested'})}
function renderCaptureLibrary(items){$('capture_library').innerHTML=items.length?items.map(c=>`<div class="capture-item"><div><strong>${esc(c.basename||c.id)}</strong><div class="mono">${esc(c.status)} · ${bytesLabel(c.rawBytes||0)}${c.error?' · '+esc(c.error):''}</div></div><div class="capture-item-actions"><button class="button" data-capture-action="raw" data-capture-id="${esc(c.id)}"${c.rawBytes?'':' disabled'}>Raw retry</button><button class="button" data-capture-action="json" data-capture-id="${esc(c.id)}"${c.status==='complete'?'':' disabled'}>JSON retry</button><button class="button" data-capture-action="delete" data-capture-id="${esc(c.id)}">Delete</button></div></div>`).join(''):'<span class="mono">No browser captures</span>';document.querySelectorAll('[data-capture-action]').forEach(b=>b.onclick=()=>{const type=b.dataset.captureAction,id=b.dataset.captureId;if(type==='delete'){if(confirm('Delete this browser capture?'))captureWorker.postMessage({type:'delete',id})}else captureWorker.postMessage({type:'retry',id,kind:type})})}
function startLog(){const stamp=captureStamp(),random=Array.from(crypto.getRandomValues(new Uint8Array(4)),x=>x.toString(16).padStart(2,'0')).join('');captureDownloads={};captureStatus=null;$('log_start').disabled=true;$('log_stop').disabled=false;captureWorker.postMessage({type:'start',id:`capture-${stamp}-${random}`,basename:`bearamin-capture-${stamp}`})}
function stopLog(){$('log_stop').disabled=true;captureWorker.postMessage({type:'stop'})}
captureWorker.onmessage=e=>{const m=e.data||{};if(m.type==='status')renderCaptureStatus(m);else if(m.type==='download')requestBrowserDownload(m);else if(m.type==='library')renderCaptureLibrary(m.captures||[]);else if(m.type==='complete'){captureActive=false;renderCaptureStatus({...m.capture,elapsedMs:(m.capture.completedAt||Date.now())-m.capture.createdAt})}else if(m.type==='error'){captureActive=false;$('capture_errors').className='mono bad';$('capture_errors').textContent=m.error||'capture error';$('capture_compact').hidden=false;$('capture_compact').className='mono capture-compact bad';$('capture_compact').textContent='Capture incomplete';$('log_start').disabled=false;$('log_stop').disabled=true}};
const setViewBeforeCapture=setView;setView=function(system){setViewBeforeCapture(system);if(system)captureWorker.postMessage({type:'library'})};
function dismissCaptureDetail(){if(captureActive)return;$('capture_stage').textContent='idle';$('capture_metrics').innerHTML='';$('capture_errors').textContent=''}$('capture_stage').title='Click completed status to dismiss';$('capture_stage').onclick=dismissCaptureDetail;
const matrixSourceSection=document.createElement('section');matrixSourceSection.innerHTML='<h2>Sensor pipeline</h2><p>Production targets use genuine 8×8 depth frames at 15 Hz. CV and display state are time-smoothed at 30 Hz between measurements.</p><div class="mono">8×8 source · 15 Hz sensing · 30 Hz output</div>';document.querySelector('#system_view .service').appendChild(matrixSourceSection);
const pollBeforePresetTransferStatus=poll;poll=async function(){await pollBeforePresetTransferStatus();const d=latestDevice;if(!d||!d.dpt_control_tx_quiet)return;$('readiness').textContent='Preset transfer';$('ready_dot').className='dot attention';$('path_status').textContent='Preset storage active · motion control remains live';const rows=$('status_rows').querySelectorAll('.status-row');if(rows[5])rows[5].querySelector('strong:last-child').textContent='Preset transfer'}
$('settings_load').onclick=showLoadPanel;$('settings_save').onclick=()=>openPreset('save');$('settings_save_sd').onclick=()=>savePreset();$('preset_default_save').onclick=()=>savePreset('default');$('settings_download').onclick=downloadSettingsFile;$('settings_upload').onclick=()=>$('settings_file').click();$('settings_file').onchange=e=>{loadSettingsFile(e.target.files&&e.target.files[0]);e.target.value=''};document.querySelectorAll('.preset_close').forEach(b=>b.onclick=closePresets);$('preset_modal').onclick=e=>{if(e.target.id==='preset_modal')closePresets()};$('system_open').onclick=()=>setView(true);$('system_close').onclick=()=>setView(false);$('announce_modes').onchange=e=>sendSettings('announce_modes='+(e.target.checked?1:0));$('eng').onclick=()=>setEngineering(true);$('diag_close').onclick=()=>setEngineering(false);$('matrix_ota_btn').onclick=triggerMatrixOta;$('support_download').onclick=downloadSupport;$('cv_slew').oninput=e=>{$('cv_slew_value').textContent=e.target.value+' / 32'};$('cv_slew').onchange=e=>sendSettings('cv_slew='+encodeURIComponent(e.target.value));$('log_start').onclick=startLog;$('log_stop').onclick=stopLog;document.addEventListener('keydown',e=>{if(e.key==='Escape')closePresets()});window.addEventListener('beforeunload',()=>{if(navigator.sendBeacon)navigator.sendBeacon('/engineering/stream?enabled=0')});setInterval(poll,500);poll();
const renderPanelWithRequestedCopy=renderPanel;function editingPanelControl(){const a=document.activeElement;return !!(a&&(a.tagName==='SELECT'||a.type==='range'||a.type==='radio'||a.type==='checkbox'))}renderPanel=function(d){if(editingPanelControl())return;renderPanelWithRequestedCopy(d);renderResonatorControls(d);document.querySelectorAll('#cv_in .lane:last-child .lane-value').forEach(e=>e.remove())};function lines(v){v=v==='Bearamin amount'?'Bearamin CV amount':v==='Reverb mix'?'Reverb wet/dry':v;return String(v).split('|').map(x=>`<div>${esc(x)}</div>`).join('')}document.querySelector('.response h2').textContent='Gesture response (slew)';
settingsKeys.push('res_feedback_amount','res_feedback_source','res_feedback_depth','res_drone_enabled','res_drone_waveform','res_drone_level','res_drone_source','res_drone_depth','res_reverb_time','res_reverb_time_source','res_reverb_time_depth','res_reverb_diffusion','res_reverb_diffusion_source','res_reverb_diffusion_depth','res_reverb_lp','res_reverb_lp_source','res_reverb_lp_depth');const settingsToParamsBase=settingsToParams;settingsToParams=function(s){s=s||{};const out=settingsToParamsBase(s).split('&');[['res_feedback_amount',Math.min(95,s.res_feedback_amount??0)],['res_feedback_source',s.res_feedback_source||'none'],['res_feedback_depth',s.res_feedback_depth??0],['res_drone_enabled',s.res_drone_enabled?1:0],['res_drone_waveform',s.res_drone_waveform==='saw'?'saw':'sine'],['res_drone_level',s.res_drone_level??20],['res_drone_source',s.res_drone_source||'none'],['res_drone_depth',s.res_drone_depth??0],['res_reverb_time',s.res_reverb_time??67],['res_reverb_time_source',s.res_reverb_time_source||'none'],['res_reverb_time_depth',s.res_reverb_time_depth??0],['res_reverb_diffusion',s.res_reverb_diffusion??63],['res_reverb_diffusion_source',s.res_reverb_diffusion_source||'none'],['res_reverb_diffusion_depth',s.res_reverb_diffusion_depth??0],['res_reverb_lp',s.res_reverb_lp??60],['res_reverb_lp_source',s.res_reverb_lp_source||'none'],['res_reverb_lp_depth',s.res_reverb_lp_depth??0]].forEach(([k,v])=>out.push(p(k,v)));return out.join('&')};const renderResonatorControlsBase=renderResonatorControls;renderResonatorControls=function(d){const s=$('resonator_web');if(s&&s.contains(document.activeElement))return;renderResonatorControlsBase(d)};
settingsKeys.push('res_frequency_quantization');
const settingsToParamsWithQuantization=settingsToParams;
settingsToParams=function(s){s=s||{};return settingsToParamsWithQuantization(s)+'&'+p('res_frequency_quantization',s.res_frequency_quantization||'off')};
function frequencyRootNoteName(d){
  const value=Math.max(0,Math.min(255,Number((d.dpt_knobs||[])[5])||0));
  const midi=Math.round(12+value/255*48);
  return ['C','C♯','D','D♯','E','F','F♯','G','G♯','A','A♯','B'][((midi%12)+12)%12]+(Math.floor(midi/12)-1);
}
const renderResonatorControlsWithQuantization=renderResonatorControls;
renderResonatorControls=function(d){
  renderResonatorControlsWithQuantization(d);
  if(+d.dpt_mode<0||+d.dpt_mode>2||document.getElementById('resonator_frequency_detail'))return;
  const lane=document.querySelectorAll('#cv_in .lane')[5];
  if(!lane)return;
  const quantization=d.res_frequency_quantization||'off';
  const options=[['off','Off'],['chromatic','Chromatic'],['major','Major'],['pentatonic_major','Pentatonic Major']];
  const detail=document.createElement('div');
  detail.id='resonator_frequency_detail';
  detail.className='frequency-detail';
  detail.innerHTML='<div class="frequency-detail-head"><div><div class="eyebrow">Pitch behavior</div><strong>Frequency quantization</strong></div>'+(quantization==='off'?'':`<span class="note-chip">Root ${frequencyRootNoteName(d)}</span>`)+
    '</div><p>CV6 chooses the displayed root and key. Bearamin XYZ movement is then constrained to notes in the selected scale.</p>'+
    `<fieldset class="web-choice"><legend>Scale</legend><div class="choice-options">${options.map(([value,label])=>`<label><input type="radio" name="res_frequency_quantization" value="${value}" data-res-setting="res_frequency_quantization"${value===quantization?' checked':''}> ${label}</label>`).join('')}</div></fieldset>`;
  lane.appendChild(detail);
  detail.querySelectorAll('[data-res-setting]').forEach(e=>e.onchange=()=>{e.blur();sendSettings(encodeURIComponent(e.dataset.resSetting)+'='+encodeURIComponent(e.value))});
};
</script><style>
.web-controls{margin-top:18px}.web-control-grid{display:grid;grid-template-columns:1fr;gap:0}.web-card{border:1px solid var(--line);border-left:3px solid var(--gold);border-radius:9px;padding:14px;background:#0d0b08}.web-card h3,.reverb-detail h3{margin:2px 0 4px;color:var(--gold)}.web-card>p,.reverb-detail>p{color:var(--muted);font-size:12px;margin-bottom:10px}.drone-card{border-left-color:var(--cyan)}.web-divider{display:flex;align-items:center;gap:10px;margin:17px 0;color:var(--dim);font-size:10px;font-weight:800;letter-spacing:.08em;text-transform:uppercase}.web-divider:before,.web-divider:after{content:"";height:1px;flex:1;background:var(--line)}.web-row{display:grid;grid-template-columns:minmax(125px,.55fr) minmax(0,1fr);gap:10px;align-items:center;margin-top:9px}.web-row>label,.web-modulation span{color:var(--muted);font-size:12px}.web-row input[type=range],.web-modulation input[type=range]{width:100%;accent-color:var(--gold)}.web-modulation{display:grid;grid-template-columns:minmax(0,1fr) minmax(180px,.8fr);gap:10px;margin-top:11px}.web-modulation>label{display:grid;gap:4px}.depth-help{margin-top:7px!important;padding-left:9px;border-left:2px solid var(--line)}.web-choice{margin:12px 0 0;border:0;border-top:1px solid var(--line);padding:10px 0 0}.web-choice legend{padding:0 8px 0 0;color:var(--muted);font-size:12px}.choice-options{display:flex;gap:8px;flex-wrap:wrap}.choice-options label{border:1px solid var(--line);border-radius:999px;padding:6px 10px;background:#15110b;cursor:pointer}.choice-options input,.web-check input{accent-color:var(--gold)}.web-check{display:flex;gap:7px;align-items:flex-start;margin-top:11px;color:var(--muted);font-size:12px}.danger-copy{color:var(--red);font-weight:800}.control-status{color:var(--muted);font-size:12px}.control-status.good{color:var(--green)}.reverb-detail{grid-column:3/-1;margin:12px 0 4px 16px;padding:13px;border-left:3px solid var(--cyan);border-radius:0 9px 9px 0;background:linear-gradient(90deg,rgba(112,214,216,.08),rgba(13,11,8,.72))}.reverb-detail .web-modulation{padding-bottom:11px;border-bottom:1px solid var(--line)}.reverb-detail .web-modulation:last-child{padding-bottom:0;border-bottom:0}@media(max-width:900px){.web-modulation{grid-template-columns:1fr}.reverb-detail{grid-column:1/-1;margin-left:0}.web-row{grid-template-columns:1fr}}
.frequency-detail{grid-column:3/-1;margin:12px 0 4px 16px;padding:12px 13px;border-left:3px solid var(--gold);border-radius:0 9px 9px 0;background:linear-gradient(90deg,rgba(240,184,78,.09),rgba(13,11,8,.72))}.frequency-detail-head{display:flex;align-items:center;justify-content:space-between;gap:12px}.frequency-detail>p{color:var(--muted);font-size:12px;margin:6px 0 0}.note-chip{display:inline-grid;place-items:center;min-width:48px;padding:5px 9px;border:1px solid var(--gold);border-radius:999px;color:var(--gold);font:800 14px SFMono-Regular,Consolas,monospace}
</style><style>
.map-controls{grid-template-columns:minmax(130px,1fr) minmax(150px,.9fr);align-items:start}.lane-control-head{display:grid;grid-template-columns:37px 26px minmax(0,1fr) auto;gap:9px;padding:0 0 5px}.map-control-head{grid-column:3;display:grid;grid-template-columns:minmax(130px,1fr) minmax(150px,.9fr);gap:7px}.map-control-head span{color:var(--dim);font-size:10px;font-weight:800;text-transform:uppercase;letter-spacing:.08em}.lane-depth{position:relative;min-width:0;padding-bottom:18px}.lane-depth input{display:block;width:100%;height:24px;margin:0;accent-color:var(--gold)}.lane-depth output{position:absolute;top:23px;left:var(--depth-position);transform:translateX(-50%);color:var(--gold);font:10px SFMono-Regular,Consolas,monospace;white-space:nowrap}.web-controls{width:calc(66% - 9px)}.web-card h3{margin:2px 0 4px;color:var(--gold)}.reverb-detail{padding:0}.reverb-detail summary{padding:12px 13px;color:var(--gold);font-weight:800;cursor:pointer}.reverb-detail-body{padding:0 13px 13px}.reverb-detail-body>p{color:var(--muted);font-size:12px;margin-bottom:10px}@media(max-width:900px){.lane-control-head{grid-template-columns:31px 23px minmax(0,1fr)}.web-controls{width:100%}.frequency-detail{grid-column:3/-1;margin-left:0}}
</style></body></html>
)HTML";

bool matrixStale() {
  return lastStateMs == 0 || millis() - lastStateMs > kMatrixStaleMs;
}

bool stateFlag(uint16_t flag) {
  return (latestState.stateFlags & flag) != 0;
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

bool sensorBusFlag(uint8_t flag) {
  return (latestSensorDiagnostics.busFlags & flag) != 0;
}

bool frameFlag(const LogFrameEntry &entry, uint16_t flag) {
  return (entry.stateFlags & flag) != 0;
}

const char *logCvSourceName(const LogFrameEntry &entry) {
  if (frameFlag(entry, bearamin_sidecar::STATE_MATRIX_PRESENT)) {
    return "matrix8x8";
  }
  return "no_target";
}

uint16_t logCvBridgeZ(const LogFrameEntry &entry) {
  if (frameFlag(entry, bearamin_sidecar::STATE_MATRIX_PRESENT)) {
    return entry.matrixZNorm;
  }
  return 0;
}

String jsonBool(bool value) {
  return value ? "true" : "false";
}

String jsonString(const String &value) {
  String out = "\"";
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  out += '"';
  return out;
}

const char *wifiDiagPhaseName(uint8_t phase) {
  switch (phase) {
    case WIFI_DIAG_BOOT_START: return "boot_start";
    case WIFI_DIAG_CONNECT_WINDOW_START: return "connect_window_start";
    case WIFI_DIAG_SCAN_FAILED: return "scan_failed";
    case WIFI_DIAG_SCAN_NO_SSID: return "scan_no_ssid";
    case WIFI_DIAG_SCAN_SELECTED: return "scan_selected";
    case WIFI_DIAG_CONNECT_BEGIN: return "connect_begin";
    case WIFI_DIAG_CONNECT_READY: return "connect_ready";
    case WIFI_DIAG_CONNECT_TIMEOUT: return "connect_timeout";
    case WIFI_DIAG_CONNECT_WINDOW_FAILED: return "connect_window_failed";
    case WIFI_DIAG_SOFTAP_START: return "softap_start";
    case WIFI_DIAG_SOFTAP_STOP: return "softap_stop";
    case WIFI_DIAG_SOFTAP_RESCAN: return "softap_rescan";
    case WIFI_DIAG_MDNS_READY: return "mdns_ready";
    case WIFI_DIAG_MDNS_FAILED: return "mdns_failed";
    case WIFI_DIAG_WEB_READY: return "web_ready";
    case WIFI_DIAG_OTA_READY: return "ota_ready";
    case WIFI_DIAG_POLL_LOST: return "poll_lost";
    case WIFI_DIAG_EVENT_STA_START: return "event_sta_start";
    case WIFI_DIAG_EVENT_STA_CONNECTED: return "event_sta_connected";
    case WIFI_DIAG_EVENT_STA_GOT_IP: return "event_sta_got_ip";
    case WIFI_DIAG_EVENT_STA_DISCONNECTED: return "event_sta_disconnected";
    case WIFI_DIAG_EVENT_STA_LOST_IP: return "event_sta_lost_ip";
    case WIFI_DIAG_EVENT_AP_START: return "event_ap_start";
    case WIFI_DIAG_EVENT_AP_STOP: return "event_ap_stop";
    case WIFI_DIAG_EVENT_AP_CLIENT_CONNECTED: return "event_ap_client_connected";
    case WIFI_DIAG_EVENT_AP_CLIENT_DISCONNECTED: return "event_ap_client_disconnected";
    case WIFI_DIAG_EVENT_SCAN_DONE: return "event_scan_done";
    case WIFI_DIAG_POLL_RECOVERED: return "poll_recovered";
  }
  return "unknown";
}

const char *resetReasonName(uint32_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
  }
  return "unknown";
}

const char *wifiStatusName(int status) {
  switch (status) {
    case WL_IDLE_STATUS: return "idle";
    case WL_NO_SSID_AVAIL: return "no_ssid";
    case WL_SCAN_COMPLETED: return "scan_completed";
    case WL_CONNECTED: return "connected";
    case WL_CONNECT_FAILED: return "connect_failed";
    case WL_CONNECTION_LOST: return "connection_lost";
    case WL_DISCONNECTED: return "disconnected";
    case WL_STOPPED: return "stopped";
    case WL_NO_SHIELD: return "no_shield";
  }
  return "unknown";
}

const char *wifiModeName(int mode) {
  switch (mode) {
    case WIFI_OFF: return "off";
    case WIFI_STA: return "sta";
    case WIFI_AP: return "ap";
    case WIFI_AP_STA: return "ap_sta";
  }
  return "unknown";
}

uint32_t ipValue(IPAddress ip) {
  return static_cast<uint32_t>(ip);
}

void formatBssid(char *out, size_t size, const uint8_t bssid[6]) {
  if (bssid == nullptr || size < 18) {
    if (size > 0) {
      out[0] = '\0';
    }
    return;
  }
  snprintf(out,
           size,
           "%02x:%02x:%02x:%02x:%02x:%02x",
           bssid[0],
           bssid[1],
           bssid[2],
           bssid[3],
           bssid[4],
           bssid[5]);
}

void formatIp(char *out, size_t size, uint32_t ip) {
  if (ip == 0 || size < 16) {
    if (size > 0) {
      out[0] = '\0';
    }
    return;
  }
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&ip);
  snprintf(out, size, "%u.%u.%u.%u", bytes[0], bytes[1], bytes[2], bytes[3]);
}

String bssidString(const uint8_t bssid[6]) {
  char text[18] = {};
  formatBssid(text, sizeof(text), bssid);
  return String(text);
}

String ipString(uint32_t ip) {
  if (ip == 0) {
    return "";
  }
  IPAddress address(ip);
  return address.toString();
}

void diagLogPrintf(const char *format, ...) {
  char line[384];
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (written <= 0) {
    return;
  }
  const size_t length = static_cast<size_t>(
      std::min<int>(written, static_cast<int>(sizeof(line) - 1)));
  Serial.write(reinterpret_cast<const uint8_t *>(line), length);
  if (debugSerialReady) {
    DebugSerial.write(reinterpret_cast<const uint8_t *>(line), length);
  }
}

void diagLogPrintln(const char *text) {
  diagLogPrintf("%s\n", text);
}

void printWifiDiagEvent(const WifiDiagEvent &event) {
  char bssid[18] = {};
  char ip[16] = {};
  formatBssid(bssid, sizeof(bssid), event.bssid);
  formatIp(ip, sizeof(ip), event.ip);
  diagLogPrintf(
      "wifi_diag seq=%lu ms=%lu phase=%s attempt=%lu mode=%s status=%s scans=%d matches=%d bssid=%s channel=%ld rssi=%ld reason=%u disconnect_rssi=%d ip=%s elapsed_ms=%lu\n",
      static_cast<unsigned long>(event.seq),
      static_cast<unsigned long>(event.ms),
      wifiDiagPhaseName(event.phase),
      static_cast<unsigned long>(event.attempt),
      wifiModeName(event.wifiMode),
      wifiStatusName(event.wifiStatus),
      static_cast<int>(event.scanCount),
      static_cast<int>(event.matchCount),
      bssid,
      static_cast<long>(event.channel),
      static_cast<long>(event.rssi),
      static_cast<unsigned>(event.disconnectReason),
      static_cast<int>(event.disconnectRssi),
      ip,
      static_cast<unsigned long>(event.elapsedMs));
}

void recordWifiDiag(WifiDiagPhase phase,
                    uint32_t attempt,
                    int32_t scanCount,
                    int32_t matchCount,
                    int32_t channel,
                    int32_t rssi,
                    const uint8_t *bssid,
                    uint8_t disconnectReason,
                    int32_t disconnectRssi,
                    uint32_t elapsedMs,
                    uint32_t ip) {
  WifiDiagEvent event = {};
  event.seq = ++wifiDiagSequence;
  event.ms = millis();
  event.attempt = attempt;
  event.elapsedMs = elapsedMs;
  event.phase = phase;
  event.wifiStatus = static_cast<int16_t>(WiFi.status());
  event.wifiMode = static_cast<int16_t>(WiFi.getMode());
  event.scanCount = static_cast<int16_t>(scanCount);
  event.matchCount = static_cast<int16_t>(matchCount);
  event.rssi = rssi;
  event.channel = channel;
  if (bssid != nullptr) {
    memcpy(event.bssid, bssid, sizeof(event.bssid));
    memcpy(wifiDiagLastBssid, bssid, sizeof(wifiDiagLastBssid));
  }
  event.disconnectReason = disconnectReason;
  event.disconnectRssi = static_cast<int16_t>(disconnectRssi);
  event.ip = ip;

  wifiDiagEvents[wifiDiagWriteIndex] = event;
  wifiDiagWriteIndex = (wifiDiagWriteIndex + 1) % kWifiDiagRingSize;
  if (wifiDiagCount < kWifiDiagRingSize) {
    ++wifiDiagCount;
  }
  wifiDiagLastPhase = phase;
  if (scanCount >= -1) {
    wifiDiagLastScanCount = static_cast<int16_t>(scanCount);
  }
  if (matchCount >= -1) {
    wifiDiagLastMatchCount = static_cast<int16_t>(matchCount);
  }
  if (rssi > -1000) {
    wifiDiagLastRssi = rssi;
  }
  if (channel > 0) {
    wifiDiagLastChannel = channel;
  }
  if (disconnectReason != 0) {
    wifiDiagLastDisconnectReason = disconnectReason;
    wifiDiagLastDisconnectRssi = static_cast<int16_t>(disconnectRssi);
  }
  printWifiDiagEvent(event);
}

String networkModeText() {
  if (wifiReady) {
    return softApReady ? "ap_sta" : "sta";
  }
  return softApReady ? "softap" : "offline";
}

uint32_t matrixCoreAgeMs(uint32_t now) {
  return lastStateMs == 0 ? 0 : now - lastStateMs;
}

void markWifiOutageStart(uint32_t now) {
  if (wifiDiagOutageActive) {
    return;
  }
  wifiDiagOutageActive = true;
  wifiDiagLastLostMs = now;
  wifiDiagMatrixFramesAtLoss = matrixFrames;
  wifiDiagCoreAgeAtLossMs = matrixCoreAgeMs(now);
}

void markWifiOutageEnd(uint32_t now) {
  if (!wifiDiagOutageActive) {
    return;
  }
  wifiDiagOutageActive = false;
  wifiDiagLastRecoveredMs = now;
  wifiDiagLastOutageMs = now - wifiDiagLastLostMs;
  if (wifiDiagLastOutageMs > wifiDiagLongestOutageMs) {
    wifiDiagLongestOutageMs = wifiDiagLastOutageMs;
  }
  wifiDiagMatrixFramesAtRecovery = matrixFrames;
  wifiDiagMatrixFramesDuringLastOutage = matrixFrames - wifiDiagMatrixFramesAtLoss;
  wifiDiagCoreAgeAtRecoveryMs = matrixCoreAgeMs(now);
}

void updateSidecarHealth(uint32_t now) {
  ++sidecarLoopCount;
  if (sidecarLastLoopMs != 0) {
    const uint32_t gap = now - sidecarLastLoopMs;
    if (gap > sidecarMaxLoopGapMs) {
      sidecarMaxLoopGapMs = gap;
    }
  }
  sidecarLastLoopMs = now;
  const uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < sidecarMinFreeHeap) {
    sidecarMinFreeHeap = freeHeap;
  }
}

void recordWebRequest() {
  ++webRequestCount;
  lastWebRequestMs = millis();
}

void appendSidecarHealthJson(String &body) {
  const uint32_t now = millis();
  const uint32_t currentOutageMs = wifiDiagOutageActive && wifiDiagLastLostMs != 0
                                      ? now - wifiDiagLastLostMs
                                      : 0;
  body += "{\"uptime_ms\":";
  body += now;
  body += ",\"reset_reason\":";
  body += sidecarBootResetReason;
  body += ",\"reset_reason_name\":";
  body += jsonString(resetReasonName(sidecarBootResetReason));
  body += ",\"free_heap\":";
  body += ESP.getFreeHeap();
  body += ",\"min_free_heap\":";
  body += sidecarMinFreeHeap == UINT32_MAX ? ESP.getFreeHeap() : sidecarMinFreeHeap;
  body += ",\"loop_count\":";
  body += sidecarLoopCount;
  body += ",\"max_loop_gap_ms\":";
  body += sidecarMaxLoopGapMs;
  body += ",\"web_request_count\":";
  body += webRequestCount;
  body += ",\"last_web_request_ms\":";
  body += lastWebRequestMs;
  body += ",\"last_web_request_age_ms\":";
  body += lastWebRequestMs == 0 ? 0 : now - lastWebRequestMs;
  body += ",\"wifi_outage_active\":";
  body += jsonBool(wifiDiagOutageActive);
  body += ",\"wifi_last_lost_ms\":";
  body += wifiDiagLastLostMs;
  body += ",\"wifi_last_recovered_ms\":";
  body += wifiDiagLastRecoveredMs;
  body += ",\"wifi_current_outage_ms\":";
  body += currentOutageMs;
  body += ",\"wifi_last_outage_ms\":";
  body += wifiDiagLastOutageMs;
  body += ",\"wifi_longest_outage_ms\":";
  body += wifiDiagLongestOutageMs;
  body += ",\"matrix_frames_at_wifi_loss\":";
  body += wifiDiagMatrixFramesAtLoss;
  body += ",\"matrix_frames_at_wifi_recovery\":";
  body += wifiDiagMatrixFramesAtRecovery;
  body += ",\"matrix_frames_during_last_wifi_outage\":";
  body += wifiDiagMatrixFramesDuringLastOutage;
  body += ",\"core_age_at_wifi_loss_ms\":";
  body += wifiDiagCoreAgeAtLossMs;
  body += ",\"core_age_at_wifi_recovery_ms\":";
  body += wifiDiagCoreAgeAtRecoveryMs;
  body += ",\"core_age_ms\":";
  body += matrixCoreAgeMs(now);
  body += ",\"matrix_state_sequence\":";
  body += latestState.sequence;
  body += "}";
}

void appendWifiDiagEventJson(String &body, const WifiDiagEvent &event, bool first) {
  body += first ? "" : ",";
  body += "{\"seq\":";
  body += event.seq;
  body += ",\"ms\":";
  body += event.ms;
  body += ",\"phase\":";
  body += jsonString(wifiDiagPhaseName(event.phase));
  body += ",\"attempt\":";
  body += event.attempt;
  body += ",\"mode\":";
  body += jsonString(wifiModeName(event.wifiMode));
  body += ",\"status\":";
  body += jsonString(wifiStatusName(event.wifiStatus));
  body += ",\"scan_count\":";
  body += event.scanCount;
  body += ",\"match_count\":";
  body += event.matchCount;
  body += ",\"bssid\":";
  body += jsonString(bssidString(event.bssid));
  body += ",\"channel\":";
  body += event.channel;
  body += ",\"rssi\":";
  body += event.rssi;
  body += ",\"disconnect_reason\":";
  body += event.disconnectReason;
  body += ",\"disconnect_rssi\":";
  body += event.disconnectRssi;
  body += ",\"ip\":";
  body += jsonString(ipString(event.ip));
  body += ",\"elapsed_ms\":";
  body += event.elapsedMs;
  body += "}";
}

void appendWifiDiagJson(String &body, bool includeEvents) {
  wifiReady = WiFi.status() == WL_CONNECTED;
  uint8_t currentBssid[6] = {};
  if (wifiReady && WiFi.BSSID() != nullptr) {
    memcpy(currentBssid, WiFi.BSSID(), sizeof(currentBssid));
  } else {
    memcpy(currentBssid, wifiDiagLastBssid, sizeof(currentBssid));
  }

  body += "{\"ready\":";
  body += jsonBool(wifiReady);
  body += ",\"network_mode\":";
  body += jsonString(networkModeText());
  body += ",\"wifi_band\":";
  body += jsonString(
      !wifiReady
          ? "off"
          : (bearamin_wifi::is5GhzChannel(WiFi.channel()) ? "5ghz"
                                                          : "2.4ghz"));
  body += ",\"status\":";
  body += jsonString(wifiStatusName(WiFi.status()));
  body += ",\"mode\":";
  body += jsonString(wifiModeName(WiFi.getMode()));
  body += ",\"last_event\":";
  body += jsonString(wifiDiagPhaseName(wifiDiagLastPhase));
  body += ",\"connect_attempts\":";
  body += wifiDiagConnectAttempts;
  body += ",\"successful_connects\":";
  body += wifiDiagSuccessfulConnects;
  body += ",\"failed_scans\":";
  body += wifiDiagFailedScans;
  body += ",\"no_ssid_scans\":";
  body += wifiDiagNoSsidScans;
  body += ",\"disconnects\":";
  body += wifiDiagDisconnects;
  body += ",\"last_disconnect_reason\":";
  body += wifiDiagLastDisconnectReason;
  body += ",\"last_disconnect_rssi\":";
  body += wifiDiagLastDisconnectRssi;
  body += ",\"last_scan_count\":";
  body += wifiDiagLastScanCount;
  body += ",\"last_match_count\":";
  body += wifiDiagLastMatchCount;
  body += ",\"current_channel\":";
  body += wifiReady ? WiFi.channel() : wifiDiagLastChannel;
  body += ",\"current_rssi\":";
  body += wifiReady ? WiFi.RSSI() : wifiDiagLastRssi;
  body += ",\"current_bssid\":";
  body += jsonString(bssidString(currentBssid));
  body += ",\"sta_ip\":";
  body += jsonString(wifiReady ? WiFi.localIP().toString() : "");
  body += ",\"softap_ready\":";
  body += jsonBool(softApReady);
  body += ",\"softap_clients\":";
  body += softApReady ? WiFi.softAPgetStationNum() : 0;
  body += ",\"outage_active\":";
  body += jsonBool(wifiDiagOutageActive);
  body += ",\"last_lost_ms\":";
  body += wifiDiagLastLostMs;
  body += ",\"last_recovered_ms\":";
  body += wifiDiagLastRecoveredMs;
  body += ",\"current_outage_ms\":";
  body += (wifiDiagOutageActive && wifiDiagLastLostMs != 0) ? millis() - wifiDiagLastLostMs : 0;
  body += ",\"last_outage_ms\":";
  body += wifiDiagLastOutageMs;
  body += ",\"longest_outage_ms\":";
  body += wifiDiagLongestOutageMs;
  body += ",\"matrix_frames_during_last_outage\":";
  body += wifiDiagMatrixFramesDuringLastOutage;
  body += ",\"latest_seq\":";
  body += wifiDiagSequence;
  body += ",\"retained\":";
  body += wifiDiagCount;
  if (includeEvents) {
    body += ",\"events\":[";
    const uint8_t oldestIndex = (wifiDiagWriteIndex + kWifiDiagRingSize - wifiDiagCount) % kWifiDiagRingSize;
    bool first = true;
    for (uint8_t i = 0; i < wifiDiagCount; ++i) {
      appendWifiDiagEventJson(body, wifiDiagEvents[(oldestIndex + i) % kWifiDiagRingSize], first);
      first = false;
    }
    body += "]";
  }
  body += "}";
}

bool parseBoolArg(const String &value) {
  return value == "1" || value == "true" || value == "on" ||
         value == "yes" || value == "enabled" || value == "unipolar";
}

bool isPresetNameChar(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') || c == '_' || c == '-';
}

String sanitizePresetName(const String &raw) {
  String out;
  out.reserve(kPresetMaxNameLength);
  for (size_t i = 0; i < raw.length() && out.length() < kPresetMaxNameLength; ++i) {
    const char c = raw[i];
    if (isPresetNameChar(c)) {
      out += c;
    }
  }
  return out;
}

char hexNibble(uint8_t value) {
  value &= 0x0Fu;
  return value < 10 ? static_cast<char>('0' + value)
                    : static_cast<char>('A' + value - 10);
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}

String hexEncode(const uint8_t *data, size_t length) {
  String out;
  out.reserve(length * 2);
  for (size_t i = 0; i < length; ++i) {
    out += hexNibble(static_cast<uint8_t>(data[i] >> 4));
    out += hexNibble(data[i]);
  }
  return out;
}

bool appendHexDecoded(const String &hex, String &out) {
  if ((hex.length() % 2) != 0) {
    return false;
  }
  for (size_t i = 0; i < hex.length(); i += 2) {
    const int hi = hexValue(hex[i]);
    const int lo = hexValue(hex[i + 1]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    out += static_cast<char>((hi << 4) | lo);
  }
  return true;
}

uint16_t crc16String(const String &value) {
  return bearamin_sidecar::crc16Ccitt(
      reinterpret_cast<const uint8_t *>(value.c_str()), value.length());
}

uint8_t clampFmParam(uint8_t value) {
  return value <= bearamin_bridge::FM_PARAM_CV_SLEW
             ? value
             : bearamin_bridge::FM_PARAM_PITCH;
}

const char *fmParamName(uint8_t value) {
  switch (clampFmParam(value)) {
    case bearamin_bridge::FM_PARAM_FM1_AMOUNT:
      return "fm1_amount";
    case bearamin_bridge::FM_PARAM_FM1_FREQUENCY:
      return "fm1_frequency";
    case bearamin_bridge::FM_PARAM_FM2_AMOUNT:
      return "fm2_amount";
    case bearamin_bridge::FM_PARAM_FM2_FREQUENCY:
      return "fm2_frequency";
    case bearamin_bridge::FM_PARAM_STEREO:
      return "stereo";
    case bearamin_bridge::FM_PARAM_CV_SLEW:
      return "cv_slew";
    case bearamin_bridge::FM_PARAM_PITCH:
    default:
      return "pitch";
  }
}

uint8_t parseFmParamArg(const String &value, uint8_t fallback) {
  if (value == "pitch") return bearamin_bridge::FM_PARAM_PITCH;
  if (value == "fm1_amount") return bearamin_bridge::FM_PARAM_FM1_AMOUNT;
  if (value == "fm1_frequency") return bearamin_bridge::FM_PARAM_FM1_FREQUENCY;
  if (value == "fm2_amount") return bearamin_bridge::FM_PARAM_FM2_AMOUNT;
  if (value == "fm2_frequency") return bearamin_bridge::FM_PARAM_FM2_FREQUENCY;
  if (value == "stereo") return bearamin_bridge::FM_PARAM_STEREO;
  if (value == "cv_slew") return bearamin_bridge::FM_PARAM_CV_SLEW;
  if (value.length() > 0 && isDigit(value[0])) {
    return clampFmParam(static_cast<uint8_t>(value.toInt()));
  }
  return fallback;
}

uint8_t clampResonatorMap(uint8_t value) {
  return value <= 27 ? value : 0;
}

bearamin_bridge::ResonatorAxis resonatorMapAxis(uint8_t value) {
  return bearamin_bridge::resonatorMappingAxis(clampResonatorMap(value));
}

uint8_t resonatorMapStrength(uint8_t value) {
  return bearamin_bridge::resonatorMappingStrength(clampResonatorMap(value));
}

bool resonatorMapUnipolar(uint8_t value) {
  return bearamin_bridge::resonatorMappingUnipolar(clampResonatorMap(value));
}

uint8_t encodeResonatorMap(bearamin_bridge::ResonatorAxis axis, bool unipolar, uint8_t strength) {
  return bearamin_bridge::encodeResonatorMapping(axis, unipolar, strength);
}

const char *resonatorMapAxisName(uint8_t value) {
  switch (resonatorMapAxis(value)) {
    case bearamin_bridge::RESONATOR_AXIS_X: return resonatorMapUnipolar(value) ? "x_unipolar" : "x_bipolar";
    case bearamin_bridge::RESONATOR_AXIS_Y: return resonatorMapUnipolar(value) ? "y_unipolar" : "y_bipolar";
    case bearamin_bridge::RESONATOR_AXIS_Z: return resonatorMapUnipolar(value) ? "z_unipolar" : "z_bipolar";
    case bearamin_bridge::RESONATOR_AXIS_PITCH: return "pitch";
    case bearamin_bridge::RESONATOR_AXIS_ROLL: return "roll";
    case bearamin_bridge::RESONATOR_AXIS_SPANK: return "spank";
    case bearamin_bridge::RESONATOR_AXIS_NONE:
    default:
      return "none";
  }
}

const char *resonatorMapStrengthName(uint8_t value) {
  switch (resonatorMapStrength(value)) {
    case bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL: return "--";
    case bearamin_bridge::RESONATOR_STRENGTH_NEG_HALF: return "-";
    case bearamin_bridge::RESONATOR_STRENGTH_POS_FULL: return "++";
    case bearamin_bridge::RESONATOR_STRENGTH_POS_HALF:
    default:
      return "+";
  }
}

uint8_t parseResonatorMapAxisArg(const String &value, uint8_t fallback) {
  const uint8_t strength = resonatorMapStrength(fallback);
  if (value == "none") return 0;
  if (value == "x_bipolar") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_X, false, strength);
  if (value == "y_bipolar") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_Y, false, strength);
  if (value == "z_bipolar") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_Z, false, strength);
  if (value == "x_unipolar") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_X, true, strength);
  if (value == "y_unipolar") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_Y, true, strength);
  if (value == "z_unipolar") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_Z, true, strength);
  if (value == "pitch") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_PITCH, false, strength);
  if (value == "roll") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_ROLL, false, strength);
  if (value == "spank") return encodeResonatorMap(bearamin_bridge::RESONATOR_AXIS_SPANK, false, strength);
  return 0;
}

uint8_t parseResonatorMapStrengthArg(const String &value, uint8_t fallback) {
  const bearamin_bridge::ResonatorAxis axis = resonatorMapAxis(fallback);
  const bool unipolar = resonatorMapUnipolar(fallback);
  if (value == "--") {
    return encodeResonatorMap(axis, unipolar, bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL);
  }
  if (value == "-") {
    return encodeResonatorMap(axis, unipolar, bearamin_bridge::RESONATOR_STRENGTH_NEG_HALF);
  }
  if (value == "++") {
    return encodeResonatorMap(axis, unipolar, bearamin_bridge::RESONATOR_STRENGTH_POS_FULL);
  }
  return encodeResonatorMap(axis, unipolar, bearamin_bridge::RESONATOR_STRENGTH_POS_HALF);
}

uint8_t gestureMapForModeLane(uint8_t mode, uint8_t lane) {
  if (mode < kGestureConfigCount && lane < 6) {
    return clampResonatorMap(modeGestureMaps[mode][lane]);
  }
  return 0;
}

void setGestureMapForModeLane(uint8_t mode, uint8_t lane, uint8_t value) {
  if (mode >= kGestureConfigCount || lane >= 6) {
    return;
  }
  modeGestureMaps[mode][lane] = clampResonatorMap(value);
}

int8_t gestureDepthForModeLane(uint8_t mode, uint8_t lane) {
  return mode < kGestureConfigCount && lane < 6 ? modeGestureDepths[mode][lane] : 0;
}

uint8_t packedStrengthForDepth(int8_t depth) {
  if (depth < 0) {
    return depth <= -75 ? bearamin_bridge::RESONATOR_STRENGTH_NEG_FULL
                        : bearamin_bridge::RESONATOR_STRENGTH_NEG_HALF;
  }
  return depth >= 75 ? bearamin_bridge::RESONATOR_STRENGTH_POS_FULL
                     : bearamin_bridge::RESONATOR_STRENGTH_POS_HALF;
}

void setGestureDepthForModeLane(uint8_t mode, uint8_t lane, long requested) {
  if (mode >= kGestureConfigCount || lane >= 6) return;
  if (requested < -100) requested = -100;
  if (requested > 100) requested = 100;
  modeGestureDepths[mode][lane] = static_cast<int8_t>(requested);
  const uint8_t current = gestureMapForModeLane(mode, lane);
  if (resonatorMapAxis(current) != bearamin_bridge::RESONATOR_AXIS_NONE) {
    modeGestureMaps[mode][lane] = encodeResonatorMap(
        resonatorMapAxis(current), resonatorMapUnipolar(current),
        packedStrengthForDepth(modeGestureDepths[mode][lane]));
  }
}

int8_t gestureConfigForDptMode(uint8_t mode) {
  switch (mode) {
    case 0:
    case 1:
    case 2: return kGestureConfigRingsResonator;
    case 3: return kGestureConfigBearaminFm;
    default: return -1;
  }
}

const char *gestureConfigKey(uint8_t config) {
  switch (config) {
    case kGestureConfigBearaminFm: return "bearamin_fm";
    case kGestureConfigRingsResonator: return "rings_resonator";
    default: return "";
  }
}

uint32_t packedGestureMappingsForMode(uint8_t mode) {
  return bearamin_bridge::packResonatorMappings(
      gestureMapForModeLane(mode, 0),
      gestureMapForModeLane(mode, 1),
      gestureMapForModeLane(mode, 2),
      gestureMapForModeLane(mode, 3),
      gestureMapForModeLane(mode, 4),
      gestureMapForModeLane(mode, 5));
}

uint8_t clampPercent(long value, uint8_t maximum = 100) {
  if (value < 0) return 0;
  if (value > maximum) return maximum;
  return static_cast<uint8_t>(value);
}

uint8_t encodeSignedDepth(long value) {
  if (value < -100) value = -100;
  if (value > 100) value = 100;
  return static_cast<uint8_t>(static_cast<int8_t>(value));
}

const char *resonatorWebSourceName(uint8_t value) {
  switch (value) {
    case bearamin_bridge::RESONATOR_WEB_SOURCE_X_BIPOLAR: return "x_bipolar";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_Y_BIPOLAR: return "y_bipolar";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_Z_BIPOLAR: return "z_bipolar";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_X_UNIPOLAR: return "x_unipolar";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_Y_UNIPOLAR: return "y_unipolar";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_Z_UNIPOLAR: return "z_unipolar";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_CV5_WET: return "cv5_wet";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_PITCH: return "pitch";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_ROLL: return "roll";
    case bearamin_bridge::RESONATOR_WEB_SOURCE_SPANK: return "spank";
    default: return "none";
  }
}

uint8_t parseResonatorWebSource(const String &value, bool allowCv5, uint8_t fallback) {
  if (value == "none") return bearamin_bridge::RESONATOR_WEB_SOURCE_NONE;
  if (value == "x_bipolar") return bearamin_bridge::RESONATOR_WEB_SOURCE_X_BIPOLAR;
  if (value == "y_bipolar") return bearamin_bridge::RESONATOR_WEB_SOURCE_Y_BIPOLAR;
  if (value == "z_bipolar") return bearamin_bridge::RESONATOR_WEB_SOURCE_Z_BIPOLAR;
  if (value == "x_unipolar") return bearamin_bridge::RESONATOR_WEB_SOURCE_X_UNIPOLAR;
  if (value == "y_unipolar") return bearamin_bridge::RESONATOR_WEB_SOURCE_Y_UNIPOLAR;
  if (value == "z_unipolar") return bearamin_bridge::RESONATOR_WEB_SOURCE_Z_UNIPOLAR;
  if (value == "pitch") return bearamin_bridge::RESONATOR_WEB_SOURCE_PITCH;
  if (value == "roll") return bearamin_bridge::RESONATOR_WEB_SOURCE_ROLL;
  if (value == "spank") return bearamin_bridge::RESONATOR_WEB_SOURCE_SPANK;
  if (allowCv5 && value == "cv5_wet") return bearamin_bridge::RESONATOR_WEB_SOURCE_CV5_WET;
  return fallback;
}

const char *resonatorDroneWaveformName(uint8_t flags) {
  switch ((flags & bearamin_bridge::RESONATOR_WEB_DRONE_WAVEFORM_MASK) >>
          bearamin_bridge::RESONATOR_WEB_DRONE_WAVEFORM_SHIFT) {
    case 1: return "saw";
    default: return "sine";
  }
}

uint8_t parseResonatorDroneWaveform(const String &value) {
  if (value == "saw") return 1;
  return 0;
}

const char *resonatorFrequencyQuantizationName(uint8_t flags) {
  switch ((flags & bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_MASK) >>
          bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_SHIFT) {
    case bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_CHROMATIC:
      return "chromatic";
    case bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_MAJOR:
      return "major";
    case bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_MAJOR_PENTATONIC:
      return "pentatonic_major";
    default:
      return "off";
  }
}

uint8_t parseResonatorFrequencyQuantization(const String &value) {
  if (value == "chromatic") {
    return bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_CHROMATIC;
  }
  if (value == "major") {
    return bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_MAJOR;
  }
  if (value == "pentatonic_major") {
    return bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_MAJOR_PENTATONIC;
  }
  return bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_OFF;
}

void setResonatorFrequencyQuantization(uint8_t quantization) {
  resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] &=
      static_cast<uint8_t>(
          ~bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_MASK);
  resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] |=
      static_cast<uint8_t>(
          (quantization <<
           bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_SHIFT) &
          bearamin_bridge::RESONATOR_WEB_FREQUENCY_QUANTIZATION_MASK);
}

bool resonatorWebFlag(uint8_t flag) {
  return (resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] & flag) != 0;
}

void setResonatorWebFlag(uint8_t flag, bool enabled) {
  if (enabled) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] |= flag;
  } else {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS] &= static_cast<uint8_t>(~flag);
  }
}

uint8_t effectiveDptMode() {
  return latestState.dptModeIndex;
}

uint16_t dptControlFlagsForMode(uint8_t mode) {
  if (mode == 3) {
    return bearamin_bridge::encodeFmDroneControls(
        fmDroneRangerMode,
        static_cast<bearamin_bridge::FmParam>(clampFmParam(fmDroneMapX)),
        static_cast<bearamin_bridge::FmParam>(clampFmParam(fmDroneMapY)),
        static_cast<bearamin_bridge::FmParam>(clampFmParam(fmDroneMapZ)));
  }
  if (mode <= 2) {
    const uint8_t config = static_cast<uint8_t>(gestureConfigForDptMode(mode));
    return bearamin_bridge::encodeResonatorControls(
        resonatorMapAxis(gestureMapForModeLane(config, 0)),
        resonatorMapAxis(gestureMapForModeLane(config, 1)),
        resonatorMapAxis(gestureMapForModeLane(config, 2)),
        resonatorMapAxis(gestureMapForModeLane(config, 3)),
        resonatorMapAxis(gestureMapForModeLane(config, 4)),
        resonatorMapAxis(gestureMapForModeLane(config, 5)));
  }
  if (mode == 6) {
    return 0;
  }
  return 0;
}

void appendSettingsJson(String &body) {
  body += ",\"firmware_version\":";
  body += jsonString(BEARAMIN_FIRMWARE_VERSION);
  body += ",\"fm_drone_ranger\":";
  body += jsonBool(fmDroneRangerMode);
  body += ",\"fm_map_x\":";
  body += jsonString(fmParamName(fmDroneMapX));
  body += ",\"fm_map_y\":";
  body += jsonString(fmParamName(fmDroneMapY));
  body += ",\"fm_map_z\":";
  body += jsonString(fmParamName(fmDroneMapZ));
  body += ",\"announce_modes\":";
  body += jsonBool(modeAnnouncementsEnabled);
  body += ",\"announce_modes_applied\":";
  body += jsonBool(stateFlag(
      bearamin_sidecar::STATE_MODE_ANNOUNCEMENTS_ENABLED));
  for (uint8_t mode = 0; mode < kGestureConfigCount; ++mode) {
    for (uint8_t lane = 0; lane < 6; ++lane) {
      char key[48];
      snprintf(key, sizeof(key), ",\"map_%s_%u_axis\":", gestureConfigKey(mode), lane);
      body += key;
      body += jsonString(resonatorMapAxisName(gestureMapForModeLane(mode, lane)));
      snprintf(key, sizeof(key), ",\"map_%s_%u_depth\":", gestureConfigKey(mode), lane);
      body += key;
      body += static_cast<int16_t>(gestureDepthForModeLane(mode, lane));
    }
  }
  body += ",\"cv_slew_step\":";
  body += cvSlewStep;
  body += ",\"res_feedback_amount\":";
  body += resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_BASE];
  body += ",\"res_feedback_source\":";
  body += jsonString(resonatorWebSourceName(
      resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_SOURCE]));
  body += ",\"res_feedback_depth\":";
  body += static_cast<int16_t>(static_cast<int8_t>(
      resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_DEPTH]));
  body += ",\"res_frequency_quantization\":";
  body += jsonString(resonatorFrequencyQuantizationName(
      resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS]));
  body += ",\"res_drone_enabled\":";
  body += jsonBool(resonatorWebFlag(bearamin_bridge::RESONATOR_WEB_DRONE_ENABLED));
  body += ",\"res_drone_waveform\":";
  body += jsonString(resonatorDroneWaveformName(
      resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS]));
  body += ",\"res_drone_level\":";
  body += resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_LEVEL];
  body += ",\"res_drone_source\":";
  body += jsonString(resonatorWebSourceName(
      resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_SOURCE]));
  body += ",\"res_drone_depth\":";
  body += static_cast<int16_t>(static_cast<int8_t>(
      resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_DEPTH]));
  const uint8_t reverbOffsets[3][3] = {
      {bearamin_bridge::RESONATOR_WEB_REVERB_TIME,
       bearamin_bridge::RESONATOR_WEB_REVERB_TIME_SOURCE,
       bearamin_bridge::RESONATOR_WEB_REVERB_TIME_DEPTH},
      {bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION,
       bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_SOURCE,
       bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_DEPTH},
      {bearamin_bridge::RESONATOR_WEB_REVERB_LP,
       bearamin_bridge::RESONATOR_WEB_REVERB_LP_SOURCE,
       bearamin_bridge::RESONATOR_WEB_REVERB_LP_DEPTH},
  };
  const char *reverbNames[3] = {"time", "diffusion", "lp"};
  for (uint8_t i = 0; i < 3; ++i) {
    body += ",\"res_reverb_";
    body += reverbNames[i];
    body += "\":";
    body += resonatorWebControls[reverbOffsets[i][0]];
    body += ",\"res_reverb_";
    body += reverbNames[i];
    body += "_source\":";
    body += jsonString(resonatorWebSourceName(resonatorWebControls[reverbOffsets[i][1]]));
    body += ",\"res_reverb_";
    body += reverbNames[i];
    body += "_depth\":";
    body += static_cast<int16_t>(static_cast<int8_t>(resonatorWebControls[reverbOffsets[i][2]]));
  }
}

String stateJson() {
  const bool stale = matrixStale();
  String body;
  body.reserve(5000);
  body += "{\"device\":\"Bearamin\",\"role\":";
  body += jsonString(BEARAMIN_DEVICE_ROLE);
  body += ",\"mac\":";
  body += jsonString(WiFi.macAddress());
  body += ",\"core_link_status\":\"";
  body += stale ? "stale" : "ready";
  body += "\",\"core_link_stale\":";
  body += jsonBool(stale);
  body += ",\"core_age_ms\":";
  body += matrixCoreAgeMs(millis());
  body += ",\"matrix_frames\":";
  body += matrixFrames;
  body += ",\"matrix_bytes\":";
  body += matrixBytes;
  body += ",\"dpt_ready\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_DPT_READY));
  body += ",\"dpt_control_tx_quiet\":";
  body += jsonBool(!stale && stateFlag(
      bearamin_sidecar::STATE_DPT_CONTROL_TX_QUIET));
  body += ",\"dpt_mode\":";
  body += latestState.dptModeIndex;
  body += ",\"dpt_protocol_version\":";
  body += latestState.dptProtocolVersion;
  body += ",\"resonator_controls_revision\":";
  body += resonatorControlsRevision;
  body += ",\"dpt_resonator_controls_revision\":";
  body += latestState.dptResonatorControlsRevision;
  body += ",\"resonator_controls_applied\":";
  body += jsonBool(!stale && resonatorControlsRevision == latestState.dptResonatorControlsRevision);
  body += ",\"dpt_knobs\":[";
  for (uint8_t i = 0; i < bearamin_bridge::kDptControlCount; ++i) {
    if (i > 0) {
      body += ',';
    }
    body += latestState.dptControls[i];
  }
  body += "]";
  body += ",\"dpt_probe_tx_count\":";
  body += latestState.dptProbeTxCount;
  body += ",\"dpt_frame_tx_count\":";
  body += latestState.dptFrameTxCount;
  body += ",\"dpt_preset_tx_count\":";
  body += latestState.dptPresetTxCount;
  body += ",\"dpt_rx_byte_count\":";
  body += latestState.dptRxByteCount;
  body += ",\"dpt_status_line_count\":";
  body += latestState.dptStatusLineCount;
  body += ",\"dpt_preset_line_count\":";
  body += latestState.dptPresetLineCount;
  body += ",\"dpt_rx_debug\":";
  body += jsonString(dptRxDebugLine);
  body += ",\"dpt_performance_frames\":";
  body += dptPerformanceFrames;
  body += ",\"dpt_performance_age_ms\":";
  body += lastDptPerformanceMs == 0 ? 0xFFFFFFFFu : millis() - lastDptPerformanceMs;
  body += ",\"dpt_audio_callback_count\":";
  body += latestDptPerformance.callbackCount;
  body += ",\"dpt_audio_max_callback_us\":";
  body += latestDptPerformance.maxCallbackUs;
  body += ",\"dpt_audio_deadline_misses\":";
  body += latestDptPerformance.deadlineMisses;
  body += ",\"dpt_audio_late_starts\":";
  body += latestDptPerformance.lateStarts;
  body += ",\"dpt_audio_missed_periods\":";
  body += latestDptPerformance.missedPeriods;
  body += ",\"dpt_audio_max_start_gap_us\":";
  body += latestDptPerformance.maxStartGapUs;
  body += ",\"dpt_return_tx_calls\":";
  body += latestDptPerformance.txCallCount;
  body += ",\"dpt_return_tx_max_irq_off_us\":";
  body += latestDptPerformance.maxIrqOffUs;
  body += ",\"dpt_return_tx_max_call_us\":";
  body += latestDptPerformance.maxTxCallUs;
  body += ",\"matrix_ready\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_MATRIX_READY));
  body += ",\"matrix_source_width\":8";
  body += ",\"matrix_source_height\":8";
  body += ",\"matrix_source_hz\":";
  body += latestMatrixPerformance.sourceHz;
  body += ",\"matrix_output_hz\":";
  body += latestMatrixPerformance.outputHz;
  body += ",\"matrix_effective_hz\":";
  body += static_cast<double>(latestMatrixPerformance.effectiveRateMilliHz) /
          1000.0;
  body += ",\"synthetic_gate\":";
  body += jsonBool(
      !stale && stateFlag(bearamin_sidecar::STATE_SYNTHETIC_GATE));
  body += ",\"matrix_interval_max_us\":";
  body += latestMatrixPerformance.intervalMaxUs;
  body += ",\"matrix_read_max_us\":";
  body += latestMatrixPerformance.readMaxUs;
  body += ",\"matrix_process_max_us\":";
  body += latestMatrixPerformance.processMaxUs;
  body += ",\"matrix_loop_gap_max_us\":";
  body += latestMatrixPerformance.loopGapMaxUs;
  body += ",\"matrix_read_errors\":";
  body += latestMatrixPerformance.readErrors;
  body += ",\"matrix_ready_poll_status\":";
  body += latestMatrixPerformance.readyPollStatus;
  body += ",\"matrix_ready_poll_value\":";
  body += latestMatrixPerformance.readyPollValue;
  body += ",\"matrix_free_heap_bytes\":";
  body += latestMatrixPerformance.freeHeapBytes;
  body += ",\"matrix_minimum_free_heap_bytes\":";
  body += latestMatrixPerformance.minimumFreeHeapBytes;
  body += ",\"matrix_recovery_stack_headroom_words\":";
  body += latestMatrixPerformance.recoveryStackWords;
  body += ",\"matrix_performance_frames\":";
  body += matrixPerformanceFrames;
  body += ",\"matrix_performance_age_ms\":";
  body += lastMatrixPerformanceMs == 0 ? 0xFFFFFFFFu :
      millis() - lastMatrixPerformanceMs;
  body += ",\"matrix_present\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_MATRIX_PRESENT));
  body += ",\"matrix_x_milli\":";
  body += latestState.matrixXMilli;
  body += ",\"matrix_y_milli\":";
  body += latestState.matrixYMilli;
  body += ",\"matrix_z_mm\":";
  body += latestState.matrixZMm;
  body += ",\"matrix_x_norm\":";
  body += latestState.matrixXNorm;
  body += ",\"matrix_y_norm\":";
  body += latestState.matrixYNorm;
  body += ",\"matrix_z_norm\":";
  body += latestState.matrixZNorm;
  body += ",\"matrix_valid_count\":";
  body += latestState.matrixValidCount;
  body += ",\"matrix_blob_count\":";
  body += latestState.matrixBlobCount;
  body += ",\"pitch_norm\":";
  body += latestState.pitchNorm;
  body += ",\"roll_norm\":";
  body += latestState.rollNorm;
  body += ",\"orientation_confidence\":";
  body += latestState.orientationConfidence;
  body += ",\"spank_gate\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_SPANK_GATE));
  body += ",\"imu_ready\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_IMU_READY));
  body += ",\"impact_gate\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_IMPACT_GATE));
  body += ",\"impact_velocity\":";
  body += latestState.impactVelocityNorm;
  body += ",\"impact_age_ms\":";
  body += latestState.impactAgeMs;
  body += ",\"matrixcore_ota_mode\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_MATRIX_OTA_MODE));
  body += ",\"matrixcore_ota_active\":";
  body += jsonBool(!stale && stateFlag(bearamin_sidecar::STATE_MATRIX_OTA_ACTIVE));
  body += ",\"accel_mg\":[";
  body += latestState.accelXMilliG;
  body += ',';
  body += latestState.accelYMilliG;
  body += ',';
  body += latestState.accelZMilliG;
  body += "],\"sample_cells_mm\":[";
  for (uint8_t i = 0; i < 8; ++i) {
    if (i > 0) {
      body += ',';
    }
    body += latestState.cellsMm[i];
  }
  body += "],\"frame_errors\":";
  body += latestState.frameErrors;
  body += ",\"i2c_recoveries\":";
  body += latestState.i2cRecoveries;
  body += ",\"sensor_diagnostics_frames\":";
  body += matrixSensorDiagnosticsFrames;
  body += ",\"sensor_diagnostics_age_ms\":";
  body += lastSensorDiagnosticsMs == 0 ? 65535 : std::min<uint32_t>(
      millis() - lastSensorDiagnosticsMs, 65535);
  body += ",\"optical_recovery_failures\":";
  body += latestSensorDiagnostics.recoveryFailures;
  body += ",\"optical_recovery_active\":";
  body += jsonBool(!stale && stateFlag(
      bearamin_sidecar::STATE_OPTICAL_RECOVERY_ACTIVE));
  body += ",\"optical_recovery_reason\":";
  body += jsonString(sensorRecoveryReasonName(latestSensorDiagnostics.recoveryReason));
  body += ",\"matrix_age_ms\":";
  body += latestSensorDiagnostics.matrixAgeMs;
  body += ",\"optical_recovery_sda_high\":";
  body += jsonBool(sensorBusFlag(bearamin_sidecar::SENSOR_BUS_SDA_HIGH));
  body += ",\"optical_recovery_scl_high\":";
  body += jsonBool(sensorBusFlag(bearamin_sidecar::SENSOR_BUS_SCL_HIGH));
  body += ",\"optical_recovery_default_address_ack\":";
  body += jsonBool(sensorBusFlag(
      bearamin_sidecar::SENSOR_BUS_DEFAULT_ADDRESS_ACK));
  body += ",\"matrix_cell_frames\":";
  body += matrixCellFrames;
  body += ",\"engineering_stream\":";
  body += jsonBool(engineeringStreamEnabled);
  body += ",\"telemetry_logging\":";
  body += jsonBool(telemetryLoggingEnabled);
  body += ",\"telemetry_log_rows\":";
  body += logCount;
  body += ",\"wifi_ready\":";
  body += jsonBool(wifiReady);
  body += ",\"web_ready\":";
  body += jsonBool(webReady);
  body += ",\"ota_ready\":";
  body += jsonBool(otaReady);
  body += ",\"network_mode\":";
  body += jsonString(wifiReady ? WiFi.SSID() : networkModeText());
  body += ",\"wifi_ssid\":";
  body += jsonString(wifiReady ? WiFi.SSID() : String(WIFI_SSID));
  body += ",\"wifi_band\":";
  body += jsonString(
      !wifiReady
          ? "off"
          : (bearamin_wifi::is5GhzChannel(WiFi.channel()) ? "5ghz"
                                                          : "2.4ghz"));
  body += ",\"preset_sd_ready\":";
  body += jsonBool(presetSdReady);
  body += ",\"preset_sd_spi_hz\":";
  body += String(presetSdActiveSpiHz);
  body += ",\"preset_default_available\":";
  body += jsonBool(presetDefaultAvailable);
  body += ",\"preset_default_attempted\":";
  body += jsonBool(presetDefaultAttempted);
  body += ",\"preset_default_loaded\":";
  body += jsonBool(presetDefaultLoaded);
  body += ",\"boot_control_sync_complete\":";
  body += jsonBool(bootControlSyncComplete);
  body += ",\"boot_control_sync_source\":";
  body += jsonString(
      !bootControlSyncComplete
          ? "pending"
          : (bootControlSyncUsedRuntime ? "matrix_runtime" : "boot_defaults"));
  body += ",\"runtime_settings_received\":";
  body += jsonBool(runtimeSettingsReceived);
  body += ",\"preset_error\":";
  body += jsonString(presetLastError);
  body += ",\"preset_last_line\":";
  body += jsonString(presetLastLine);
  body += ",\"sta_ip\":";
  body += jsonString(wifiReady ? WiFi.localIP().toString() : "");
  body += ",\"softap_ready\":";
  body += jsonBool(softApReady);
  body += ",\"softap_ssid\":";
  body += jsonString(softApReady ? String(kSoftApSsid) : "");
  body += ",\"softap_ip\":";
  body += jsonString(softApReady ? WiFi.softAPIP().toString() : "");
  body += ",\"softap_clients\":";
  body += softApReady ? WiFi.softAPgetStationNum() : 0;
  body += ",\"wifi_diag\":";
  appendWifiDiagJson(body, false);
  body += ",\"sidecar_health\":";
  appendSidecarHealthJson(body);
  appendSettingsJson(body);
  body += "}";
  return body;
}

void sendJson(const String &body) {
  recordWebRequest();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", body);
}

void handleRoot() {
  recordWebRequest();
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleCaptureWorker() {
  recordWebRequest();
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(
      200, "application/javascript; charset=utf-8", CAPTURE_WORKER_JS);
}

void handleFavicon() {
  recordWebRequest();
  server.sendHeader("Cache-Control", "public, max-age=31536000, immutable");
  server.send_P(200, "image/svg+xml", kSoundfieldLogoSvg);
}

void handleStatusJson() {
  sendJson(stateJson());
}

void handleWifiDiagJson() {
  String body;
  body.reserve(1600 + static_cast<uint16_t>(wifiDiagCount) * 240u);
  appendWifiDiagJson(body, true);
  sendJson(body);
}

void handleDeviceJson() {
  handleStatusJson();
}

void handleLorenz() {
  recordWebRequest();
  sendCommand(bearamin_sidecar::COMMAND_ENTER_LORENZ, 0);
  server.send(204);
}

void handleTelemetryJson() {
  handleStatusJson();
}

void handleMatrixJson() {
  handleStatusJson();
}

void rememberTelemetrySnapshot(
    const bearamin_capture::ProcessedSensorSnapshotV1 &snapshot,
    uint64_t coreUs) {
  if (!telemetryLoggingEnabled) {
    return;
  }

  LogFrameEntry &entry = logFrames[logWriteIndex];
  entry.seq = ++logSequence;
  entry.sidecarMs = millis();
  entry.coreMs = static_cast<uint32_t>(coreUs / 1000ULL);
  entry.stateFlags =
      (latestState.stateFlags & bearamin_sidecar::STATE_DPT_READY) |
      ((snapshot.sensorFlags & bearamin_capture::SENSOR_MATRIX_READY)
           ? bearamin_sidecar::STATE_MATRIX_READY
           : 0) |
      ((snapshot.sensorFlags & bearamin_capture::SENSOR_MATRIX_PRESENT)
           ? bearamin_sidecar::STATE_MATRIX_PRESENT
           : 0) |
      ((snapshot.sensorFlags & bearamin_capture::SENSOR_IMU_READY)
           ? bearamin_sidecar::STATE_IMU_READY
           : 0) |
      ((snapshot.sensorFlags & bearamin_capture::SENSOR_IMPACT_GATE)
           ? bearamin_sidecar::STATE_IMPACT_GATE
           : 0);
  entry.dptMode = latestState.dptModeIndex;
  entry.dptProtocolVersion = latestState.dptProtocolVersion;
  entry.dptFlags = latestState.dptFlags;
  entry.matrixXMilli = snapshot.matrixXMilli;
  entry.matrixYMilli = snapshot.matrixYMilli;
  entry.matrixZMm = snapshot.matrixZMm;
  entry.matrixXNorm = snapshot.matrixXNorm;
  entry.matrixYNorm = snapshot.matrixYNorm;
  entry.matrixZNorm = snapshot.matrixZNorm;
  entry.pitchNorm = 2048;
  entry.rollNorm = 2048;
  entry.orientationConfidence = 0;
  entry.matrixValidCount = snapshot.matrixValidCount;
  entry.matrixBlobCount = snapshot.matrixBlobCount;
  entry.impactVelocityNorm = snapshot.impactVelocityNorm;
  entry.impactAgeMs = snapshot.impactAgeMs;
  entry.accelXMilliG = snapshot.accelXMilliG;
  entry.accelYMilliG = snapshot.accelYMilliG;
  entry.accelZMilliG = snapshot.accelZMilliG;
  memcpy(entry.sampleCellsMm, latestState.cellsMm, sizeof(entry.sampleCellsMm));

  entry.cellsPresent = lastCellsMs != 0 && millis() - lastCellsMs <= kLogCellsFreshMs;
  if (entry.cellsPresent) {
    memcpy(entry.cellsMm, latestCells.cellsMm, sizeof(entry.cellsMm));
  } else {
    memset(entry.cellsMm, 0, sizeof(entry.cellsMm));
  }

  logWriteIndex = (logWriteIndex + 1) % kLogRingSize;
  if (logCount < kLogRingSize) {
    ++logCount;
  }
}

void rememberTelemetrySnapshot(
    const bearamin_capture::ProcessedSensorSnapshotV2 &snapshot,
    uint64_t coreUs) {
  rememberTelemetrySnapshot(
      static_cast<const bearamin_capture::ProcessedSensorSnapshotV1 &>(snapshot),
      coreUs);
  if (!telemetryLoggingEnabled || logCount == 0) return;
  const size_t index =
      (logWriteIndex + kLogRingSize - 1) % kLogRingSize;
  logFrames[index].pitchNorm = snapshot.pitchNorm;
  logFrames[index].rollNorm = snapshot.rollNorm;
  logFrames[index].orientationConfidence = snapshot.orientationConfidence;
  if ((snapshot.sensorFlags & bearamin_capture::SENSOR_SPANK_GATE) != 0) {
    logFrames[index].stateFlags |= bearamin_sidecar::STATE_SPANK_GATE;
  }
}

void appendLogFrameJson(String &body, const LogFrameEntry &entry, bool first) {
  body += first ? "" : ",";
  body += "{\"seq\":";
  body += entry.seq;
  body += ",\"ms\":";
  body += entry.sidecarMs;
  body += ",\"core_ms\":";
  body += entry.coreMs;
  body += ",\"cv_mode\":\"split_core\"";
  body += ",\"cv_source\":";
  body += jsonString(logCvSourceName(entry));
  body += ",\"cv_bridge_z\":";
  body += logCvBridgeZ(entry);
  body += ",\"cv_bridge_x\":";
  body += entry.matrixXNorm;
  body += ",\"cv_bridge_y\":";
  body += entry.matrixYNorm;
  body += ",\"matrix8x8_ready\":";
  body += jsonBool(frameFlag(entry, bearamin_sidecar::STATE_MATRIX_READY));
  body += ",\"matrix8x8_present\":";
  body += jsonBool(frameFlag(entry, bearamin_sidecar::STATE_MATRIX_PRESENT));
  body += ",\"matrix8x8_nearest_mm\":";
  body += entry.matrixZMm;
  body += ",\"matrix8x8_x_milli\":";
  body += entry.matrixXMilli;
  body += ",\"matrix8x8_y_milli\":";
  body += entry.matrixYMilli;
  body += ",\"matrix8x8_x_norm\":";
  body += entry.matrixXNorm;
  body += ",\"matrix8x8_y_norm\":";
  body += entry.matrixYNorm;
  body += ",\"matrix8x8_z_norm\":";
  body += entry.matrixZNorm;
  body += ",\"matrix8x8_valid_count\":";
  body += static_cast<unsigned>(entry.matrixValidCount);
  body += ",\"matrix8x8_blob_count\":";
  body += static_cast<unsigned>(entry.matrixBlobCount);
  body += ",\"pitch_norm\":";
  body += entry.pitchNorm;
  body += ",\"roll_norm\":";
  body += entry.rollNorm;
  body += ",\"orientation_confidence\":";
  body += static_cast<unsigned>(entry.orientationConfidence);
  body += ",\"spank_gate\":";
  body += jsonBool(frameFlag(entry, bearamin_sidecar::STATE_SPANK_GATE));
  body += ",\"dpt_ready\":";
  body += jsonBool(frameFlag(entry, bearamin_sidecar::STATE_DPT_READY));
  body += ",\"dpt_mode\":";
  body += static_cast<unsigned>(entry.dptMode);
  body += ",\"dpt_protocol_version\":";
  body += static_cast<unsigned>(entry.dptProtocolVersion);
  body += ",\"dpt_flags\":";
  body += entry.dptFlags;
  body += ",\"impact_gate\":";
  body += jsonBool(frameFlag(entry, bearamin_sidecar::STATE_IMPACT_GATE));
  body += ",\"impact_velocity\":";
  body += entry.impactVelocityNorm;
  body += ",\"impact_age_ms\":";
  body += entry.impactAgeMs;
  body += ",\"accel_mg\":[";
  body += entry.accelXMilliG;
  body += ',';
  body += entry.accelYMilliG;
  body += ',';
  body += entry.accelZMilliG;
  body += "],\"sample_cells_mm\":[";
  for (uint8_t i = 0; i < 8; ++i) {
    if (i > 0) body += ',';
    body += entry.sampleCellsMm[i];
  }
  body += "],\"matrix8x8_cells_present\":";
  body += jsonBool(entry.cellsPresent);
  body += ",\"cells_mm\":[";
  for (uint8_t i = 0; i < 64; ++i) {
    if (i > 0) body += ',';
    body += entry.cellsMm[i];
  }
  body += "]}";
}

void handleFramesJson() {
  uint32_t since = 0;
  if (server.hasArg("since")) {
    since = static_cast<uint32_t>(strtoul(server.arg("since").c_str(), nullptr, 10));
  }
  uint8_t limit = 24;
  if (server.hasArg("limit")) {
    const long requested = server.arg("limit").toInt();
    if (requested > 0) {
      limit = static_cast<uint8_t>(requested > kLogMaxFramesPerResponse ? kLogMaxFramesPerResponse : requested);
    }
  }

  String body;
  body.reserve(1200 + static_cast<uint16_t>(limit) * 1050u);
  body += "{\"latest_seq\":";
  body += logSequence;
  body += ",\"retained\":";
  body += logCount;
  body += ",\"logging\":";
  body += jsonBool(telemetryLoggingEnabled);
  body += ",\"frames\":[";
  const uint16_t oldestIndex = (logWriteIndex + kLogRingSize - logCount) % kLogRingSize;
  bool first = true;
  uint8_t emitted = 0;
  for (uint16_t i = 0; i < logCount && emitted < limit; ++i) {
    const LogFrameEntry &entry = logFrames[(oldestIndex + i) % kLogRingSize];
    if (entry.seq <= since) {
      continue;
    }
    appendLogFrameJson(body, entry, first);
    first = false;
    ++emitted;
  }
  body += "]}";
  sendJson(body);
}

void handleTelemetryReset() {
  if (captureSessionActive || captureSessionStopping) {
    server.send(
        409,
        "application/json; charset=utf-8",
        "{\"ok\":false,\"error\":\"capture_already_active\"}");
    return;
  }
  logWriteIndex = 0;
  logCount = 0;
  logSequence = 0;
  String id = "legacy-";
  id += millis();
  resetCaptureSession(id);
  telemetryLoggingEnabled = true;
  sendCommand(
      bearamin_sidecar::COMMAND_CAPTURE_START,
      static_cast<uint16_t>(captureSessionToken & 0xFFFFu),
      static_cast<uint16_t>(captureSessionToken >> 16));
  sendJson(
      "{\"ok\":true,\"logging\":true,\"source\":\"processed_snapshot\"}");
}

void handleTelemetryStop() {
  if (captureSessionActive && !captureSessionStopping) {
    captureSessionStopping = true;
    sendCommand(bearamin_sidecar::COMMAND_CAPTURE_STOP, 0);
  }
  telemetryLoggingEnabled = false;
  sendJson("{\"ok\":true,\"logging\":false}");
}

void handlePerfJson() {
  const uint32_t now = millis();
  const uint8_t obsActiveSubscribers =
      obsSubscriberLeases.activeCount(now);
  const uint8_t obsV3Subscribers = obsSubscriberLeases.activeCountForVersion(
      now, bearamin_obs::kVersionV3);
  const uint8_t obsV4Subscribers = obsSubscriberLeases.activeCountForVersion(
      now, bearamin_obs::kVersionV4);
  const uint8_t obsStreamUnion =
      obsSubscriberLeases.streamUnion(now);
  String body = "{\"matrix_bytes\":";
  body += matrixBytes;
  body += ",\"matrix_frames\":";
  body += matrixFrames;
  body += ",\"matrix_acks\":";
  body += matrixAcks;
  body += ",\"parser_events\":";
  body += matrixParserEvents;
  body += ",\"heap\":";
  body += ESP.getFreeHeap();
  body += ",\"obs_overlay\":{\"udp_ready\":";
  body += jsonBool(obsUdpReady);
  body += ",\"port\":";
  body += bearamin_obs::kUdpPort;
  body += ",\"multicast_group\":\"";
  body += static_cast<unsigned int>(
      bearamin_obs::kMulticastAddress[0]);
  body += ".";
  body += static_cast<unsigned int>(
      bearamin_obs::kMulticastAddress[1]);
  body += ".";
  body += static_cast<unsigned int>(
      bearamin_obs::kMulticastAddress[2]);
  body += ".";
  body += static_cast<unsigned int>(
      bearamin_obs::kMulticastAddress[3]);
  body += "\"";
  body += ",\"multicast_port\":";
  body += bearamin_obs::kMulticastPort;
  body += ",\"subscribed\":";
  body += jsonBool(obsActiveSubscribers > 0);
  body += ",\"active_subscribers\":";
  body += static_cast<unsigned int>(obsActiveSubscribers);
  body += ",\"v3_subscribers\":";
  body += static_cast<unsigned int>(obsV3Subscribers);
  body += ",\"v4_subscribers\":";
  body += static_cast<unsigned int>(obsV4Subscribers);
  body += ",\"protocol_min\":";
  body += bearamin_obs::kVersionV3;
  body += ",\"protocol_max\":";
  body += bearamin_obs::kVersionV4;
  body += ",\"subscriber_capacity\":";
  body += static_cast<unsigned int>(
      bearamin_obs::kMaxSubscribers);
  body += ",\"streams\":";
  body += static_cast<unsigned int>(obsStreamUnion);
  body += ",\"client_nonce\":";
  body += obsLastClientNonce;
  body += ",\"subscriptions\":";
  body += obsSubscriptions;
  body += ",\"lease_expirations\":";
  body += obsLeaseExpirations;
  body += ",\"table_full_rejections\":";
  body += obsTableFullRejections;
  body += ",\"state_packets\":";
  body += obsStatePackets;
  body += ",\"cell_packets\":";
  body += obsCellPackets;
  body += ",\"multicast_state_packets\":";
  body += obsStatePackets;
  body += ",\"multicast_cell_packets\":";
  body += obsCellPackets;
  body += ",\"send_failures\":";
  body += obsSendFailures;
  body += ",\"multicast_send_failures\":";
  body += obsSendFailures;
  body += ",\"malformed_packets\":";
  body += obsMalformedPackets;
  body += ",\"max_send_us\":";
  body += obsMaxSendUs;
  body += ",\"max_multicast_send_us\":";
  body += obsMaxSendUs;
  body += "}";
  body += ",\"sidecar_health\":";
  appendSidecarHealthJson(body);
  body += "}";
  sendJson(body);
}

void sendCommand(uint8_t commandId,
                 uint16_t arg0,
                 uint16_t arg1,
                 uint16_t arg2,
                 uint16_t arg3) {
  bearamin_sidecar::CommandFrame command = {};
  command.sequence = static_cast<uint16_t>(commandSequence++);
  command.command = commandId;
  command.arg0 = arg0;
  command.arg1 = arg1;
  command.arg2 = arg2;
  command.arg3 = arg3;
  uint8_t frame[bearamin_sidecar::kCommandFrameSize] = {};
  const size_t length = bearamin_sidecar::encodeCommandFrame(
      command, bearamin_sidecar::FRAME_COMMAND, frame, sizeof(frame));
  if (length > 0) {
    MatrixSerial.write(frame, length);
  }
}

bool browserEngineeringRequestFresh(uint32_t now) {
  return lastEngineeringRequestMs != 0 &&
         now - lastEngineeringRequestMs < kEngineeringStreamTimeoutMs;
}

void setEngineeringStreamTransport(bool enabled) {
  if (engineeringStreamEnabled == enabled) {
    return;
  }
  engineeringStreamEnabled = enabled;
  sendCommand(
      bearamin_sidecar::COMMAND_SET_ENGINEERING_STREAM, enabled ? 1 : 0);
}

void reconcileEngineeringStream(uint32_t now) {
  const bool overlayCells =
      (obsSubscriberLeases.streamUnion(now) &
       bearamin_obs::STREAM_CELLS) != 0;
  const bool preserveCaptureStream =
      telemetryLoggingEnabled && engineeringStreamEnabled;
  setEngineeringStreamTransport(
      overlayCells || browserEngineeringRequestFresh(now) ||
      preserveCaptureStream);
}

void expireObsSubscriptions(uint32_t now) {
  const uint8_t expired = obsSubscriberLeases.expire(now);
  if (expired == 0) {
    return;
  }
  obsLeaseExpirations += expired;
  reconcileEngineeringStream(now);
}

void pollObsSubscriptions(uint32_t now) {
  if (!obsUdpReady) {
    return;
  }
  for (uint8_t handled = 0; handled < 4; ++handled) {
    const int packetSize = obsTelemetryUdp.parsePacket();
    if (packetSize <= 0) {
      break;
    }
    const IPAddress remoteIp = obsTelemetryUdp.remoteIP();
    uint8_t packet[bearamin_obs::kSubscribePacketSize] = {};
    const int readLength = obsTelemetryUdp.read(packet, sizeof(packet));
    while (obsTelemetryUdp.available()) {
      obsTelemetryUdp.read();
    }
    bearamin_obs::Subscribe subscription = {};
    if (packetSize != static_cast<int>(sizeof(packet)) ||
        readLength != static_cast<int>(sizeof(packet)) ||
        !bearamin_obs::decodeSubscribe(
            packet, sizeof(packet), subscription)) {
      ++obsMalformedPackets;
      continue;
    }
    const bearamin_obs::LeaseUpdateResult result =
        obsSubscriberLeases.update(
            static_cast<uint32_t>(remoteIp),
            subscription.clientNonce,
            subscription.streams,
            subscription.version,
            now);
    if (result == bearamin_obs::LEASE_TABLE_FULL) {
      ++obsTableFullRejections;
      continue;
    }
    obsLastClientNonce = subscription.clientNonce;
    ++obsSubscriptions;
    reconcileEngineeringStream(now);
  }
}

bool sendObsPacket(const uint8_t *packet, size_t length) {
  const uint32_t startedUs = micros();
  bool ok = obsTelemetryUdp.beginPacket(
                obsMulticastIp,
                bearamin_obs::kMulticastPort) == 1;
  if (ok) {
    ok = obsTelemetryUdp.write(packet, length) == length;
  }
  if (ok) {
    ok = obsTelemetryUdp.endPacket() == 1;
  } else {
    obsTelemetryUdp.stop();
    obsUdpReady = obsTelemetryUdp.begin(bearamin_obs::kUdpPort) == 1;
  }
  const uint32_t elapsedUs = micros() - startedUs;
  if (elapsedUs > obsMaxSendUs) {
    obsMaxSendUs = elapsedUs;
  }
  if (!ok) {
    ++obsSendFailures;
  }
  return ok;
}

void sendPendingObsTelemetry(uint32_t now) {
  if (!obsUdpReady) {
    return;
  }
  const uint8_t v3Streams = obsSubscriberLeases.streamUnionForVersion(
      now, bearamin_obs::kVersionV3);
  const uint8_t v4Streams = obsSubscriberLeases.streamUnionForVersion(
      now, bearamin_obs::kVersionV4);
  if (v3Streams == 0 && v4Streams == 0) {
    return;
  }
  if (obsStatePending) {
    obsStatePending = false;
    bearamin_obs::State state = {};
    state.sourceUptimeMs = latestState.uptimeMs;
    state.stateFlags = latestState.stateFlags;
    state.sensorAgeMs = latestState.sensorAgeMs;
    state.matrixXNorm = latestState.matrixXNorm;
    state.matrixYNorm = latestState.matrixYNorm;
    state.matrixZNorm = latestState.matrixZNorm;
    state.matrixXMilli = latestState.matrixXMilli;
    state.matrixYMilli = latestState.matrixYMilli;
    state.matrixZMm = latestState.matrixZMm;
    state.impactVelocityNorm = latestState.impactVelocityNorm;
    state.impactAgeMs = latestState.impactAgeMs;
    state.pitchNorm = latestState.pitchNorm;
    state.rollNorm = latestState.rollNorm;
    state.matrixValidCount = latestState.matrixValidCount;
    state.matrixBlobCount = latestState.matrixBlobCount;
    for (uint8_t axis = 0; axis < 5; ++axis) {
      state.trajectoryStartNorm[axis] = latestState.trajectoryStartNorm[axis];
    }
    state.slewTauMs = latestState.trajectoryTauMs;
    if ((v3Streams & bearamin_obs::STREAM_STATE) != 0) {
      bearamin_obs::State v3State = state;
      v3State.matrixXNorm = state.trajectoryStartNorm[0];
      v3State.matrixYNorm = state.trajectoryStartNorm[1];
      v3State.matrixZNorm = state.trajectoryStartNorm[2];
      v3State.pitchNorm = state.trajectoryStartNorm[3];
      v3State.rollNorm = state.trajectoryStartNorm[4];
      v3State.matrixXMilli = static_cast<uint16_t>(
          (static_cast<uint32_t>(v3State.matrixXNorm) * 1000u + 2047u) /
          4095u);
      v3State.matrixYMilli = static_cast<uint16_t>(
          (static_cast<uint32_t>(v3State.matrixYNorm) * 1000u + 2047u) /
          4095u);
      uint8_t packet[bearamin_obs::kStatePacketSize] = {};
      const size_t length = bearamin_obs::encodeState(
          v3State, obsStateSequence++, packet, sizeof(packet));
      if (length > 0 && sendObsPacket(packet, length)) {
        ++obsStatePackets;
      }
    }
    if ((v4Streams & bearamin_obs::STREAM_STATE) != 0 &&
        latestState.trajectoryValid) {
      uint8_t packet[bearamin_obs::kStatePacketSizeV4] = {};
      const size_t length = bearamin_obs::encodeStateV4(
          state, obsStateSequence++, packet, sizeof(packet));
      if (length > 0 && sendObsPacket(packet, length)) {
        ++obsStatePackets;
      }
    }
  }
  if (obsCellsPending) {
    obsCellsPending = false;
    bearamin_obs::Cells cells = {};
    cells.sourceUptimeMs = latestState.uptimeMs;
    cells.sensorAgeMs = latestCells.sensorAgeMs;
    cells.matrixValidCount = latestCells.validCount;
    cells.matrixBlobCount = latestCells.blobCount;
    for (uint8_t i = 0; i < 64; ++i) {
      cells.cellsMm[i] = latestCells.cellsMm[i];
    }
    if ((v3Streams & bearamin_obs::STREAM_CELLS) != 0) {
      uint8_t packet[bearamin_obs::kCellsPacketSize] = {};
      const size_t length = bearamin_obs::encodeCells(
          cells, obsCellSequence++, packet, sizeof(packet));
      if (length > 0 && sendObsPacket(packet, length)) {
        ++obsCellPackets;
      }
    }
    if ((v4Streams & bearamin_obs::STREAM_CELLS) != 0 &&
        latestState.trajectoryValid) {
      uint8_t packet[bearamin_obs::kCellsPacketSize] = {};
      const size_t length = bearamin_obs::encodeCells(
          cells, obsCellSequence++, packet, sizeof(packet),
          bearamin_obs::kVersionV4);
      if (length > 0 && sendObsPacket(packet, length)) {
        ++obsCellPackets;
      }
    }
  }
}

bool validCaptureSessionId(const String &id) {
  if (id.length() == 0 || id.length() > 64) {
    return false;
  }
  for (size_t i = 0; i < id.length(); ++i) {
    const char c = id[i];
    if (!isalnum(static_cast<unsigned char>(c)) &&
        c != '-' && c != '_') {
      return false;
    }
  }
  return true;
}

String captureStatusJson() {
  String body;
  body.reserve(640);
  body += "{\"session_id\":";
  body += captureSessionId.length() == 0
              ? "null"
              : jsonString(captureSessionId);
  body += ",\"active\":";
  body += jsonBool(captureSessionActive);
  body += ",\"stopping\":";
  body += jsonBool(captureSessionStopping);
  body += ",\"finalized\":";
  body += jsonBool(captureSessionFinalized);
  body += ",\"psram\":";
  body += jsonBool(captureUsingPsram);
  body += ",\"capacity_bytes\":";
  body += static_cast<unsigned long>(captureStore.Capacity());
  body += ",\"used_bytes\":";
  body += static_cast<unsigned long>(captureStore.Used());
  body += ",\"high_water_bytes\":";
  body += static_cast<unsigned long>(captureStore.HighWater());
  body += ",\"first_seq\":";
  body += captureFirstSequence;
  body += ",\"latest_seq\":";
  body += captureLatestSequence;
  body += ",\"final_seq\":";
  if (captureSessionFinalized || captureFinalSequence != 0) {
    body += captureFinalSequence;
  } else {
    body += "null";
  }
  body += ",\"ack_seq\":";
  body += captureAcknowledgedSequence;
  body += ",\"record_count\":";
  body += captureRecordCount;
  body += ",\"drops\":";
  body += captureDropCount;
  body += ",\"pending_drops\":";
  body += capturePendingDropCount;
  body += ",\"sequence_errors\":";
  body += captureSequenceErrors;
  body += ",\"crc_errors\":";
  body += captureParser.CrcErrors();
  body += ",\"framing_errors\":";
  body += captureParser.FramingErrors();
  body += ",\"store_errors\":";
  body += captureStore.Corruptions();
  body += "}";
  return body;
}

void handleCaptureStart() {
  const String id = server.arg("id");
  if (!validCaptureSessionId(id)) {
    server.send(
        400,
        "application/json; charset=utf-8",
        "{\"ok\":false,\"error\":\"invalid_session_id\"}");
    return;
  }
  if (captureSessionActive || captureSessionStopping) {
    server.send(
        409,
        "application/json; charset=utf-8",
        "{\"ok\":false,\"error\":\"capture_already_active\"}");
    return;
  }
  resetCaptureSession(id);
  telemetryLoggingEnabled = true;
  sendCommand(
      bearamin_sidecar::COMMAND_CAPTURE_START,
      static_cast<uint16_t>(captureSessionToken & 0xFFFFu),
      static_cast<uint16_t>(captureSessionToken >> 16));
  String body = "{\"ok\":true,\"session_id\":";
  body += jsonString(captureSessionId);
  body += ",\"token\":";
  body += captureSessionToken;
  body += "}";
  sendJson(body);
}

void handleCaptureStop() {
  const String id = server.arg("id");
  if (id != captureSessionId ||
      (!captureSessionActive && !captureSessionStopping)) {
    server.send(
        409,
        "application/json; charset=utf-8",
        "{\"ok\":false,\"error\":\"capture_not_active\"}");
    return;
  }
  if (!captureSessionStopping) {
    captureSessionStopping = true;
    sendCommand(bearamin_sidecar::COMMAND_CAPTURE_STOP, 0);
  }
  const uint32_t startedMs = millis();
  while (!captureSessionFinalized &&
         millis() - startedMs < kCaptureStopWaitMs) {
    pollMatrixUart();
    delay(1);
  }
  telemetryLoggingEnabled = false;
  String body = "{\"ok\":";
  body += jsonBool(captureSessionFinalized);
  body += ",\"pending\":";
  body += jsonBool(!captureSessionFinalized);
  body += ",\"final_seq\":";
  if (captureSessionFinalized || captureFinalSequence != 0) {
    body += captureFinalSequence;
  } else {
    body += "null";
  }
  body += ",\"records\":";
  body += captureRecordCount;
  body += ",\"drops\":";
  body += captureDropCount;
  body += "}";
  server.send(
      captureSessionFinalized ? 200 : 202,
      "application/json; charset=utf-8",
      body);
}

void handleCaptureStatus() {
  sendJson(captureStatusJson());
}

void handleCaptureData() {
  const String id = server.arg("id");
  if (id != captureSessionId || id.length() == 0) {
    server.send(
        404,
        "application/json; charset=utf-8",
        "{\"ok\":false,\"error\":\"capture_not_found\"}");
    return;
  }
  uint32_t after = 0xFFFFFFFFu;
  if (server.hasArg("after")) {
    after = static_cast<uint32_t>(
        strtoul(server.arg("after").c_str(), nullptr, 10));
  }
  if (server.hasArg("ack")) {
    const uint32_t ack = static_cast<uint32_t>(
        strtoul(server.arg("ack").c_str(), nullptr, 10));
    if (ack != 0xFFFFFFFFu &&
        static_cast<int32_t>(ack - captureAcknowledgedSequence) >= 0) {
      captureStore.AckThrough(ack);
      captureAcknowledgedSequence = ack;
      flushPendingCaptureEnd();
    }
  }
  size_t limit = kCaptureHttpMaxBytes;
  if (server.hasArg("limit")) {
    const long requested = server.arg("limit").toInt();
    if (requested > 0 &&
        static_cast<size_t>(requested) < limit) {
      limit = static_cast<size_t>(requested);
    }
  }
  uint32_t lastSequence = after;
  const size_t bytes =
      captureStore.CopyAfter(
          after, captureHttpBuffer, limit, lastSequence);
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Bearamin-Last-Sequence", String(lastSequence));
  server.setContentLength(bytes);
  server.send(200, "application/octet-stream", "");
  if (bytes > 0) {
    server.client().write(captureHttpBuffer, bytes);
  }
}

void sendCurrentDptControls() {
  const uint8_t mode = effectiveDptMode();
  const uint16_t flags = dptControlFlagsForMode(mode);
  sendCommand(
      bearamin_sidecar::COMMAND_SET_MODE_ANNOUNCEMENTS,
      modeAnnouncementsEnabled ? 1 : 0);
  uint16_t reserved0 = 0;
  uint8_t reserved1 = 0;
  uint8_t reserved2 = 0;
  const int8_t config = gestureConfigForDptMode(mode);
  if (config >= 0) {
    const uint8_t mappedConfig = static_cast<uint8_t>(config);
    const uint32_t packed = packedGestureMappingsForMode(mappedConfig);
    reserved0 = static_cast<uint16_t>(packed & 0xFFFFu);
    reserved1 = static_cast<uint8_t>((packed >> 16) & 0xFFu);
    reserved2 = bearamin_bridge::CONTROL_RESONATOR_MAPPING_MAGIC;
    const uint16_t reserved1AndSourceFlags2 =
        static_cast<uint16_t>(reserved1) |
        (static_cast<uint16_t>((packed >> 24) & 0xFFu) << 8);
    sendCommand(
        bearamin_sidecar::COMMAND_SET_CONTROL_FLAGS, flags, reserved0, reserved1AndSourceFlags2, reserved2);
    uint16_t depthWords[3] = {};
    for (uint8_t i = 0; i < 6; ++i) {
      depthWords[i / 2] |= static_cast<uint16_t>(
          static_cast<uint8_t>(gestureDepthForModeLane(mappedConfig, i))) << ((i % 2) * 8);
    }
    sendCommand(bearamin_sidecar::COMMAND_SET_LANE_DEPTHS, mode,
                depthWords[0], depthWords[1], depthWords[2]);
    return;
  }
  sendCommand(bearamin_sidecar::COMMAND_SET_CONTROL_FLAGS, flags, reserved0, reserved1, reserved2);
}

void sendCurrentResonatorWebControls() {
  for (uint8_t page = 0; page < 3; ++page) {
    uint16_t words[3] = {};
    const uint8_t start = static_cast<uint8_t>(page * 6);
    for (uint8_t i = 0; i < 6; ++i) {
      const uint8_t index = static_cast<uint8_t>(start + i);
      const uint8_t value = index < bearamin_bridge::kResonatorWebControlSize
                                ? resonatorWebControls[index]
                                : 0;
      words[i / 2] |= static_cast<uint16_t>(value) << ((i % 2) * 8);
    }
    sendCommand(
        bearamin_sidecar::COMMAND_SET_RESONATOR_WEB_PAGE,
        static_cast<uint16_t>(resonatorControlsRevision) |
            (static_cast<uint16_t>(page) << 8),
        words[0],
        words[1],
        words[2]);
  }
  lastResonatorControlSendMs = millis();
}

void requestRuntimeSettings() {
  lastRuntimeSettingsRequestMs = millis();
  sendCommand(bearamin_sidecar::COMMAND_REQUEST_RUNTIME_SETTINGS, 0);
}

void adoptRuntimeSettings(
    const bearamin_sidecar::RuntimeSettingsFrame &settings) {
  latestRuntimeSettings = settings;
  runtimeSettingsReceived = true;
  if (!settings.controlsInitializedBySidecar) {
    sendCurrentDptControls();
    sendCurrentResonatorWebControls();
    bootControlSyncComplete = true;
    bootControlSyncUsedRuntime = false;
    return;
  }
  modeAnnouncementsEnabled = settings.modeAnnouncementsEnabled;
  cvSlewStep = settings.cvSlewStep > 32 ? 32 : settings.cvSlewStep;
  resonatorControlsRevision = settings.resonatorControlsRevision;
  memcpy(resonatorWebControls,
         settings.resonatorWebControls,
         sizeof(resonatorWebControls));

  const int8_t config = gestureConfigForDptMode(settings.dptModeIndex);
  if (config >= 0) {
    const uint8_t mappedConfig = static_cast<uint8_t>(config);
    for (uint8_t lane = 0; lane < bearamin_bridge::kLaneDepthCount; ++lane) {
      modeGestureDepths[mappedConfig][lane] = settings.laneDepths[lane];
    }
    if (settings.dptReserved2 ==
        bearamin_bridge::CONTROL_RESONATOR_MAPPING_MAGIC) {
      const uint32_t packed =
          static_cast<uint32_t>(settings.dptReserved0) |
          (static_cast<uint32_t>(settings.dptReserved1) << 16) |
          (static_cast<uint32_t>(settings.dptSourceFlags2) << 24);
      for (uint8_t lane = 0; lane < 6; ++lane) {
        modeGestureMaps[mappedConfig][lane] =
            static_cast<uint8_t>((packed >> (lane * 5)) & 0x1Fu);
      }
    }
  }
  if (settings.dptModeIndex == 3) {
    fmDroneRangerMode =
        (settings.dptControlFlags &
         bearamin_bridge::CONTROL_FM_DRONE_RANGER) != 0;
    fmDroneMapX = clampFmParam(static_cast<uint8_t>(
        (settings.dptControlFlags >> bearamin_bridge::CONTROL_FM_X_SHIFT) &
        0x7u));
    fmDroneMapY = clampFmParam(static_cast<uint8_t>(
        (settings.dptControlFlags >> bearamin_bridge::CONTROL_FM_Y_SHIFT) &
        0x7u));
    fmDroneMapZ = clampFmParam(static_cast<uint8_t>(
        (settings.dptControlFlags >> bearamin_bridge::CONTROL_FM_Z_SHIFT) &
        0x7u));
  }

  bootControlSyncComplete = true;
  bootControlSyncUsedRuntime = true;
  presetLastError = "";
}

void beginBootControlSync() {
  if (bootControlSyncComplete || lastRuntimeSettingsRequestMs != 0) {
    return;
  }
  requestRuntimeSettings();
}

void pollBootControlSync(uint32_t now) {
  if (bootControlSyncComplete || lastStateMs == 0 ||
      matrixStale() || lastRuntimeSettingsRequestMs == 0 ||
      now - lastRuntimeSettingsRequestMs < kRuntimeSettingsRetryMs) {
    return;
  }
  requestRuntimeSettings();
}

bool findJsonObjectForKey(const String &json, const String &key, String &objectText);
bool extractJsonValue(const String &json, const String &key, String &value);

// Retired v9 relay protocol.  Presets must not transit Sidecar -> MatrixCore
// -> DPT; keep this block only temporarily as source-history while RevD is
// brought up, and do not compile it into the target.
#if 0
void queuePresetLine(const String &line) {
  const uint16_t next = (presetLineHead + 1) % kPresetLineQueueSize;
  if (next == presetLineTail) {
    presetLineTail = (presetLineTail + 1) % kPresetLineQueueSize;
  }
  presetLineQueue[presetLineHead] = line;
  presetLineHead = next;
}

bool popPresetLine(String &line) {
  if (presetLineTail == presetLineHead) {
    return false;
  }
  line = presetLineQueue[presetLineTail];
  presetLineQueue[presetLineTail] = "";
  presetLineTail = (presetLineTail + 1) % kPresetLineQueueSize;
  return true;
}

void clearPresetLines() {
  presetLineHead = 0;
  presetLineTail = 0;
  for (uint16_t i = 0; i < kPresetLineQueueSize; ++i) {
    presetLineQueue[i] = "";
  }
}

void queuePresetLineWithEmbeddedResponses(const String &line) {
  size_t cursor = 0;
  bearamin_sidecar::PresetResponseSpan span = {};
  bool queuedResponse = false;
  while (bearamin_sidecar::nextPresetResponseSpan(
      line.c_str(), line.length(), cursor, span)) {
    queuePresetLine(line.substring(span.start, span.start + span.length));
    queuedResponse = true;
  }
  if (!queuedResponse) {
    queuePresetLine(line);
  }
}

void servicePresetWait() {
  pollMatrixUart();
  if (otaReady) {
    ArduinoOTA.handle();
  }
  delay(1);
}

void sendPresetLineToMatrix(const String &line) {
  size_t offset = 0;
  while (offset < line.length()) {
    bearamin_sidecar::PresetFrame preset = {};
    preset.sequence = presetSequence++;
    preset.op = bearamin_sidecar::PRESET_OP_LINE;
    while (preset.length < bearamin_sidecar::kPresetLinePayloadSize &&
           offset < line.length()) {
      preset.data[preset.length++] = static_cast<uint8_t>(line[offset++]);
    }
    uint8_t frame[bearamin_sidecar::kPresetFrameSize] = {};
    const size_t length = bearamin_sidecar::encodePresetFrame(preset, frame, sizeof(frame));
    if (length > 0) {
      MatrixSerial.write(frame, length);
    }
  }
}

void pacePresetWriteLine() {
  const uint32_t start = millis();
  while (millis() - start < kPresetWriteLinePaceMs) {
    servicePresetWait();
  }
}

bool splitCsvToken(const String &line, int tokenIndex, String &value) {
  int start = 0;
  int index = 0;
  while (start <= static_cast<int>(line.length())) {
    int end = line.indexOf(',', start);
    if (end < 0) {
      end = line.length();
    }
    if (index == tokenIndex) {
      value = line.substring(start, end);
      return true;
    }
    ++index;
    start = end + 1;
  }
  return false;
}

bool lineMatchesSeq(const String &line, const String &prefix, uint16_t seq) {
  String seqText;
  return line.startsWith(prefix + ",") &&
         splitCsvToken(line, 1, seqText) &&
         static_cast<uint16_t>(seqText.toInt()) == seq;
}

bool lineMatchesSeqKind(const String &line, const String &prefix, uint16_t seq, const String &kind) {
  String token;
  return lineMatchesSeq(line, prefix, seq) &&
         splitCsvToken(line, 2, token) &&
         token == kind;
}

bool lineMatchesSeqError(const String &line, uint16_t seq) {
  return lineMatchesSeq(line, "BPERR", seq);
}

bool isPresetResponseLine(const String &line, uint16_t seq) {
  if (lineMatchesSeqError(line, seq) || lineMatchesSeq(line, "BPD", seq)) {
    return true;
  }
  String kind;
  if (lineMatchesSeq(line, "BPLS", seq) && splitCsvToken(line, 2, kind)) {
    return kind == "ITEM" || kind == "END";
  }
  if (lineMatchesSeq(line, "BPRD", seq) && splitCsvToken(line, 2, kind)) {
    return kind == "BEGIN" || kind == "END";
  }
  if (lineMatchesSeq(line, "BPWR", seq) && splitCsvToken(line, 2, kind)) {
    return kind == "READY" || kind == "OK";
  }
  if (lineMatchesSeq(line, "BPAK", seq) && splitCsvToken(line, 2, kind)) {
    return kind.length() > 0;
  }
  return false;
}

bool waitPresetLine(uint16_t seq, String &line, uint32_t timeoutMs = kPresetTimeoutMs) {
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    String candidate;
    if (popPresetLine(candidate)) {
      if (isPresetResponseLine(candidate, seq)) {
        line = candidate;
        return true;
      }
      continue;
    }
    servicePresetWait();
  }
  line = "";
  return false;
}

bool waitPresetLineWithRetry(uint16_t seq,
                             const String &requestLine,
                             String &line,
                             uint32_t timeoutMs = kPresetTimeoutMs,
                             uint32_t retryMs = kPresetRequestRetryMs) {
  const uint32_t start = millis();
  uint32_t lastRetryMs = start;
  while (millis() - start < timeoutMs) {
    String candidate;
    if (popPresetLine(candidate)) {
      if (isPresetResponseLine(candidate, seq)) {
        line = candidate;
        return true;
      }
      continue;
    }
    const uint32_t now = millis();
    if (now - lastRetryMs >= retryMs) {
      sendPresetLineToMatrix(requestLine);
      lastRetryMs = now;
    }
    servicePresetWait();
  }
  line = "";
  return false;
}

bool presetLineError(const String &line, uint16_t seq, String &error) {
  if (!lineMatchesSeq(line, "BPERR", seq)) {
    return false;
  }
  String code;
  splitCsvToken(line, 2, code);
  error = code.length() > 0 ? code : "preset_error";
  presetLastError = error;
  presetSdReady = !error.startsWith("sd");
  return true;
}

bool waitPresetChunkAck(uint16_t seq, size_t offset, const String &requestLine, String &error) {
  const uint32_t start = millis();
  uint32_t lastRetryMs = start;
  while (millis() - start < kPresetTimeoutMs) {
    String candidate;
    if (popPresetLine(candidate)) {
      if (presetLineError(candidate, seq, error)) {
        return false;
      }
      if (lineMatchesSeq(candidate, "BPAK", seq)) {
        String offsetText;
        splitCsvToken(candidate, 2, offsetText);
        if (static_cast<size_t>(offsetText.toInt()) == offset) {
          return true;
        }
      }
      continue;
    }
    const uint32_t now = millis();
    if (now - lastRetryMs >= kPresetRequestRetryMs) {
      sendPresetLineToMatrix(requestLine);
      lastRetryMs = now;
    }
    servicePresetWait();
  }
  error = "chunk_timeout";
  presetLastError = error;
  return false;
}

bool requestPresetList(String &presetsJson,
                       String &error,
                       String *presetNames = nullptr,
                       size_t *presetNameCount = nullptr,
                       size_t presetNameCapacity = 0) {
  const uint16_t seq = presetSequence++;
  clearPresetLines();
  String requestLine = "BPLS," + String(seq) + "\n";
  sendPresetLineToMatrix(requestLine);
  String listedNames[kPresetMaxListedPresets];
  bool listedNameReceived[kPresetMaxListedPresets] = {};
  size_t namesFound = 0;
  uint8_t restartAttempts = 0;
  if (presetNameCount != nullptr) {
    *presetNameCount = 0;
  }
  while (true) {
    String line;
    if (!waitPresetLineWithRetry(
            seq, requestLine, line, kPresetTimeoutMs, kPresetListRetryMs)) {
      error = "timeout";
      presetLastError = error;
      presetSdReady = false;
      return false;
    }
    if (presetLineError(line, seq, error)) {
      return false;
    }
    String kind;
    splitCsvToken(line, 2, kind);
    if (kind == "END") {
      String countText;
      const size_t expectedCount =
          splitCsvToken(line, 3, countText)
              ? static_cast<size_t>(countText.toInt())
              : namesFound;
      if (expectedCount > kPresetMaxListedPresets) {
        error = "list_large";
        presetLastError = error;
        return false;
      }
      bool complete = expectedCount == namesFound;
      for (size_t i = 0; complete && i < expectedCount; ++i) {
        complete = listedNameReceived[i];
      }
      if (!complete) {
        if (++restartAttempts >= kPresetDefaultMaxAttempts) {
          error = "list_incomplete";
          presetLastError = error;
          return false;
        }
        requestLine = "BPLS," + String(seq) + ",RETRY\n";
        sendPresetLineToMatrix(requestLine);
        continue;
      }
      String body = "{\"ok\":true,\"sd_ready\":true,\"presets\":[";
      for (size_t i = 0; i < expectedCount; ++i) {
        if (i > 0) {
          body += ',';
        }
        body += "{\"name\":";
        body += jsonString(listedNames[i]);
        body += ",\"filename\":";
        body += jsonString(listedNames[i] + ".json");
        body += "}";
        if (presetNames != nullptr && i < presetNameCapacity) {
          presetNames[i] = listedNames[i];
        }
      }
      if (presetNameCount != nullptr) {
        *presetNameCount =
            expectedCount < presetNameCapacity ? expectedCount : presetNameCapacity;
      }
      body += "],\"error\":\"\"}";
      presetsJson = body;
      presetLastError = "";
      presetSdReady = true;
      return true;
    }
    if (kind == "ITEM") {
      String indexText;
      String name;
      String crcText;
      if (!splitCsvToken(line, 3, indexText) ||
          !splitCsvToken(line, 4, name) ||
          !splitCsvToken(line, 5, crcText) ||
          name.length() == 0) {
        continue;
      }
      const size_t index = static_cast<size_t>(indexText.toInt());
      const uint16_t receivedCrc =
          static_cast<uint16_t>(strtoul(crcText.c_str(), nullptr, 16));
      if (index >= kPresetMaxListedPresets ||
          receivedCrc != crc16String(name)) {
        continue;
      }
      listedNames[index] = name;
      if (!listedNameReceived[index]) {
        listedNameReceived[index] = true;
        ++namesFound;
      }
    }
  }
}

bool readPresetFromSdAttempt(const String &name, String &json, String &error) {
  const uint16_t seq = presetSequence++;
  clearPresetLines();
  const String requestLine = "BPLS," + String(seq) + ",READ," + name + "\n";
  sendPresetLineToMatrix(requestLine);
  String line;
  if (!waitPresetLineWithRetry(
          seq, requestLine, line, kPresetFirstResponseTimeoutMs)) {
    error = "timeout";
    presetLastError = error;
    presetSdReady = false;
    return false;
  }
  if (presetLineError(line, seq, error)) {
    return false;
  }
  if (!lineMatchesSeqKind(line, "BPRD", seq, "BEGIN")) {
    error = "chunk";
    presetLastError = error;
    return false;
  }

  String sizeText;
  String crcText;
  splitCsvToken(line, 3, sizeText);
  splitCsvToken(line, 4, crcText);
  const size_t expectedSize = static_cast<size_t>(sizeText.toInt());
  uint16_t expectedCrc = static_cast<uint16_t>(
      strtoul(crcText.c_str(), nullptr, 16));
  bool expectedCrcReady = crcText.length() > 0 && crcText != "0000";
  if (expectedSize > kPresetMaxJsonBytes) {
    error = "large";
    presetLastError = error;
    return false;
  }

  json = "";
  json.reserve(expectedSize + 1);
  size_t lastChunkOffset = static_cast<size_t>(-1);
  String lastChunkHex;
  String chunkRequestLine = "BPRQ," + String(seq) + ",0\n";
  sendPresetLineToMatrix(chunkRequestLine);
  while (true) {
    if (!waitPresetLineWithRetry(
        seq, chunkRequestLine, line, kPresetReadChunkTimeoutMs)) {
      error = "timeout";
      presetLastError = error;
      presetSdReady = false;
      return false;
    }
    if (presetLineError(line, seq, error)) {
      return false;
    }
    if (lineMatchesSeq(line, "BPRD", seq)) {
      String kind;
      splitCsvToken(line, 2, kind);
      if (kind != "END") {
        continue;
      }
      String endCrcText;
      splitCsvToken(line, 3, endCrcText);
      if (endCrcText.length() > 0) {
        expectedCrc = static_cast<uint16_t>(
            strtoul(endCrcText.c_str(), nullptr, 16));
        expectedCrcReady = true;
      }
      if (json.length() != expectedSize) {
        error = "chunk";
        presetLastError = error;
        return false;
      }
      if (!expectedCrcReady || crc16String(json) != expectedCrc) {
        error = "crc";
        presetLastError = error;
        return false;
      }
      presetLastError = "";
      presetSdReady = true;
      return true;
    }
    if (!lineMatchesSeq(line, "BPD", seq)) {
      continue;
    }

    String offsetText;
    String hex;
    String chunkCrcText;
    splitCsvToken(line, 2, offsetText);
    splitCsvToken(line, 3, hex);
    splitCsvToken(line, 4, chunkCrcText);
    const size_t receivedOffset = static_cast<size_t>(offsetText.toInt());
    const size_t expectedOffset = json.length();
    if (receivedOffset == lastChunkOffset && hex == lastChunkHex) {
      sendPresetLineToMatrix(chunkRequestLine);
      continue;
    }
    if (receivedOffset < expectedOffset) {
      sendPresetLineToMatrix(chunkRequestLine);
      continue;
    }
    if (receivedOffset > expectedOffset) {
      error = "chunk";
      presetLastError = error;
      return false;
    }
    String decoded;
    decoded.reserve(kPresetHexChunkBytes);
    const uint16_t receivedChunkCrc = static_cast<uint16_t>(
        strtoul(chunkCrcText.c_str(), nullptr, 16));
    if (chunkCrcText.length() == 0 ||
        !appendHexDecoded(hex, decoded) ||
        crc16String(decoded) != receivedChunkCrc) {
      sendPresetLineToMatrix(chunkRequestLine);
      continue;
    }
    json += decoded;
    if (json.length() > expectedSize) {
      json.remove(expectedOffset);
      error = "chunk";
      presetLastError = error;
      return false;
    }
    lastChunkOffset = receivedOffset;
    lastChunkHex = hex;
    chunkRequestLine = "BPRQ," + String(seq) + "," +
                       String(json.length()) + "\n";
    sendPresetLineToMatrix(chunkRequestLine);
  }
}

bool readPresetFromSd(const String &name, String &json, String &error) {
  for (uint8_t attempt = 0; attempt < kPresetReadMaxAttempts; ++attempt) {
    if (readPresetFromSdAttempt(name, json, error)) {
      return true;
    }
    if (error != "chunk" && error != "crc" && error != "timeout") {
      return false;
    }
  }
  return false;
}

bool writePresetToSd(const String &name, const String &json, String &error) {
  if (json.length() > kPresetMaxJsonBytes) {
    error = "large";
    presetLastError = error;
    return false;
  }
  const uint16_t seq = presetSequence++;
  clearPresetLines();
  const String beginLine = "BPWR," + String(seq) + "," + name + "," +
                           String(json.length()) + "," + String(crc16String(json), HEX) + "\n";
  sendPresetLineToMatrix(beginLine);
  String line;
  const uint32_t beginStart = millis();
  bool beginReady = false;
  while (millis() - beginStart < kPresetTimeoutMs) {
    const uint32_t elapsed = millis() - beginStart;
    const uint32_t remaining = elapsed >= kPresetTimeoutMs ? 1 : kPresetTimeoutMs - elapsed;
    if (!waitPresetLineWithRetry(seq, beginLine, line, remaining)) {
      error = "timeout";
      presetLastError = error;
      return false;
    }
    if (presetLineError(line, seq, error)) {
      return false;
    }
    if (lineMatchesSeqKind(line, "BPWR", seq, "READY")) {
      beginReady = true;
      break;
    }
  }
  if (!beginReady) {
    error = "write_begin";
    presetLastError = error;
    return false;
  }
  for (size_t offset = 0; offset < json.length(); offset += kPresetHexChunkBytes) {
    const size_t chunk = json.length() - offset > kPresetHexChunkBytes
                             ? kPresetHexChunkBytes
                             : json.length() - offset;
    const uint16_t chunkCrc = bearamin_sidecar::crc16Ccitt(
        reinterpret_cast<const uint8_t *>(json.c_str() + offset), chunk);
    const String chunkLine = "BPD," + String(seq) + "," + String(offset) + "," +
                             hexEncode(reinterpret_cast<const uint8_t *>(json.c_str() + offset), chunk) +
                             "," + String(chunkCrc, HEX) + "\n";
    sendPresetLineToMatrix(chunkLine);
    pacePresetWriteLine();
    if (!waitPresetChunkAck(seq, offset, chunkLine, error)) {
      return false;
    }
  }
  pacePresetWriteLine();
  const String endLine = "BPWE," + String(seq) + "\n";
  sendPresetLineToMatrix(endLine);
  while (true) {
    if (!waitPresetLineWithRetry(seq, endLine, line)) {
      error = "timeout";
      presetLastError = error;
      return false;
    }
    if (presetLineError(line, seq, error)) {
      return false;
    }
    if (lineMatchesSeq(line, "BPD", seq)) {
      continue;
    }
    if (lineMatchesSeqKind(line, "BPWR", seq, "OK")) {
      presetLastError = "";
      presetSdReady = true;
      return true;
    }
  }
}
#endif

void pollMatrixUart();

bool defaultRequest(uint8_t op, const uint8_t *data, uint8_t length,
                    bearamin_sidecar::PresetFrame &reply, String &error) {
  bearamin_sidecar::PresetFrame request = {};
  request.sequence = ++defaultRequestSequence;
  request.op = op;
  request.length = length;
  if (length) memcpy(request.data, data, length);
  uint8_t frame[bearamin_sidecar::kPresetFrameSize] = {};
  const size_t bytes = bearamin_sidecar::encodePresetFrame(request, frame, sizeof(frame));
  defaultReplyReady = false;
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    MatrixSerial.write(frame, bytes);
    const uint32_t start = millis();
    while (millis() - start < 1000) {
      pollMatrixUart();
      if (defaultReplyReady && defaultReply.sequence == request.sequence &&
          defaultReply.op == op && defaultReply.length > 0) {
        reply = defaultReply;
        defaultReplyReady = false;
        if (reply.data[0] == bearamin_default::kOk) { error = ""; return true; }
        error = reply.data[0] == bearamin_default::kMissing ? "missing" :
                reply.data[0] == bearamin_default::kInvalid ? "invalid" : "storage";
        return false;
      }
      delay(1);
    }
  }
  error = "timeout";
  return false;
}

bool readDefaultPreset(String &json, String &error) {
  bearamin_sidecar::PresetFrame reply = {};
  if (!defaultRequest(bearamin_default::kReadBegin, nullptr, 0, reply, error)) {
    if (error == "missing" || error == "invalid") presetDefaultAvailable = false;
    return false;
  }
  if (reply.length != 5) { error = "invalid"; return false; }
  const size_t size = bearamin_sidecar::readLe16(reply.data, 1);
  const uint16_t crc = bearamin_sidecar::readLe16(reply.data, 3);
  if (size == 0 || size > bearamin_default::kMaxBytes) { error = "invalid"; return false; }
  json = "";
  if (!json.reserve(size)) { error = "memory"; return false; }
  while (json.length() < size) {
    uint8_t offset[2];
    bearamin_sidecar::writeLe16(offset, 0, static_cast<uint16_t>(json.length()));
    if (!defaultRequest(bearamin_default::kReadChunk, offset, sizeof(offset), reply, error)) return false;
    if (reply.length <= 3 || bearamin_sidecar::readLe16(reply.data, 1) != json.length() ||
        json.length() + reply.length - 3 > size) { error = "invalid"; return false; }
    json.concat(reinterpret_cast<const char *>(reply.data + 3), reply.length - 3);
  }
  if (bearamin_sidecar::crc16Ccitt(reinterpret_cast<const uint8_t *>(json.c_str()), size) != crc) {
    error = "invalid";
    return false;
  }
  presetDefaultAvailable = true;
  return true;
}

bool writeDefaultPreset(const String &json, String &error) {
  if (json.length() == 0 || json.length() > bearamin_default::kMaxBytes) {
    error = "large"; return false;
  }
  bearamin_sidecar::PresetFrame reply = {};
  uint8_t data[bearamin_sidecar::kPresetLinePayloadSize] = {};
  bearamin_sidecar::writeLe16(data, 0, static_cast<uint16_t>(json.length()));
  bearamin_sidecar::writeLe16(data, 2, bearamin_sidecar::crc16Ccitt(
      reinterpret_cast<const uint8_t *>(json.c_str()), json.length()));
  if (!defaultRequest(bearamin_default::kWriteBegin, data, 4, reply, error)) return false;
  for (size_t offset = 0; offset < json.length(); offset += bearamin_default::kWriteChunkBytes) {
    const size_t count = std::min(json.length() - offset, bearamin_default::kWriteChunkBytes);
    bearamin_sidecar::writeLe16(data, 0, static_cast<uint16_t>(offset));
    memcpy(data + 2, json.c_str() + offset, count);
    if (!defaultRequest(bearamin_default::kWriteChunk, data, count + 2, reply, error)) return false;
  }
  if (!defaultRequest(bearamin_default::kCommit, nullptr, 0, reply, error)) return false;
  // Verify the committed record over the same path used at boot.
  String check;
  if (!readDefaultPreset(check, error) || check != json) {
    if (error.length() == 0) error = "verify";
    return false;
  }
  return true;
}

bool presetJsonV10(const String &json) {
  String value;
  String settings;
  return json.length() > 0 && json.length() <= kPresetMaxJsonBytes &&
         extractJsonValue(json, "format", value) &&
         value == "bearamin-settings" &&
         extractJsonValue(json, "version", value) && value.toInt() == 10 &&
         findJsonObjectForKey(json, "settings", settings);
}

bool presetPathForName(const String &name, String &path) {
  if (name == "default" || name.length() == 0 || name.length() >= kPresetMaxNameLength ||
      sanitizePresetName(name) != name) {
    return false;
  }
  path = String(kPresetDirectory) + "/" + name + ".json";
  return true;
}

bool presetFileV10(const String &path) {
  File file = SD.open(path, FILE_READ);
  if (!file || file.isDirectory() || file.size() == 0 ||
      file.size() > kPresetMaxJsonBytes) {
    if (file) file.close();
    return false;
  }
  String json;
  json.reserve(file.size() + 1);
  while (file.available()) json += static_cast<char>(file.read());
  file.close();
  return presetJsonV10(json);
}

void recoverPresetTransactions() {
  File directory = SD.open(kPresetDirectory);
  if (!directory || !directory.isDirectory()) { if (directory) directory.close(); return; }
  for (File entry = directory.openNextFile(); entry; entry = directory.openNextFile()) {
    String path = entry.path();
    const bool regular = !entry.isDirectory();
    entry.close();
    if (!regular || path.endsWith("/default.json.tmp") ||
        path.endsWith("/default.json.bak")) continue;
    if (path.endsWith(".json.tmp")) {
      const String target = path.substring(0, path.length() - 4);
      const String backup = target + ".bak";
      if (presetFileV10(target)) { SD.remove(path); SD.remove(backup); }
      else if (presetFileV10(path)) { SD.remove(target); SD.rename(path, target); SD.remove(backup); }
      else if (presetFileV10(backup)) { SD.remove(target); SD.remove(path); SD.rename(backup, target); }
      else { SD.remove(path); SD.remove(backup); }
    } else if (path.endsWith(".json.bak")) {
      const String target = path.substring(0, path.length() - 4);
      if (presetFileV10(target)) SD.remove(path);
      else if (presetFileV10(path)) { SD.remove(target); SD.rename(path, target); }
      else SD.remove(path);
    }
  }
  directory.close();
}

bool initializePresetStorage();

bool readPresetFromSd(const String &name, String &json, String &error,
                      bool retried = false) {
  String path;
  if (!presetSdReady && !initializePresetStorage()) {
    error = presetLastError.length() > 0 ? presetLastError : "sd";
    return false;
  }
  if (!presetPathForName(name, path) || !SD.exists(path)) {
    error = "missing";
    return false;
  }
  File file = SD.open(path, FILE_READ);
  if (!file || file.isDirectory() || file.size() == 0 ||
      file.size() > kPresetMaxJsonBytes) {
    if (file) file.close();
    error = "invalid";
    if (!retried) {
      presetSdReady = false;
      if (initializePresetStorage()) {
        return readPresetFromSd(name, json, error, true);
      }
    }
    return false;
  }
  json = "";
  json.reserve(file.size() + 1);
  while (file.available()) json += static_cast<char>(file.read());
  file.close();
  if (!presetJsonV10(json)) {
    error = "preset_version";
    return false;
  }
  error = "";
  return true;
}

bool writePresetToSd(const String &name, const String &json, String &error,
                     bool retried = false) {
  String target;
  if (!presetSdReady && !initializePresetStorage()) {
    error = presetLastError.length() > 0 ? presetLastError : "sd";
    return false;
  }
  if (!presetPathForName(name, target)) { error = "name"; return false; }
  if (!presetJsonV10(json)) { error = "preset_version"; return false; }
  const String temporary = target + ".tmp";
  const String backup = target + ".bak";
  SD.remove(temporary);
  File file = SD.open(temporary, FILE_WRITE);
  if (!file || file.print(json) != json.length()) {
    if (file) file.close();
    SD.remove(temporary);
    error = "write";
    if (!retried) {
      presetSdReady = false;
      if (initializePresetStorage()) {
        return writePresetToSd(name, json, error, true);
      }
    }
    return false;
  }
  file.close();
  String verify;
  // Verify the temporary file directly without letting a .tmp become a
  // user-loadable preset name.
  File check = SD.open(temporary, FILE_READ);
  verify = "";
  if (check && check.size() <= kPresetMaxJsonBytes) {
    while (check.available()) verify += static_cast<char>(check.read());
  }
  if (check) check.close();
  if (!presetJsonV10(verify)) {
    SD.remove(temporary);
    error = "verify";
    if (!retried) {
      presetSdReady = false;
      if (initializePresetStorage()) {
        return writePresetToSd(name, json, error, true);
      }
    }
    return false;
  }
  SD.remove(backup);
  if (SD.exists(target) && !SD.rename(target, backup)) {
    SD.remove(temporary);
    error = "backup";
    if (!retried) {
      presetSdReady = false;
      if (initializePresetStorage()) {
        return writePresetToSd(name, json, error, true);
      }
    }
    return false;
  }
  if (!SD.rename(temporary, target)) {
    if (SD.exists(backup)) SD.rename(backup, target);
    error = "commit";
    if (!retried) {
      presetSdReady = false;
      if (initializePresetStorage()) {
        return writePresetToSd(name, json, error, true);
      }
    }
    return false;
  }
  SD.remove(backup);
  presetLastError = "";
  return true;
}

bool initializePresetStorage() {
  presetSdReady = false;
  presetSdActiveSpiHz = 0;
  for (uint8_t attempt = 0; attempt < kPresetSdMountMaxAttempts; ++attempt) {
    const uint8_t divider = attempt < 3 ? attempt : 2;
    uint32_t spiHz = kSdSpiHz >> divider;
    if (spiHz < kPresetSdMinimumSpiHz) spiHz = kPresetSdMinimumSpiHz;
    SD.end();
    pinMode(kSdCsPin, OUTPUT);
    digitalWrite(kSdCsPin, HIGH);
    delay(kPresetSdMountRetryDelayMs);
    SPI.begin(kSdSckPin, kSdMisoPin, kSdMosiPin, kSdCsPin);
    if (!SD.begin(kSdCsPin, SPI, spiHz)) {
      presetLastError = "sd";
      continue;
    }
    presetSdReady = true;
    presetSdActiveSpiHz = spiHz;
    if (!SD.exists(kPresetDirectory) && !SD.mkdir(kPresetDirectory)) {
      SD.end();
      presetSdReady = false;
      presetLastError = "directory";
      continue;
    }
    recoverPresetTransactions();
    presetLastError = "";
    return true;
  }
  return false;
}

bool requestPresetList(String &presetsJson, String &error,
                       String * = nullptr, size_t * = nullptr, size_t = 0) {
  String names[kPresetMaxListedPresets];
  size_t count = 0;
  String json;
  String defaultError;
  if (readDefaultPreset(json, defaultError)) names[count++] = "default";
  // SD availability never hides the stored boot preset.
  if (presetSdReady || initializePresetStorage()) {
    File directory = SD.open(kPresetDirectory);
    if (directory && directory.isDirectory()) {
      for (File entry = directory.openNextFile(); entry && count < kPresetMaxListedPresets;
           entry = directory.openNextFile()) {
        String filename = entry.name();
        const bool regular = !entry.isDirectory();
        entry.close();
        const int slash = filename.lastIndexOf('/');
        if (slash >= 0) filename.remove(0, slash + 1);
        if (!regular || !filename.endsWith(".json")) continue;
        filename.remove(filename.length() - 5);
        if (filename == "default" || sanitizePresetName(filename) != filename) continue;
        bool duplicate = false;
        for (size_t i = 0; i < count; ++i) if (names[i] == filename) duplicate = true;
        if (!duplicate) names[count++] = filename;
      }
    } else {
      presetSdReady = false;
      presetLastError = "directory";
    }
    directory.close();
  }
  if (defaultError.length() && defaultError != "missing" && count == 0) {
    error = defaultError;
    return false;
  }
  for (size_t i = 1; i < count; ++i) {
    String value = names[i]; size_t p = i;
    while (p > 0 && names[p - 1].compareTo(value) > 0) { names[p] = names[p - 1]; --p; }
    names[p] = value;
  }
  presetsJson = "{\"ok\":true,\"sd_ready\":" + jsonBool(presetSdReady) + ",\"presets\":[";
  for (size_t i = 0; i < count; ++i) {
    if (i) presetsJson += ',';
    presetsJson += "{\"name\":" + jsonString(names[i]) + ",\"filename\":" + jsonString(names[i] + ".json") + "}";
  }
  presetsJson += "],\"error\":\"\"}";
  error = "";
  return true;
}

bool findJsonObjectForKey(const String &json, const String &key, String &objectText) {
  const int keyIndex = json.indexOf("\"" + key + "\"");
  if (keyIndex < 0) {
    return false;
  }
  int colon = json.indexOf(':', keyIndex);
  int start = colon < 0 ? -1 : json.indexOf('{', colon);
  if (start < 0) {
    return false;
  }
  int depth = 0;
  bool inString = false;
  bool escaped = false;
  for (int i = start; i < static_cast<int>(json.length()); ++i) {
    const char c = json[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (inString && c == '\\') {
      escaped = true;
      continue;
    }
    if (c == '"') {
      inString = !inString;
      continue;
    }
    if (inString) {
      continue;
    }
    if (c == '{') {
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0) {
        objectText = json.substring(start, i + 1);
        return true;
      }
    }
  }
  return false;
}

bool extractJsonValue(const String &json, const String &key, String &value) {
  const int keyIndex = json.indexOf("\"" + key + "\"");
  if (keyIndex < 0) {
    return false;
  }
  int i = json.indexOf(':', keyIndex);
  if (i < 0) {
    return false;
  }
  ++i;
  while (i < static_cast<int>(json.length()) && isspace(static_cast<unsigned char>(json[i]))) {
    ++i;
  }
  if (i >= static_cast<int>(json.length())) {
    return false;
  }
  if (json[i] == '"') {
    ++i;
    String out;
    bool escaped = false;
    for (; i < static_cast<int>(json.length()); ++i) {
      const char c = json[i];
      if (escaped) {
        out += c;
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        value = out;
        return true;
      } else {
        out += c;
      }
    }
    return false;
  }
  const int start = i;
  while (i < static_cast<int>(json.length()) &&
         json[i] != ',' && json[i] != '}' && json[i] != '\n' && json[i] != '\r') {
    ++i;
  }
  value = json.substring(start, i);
  value.trim();
  return value.length() > 0;
}

bool applySettingsJson(const String &json, String &error) {
  String trimmed = json;
  trimmed.trim();
  if (!trimmed.startsWith("{")) {
    error = "invalid_json";
    return false;
  }

  String value;
  if (!extractJsonValue(trimmed, "format", value) ||
      value != "bearamin-settings" ||
      !extractJsonValue(trimmed, "version", value) || value.toInt() != 10) {
    error = "preset_version";
    return false;
  }
  String settings;
  if (!findJsonObjectForKey(trimmed, "settings", settings)) {
    error = "invalid_json";
    return false;
  }

  if (extractJsonValue(settings, "fm_drone_ranger", value)) {
    fmDroneRangerMode = parseBoolArg(value);
  }
  if (extractJsonValue(settings, "fm_map_x", value)) {
    fmDroneMapX = parseFmParamArg(value, fmDroneMapX);
  }
  if (extractJsonValue(settings, "fm_map_y", value)) {
    fmDroneMapY = parseFmParamArg(value, fmDroneMapY);
  }
  if (extractJsonValue(settings, "fm_map_z", value)) {
    fmDroneMapZ = parseFmParamArg(value, fmDroneMapZ);
  }
  for (uint8_t config = 0; config < kGestureConfigCount; ++config) {
    for (uint8_t lane = 0; lane < 6; ++lane) {
      char key[48];
      snprintf(key, sizeof(key), "map_%s_%u_axis", gestureConfigKey(config), lane);
      if (extractJsonValue(settings, key, value)) {
        setGestureMapForModeLane(
            config,
            lane,
            parseResonatorMapAxisArg(value, gestureMapForModeLane(config, lane)));
      }
      snprintf(key, sizeof(key), "map_%s_%u_depth", gestureConfigKey(config), lane);
      if (extractJsonValue(settings, key, value)) {
        setGestureDepthForModeLane(config, lane, value.toInt());
      }
    }
  }
  if (extractJsonValue(settings, "cv_slew_step", value)) {
    long requested = value.toInt();
    if (requested < 0) requested = 0;
    if (requested > 32) requested = 32;
    cvSlewStep = static_cast<uint16_t>(requested);
    sendCommand(bearamin_sidecar::COMMAND_SET_CV_SLEW, cvSlewStep);
  }
  if (extractJsonValue(settings, "res_feedback_amount", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_BASE]
        = clampPercent(value.toInt(), 95);
  }
  if (resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_BASE] > 95) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_BASE] = 95;
  }
  if (extractJsonValue(settings, "res_feedback_source", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_SOURCE]
        = parseResonatorWebSource(
            value,
            true,
            resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_SOURCE]);
  }
  if (extractJsonValue(settings, "res_feedback_depth", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_DEPTH]
        = encodeSignedDepth(value.toInt());
  }
  if (extractJsonValue(settings, "res_frequency_quantization", value)) {
    setResonatorFrequencyQuantization(
        parseResonatorFrequencyQuantization(value));
  } else {
    setResonatorFrequencyQuantization(
        bearamin_bridge::RESONATOR_FREQUENCY_QUANTIZATION_OFF);
  }
  if (extractJsonValue(settings, "res_drone_enabled", value)) {
    setResonatorWebFlag(
        bearamin_bridge::RESONATOR_WEB_DRONE_ENABLED, parseBoolArg(value));
  }
  if (extractJsonValue(settings, "res_drone_waveform", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS]
        &= static_cast<uint8_t>(~bearamin_bridge::RESONATOR_WEB_DRONE_WAVEFORM_MASK);
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS]
        |= static_cast<uint8_t>(parseResonatorDroneWaveform(value)
                                << bearamin_bridge::RESONATOR_WEB_DRONE_WAVEFORM_SHIFT);
  }
  if (extractJsonValue(settings, "res_drone_level", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_LEVEL]
        = clampPercent(value.toInt());
  }
  if (extractJsonValue(settings, "res_drone_source", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_SOURCE]
        = parseResonatorWebSource(
            value,
            false,
            resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_SOURCE]);
  }
  if (extractJsonValue(settings, "res_drone_depth", value)) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_DEPTH]
        = encodeSignedDepth(value.toInt());
  }
  const uint8_t reverbOffsets[3][3] = {
      {bearamin_bridge::RESONATOR_WEB_REVERB_TIME,
       bearamin_bridge::RESONATOR_WEB_REVERB_TIME_SOURCE,
       bearamin_bridge::RESONATOR_WEB_REVERB_TIME_DEPTH},
      {bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION,
       bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_SOURCE,
       bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_DEPTH},
      {bearamin_bridge::RESONATOR_WEB_REVERB_LP,
       bearamin_bridge::RESONATOR_WEB_REVERB_LP_SOURCE,
       bearamin_bridge::RESONATOR_WEB_REVERB_LP_DEPTH},
  };
  const char *reverbNames[3] = {"time", "diffusion", "lp"};
  for (uint8_t i = 0; i < 3; ++i) {
    String key = String("res_reverb_") + reverbNames[i];
    if (extractJsonValue(settings, key, value)) {
      resonatorWebControls[reverbOffsets[i][0]] = clampPercent(value.toInt());
    }
    key += "_source";
    if (extractJsonValue(settings, key, value)) {
      resonatorWebControls[reverbOffsets[i][1]]
          = parseResonatorWebSource(value, true, resonatorWebControls[reverbOffsets[i][1]]);
    }
    key = String("res_reverb_") + reverbNames[i] + "_depth";
    if (extractJsonValue(settings, key, value)) {
      resonatorWebControls[reverbOffsets[i][2]] = encodeSignedDepth(value.toInt());
    }
  }
  sendCurrentDptControls();
  ++resonatorControlsRevision;
  sendCurrentResonatorWebControls();
  error = "";
  return true;
}

void setEngineeringStream(bool enabled) {
  const uint32_t now = millis();
  lastEngineeringRequestMs = enabled ? now : 0;
  reconcileEngineeringStream(now);
}

String matrixCellsJson() {
  const bool stale = lastCellsMs == 0 || millis() - lastCellsMs > kEngineeringStreamTimeoutMs;
  String body;
  body.reserve(900);
  body += "{\"engineering_stream\":";
  body += jsonBool(engineeringStreamEnabled);
  body += ",\"cells_stale\":";
  body += jsonBool(stale);
  body += ",\"cells_age_ms\":";
  body += lastCellsMs == 0 ? 0 : millis() - lastCellsMs;
  body += ",\"matrix_cell_frames\":";
  body += matrixCellFrames;
  body += ",\"sequence\":";
  body += latestCells.sequence;
  body += ",\"valid_count\":";
  body += latestCells.validCount;
  body += ",\"blob_count\":";
  body += latestCells.blobCount;
  body += ",\"sensor_age_ms\":";
  body += latestCells.sensorAgeMs;
  body += ",\"cells_mm\":[";
  for (uint8_t i = 0; i < 64; ++i) {
    if (i > 0) {
      body += ',';
    }
    body += latestCells.cellsMm[i];
  }
  body += "]}";
  return body;
}

void handleEngineeringStream() {
  setEngineeringStream(!server.hasArg("enabled") || server.arg("enabled").toInt() != 0);
  sendJson("{\"ok\":true}");
}

void handleEngineeringMatrixJson() {
  setEngineeringStream(true);
  sendJson(matrixCellsJson());
}

void handleDptSettings() {
  bool resonatorChanged = false;
  if (server.hasArg("fm_enabled")) {
    fmDroneRangerMode = parseBoolArg(server.arg("fm_enabled"));
  }
  if (server.hasArg("fm_x")) {
    fmDroneMapX = parseFmParamArg(server.arg("fm_x"), fmDroneMapX);
  }
  if (server.hasArg("fm_y")) {
    fmDroneMapY = parseFmParamArg(server.arg("fm_y"), fmDroneMapY);
  }
  if (server.hasArg("fm_z")) {
    fmDroneMapZ = parseFmParamArg(server.arg("fm_z"), fmDroneMapZ);
  }
  if (server.hasArg("announce_modes")) {
    modeAnnouncementsEnabled = parseBoolArg(server.arg("announce_modes"));
  }
  for (uint8_t config = 0; config < kGestureConfigCount; ++config) {
    for (uint8_t lane = 0; lane < 6; ++lane) {
      char key[48];
      snprintf(key, sizeof(key), "map_%s_%u_axis", gestureConfigKey(config), lane);
      if (server.hasArg(key)) {
        setGestureMapForModeLane(
            config,
            lane,
            parseResonatorMapAxisArg(
                server.arg(key), gestureMapForModeLane(config, lane)));
      }
      snprintf(key, sizeof(key), "map_%s_%u_depth", gestureConfigKey(config), lane);
      if (server.hasArg(key)) {
        setGestureDepthForModeLane(config, lane, server.arg(key).toInt());
      }
    }
  }
  if (server.hasArg("cv_slew") || server.hasArg("step") || server.hasArg("value")) {
    const String raw = server.hasArg("cv_slew") ? server.arg("cv_slew")
                       : server.hasArg("step")  ? server.arg("step")
                                                 : server.arg("value");
    long requested = raw.toInt();
    if (requested < 0) {
      requested = 0;
    }
    if (requested > 32) {
      requested = 32;
    }
    cvSlewStep = static_cast<uint16_t>(requested);
    sendCommand(bearamin_sidecar::COMMAND_SET_CV_SLEW, cvSlewStep);
  }
  if (server.hasArg("res_feedback_amount")) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_BASE]
        = clampPercent(server.arg("res_feedback_amount").toInt(), 95);
    resonatorChanged = true;
  }
  if (server.hasArg("res_feedback_source")) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_SOURCE]
        = parseResonatorWebSource(
            server.arg("res_feedback_source"),
            true,
            resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_SOURCE]);
    resonatorChanged = true;
  }
  if (server.hasArg("res_feedback_depth")) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_DEPTH]
        = encodeSignedDepth(server.arg("res_feedback_depth").toInt());
    resonatorChanged = true;
  }
  if (server.hasArg("res_frequency_quantization")) {
    setResonatorFrequencyQuantization(
        parseResonatorFrequencyQuantization(
            server.arg("res_frequency_quantization")));
    resonatorChanged = true;
  }
  if (server.hasArg("res_drone_enabled")) {
    setResonatorWebFlag(
        bearamin_bridge::RESONATOR_WEB_DRONE_ENABLED,
        parseBoolArg(server.arg("res_drone_enabled")));
    resonatorChanged = true;
  }
  if (server.hasArg("res_drone_waveform")) {
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS]
        &= static_cast<uint8_t>(~bearamin_bridge::RESONATOR_WEB_DRONE_WAVEFORM_MASK);
    resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS]
        |= static_cast<uint8_t>(parseResonatorDroneWaveform(server.arg("res_drone_waveform"))
                                << bearamin_bridge::RESONATOR_WEB_DRONE_WAVEFORM_SHIFT);
    resonatorChanged = true;
  }
  const char *simpleKeys[] = {
      "res_drone_level", "res_drone_depth", "res_reverb_time",
      "res_reverb_time_depth", "res_reverb_diffusion",
      "res_reverb_diffusion_depth", "res_reverb_lp", "res_reverb_lp_depth"};
  const uint8_t simpleOffsets[] = {
      bearamin_bridge::RESONATOR_WEB_DRONE_LEVEL,
      bearamin_bridge::RESONATOR_WEB_DRONE_DEPTH,
      bearamin_bridge::RESONATOR_WEB_REVERB_TIME,
      bearamin_bridge::RESONATOR_WEB_REVERB_TIME_DEPTH,
      bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION,
      bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_DEPTH,
      bearamin_bridge::RESONATOR_WEB_REVERB_LP,
      bearamin_bridge::RESONATOR_WEB_REVERB_LP_DEPTH};
  for (uint8_t i = 0; i < 8; ++i) {
    if (server.hasArg(simpleKeys[i])) {
      const bool depth = String(simpleKeys[i]).endsWith("_depth");
      resonatorWebControls[simpleOffsets[i]]
          = depth ? encodeSignedDepth(server.arg(simpleKeys[i]).toInt())
                  : clampPercent(server.arg(simpleKeys[i]).toInt());
      resonatorChanged = true;
    }
  }
  const char *sourceKeys[] = {
      "res_drone_source", "res_reverb_time_source",
      "res_reverb_diffusion_source", "res_reverb_lp_source"};
  const uint8_t sourceOffsets[] = {
      bearamin_bridge::RESONATOR_WEB_DRONE_SOURCE,
      bearamin_bridge::RESONATOR_WEB_REVERB_TIME_SOURCE,
      bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_SOURCE,
      bearamin_bridge::RESONATOR_WEB_REVERB_LP_SOURCE};
  for (uint8_t i = 0; i < 4; ++i) {
    if (server.hasArg(sourceKeys[i])) {
      resonatorWebControls[sourceOffsets[i]] = parseResonatorWebSource(
          server.arg(sourceKeys[i]), i > 0 && i < 4,
          resonatorWebControls[sourceOffsets[i]]);
      resonatorChanged = true;
    }
  }
  if (server.hasArg("control_flags")) {
    sendCommand(bearamin_sidecar::COMMAND_SET_CONTROL_FLAGS,
                static_cast<uint16_t>(strtoul(server.arg("control_flags").c_str(), nullptr, 0)));
  }
  sendCurrentDptControls();
  if (resonatorChanged) {
    ++resonatorControlsRevision;
    sendCurrentResonatorWebControls();
  }
  sendJson(stateJson());
}

void handleCvSlew() {
  handleDptSettings();
}

void handleCvMode() {
  handleDptSettings();
}

String serverPresetJson() {
  String json = "{\"format\":\"bearamin-settings\",\"version\":10,\"firmware\":";
  json += jsonString(BEARAMIN_FIRMWARE_VERSION);
  json += ",\"saved_at_ms\":" + String(millis()) + ",\"settings\":{";
  json += "\"fm_drone_ranger\":" + jsonBool(fmDroneRangerMode);
  json += ",\"fm_map_x\":" + jsonString(fmParamName(fmDroneMapX));
  json += ",\"fm_map_y\":" + jsonString(fmParamName(fmDroneMapY));
  json += ",\"fm_map_z\":" + jsonString(fmParamName(fmDroneMapZ));
  json += ",\"cv_slew_step\":" + String(cvSlewStep);
  for (uint8_t mode = 0; mode < kGestureConfigCount; ++mode) {
    for (uint8_t lane = 0; lane < 6; ++lane) {
      char key[48];
      snprintf(key, sizeof(key), "map_%s_%u_axis", gestureConfigKey(mode), lane);
      json += ",\"" + String(key) + "\":" + jsonString(resonatorMapAxisName(gestureMapForModeLane(mode, lane)));
      snprintf(key, sizeof(key), "map_%s_%u_depth", gestureConfigKey(mode), lane);
      json += ",\"" + String(key) + "\":" + String(gestureDepthForModeLane(mode, lane));
    }
  }
  json += ",\"res_feedback_amount\":" + String(resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_BASE]);
  json += ",\"res_feedback_source\":" + jsonString(resonatorWebSourceName(resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_SOURCE]));
  json += ",\"res_feedback_depth\":" + String(static_cast<int8_t>(resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FEEDBACK_DEPTH]));
  const uint8_t flags = resonatorWebControls[bearamin_bridge::RESONATOR_WEB_FLAGS];
  json += ",\"res_drone_enabled\":" + jsonBool((flags & bearamin_bridge::RESONATOR_WEB_DRONE_ENABLED) != 0);
  json += ",\"res_drone_waveform\":" + jsonString(resonatorDroneWaveformName(flags));
  json += ",\"res_drone_level\":" + String(resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_LEVEL]);
  json += ",\"res_drone_source\":" + jsonString(resonatorWebSourceName(resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_SOURCE]));
  json += ",\"res_drone_depth\":" + String(static_cast<int8_t>(resonatorWebControls[bearamin_bridge::RESONATOR_WEB_DRONE_DEPTH]));
  const char *names[] = {"time", "diffusion", "lp"};
  const uint8_t offsets[][3] = {{bearamin_bridge::RESONATOR_WEB_REVERB_TIME, bearamin_bridge::RESONATOR_WEB_REVERB_TIME_SOURCE, bearamin_bridge::RESONATOR_WEB_REVERB_TIME_DEPTH}, {bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION, bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_SOURCE, bearamin_bridge::RESONATOR_WEB_REVERB_DIFFUSION_DEPTH}, {bearamin_bridge::RESONATOR_WEB_REVERB_LP, bearamin_bridge::RESONATOR_WEB_REVERB_LP_SOURCE, bearamin_bridge::RESONATOR_WEB_REVERB_LP_DEPTH}};
  for (uint8_t i = 0; i < 3; ++i) {
    const String base = String("res_reverb_") + names[i];
    json += ",\"" + base + "\":" + String(resonatorWebControls[offsets[i][0]]);
    json += ",\"" + base + "_source\":" + jsonString(resonatorWebSourceName(resonatorWebControls[offsets[i][1]]));
    json += ",\"" + base + "_depth\":" + String(static_cast<int8_t>(resonatorWebControls[offsets[i][2]]));
  }
  json += ",\"res_frequency_quantization\":" + jsonString(resonatorFrequencyQuantizationName(flags));
  json += "}}\n";
  return json;
}

void handleMatrixCoreOta() {
  sendCommand(bearamin_sidecar::COMMAND_ENTER_MATRIX_OTA, 1);
  sendJson("{\"ok\":true,\"matrixcore_ota_requested\":true}");
}

void handleFmDroneRangerMode() {
  handleDptSettings();
}

void sendPresetErrorResponse(const String &error, int status = 500) {
  String body = "{\"ok\":false,\"sd_ready\":";
  body += jsonBool(presetSdReady);
  body += ",\"presets\":[],\"error\":";
  body += jsonString(error);
  body += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(status, "application/json; charset=utf-8", body);
}

void handlePresetsJson() {
  String body;
  String error;
  if (!requestPresetList(body, error)) {
    sendPresetErrorResponse(error);
    return;
  }
  sendJson(body);
}

void handlePresetSave() {
  const String name = sanitizePresetName(server.arg("name"));
  if (name.length() == 0) {
    sendPresetErrorResponse("name", 400);
    return;
  }
  // Storage is owned by the main unit: serialize authoritative local state,
  // never a browser-supplied JSON document.
  const String json = serverPresetJson();
  String error;
  if (!(name == "default" ? writeDefaultPreset(json, error) : writePresetToSd(name, json, error))) {
    sendPresetErrorResponse(error);
    return;
  }
  String body = "{\"ok\":true,\"sd_ready\":" + jsonBool(presetSdReady) + ",\"name\":";
  body += jsonString(name);
  body += ",\"filename\":";
  body += jsonString(name + ".json");
  body += ",\"error\":\"\"}";
  sendJson(body);
}

void handlePresetLoad() {
  const String name = sanitizePresetName(server.arg("name"));
  if (name.length() == 0) {
    sendPresetErrorResponse("name", 400);
    return;
  }
  String json;
  String error;
  if (!(name == "default" ? readDefaultPreset(json, error) : readPresetFromSd(name, json, error))) {
    sendPresetErrorResponse(error);
    return;
  }
  if (!applySettingsJson(json, error)) {
    presetLastError = error;
    sendPresetErrorResponse(error, 400);
    return;
  }
  sendJson(stateJson());
}

void registerWifiDiagnostics() {
  if (wifiDiagEventsRegistered) {
    return;
  }
  WiFi.onEvent(
      [](WiFiEvent_t event, WiFiEventInfo_t info) {
        switch (event) {
          case ARDUINO_EVENT_WIFI_STA_START:
            recordWifiDiag(WIFI_DIAG_EVENT_STA_START, wifiDiagCurrentAttempt);
            break;
          case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            recordWifiDiag(WIFI_DIAG_EVENT_STA_CONNECTED,
                           wifiDiagCurrentAttempt,
                           -1,
                           -1,
                           info.wifi_sta_connected.channel,
                           -1000,
                           info.wifi_sta_connected.bssid);
            break;
          case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            markWifiOutageEnd(millis());
            recordWifiDiag(WIFI_DIAG_EVENT_STA_GOT_IP,
                           wifiDiagCurrentAttempt,
                           -1,
                           -1,
                           wifiDiagLastChannel,
                           wifiDiagLastRssi,
                           wifiDiagLastBssid,
                           0,
                           0,
                           0,
                           info.got_ip.ip_info.ip.addr);
            break;
          case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            ++wifiDiagDisconnects;
            markWifiOutageStart(millis());
            recordWifiDiag(WIFI_DIAG_EVENT_STA_DISCONNECTED,
                           wifiDiagCurrentAttempt,
                           -1,
                           -1,
                           0,
                           info.wifi_sta_disconnected.rssi,
                           info.wifi_sta_disconnected.bssid,
                           info.wifi_sta_disconnected.reason,
                           info.wifi_sta_disconnected.rssi);
            break;
          case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            recordWifiDiag(WIFI_DIAG_EVENT_STA_LOST_IP, wifiDiagCurrentAttempt);
            break;
          case ARDUINO_EVENT_WIFI_AP_START:
            recordWifiDiag(WIFI_DIAG_EVENT_AP_START, wifiDiagCurrentAttempt);
            break;
          case ARDUINO_EVENT_WIFI_AP_STOP:
            recordWifiDiag(WIFI_DIAG_EVENT_AP_STOP, wifiDiagCurrentAttempt);
            break;
          case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            recordWifiDiag(WIFI_DIAG_EVENT_AP_CLIENT_CONNECTED, wifiDiagCurrentAttempt);
            break;
          case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            recordWifiDiag(WIFI_DIAG_EVENT_AP_CLIENT_DISCONNECTED, wifiDiagCurrentAttempt);
            break;
          case ARDUINO_EVENT_WIFI_SCAN_DONE:
            recordWifiDiag(WIFI_DIAG_EVENT_SCAN_DONE,
                           wifiDiagCurrentAttempt,
                           info.wifi_scan_done.status == 0 ? info.wifi_scan_done.number : -static_cast<int32_t>(info.wifi_scan_done.status),
                           -1);
            break;
          default:
            break;
        }
      },
      ARDUINO_EVENT_MAX);
  wifiDiagEventsRegistered = true;
}

void configureWeb() {
  server.on("/", handleRoot);
  server.on("/capture-worker.js", handleCaptureWorker);
  server.on("/favicon.svg", handleFavicon);
  server.on("/favicon.ico", handleFavicon);
  server.on("/status.json", handleStatusJson);
  server.on("/wifi/diag.json", handleWifiDiagJson);
  server.on("/device.json", handleDeviceJson);
  server.on("/lorenz", HTTP_GET, handleLorenz);
  server.on("/telemetry.json", handleTelemetryJson);
  server.on("/sensor/matrix.json", handleMatrixJson);
  server.on("/frames.json", handleFramesJson);
  server.on("/telemetry/reset", handleTelemetryReset);
  server.on("/telemetry/stop", handleTelemetryStop);
  server.on("/capture/start", HTTP_POST, handleCaptureStart);
  server.on("/capture/data", HTTP_GET, handleCaptureData);
  server.on("/capture/stop", HTTP_POST, handleCaptureStop);
  server.on("/capture/status", HTTP_GET, handleCaptureStatus);
  server.on("/perf.json", handlePerfJson);
  server.on("/engineering/status.json", handlePerfJson);
  server.on("/engineering/stream", HTTP_GET, handleEngineeringStream);
  server.on("/engineering/stream", HTTP_POST, handleEngineeringStream);
  server.on("/engineering/matrix.json", handleEngineeringMatrixJson);
  server.on("/dpt/settings", handleDptSettings);
  server.on("/presets.json", HTTP_GET, handlePresetsJson);
  server.on("/presets/save", HTTP_POST, handlePresetSave);
  server.on("/presets/load", HTTP_POST, handlePresetLoad);
  server.on("/cv/slew", handleCvSlew);
  server.on("/cv/mode", handleCvMode);
  server.on("/matrixcore/ota", HTTP_GET, handleMatrixCoreOta);
  server.on("/matrixcore/ota", HTTP_POST, handleMatrixCoreOta);
  server.on("/dpt/fm-drone-ranger", handleFmDroneRangerMode);
  server.begin();
  obsUdpReady = obsTelemetryUdp.begin(bearamin_obs::kUdpPort) == 1;
  webReady = true;
  recordWifiDiag(WIFI_DIAG_WEB_READY, wifiDiagCurrentAttempt);
}

void configureOta() {
  if (otaReady) {
    return;
  }
  ArduinoOTA.setHostname(BEARAMIN_MDNS_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }
  ArduinoOTA.begin();
  otaReady = true;
  recordWifiDiag(WIFI_DIAG_OTA_READY, wifiDiagCurrentAttempt);
}

struct WifiCandidate {
  bool found = false;
  uint8_t bssid[6] = {};
  int32_t channel = 0;
  int32_t rssi = -1000;
};

void considerWifiCandidate(WifiCandidate &candidate,
                           const uint8_t bssid[6],
                           int32_t channel,
                           int32_t rssi) {
  if (candidate.found && rssi <= candidate.rssi) {
    return;
  }
  candidate.found = true;
  memcpy(candidate.bssid, bssid, sizeof(candidate.bssid));
  candidate.channel = channel;
  candidate.rssi = rssi;
}

bool scanConfiguredWifi(WifiCandidate &preferred,
                        WifiCandidate &fallback) {
  preferred = {};
  fallback = {};
  WifiCandidate strongestAny = {};
  WifiCandidate strongestFiveGhz = {};
  WifiCandidate strongestTwoGhz = {};
  const int networkCount = WiFi.scanNetworks(false, true);
  wifiDiagLastScanCount = networkCount;
  wifiDiagLastMatchCount = 0;
  if (networkCount < 0) {
    ++wifiDiagFailedScans;
    recordWifiDiag(WIFI_DIAG_SCAN_FAILED, wifiDiagCurrentAttempt, networkCount, 0);
    WiFi.scanDelete();
    return false;
  }
  if (networkCount == 0) {
    ++wifiDiagNoSsidScans;
    recordWifiDiag(WIFI_DIAG_SCAN_NO_SSID, wifiDiagCurrentAttempt, networkCount, 0);
    WiFi.scanDelete();
    return false;
  }
  int32_t matchCount = 0;
  for (int i = 0; i < networkCount; ++i) {
    if (WiFi.SSID(i) != WIFI_SSID) {
      continue;
    }
    ++matchCount;
    const int32_t channel = WiFi.channel(i);
    const int32_t rssi = WiFi.RSSI(i);
    const uint8_t *bssid = WiFi.BSSID(i);
    considerWifiCandidate(strongestAny, bssid, channel, rssi);
    if (bearamin_wifi::is5GhzChannel(channel)) {
      considerWifiCandidate(strongestFiveGhz, bssid, channel, rssi);
    } else {
      considerWifiCandidate(strongestTwoGhz, bssid, channel, rssi);
    }
  }
  WiFi.scanDelete();
  wifiDiagLastMatchCount = matchCount;
  if (!strongestAny.found) {
    ++wifiDiagNoSsidScans;
    recordWifiDiag(WIFI_DIAG_SCAN_NO_SSID, wifiDiagCurrentAttempt, networkCount, matchCount);
    return false;
  }

  if (kWifiPrefer2Ghz && strongestTwoGhz.found) {
    preferred = strongestTwoGhz;
    if (strongestFiveGhz.found) {
      fallback = strongestFiveGhz;
    }
  } else if (kWifiPrefer5Ghz && strongestFiveGhz.found) {
    preferred = strongestFiveGhz;
    if (strongestTwoGhz.found) {
      fallback = strongestTwoGhz;
    }
  } else if (kWifiPrefer5Ghz && strongestTwoGhz.found) {
    preferred = strongestTwoGhz;
  } else {
    preferred = strongestAny;
  }
  recordWifiDiag(WIFI_DIAG_SCAN_SELECTED,
                 wifiDiagCurrentAttempt,
                 networkCount,
                 matchCount,
                 preferred.channel,
                 preferred.rssi,
                 preferred.bssid);
  return true;
}

bool connectConfiguredWifi(uint32_t timeoutMs) {
  if (strlen(WIFI_SSID) == 0) {
    return false;
  }
  WiFi.mode(softApReady ? WIFI_AP_STA : WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(BEARAMIN_MDNS_HOSTNAME);
  const uint32_t start = millis();
  recordWifiDiag(WIFI_DIAG_CONNECT_WINDOW_START, wifiDiagCurrentAttempt, -1, -1, 0, -1000, nullptr, 0, 0, 0);
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    WifiCandidate preferred = {};
    WifiCandidate fallback = {};
    if (!scanConfiguredWifi(preferred, fallback)) {
      delay(250);
      continue;
    }
    const WifiCandidate candidates[2] = {preferred, fallback};
    for (const WifiCandidate &candidate : candidates) {
      if (!candidate.found || millis() - start >= timeoutMs) {
        continue;
      }
      const uint32_t remainingMs = timeoutMs - (millis() - start);
      uint32_t attemptTimeoutMs = remainingMs;
      const bool preferredBandCandidate =
          (kWifiPrefer2Ghz &&
           !bearamin_wifi::is5GhzChannel(candidate.channel)) ||
          (!kWifiPrefer2Ghz && kWifiPrefer5Ghz &&
           bearamin_wifi::is5GhzChannel(candidate.channel));
      if (fallback.found && preferredBandCandidate) {
        attemptTimeoutMs = remainingMs / 2;
        if (attemptTimeoutMs > kWifiPreferredBandAttemptMs) {
          attemptTimeoutMs = kWifiPreferredBandAttemptMs;
        }
      }
      const uint32_t attempt = ++wifiDiagConnectAttempts;
      wifiDiagCurrentAttempt = attempt;
      recordWifiDiag(WIFI_DIAG_CONNECT_BEGIN,
                     attempt,
                     wifiDiagLastScanCount,
                     wifiDiagLastMatchCount,
                     candidate.channel,
                     candidate.rssi,
                     candidate.bssid);
      WiFi.begin(
          WIFI_SSID,
          WIFI_PASSWORD,
          candidate.channel,
          candidate.bssid,
          true);
      const uint32_t connectStart = millis();
      while (WiFi.status() != WL_CONNECTED &&
             millis() - connectStart < attemptTimeoutMs &&
             millis() - start < timeoutMs) {
        pollMatrixUart();
        delay(50);
      }
      if (WiFi.status() == WL_CONNECTED) {
        wifiReady = true;
        ++wifiDiagSuccessfulConnects;
        recordWifiDiag(WIFI_DIAG_CONNECT_READY,
                       attempt,
                       wifiDiagLastScanCount,
                       wifiDiagLastMatchCount,
                       candidate.channel,
                       candidate.rssi,
                       candidate.bssid,
                       0,
                       0,
                       millis() - connectStart,
                       ipValue(WiFi.localIP()));
        return true;
      }
      recordWifiDiag(WIFI_DIAG_CONNECT_TIMEOUT,
                     attempt,
                     wifiDiagLastScanCount,
                     wifiDiagLastMatchCount,
                     candidate.channel,
                     candidate.rssi,
                     candidate.bssid,
                     0,
                     0,
                     millis() - connectStart);
      WiFi.disconnect(false, false);
      delay(50);
    }
  }
  wifiReady = WiFi.status() == WL_CONNECTED;
  if (!wifiReady) {
    recordWifiDiag(WIFI_DIAG_CONNECT_WINDOW_FAILED,
                   wifiDiagCurrentAttempt,
                   wifiDiagLastScanCount,
                   wifiDiagLastMatchCount,
                   wifiDiagLastChannel,
                   wifiDiagLastRssi,
                   wifiDiagLastBssid,
                   0,
                   0,
                   millis() - start);
  }
  return wifiReady;
}

void stopSoftAp() {
  if (!softApReady) {
    return;
  }
  WiFi.softAPdisconnect(true);
  softApReady = false;
  recordWifiDiag(WIFI_DIAG_SOFTAP_STOP, wifiDiagCurrentAttempt);
  if (wifiReady) {
    WiFi.mode(WIFI_STA);
  }
}

void startSoftAp() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(BEARAMIN_MDNS_HOSTNAME);
  if (strlen(kSoftApPassword) >= 8) {
    softApReady = WiFi.softAP(kSoftApSsid, kSoftApPassword);
  } else {
    softApReady = WiFi.softAP(kSoftApSsid);
  }
  lastSoftApRescanMs = millis();
  recordWifiDiag(WIFI_DIAG_SOFTAP_START, wifiDiagCurrentAttempt);
  if (softApReady) {
    diagLogPrintf("softap=ready ssid=%s ip=%s\n", kSoftApSsid, WiFi.softAPIP().toString().c_str());
  } else {
    diagLogPrintln("softap=failed");
  }
}

void configureNetworkServices() {
  if (wifiReady) {
    if (MDNS.begin(BEARAMIN_MDNS_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      recordWifiDiag(WIFI_DIAG_MDNS_READY, wifiDiagCurrentAttempt, -1, -1, 0, -1000, nullptr, 0, 0, 0, ipValue(WiFi.localIP()));
    } else {
      recordWifiDiag(WIFI_DIAG_MDNS_FAILED, wifiDiagCurrentAttempt);
    }
  }
  configureOta();
  configureWeb();
}

void configureWifi() {
  const uint32_t bootStart = millis();
  recordWifiDiag(WIFI_DIAG_BOOT_START, wifiDiagCurrentAttempt);
  wifiReady = connectConfiguredWifi(kWifiTimeoutMs);
  if (!wifiReady) {
    startSoftAp();
  }
  configureNetworkServices();
  recordWifiDiag(wifiReady ? WIFI_DIAG_CONNECT_READY : WIFI_DIAG_CONNECT_WINDOW_FAILED,
                 wifiDiagCurrentAttempt,
                 wifiDiagLastScanCount,
                 wifiDiagLastMatchCount,
                 wifiDiagLastChannel,
                 wifiDiagLastRssi,
                 wifiDiagLastBssid,
                 0,
                 0,
                 millis() - bootStart,
                 wifiReady ? ipValue(WiFi.localIP()) : 0);
}

void pollNetwork(uint32_t now) {
  const bool previousWifiReady = wifiReady;
  wifiReady = WiFi.status() == WL_CONNECTED;
  if (previousWifiReady && !wifiReady) {
    markWifiOutageStart(now);
    recordWifiDiag(WIFI_DIAG_POLL_LOST, wifiDiagCurrentAttempt);
  } else if (!previousWifiReady && wifiReady) {
    markWifiOutageEnd(now);
    recordWifiDiag(WIFI_DIAG_POLL_RECOVERED,
                   wifiDiagCurrentAttempt,
                   wifiDiagLastScanCount,
                   wifiDiagLastMatchCount,
                   wifiDiagLastChannel,
                   wifiDiagLastRssi,
                   wifiDiagLastBssid,
                   0,
                   0,
                   wifiDiagLastOutageMs,
                   ipValue(WiFi.localIP()));
  }
  if (wifiReady && softApReady) {
    stopSoftAp();
    return;
  }
  if (!wifiReady && !softApReady) {
    startSoftAp();
    return;
  }
  if (!softApReady || WiFi.softAPgetStationNum() > 0 || now - lastSoftApRescanMs < kSoftApRescanMs) {
    return;
  }
  lastSoftApRescanMs = now;
  recordWifiDiag(WIFI_DIAG_SOFTAP_RESCAN, wifiDiagCurrentAttempt);
  if (connectConfiguredWifi(kSoftApReconnectWindowMs)) {
    stopSoftAp();
  }
}

void pollMatrixUart() {
  while (MatrixSerial.available()) {
    ++matrixBytes;
    const uint8_t byte = static_cast<uint8_t>(MatrixSerial.read());
    if (captureParser.InFrame() ||
        (!matrixParser.InFrame() && byte == bearamin_capture::kMagic[0])) {
      bearamin_capture::RecordView captureRecord = {};
      if (captureParser.Push(byte, captureRecord)) {
        acceptCaptureRecord(captureRecord);
        continue;
      }
      if (captureParser.InFrame()) {
        continue;
      }
      if (byte != bearamin_sidecar::kStart0) {
        continue;
      }
    }
    bearamin_sidecar::StateFrame state = {};
    bearamin_sidecar::CommandFrame command = {};
    bearamin_sidecar::MatrixCellsFrame cells = {};
    bearamin_sidecar::PresetFrame preset = {};
    bearamin_sidecar::SensorDiagnosticsFrame diagnostics = {};
    bearamin_sidecar::RuntimeSettingsFrame runtimeSettings = {};
    bearamin_sidecar::MatrixPerformanceFrame matrixPerformance = {};
    const uint8_t type = matrixParser.Push(
        byte,
        state,
        command,
        cells,
        preset,
        diagnostics,
        runtimeSettings,
        matrixPerformance);
    if (type == bearamin_sidecar::FRAME_STATE) {
      latestState = state;
      lastStateMs = millis();
      ++matrixFrames;
      obsStatePending = true;
      beginBootControlSync();
    } else if (type == bearamin_sidecar::FRAME_MATRIX_CELLS) {
      if (engineeringStreamEnabled) {
        latestCells = cells;
        lastCellsMs = millis();
        ++matrixCellFrames;
        obsCellsPending = true;
      }
    } else if (type == bearamin_sidecar::FRAME_SENSOR_DIAGNOSTICS) {
      latestSensorDiagnostics = diagnostics;
      lastSensorDiagnosticsMs = millis();
      ++matrixSensorDiagnosticsFrames;
    } else if (type == bearamin_sidecar::FRAME_RUNTIME_SETTINGS) {
      adoptRuntimeSettings(runtimeSettings);
    } else if (type == bearamin_sidecar::FRAME_MATRIX_PERFORMANCE) {
      latestMatrixPerformance = matrixPerformance;
      lastMatrixPerformanceMs = millis();
      ++matrixPerformanceFrames;
    } else if (type == bearamin_sidecar::FRAME_ACK) {
      ++matrixAcks;
    } else if (type == bearamin_sidecar::FRAME_PRESET &&
               bearamin_default::isOperation(preset.op)) {
      defaultReply = preset;
      defaultReplyReady = true;
    } else if (type == bearamin_sidecar::FRAME_PRESET &&
               preset.op == bearamin_sidecar::PRESET_OP_LINE &&
               preset.length > 0) {
      String line;
      line.reserve(preset.length);
      for (uint8_t i = 0; i < preset.length; ++i) {
        const char c = static_cast<char>(preset.data[i]);
        if (c != '\n' && c != '\r') {
          line += c;
        }
      }
      if (line.length() > 0) {
        if (line.startsWith("BPDG,")) {
          dptRxDebugLine = line;
        } else if (line.startsWith("DPTP,")) {
          bearamin_bridge::DptPerformance performance = {};
          if (bearamin_bridge::decodeDptPerformanceLine(
                  line.c_str(), performance)) {
            latestDptPerformance = performance;
            lastDptPerformanceMs = millis();
            ++dptPerformanceFrames;
          }
        }
      }
    }
    if (type != 0) {
      ++matrixParserEvents;
    }
  }
}

void pollEngineeringStreamTimeout(uint32_t now) {
  if (lastEngineeringRequestMs != 0 &&
      now - lastEngineeringRequestMs >= kEngineeringStreamTimeoutMs) {
    lastEngineeringRequestMs = 0;
  }
  reconcileEngineeringStream(now);
}

void sendSidecarStatus(uint32_t now) {
  if (now - lastSidecarStatusMs < kSidecarStatusMs) {
    return;
  }
  lastSidecarStatusMs = now;
  wifiReady = WiFi.status() == WL_CONNECTED;
  uint16_t flags = 0;
  if (wifiReady) {
    flags |= bearamin_sidecar::SIDECAR_STATUS_WIFI_READY;
  }
  if (webReady) {
    flags |= bearamin_sidecar::SIDECAR_STATUS_WEB_READY;
  }
  if (otaReady) {
    flags |= bearamin_sidecar::SIDECAR_STATUS_OTA_READY;
  }
  if (softApReady) {
    flags |= bearamin_sidecar::SIDECAR_STATUS_SOFTAP_READY;
  }
  if (wifiReady && bearamin_wifi::is5GhzChannel(WiFi.channel())) {
    flags |= bearamin_sidecar::SIDECAR_STATUS_WIFI_5GHZ;
  }
  sendCommand(bearamin_sidecar::COMMAND_SET_SIDECAR_STATUS, flags);
}

void pollDefaultPresetBoot() {
  const uint32_t now = millis();
  if (!bootControlSyncComplete ||
      bootControlSyncUsedRuntime ||
      presetDefaultLoaded ||
      presetDefaultAttempts >= kPresetDefaultMaxAttempts ||
      matrixStale() ||
      !stateFlag(bearamin_sidecar::STATE_DPT_READY) ||
      (presetDefaultNextAttemptMs != 0 &&
       static_cast<int32_t>(now - presetDefaultNextAttemptMs) < 0)) {
    return;
  }
  presetDefaultAttempted = true;
  ++presetDefaultAttempts;
  String json;
  String error;
  if (!readDefaultPreset(json, error)) {
    presetDefaultLoaded = false;
    if (error == "missing" || presetDefaultAttempts >= kPresetDefaultMaxAttempts) {
      presetLastError = error == "missing" ? "" : error;
      if (error == "missing") {
        presetDefaultAttempts = kPresetDefaultMaxAttempts;
      }
    } else {
      presetLastError = error;
      presetDefaultNextAttemptMs = now + kPresetDefaultRetryMs;
    }
    return;
  }
  if (!applySettingsJson(json, error)) {
    presetDefaultLoaded = false;
    presetLastError = error;
    presetDefaultAttempts = kPresetDefaultMaxAttempts;
    return;
  }
  presetDefaultLoaded = true;
  presetLastError = "";
}

void setup() {
  Serial.begin(kSerialBaud);
  if (kDebugTxPin >= 0) {
    DebugSerial.begin(kDebugBaud, SERIAL_8N1, kDebugRxPin, kDebugTxPin);
    debugSerialReady = true;
  }
  delay(80);
  sidecarBootResetReason = static_cast<uint32_t>(esp_reset_reason());
  sidecarMinFreeHeap = ESP.getFreeHeap();
  initializeCaptureStore();
  initializePresetStorage();
  diagLogPrintln(BEARAMIN_BOOT_LABEL);
  diagLogPrintf("sidecar_reset reason=%s(%lu) heap=%lu debug_uart=%s baud=%lu sd_spi=%s\n",
                resetReasonName(sidecarBootResetReason),
                static_cast<unsigned long>(sidecarBootResetReason),
                static_cast<unsigned long>(sidecarMinFreeHeap),
                debugSerialReady ? "enabled" : "usb-only",
                static_cast<unsigned long>(kDebugBaud),
                presetSdReady ? "ready" : "unavailable");
  MatrixSerial.setRxBufferSize(kMatrixRxBufferBytes);
  MatrixSerial.begin(kMatrixBaud, SERIAL_8N1, kMatrixRxPin, kMatrixTxPin);
  registerWifiDiagnostics();
  configureWifi();
  diagLogPrintf("matrix_uart=ready rx=GPIO%u tx=GPIO%u baud=%lu wifi=%s web=%s ota=%s\n",
                kMatrixRxPin,
                kMatrixTxPin,
                static_cast<unsigned long>(kMatrixBaud),
                wifiReady ? "ready" : "off",
                webReady ? "ready" : "off",
                otaReady ? "ready" : "off");
}

void loop() {
  const uint32_t now = millis();
  updateSidecarHealth(now);
  pollNetwork(now);
  pollMatrixUart();
  expireObsSubscriptions(now);
  pollObsSubscriptions(now);
  sendPendingObsTelemetry(now);
  pollBootControlSync(now);
  pollDefaultPresetBoot();
  pollEngineeringStreamTimeout(now);
  sendSidecarStatus(now);
  if (bootControlSyncComplete &&
      now - lastResonatorControlSendMs >= 1000 &&
      latestState.dptResonatorControlsRevision != resonatorControlsRevision) {
    sendCurrentResonatorWebControls();
  }
  if (webReady) {
    server.handleClient();
  }
  if (otaReady) {
    ArduinoOTA.handle();
  }
}
