#include <Adafruit_NeoPixel.h>
#include <Adafruit_VL53L7CX.h>
#include <Arduino.h>
#include <Wire.h>
#include <algorithm>
#include <cmath>
#include <esp_timer.h>

#include "../../third_party/bearfield-core/include/bearfield/bearfield.h"
#include "../common/bearcv_output.h"
#include "../common/bearcv_matrix_display.h"
#include "../common/bearcv_spank.h"
#include "../common/bearcv_dac_boot_policy.h"
#include "../common/chaos_lorenz.h"
#include "../common/trajectory_keyframe.h"

namespace {

#include "../targets/bearcv_dual_mcp4728_v1.h"

TwoWire dacWire(1);
Adafruit_VL53L7CX matrixSensor;
VL53L7CX_ResultsData matrixResults;
bearfield::DepthFrame8x8 matrixDisplayFrame = {};
bearcv_display::Renderer matrixDisplayRenderer;
bearcv_spank::Detector spankGesture;
Adafruit_NeoPixel display(64, kLedPin, NEO_RGB + NEO_KHZ800);
uint8_t displayBrightness = 32;
void setDisplayBrightness(uint8_t brightness) {
  displayBrightness = std::min<uint8_t>(brightness, 32);
}
uint32_t displayColor(uint8_t r, uint8_t g, uint8_t b) {
  return display.Color(bearcv_display::scaleChannel(r, displayBrightness),
                       bearcv_display::scaleChannel(g, displayBrightness),
                       bearcv_display::scaleChannel(b, displayBrightness));
}
uint32_t displayColor(uint32_t rgb) {
  return displayColor(uint8_t(rgb >> 16), uint8_t(rgb >> 8), uint8_t(rgb));
}
bearfield::Processor fieldProcessor(bearfield::BearCvV1());
bearfield::ProcessedFieldState fieldState = {};
bearamin_trajectory::Authority trajectoryAuthority;
bearamin_trajectory::EvaluatedAxes trajectoryAxes = {};
bearcv::SixCvCodes currentCodes = bearcv::safeZeroCodes();
uint32_t sourceSequence = 0;
uint64_t lastOutputUs = 0;
uint32_t lastSensorFrameMs = 0;
bool sensorReady = false;
bool dacReady = false;
bool outputFault = true;
// DAC recovery may clear its own fault, never a sensor/trajectory fault.
bool nonDacOutputFault = false;
enum class InitializerState : uint8_t { kPending, kReady, kError };
InitializerState bipolarDacState = InitializerState::kPending;
InitializerState unipolarDacState = InitializerState::kPending;
InitializerState sensorState = InitializerState::kPending;
bearcv_chaos::Unlock chaosUnlock;
bearcv_chaos::Lorenz chaosLorenz;
uint64_t chaosLastUs = 0;
float chaosInfluence = 0.0f;
float imuBaselineG = 1.0f;
bool imuReady = false;
uint32_t lastImuPollMs = 0;
uint32_t lastChaosImpactMs = 0;

bool qmiRead(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(0x6B); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x6B, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool qmiWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x6B); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

void pollChaosImu(uint32_t now) {
  if (!imuReady || now - lastImuPollMs < 20) return;
  lastImuPollMs = now;
  // ESP32 Wire.begin() does not change pins on an already active bus.
  Wire.end();
  Wire.begin(11, 12, 400000);
  uint8_t raw[6] = {};
  const bool ok = qmiRead(0x35, raw, sizeof(raw));
  Wire.end();
  Wire.begin(kSensorSdaPin, kSensorSclPin, 400000);
  if (!ok) { imuReady = false; return; }
  const int16_t ax = static_cast<int16_t>((raw[1] << 8) | raw[0]);
  const int16_t ay = static_cast<int16_t>((raw[3] << 8) | raw[2]);
  const int16_t az = static_cast<int16_t>((raw[5] << 8) | raw[4]);
  const float magnitude = std::sqrt(static_cast<float>(ax)*ax + static_cast<float>(ay)*ay + static_cast<float>(az)*az) * 16.0f / 32768.0f;
  const float impulse = std::fabs(magnitude - imuBaselineG);
  imuBaselineG += (magnitude - imuBaselineG) * 0.015f;
  if (impulse >= 1.2f && (lastChaosImpactMs == 0 || now - lastChaosImpactMs >= 80)) {
    lastChaosImpactMs = now;
    if (chaosUnlock.on(now)) chaosLorenz.seed(static_cast<uint32_t>(esp_timer_get_time()));
  }
}

bool i2cAddressPresent(TwoWire &wire, uint8_t address) {
  wire.beginTransmission(address);
  return wire.endTransmission() == 0;
}

constexpr uint16_t kDacProvisionHalfClockUs = 20;
constexpr uint16_t kDacLdacSelectSetupUs = 2;
constexpr uint32_t kDacProvisionLineTimeoutUs = 2000;
constexpr uint32_t kDacWireTimeoutMs = 5;
uint8_t addressWritesRemaining[2] = {bearcv_boot::kWriteBudget, bearcv_boot::kWriteBudget};
uint8_t safeWritesRemaining[2] = {bearcv_boot::kWriteBudget, bearcv_boot::kWriteBudget};
uint32_t lastDacRecoveryMs = 0;

using Mcp4728AddressBits = bearcv_boot::AddressBits;

enum class DacProvisionStatus : uint8_t {
  kAlreadyProvisioned,
  kProvisioned,
  kUnexpectedInitialAddressState,
  kSelectorReadFailed,
  kUnexpectedFactoryAddress,
  kAddressWriteFailed,
  kAddressVerifyFailed,
  kAmbiguousBus,
  kSafeEepromFailed,
};

const char *dacProvisionStatusText(DacProvisionStatus status) {
  switch (status) {
    case DacProvisionStatus::kAlreadyProvisioned:
      return "already provisioned";
    case DacProvisionStatus::kProvisioned:
      return "provisioned now";
    case DacProvisionStatus::kUnexpectedInitialAddressState:
      return "unexpected initial I2C address state";
    case DacProvisionStatus::kSelectorReadFailed:
      return "LDAC-selected address read failed";
    case DacProvisionStatus::kUnexpectedFactoryAddress:
      return "unexpected factory address bits";
    case DacProvisionStatus::kAddressWriteFailed:
      return "unipolar address write failed";
    case DacProvisionStatus::kAmbiguousBus:
      return "unexpected extra DAC address; safe failure";
    case DacProvisionStatus::kSafeEepromFailed:
      return "safe-output EEPROM verification failed";
    case DacProvisionStatus::kAddressVerifyFailed:
      return "post-write address verification failed";
  }
  return "unknown status";
}

void releaseDacLine(uint8_t pin) { pinMode(pin, INPUT); }

void pullDacLineLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

bool releaseDacClockAndWaitHigh() {
  releaseDacLine(kDacSclPin);
  const uint32_t start = micros();
  while (!digitalRead(kDacSclPin)) {
    if (micros() - start > kDacProvisionLineTimeoutUs) return false;
  }
  delayMicroseconds(kDacProvisionHalfClockUs);
  return true;
}

void pullDacClockLow() {
  pullDacLineLow(kDacSclPin);
  delayMicroseconds(kDacProvisionHalfClockUs);
}

bool dacStartCondition() {
  releaseDacLine(kDacSdaPin);
  if (!releaseDacClockAndWaitHigh()) return false;
  if (!digitalRead(kDacSdaPin)) return false;
  pullDacLineLow(kDacSdaPin);
  delayMicroseconds(kDacProvisionHalfClockUs);
  pullDacClockLow();
  return true;
}

bool dacStopCondition() {
  pullDacLineLow(kDacSdaPin);
  if (!releaseDacClockAndWaitHigh()) return false;
  releaseDacLine(kDacSdaPin);
  delayMicroseconds(kDacProvisionHalfClockUs);
  return true;
}

bool dacWriteByte(uint8_t value, bool selectAtLastBit = false,
                  uint8_t selectedLdacPin = 0) {
  for (int bit = 7; bit >= 0; --bit) {
    if (value & (1u << bit)) {
      releaseDacLine(kDacSdaPin);
    } else {
      pullDacLineLow(kDacSdaPin);
    }
    delayMicroseconds(kDacProvisionHalfClockUs);
    if (!releaseDacClockAndWaitHigh()) return false;
    pullDacClockLow();
  }
  releaseDacLine(kDacSdaPin);
  if (selectAtLastBit) {
    // Fig. 5-11 requires the LDAC edge during clock 8 low, immediately
    // before ACK 9; a full half-clock early makes the address command NACK.
    pullDacLineLow(selectedLdacPin);
    delayMicroseconds(kDacLdacSelectSetupUs);
  } else {
    delayMicroseconds(kDacProvisionHalfClockUs);
  }
  if (!releaseDacClockAndWaitHigh()) return false;
  const bool acknowledged = !digitalRead(kDacSdaPin);
  pullDacClockLow();
  return acknowledged;
}

bool dacReadByte(uint8_t *value) {
  uint8_t received = 0;
  releaseDacLine(kDacSdaPin);
  for (int bit = 7; bit >= 0; --bit) {
    delayMicroseconds(kDacProvisionHalfClockUs);
    if (!releaseDacClockAndWaitHigh()) return false;
    if (digitalRead(kDacSdaPin)) received |= 1u << bit;
    pullDacClockLow();
  }
  // The MCP4728 general-call address read is a single byte; NACK it.
  releaseDacLine(kDacSdaPin);
  delayMicroseconds(kDacProvisionHalfClockUs);
  if (!releaseDacClockAndWaitHigh()) return false;
  pullDacClockLow();
  *value = received;
  return true;
}

void holdDacProvisionOutputsSafe() {
  digitalWrite(kPresenceGatePin, LOW);
  digitalWrite(kSpankGatePin, LOW);
  digitalWrite(kBipolarLdacPin, HIGH);
  digitalWrite(kUnipolarLdacPin, HIGH);
}

bool readSelectedMcp4728Address(uint8_t selectedLdacPin,
                                Mcp4728AddressBits *address) {
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::SelectedRead)) return false;
#endif

  holdDacProvisionOutputsSafe();
  uint8_t packed = 0;
  const bool transferred = dacStartCondition() && dacWriteByte(0x00) &&
                           dacWriteByte(0x0C, true, selectedLdacPin) &&
                           dacStartCondition() && dacWriteByte(0xC1) &&
                           dacReadByte(&packed);
  const bool stopped = dacStopCondition();
  holdDacProvisionOutputsSafe();
  if (!transferred || !stopped) return false;
  return bearcv_boot::decodeSelectedAddress(packed, address);
}

bool writeSelectedMcp4728Address(uint8_t currentBits, uint8_t newBits,
                                 uint8_t device) {
  if (device > 1) return false;
  if (!addressWritesRemaining[device]) return false;
  --addressWritesRemaining[device]; // Count uncertain writes too; read back before retry.
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::AddressWrite)) return false;
#endif

  const uint8_t command1 = 0x60 | ((currentBits & 0x07) << 2) | 0x01;
  const uint8_t command2 = 0x60 | (newBits << 2) | 0x02;
  const uint8_t command3 = 0x60 | (newBits << 2) | 0x03;
  holdDacProvisionOutputsSafe();
  const bool transferred = dacStartCondition() &&
      dacWriteByte((0x60 | currentBits) << 1) &&
      (device == 1 ? dacWriteByte(command1, true, kUnipolarLdacPin)
                   : dacWriteByte(command1, true, kBipolarLdacPin)) &&
      dacWriteByte(command2) && dacWriteByte(command3);
  const bool stopped = dacStopCondition();
  holdDacProvisionOutputsSafe();
  return transferred && stopped;
}

bool writeUnipolarMcp4728Address(uint8_t currentBits) {
  const uint8_t newBits = kUnipolarDacAddress & 0x07;
  return writeSelectedMcp4728Address(currentBits, newBits, 1);
}

bool recoverDacBus() {
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::Recover)) return false;
#endif

  dacWire.end();
  holdDacProvisionOutputsSafe();
  releaseDacLine(kDacSdaPin);
  // NXP I2C bus clear: at most nine clocks, followed by STOP; never drive high.
  for (uint8_t i = 0; i < 9 && !digitalRead(kDacSdaPin); ++i) {
    pullDacClockLow();
    if (!releaseDacClockAndWaitHigh()) return false;
  }
  const bool stopped = dacStopCondition();
  return stopped && digitalRead(kDacSdaPin) && digitalRead(kDacSclPin);
}

bool startDacWire(uint32_t frequency = 100000) {
  const bool ok = dacWire.begin(kDacSdaPin, kDacSclPin, frequency);
  dacWire.setTimeOut(kDacWireTimeoutMs);
  return ok;
}

bool waitDacReady(uint8_t bits) {
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::Busy)) return false;
#endif

  if (!startDacWire()) return false;
  const uint32_t start = millis();
  bool ready = false;
  do {
    if (dacWire.requestFrom(static_cast<uint8_t>(0x60 | bits), uint8_t{1}) == 1)
      ready = (dacWire.read() & 0x80) != 0; // RDY/BSY, not an ACK-only delay.
    if (ready) break;
    delay(2);
  } while (millis() - start < bearcv_boot::kBusyTimeoutMs);
  dacWire.end();
  return ready;
}

struct DacBootTransport {
  bool recover() { return recoverDacBus(); }
  bool readPair(Mcp4728AddressBits *bipolar, Mcp4728AddressBits *unipolar) {
    dacWire.end();
    return readSelectedMcp4728Address(kBipolarLdacPin, bipolar) &&
           readSelectedMcp4728Address(kUnipolarLdacPin, unipolar);
  }
  bool scan(uint8_t *mask) {
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::Scan)) return false;
#endif

    if (!startDacWire()) return false;
    *mask = 0;
    for (uint8_t bits = 0; bits < 8; ++bits) {
      dacWire.beginTransmission(0x60 | bits);
      const uint8_t error = dacWire.endTransmission();
      if (error == 0) *mask |= 1u << bits;
      else if (error != 2) { dacWire.end(); return false; } // NACK only is absence.
    }
    dacWire.end();
    return true;
  }
  bool waitReady(uint8_t bits) { return waitDacReady(bits); }
  bool writeAddress(bool uni, Mcp4728AddressBits value) {
    if (uni) {
      // Use the active address if volatile/EEPROM bits disagree after an upset.
      const Mcp4728AddressBits unipolar = value;
      return unipolar.input == unipolar.eeprom
          ? writeUnipolarMcp4728Address(unipolar.eeprom)
          : writeUnipolarMcp4728Address(unipolar.input);
    }
    return writeSelectedMcp4728Address(value.input, 0, 0);
  }
};

DacProvisionStatus provisionDacAddressesOnFirstBoot() {
  // The historical API name is retained; this is executed on EVERY boot/recovery.
  DacBootTransport transport;
  const auto result = bearcv_boot::ensureAddresses(transport);
  Mcp4728AddressBits bipolar = {}, unipolar = {};
  // Independent final device-selected read, including already-provisioned boards.
  if (result == bearcv_boot::Result::kReady || result == bearcv_boot::Result::kRepaired) {
    if (!readSelectedMcp4728Address(kBipolarLdacPin, &bipolar) ||
        !readSelectedMcp4728Address(kUnipolarLdacPin, &unipolar) ||
        !(bipolar.eeprom == 0 && bipolar.input == 0 &&
          unipolar.eeprom == 1 && unipolar.input == 1))
      return DacProvisionStatus::kAddressVerifyFailed;
    return result == bearcv_boot::Result::kReady
        ? DacProvisionStatus::kAlreadyProvisioned : DacProvisionStatus::kProvisioned;
  }
  return result == bearcv_boot::Result::kAmbiguous
      ? DacProvisionStatus::kAmbiguousBus : DacProvisionStatus::kAddressVerifyFailed;
}

bool configureMcp4728Channel(uint8_t address,
                             uint8_t channel,
                             uint16_t code,
                             bool update) {
  code = std::min<uint16_t>(code, bearcv::kDacMaximum);
  dacWire.beginTransmission(address);
  dacWire.write(static_cast<uint8_t>(0x40u | ((channel & 0x03u) << 1) |
                                     (update ? 0u : 1u)));
  dacWire.write(static_cast<uint8_t>(0x90u | ((code >> 8) & 0x0Fu)));
  dacWire.write(static_cast<uint8_t>(code & 0xFFu));
  return dacWire.endTransmission() == 0;
}

bool configureMcp4728(uint8_t address, const uint16_t codes[4]) {
  for (uint8_t channel = 0; channel < 4; ++channel) {
    if (!configureMcp4728Channel(address, channel, codes[channel],
                                 channel == 3)) {
      return false;
    }
  }
  return true;
}

bool fastWriteMcp4728(uint8_t address, const uint16_t codes[4]) {
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(address == kBipolarDacAddress
      ? bearcv_diag::Stage::BipolarWrite : bearcv_diag::Stage::UnipolarWrite)) return false;
#endif
  dacWire.beginTransmission(address);
  for (uint8_t channel = 0; channel < 4; ++channel) {
    const uint16_t code = std::min<uint16_t>(codes[channel], bearcv::kDacMaximum);
    dacWire.write(static_cast<uint8_t>((code >> 8) & 0x0Fu));
    dacWire.write(static_cast<uint8_t>(code & 0xFFu));
  }
  return dacWire.endTransmission() == 0;
}

void latchDacOutputs() {
  digitalWrite(kBipolarLdacPin, LOW);
  digitalWrite(kUnipolarLdacPin, LOW);
  delayMicroseconds(2);
  digitalWrite(kBipolarLdacPin, HIGH);
  digitalWrite(kUnipolarLdacPin, HIGH);
}

bool writeCvCodes(const bearcv::SixCvCodes &codes) {
  const uint16_t bipolar[4] = {
      codes.xBipolar, codes.yBipolar, codes.zBipolar,
      bearcv::kBipolarBiasCode};
  const uint16_t unipolar[4] = {
      codes.xUnipolar, codes.yUnipolar, codes.zUnipolar, 0};
  const bool bipolarOk = fastWriteMcp4728(kBipolarDacAddress, bipolar);
  const bool unipolarOk = fastWriteMcp4728(kUnipolarDacAddress, unipolar);
  if (!bipolarOk) bipolarDacState = InitializerState::kError;
  if (!unipolarOk) unipolarDacState = InitializerState::kError;
  const bool ok = bipolarOk && unipolarOk;
  if (ok) latchDacOutputs();
  else { dacReady = false; outputFault = true; holdDacProvisionOutputsSafe(); }
  return ok;
}

void forceSafeOutputs() {
  digitalWrite(kPresenceGatePin, LOW);
  digitalWrite(kSpankGatePin, LOW);
  currentCodes = bearcv::safeZeroCodes();
  if (dacReady) writeCvCodes(currentCodes);
}

bool verifySafeDacEeprom(uint8_t address, const uint16_t codes[4], uint8_t device) {
  for (uint8_t attempt = 0; attempt < bearcv_boot::kAttempts; ++attempt) {
    if (!waitDacReady(address & 7) || !startDacWire()) continue;
    uint8_t bytes[24] = {};
    const bool readOk =
#ifdef BEARCV_DAC_FAULT_TEST
        !bearcvDiagnosticIntercept(bearcv_diag::Stage::EepromRead) &&
#endif
        dacWire.requestFrom(address, uint8_t{24}) == 24;
    if (readOk) for (auto &value : bytes) value = dacWire.read();
    // Never overwrite on an incomplete/unreadable EEPROM response.
    if (!readOk) { dacWire.end(); recoverDacBus(); continue; }
    if (bearcv_boot::safeEeprom(bytes, address & 7, codes)) return true;
    // A valid, ready per-channel response must precede any EEPROM repair.
    bool valid = true;
    for (uint8_t ch = 0; ch < 4; ++ch) for (uint8_t bank = 0; bank < 2; ++bank) {
      const uint8_t header = bytes[ch * 6 + bank * 3];
      valid &= (header & 0x80) && (header & 0x30) == (ch << 4) &&
               (header & 7) == (address & 7);
    }
    if (!valid || !safeWritesRemaining[device]) return false;
    --safeWritesRemaining[device];
#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::EepromWrite)) return false;
#endif

    dacWire.beginTransmission(address);
    dacWire.write(0x51); // Sequential EEPROM A-D; UDAC=1, LDAC stays high.
    for (uint8_t ch = 0; ch < 4; ++ch) {
      dacWire.write(static_cast<uint8_t>(0x90 | (codes[ch] >> 8)));
      dacWire.write(static_cast<uint8_t>(codes[ch] & 0xff));
    }
    const uint8_t error = dacWire.endTransmission();
    dacWire.end();
    // Even a lost ACK may have committed: poll and read again before rewriting.
    if (error && !recoverDacBus()) continue;
  }
  // The final permitted write still gets its own busy poll/readback.
  if (!waitDacReady(address & 7) || !startDacWire()) return false;
  uint8_t bytes[24] = {};

#ifdef BEARCV_DAC_FAULT_TEST
  if (bearcvDiagnosticIntercept(bearcv_diag::Stage::EepromRead)) return false;
#endif
  if (dacWire.requestFrom(address, uint8_t{24}) != 24) return false;
  for (auto &value : bytes) value = dacWire.read();
  return bearcv_boot::safeEeprom(bytes, address & 7, codes);
}

bool beginDacs(DacProvisionStatus *provisionStatus) {
  holdDacProvisionOutputsSafe();
  *provisionStatus = provisionDacAddressesOnFirstBoot();
  if (*provisionStatus != DacProvisionStatus::kAlreadyProvisioned &&
      *provisionStatus != DacProvisionStatus::kProvisioned) {
    bipolarDacState = InitializerState::kError;
    unipolarDacState = InitializerState::kError;
    return false;
  }
  if (!startDacWire(400000)) return false;
  const bool bipolarPresent = i2cAddressPresent(dacWire, kBipolarDacAddress);
  const bool unipolarPresent = i2cAddressPresent(dacWire, kUnipolarDacAddress);
  if (!bipolarPresent) bipolarDacState = InitializerState::kError;
  if (!unipolarPresent) unipolarDacState = InitializerState::kError;
  if (!bipolarPresent || !unipolarPresent) {
    return false;
  }
  const uint16_t bipolar[4] = {
      bearcv::kBipolarZeroCode, bearcv::kBipolarZeroCode,
      bearcv::kBipolarZeroCode, bearcv::kBipolarBiasCode};
  const uint16_t unipolar[4] = {0, 0, 0, 0};
  const bool bipolarConfigured = configureMcp4728(kBipolarDacAddress, bipolar);
  const bool unipolarConfigured = configureMcp4728(kUnipolarDacAddress, unipolar);
  if (!bipolarConfigured) bipolarDacState = InitializerState::kError;
  if (!unipolarConfigured) unipolarDacState = InitializerState::kError;
  if (!bipolarConfigured || !unipolarConfigured) return false;
  if (!writeCvCodes(bearcv::safeZeroCodes())) return false;
  if (!verifySafeDacEeprom(kBipolarDacAddress, bipolar, 0) ||
      !verifySafeDacEeprom(kUnipolarDacAddress, unipolar, 1)) {
    bipolarDacState = InitializerState::kError;
    unipolarDacState = InitializerState::kError;
    *provisionStatus = DacProvisionStatus::kSafeEepromFailed;
    return false;
  }
  if (!startDacWire(400000) || !writeCvCodes(bearcv::safeZeroCodes())) return false;
  bipolarDacState = InitializerState::kReady;
  unipolarDacState = InitializerState::kReady;
  return true;
}

bool beginSensor() {
  pinMode(kSensorInterruptPin, INPUT_PULLUP);
  pinMode(kSensorLprPin, OUTPUT);
  digitalWrite(kSensorLprPin, HIGH);
  Wire.end();
  Wire.begin(kSensorSdaPin, kSensorSclPin, 400000);
  delay(250);
  return matrixSensor.begin(0x29, &Wire, 400000) &&
         matrixSensor.setResolution(64) &&
         matrixSensor.setRangingFrequency(kMatrixSourceHz) &&
         matrixSensor.setTargetOrder(VL53L7CX_TARGET_ORDER_STRONGEST) &&
         matrixSensor.setIntegrationTime(10) &&
         matrixSensor.startRanging();
}

void updateSpankGesture(const bearfield::DepthFrame8x8 &frame, uint32_t nowMs,
                        uint16_t maximumDepthMm=150, float minimumApproach=250,
                        uint16_t refractoryMs=300, uint8_t minimumCells=4) {
  uint16_t depths[64] = {};
  uint8_t count = 0;
  // Match MatrixCore's active Bearfield onset input: ALL valid depths.
  // Selecting only the nearest blob amplifies partial/lateral hand motion.
  for (uint8_t i=0; i<64; ++i) {
    const uint16_t mm=frame.distanceMm[i];
    if ((frame.validMask & (UINT64_C(1)<<i)) && mm && mm<=2000)
      depths[count++]=mm;
  }
  std::sort(depths, depths+count);
  const bool confident=fieldState.present && fieldState.blobCount>=minimumCells &&
                       count>=minimumCells;
  const uint16_t median=!count ? 0 : (count&1) ? depths[count/2] :
      uint16_t((uint32_t(depths[count/2-1])+depths[count/2])/2);
  const bool wasHeld=spankGesture.gate();
  fieldState.spankHeld=spankGesture.update(confident,median,nowMs,
      maximumDepthMm,minimumApproach,refractoryMs);
  fieldState.spankStarted=!wasHeld && fieldState.spankHeld;
}

void readSensorFrame() {
  if (!sensorReady || !matrixSensor.isDataReady()) return;
  if (!matrixSensor.getRangingData(&matrixResults)) {
    sensorReady = false;
    spankGesture.reset();
    fieldState.spankHeld = false;
    sensorState = InitializerState::kError;
    nonDacOutputFault = true;
    outputFault = true;
    forceSafeOutputs();
    return;
  }
  bearfield::DepthFrame8x8 raw = {};
  raw.sequence = ++sourceSequence;
  raw.capturedUs = static_cast<uint64_t>(esp_timer_get_time());
  for (uint8_t index = 0; index < 64; ++index) {
    const int16_t distance = matrixResults.distance_mm[index];
    raw.distanceMm[index] = distance > 0 ? static_cast<uint16_t>(distance) : 0;
    const uint8_t status = matrixResults.target_status[index];
    if ((status == 5 || status == 9) && distance > 0 && distance <= 2000) {
      raw.validMask |= UINT64_C(1) << index;
    }
  }
  const bearfield::DepthFrame8x8 canonical =
      bearfield::orientFrame(raw, kSensorMount);
  matrixDisplayFrame = canonical;
  fieldProcessor.process(canonical, raw.capturedUs);
  fieldState = fieldProcessor.target();
  updateSpankGesture(canonical, millis());
  chaosInfluence = (static_cast<float>(fieldState.xNorm12) - 2048.0f) / 4095.0f +
      (static_cast<float>(fieldState.yNorm12) - 2048.0f) / 8190.0f;
  const uint32_t sourceEpochMs = millis();
  const uint16_t targetNorm[bearamin_trajectory::kAxisCount] = {
      fieldState.xNorm12, fieldState.yNorm12, fieldState.zNorm12,
      fieldState.pitchNorm12, fieldState.rollNorm12};
  const uint16_t tauMs = bearcv::trajectoryTauMs(
      bearcv::normalizedKnob(analogRead(kSlewAdcPin)));
  if (!trajectoryAuthority.publishTarget(targetNorm, sourceEpochMs, tauMs)) {
    nonDacOutputFault = true;
    outputFault = true;
    forceSafeOutputs();
    return;
  }
  lastSensorFrameMs = sourceEpochMs;
}

void updateDisplay(bool trajectoryValid) {
  display.clear();
  if (outputFault) {
    for (uint8_t i = 0; i < 8; ++i) {
      display.setPixelColor(bearfield::displayPixelIndex(i, i, kDisplayMount),
                            displayColor(48, 0, 0));
    }
  } else {
    const bool fresh = sensorReady && lastSensorFrameMs &&
        millis() - lastSensorFrameMs <= kSensorStaleMs;
    matrixDisplayRenderer.render(
        matrixDisplayFrame.distanceMm, matrixDisplayFrame.validMask, fresh,
        trajectoryValid && fieldState.present,
        trajectoryAxes.norm[bearamin_trajectory::AXIS_X],
        trajectoryAxes.norm[bearamin_trajectory::AXIS_Y],
        trajectoryAxes.norm[bearamin_trajectory::AXIS_Z], fieldState.zMm,
        fieldState.spankHeld);
    for (uint8_t y = 0; y < 8; ++y) for (uint8_t x = 0; x < 8; ++x) {
      const auto &p = matrixDisplayRenderer.pixels[y * 8 + x];
      display.setPixelColor(bearfield::displayPixelIndex(x, y, kDisplayMount),
                            displayColor(p.red, p.green, p.blue));
    }
  }
  const auto drawStatus = [](uint8_t x, InitializerState state,
                             uint32_t pendingColor, uint32_t readyColor,
                             uint32_t errorColor) {
    if (state == InitializerState::kReady && readyColor == 0) return;
    const uint32_t color = state == InitializerState::kPending
                               ? pendingColor
                               : state == InitializerState::kReady
                                     ? readyColor
                                     : errorColor;
    display.setPixelColor(bearfield::displayPixelIndex(x, 0, kDisplayMount),
                          displayColor(color));
  };
  constexpr uint32_t kStatusRed = 0x300000;
  constexpr uint32_t kStatusOrange = 0x301000;
  constexpr uint32_t kStatusYellow = 0x303000;
  constexpr uint32_t kStatusGreen = 0x003000;
  // UNI is top-left and BIP is top-right. A ready DAC leaves its pixel clear.
  drawStatus(0, unipolarDacState, kStatusOrange, 0, kStatusRed);
  drawStatus(7, bipolarDacState, kStatusOrange, 0, kStatusRed);
  // Both middle pixels are one sensor state, so they always match.
  drawStatus(3, sensorState, kStatusYellow, kStatusGreen, kStatusRed);
  drawStatus(4, sensorState, kStatusYellow, kStatusGreen, kStatusRed);
  display.show();
}

void serviceOutputs(uint64_t nowUs) {
  constexpr uint64_t kOutputPeriodUs = 1000000ULL / kOutputHz;
  if (lastOutputUs != 0 && nowUs - lastOutputUs < kOutputPeriodUs) return;
  lastOutputUs = nowUs;
  const uint32_t nowMs = millis();
  const bool sourceStale = lastSensorFrameMs == 0 ||
      nowMs - lastSensorFrameMs > kSensorStaleMs;
  if (sourceStale) { spankGesture.reset(); fieldState.spankHeld = false; }
  const float range = bearcv::normalizedKnob(analogRead(kRangeAdcPin));
  if (chaosUnlock.active()) {
    const float elapsed = chaosLastUs == 0 ? 1.0f / kOutputHz :
        static_cast<float>(nowUs - chaosLastUs) / 1000000.0f;
    chaosLastUs = nowUs;
    const bearcv_chaos::Axes axes = chaosLorenz.tick(
        elapsed, bearcv::normalizedKnob(analogRead(kSlewAdcPin)), chaosInfluence);
    currentCodes = bearcv::targetCodes(!outputFault, axes.x, axes.y, axes.z, range);
    if (dacReady && !writeCvCodes(currentCodes)) {
      outputFault = true;
      dacReady = false;
    }
    digitalWrite(kPresenceGatePin, !outputFault && fieldState.present ? HIGH : LOW);
    digitalWrite(kSpankGatePin, !outputFault && fieldState.spankHeld ? HIGH : LOW);
    updateDisplay(true);
    return;
  }
  const uint16_t tauMs = bearcv::trajectoryTauMs(
      bearcv::normalizedKnob(analogRead(kSlewAdcPin)));
  if (!sourceStale && trajectoryAuthority.hasKeyframe() &&
      !trajectoryAuthority.changeTau(tauMs, nowMs)) {
    nonDacOutputFault = true;
    outputFault = true;
  }
  const bool trajectoryValid = !sourceStale &&
      trajectoryAuthority.hasKeyframe() &&
      bearamin_trajectory::evaluateAtSourceTime(
          trajectoryAuthority.keyframe(), nowMs, trajectoryAxes);
  const bearcv::SixCvCodes targets = bearcv::targetCodes(
      !outputFault && trajectoryValid && fieldState.present,
      trajectoryAxes.norm[bearamin_trajectory::AXIS_X],
      trajectoryAxes.norm[bearamin_trajectory::AXIS_Y],
      trajectoryAxes.norm[bearamin_trajectory::AXIS_Z], range);
  currentCodes = targets;
  if (dacReady && !writeCvCodes(currentCodes)) {
    outputFault = true;
    dacReady = false;
  }
  digitalWrite(kPresenceGatePin,
               !outputFault && trajectoryValid && fieldState.present ? HIGH : LOW);
  digitalWrite(kSpankGatePin,
               !outputFault && trajectoryValid && fieldState.spankHeld ? HIGH : LOW);
  updateDisplay(trajectoryValid);
}

}  // namespace

#if !defined(BEARCV_JACK_SWEEP) && !defined(BEARCV_DAC_FAULT_TEST)
void setup() {
  pinMode(kPresenceGatePin, OUTPUT);
  pinMode(kSpankGatePin, OUTPUT);
  pinMode(kBipolarLdacPin, OUTPUT);
  pinMode(kUnipolarLdacPin, OUTPUT);
  holdDacProvisionOutputsSafe();
  forceSafeOutputs();
  Serial.begin(115200);
  const uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 1000) delay(10);
  Serial.println("BearCV production boot");
  analogReadResolution(12);
  display.begin();
  // Brightness is capped and scaled in displayColor, without a second
  // NeoPixel truncation that would erase the dim white occupancy dots.
  display.setBrightness(255);
  setDisplayBrightness(32);
  display.clear();
  display.show();
  chaosUnlock.begin(millis());
  // ESP32 Wire.begin() does not change pins on an already active bus.
  Wire.end();
  Wire.begin(11, 12, 400000);
  uint8_t who = 0;
  imuReady = qmiRead(0x00, &who, 1) && who == 0x05;
  if (imuReady) imuReady = qmiWrite(0x02, 0x40) && qmiWrite(0x03, 0x2C) &&
      qmiWrite(0x04, 0x36) && qmiWrite(0x06, 0x00) && qmiWrite(0x08, 0x03);
  Wire.end();
  Wire.begin(kSensorSdaPin, kSensorSclPin, 400000);
  DacProvisionStatus dacProvisionStatus =
      DacProvisionStatus::kUnexpectedInitialAddressState;
  dacReady = beginDacs(&dacProvisionStatus);
  Serial.print("MCP4728 address provisioning: ");
  Serial.println(dacProvisionStatusText(dacProvisionStatus));
  Serial.println(dacReady ? "MCP4728 initialization: ready"
                          : "MCP4728 initialization: safe failure");
  sensorReady = beginSensor();
  sensorState = sensorReady ? InitializerState::kReady : InitializerState::kError;
  nonDacOutputFault = !sensorReady;
  outputFault = !dacReady || nonDacOutputFault;
  if (outputFault) forceSafeOutputs();
  updateDisplay(false);
}

void loop() {
  const uint32_t now = millis();
  if (!dacReady && now - lastDacRecoveryMs >= bearcv_boot::kRetryMs) {
    lastDacRecoveryMs = now;
    DacProvisionStatus status = DacProvisionStatus::kAddressVerifyFailed;
    dacReady = beginDacs(&status);
    if (dacReady) outputFault = nonDacOutputFault || !sensorReady;
    Serial.printf("DAC recovery: %s\n", dacProvisionStatusText(status));
  }
  pollChaosImu(millis());
  readSensorFrame();
  serviceOutputs(static_cast<uint64_t>(esp_timer_get_time()));
}

#elif defined(BEARCV_JACK_SWEEP)
#include "../BearCVDacJackSweep/jack_sweep_impl.h"
#else
#include "../BearCVDacFaultTest/fault_test_impl.h"
#endif
