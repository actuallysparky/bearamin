#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L7CX.h>
#include "../targets/bearamin_virtual_c3.h"
#include "bearamin_virtual_wire.h"

Adafruit_VL53L7CX sensor;
VL53L7CX_ResultsData results;
char line[bearamin_virtual::kFrameLineMax];
uint32_t bootId;
uint32_t sequenceNumber = 0;
uint32_t lastRetryMs = 0;
uint32_t lastStatusMs = 0;
bool ready = false;
bool everStarted = false;

void sendLine(size_t length) {
  if (length && Serial && Serial.availableForWrite() >= static_cast<int>(length))
    Serial.write(reinterpret_cast<const uint8_t *>(line), length);
}

bool startSensor() {
  digitalWrite(bearamin_virtual::kLpn, HIGH);
  delay(250);
  return sensor.begin(0x29, &Wire, 400000) &&
         sensor.setResolution(64) &&
         sensor.setRangingFrequency(bearamin_virtual::kSensorHz) &&
         sensor.setTargetOrder(VL53L7CX_TARGET_ORDER_STRONGEST) &&
         sensor.setIntegrationTime(10) && sensor.startRanging();
}

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  bootId = esp_random();
  pinMode(bearamin_virtual::kLpn, OUTPUT);
  pinMode(bearamin_virtual::kInt, INPUT_PULLUP);
  digitalWrite(bearamin_virtual::kLpn, HIGH);
  Wire.begin(bearamin_virtual::kSda, bearamin_virtual::kScl, 400000);
  lastRetryMs = millis() - bearamin_virtual::kRetryMs;
  lastStatusMs = millis() - bearamin_virtual::kStatusMs;
}

void loop() {
  const uint32_t now = millis();
  if (!ready && now - lastRetryMs >= bearamin_virtual::kRetryMs) {
    lastRetryMs = now;
    sendLine(bearamin_virtual::statusLine(line, sizeof(line), bootId,
             sequenceNumber++, static_cast<uint64_t>(esp_timer_get_time()), false));
    ready = startSensor();
    everStarted = true;
  }
  if (!ready) {
    if (now - lastStatusMs >= bearamin_virtual::kStatusMs) {
      lastStatusMs = now;
      sendLine(bearamin_virtual::statusLine(line, sizeof(line), bootId,
               sequenceNumber++, static_cast<uint64_t>(esp_timer_get_time()), everStarted));
    }
    delay(2);
    return;
  }
  if (sensor.isDataReady()) {
    if (!sensor.getRangingData(&results)) {
      ready = false;
      lastRetryMs = now;
      sendLine(bearamin_virtual::statusLine(line, sizeof(line), bootId,
               sequenceNumber++, static_cast<uint64_t>(esp_timer_get_time()), true));
      lastStatusMs = now;
    } else {
      const uint64_t capturedUs = static_cast<uint64_t>(esp_timer_get_time());
      sendLine(bearamin_virtual::frameLine(line, sizeof(line), bootId,
               sequenceNumber++, capturedUs, results.distance_mm, results.target_status));
    }
  }
  delay(1);
}
