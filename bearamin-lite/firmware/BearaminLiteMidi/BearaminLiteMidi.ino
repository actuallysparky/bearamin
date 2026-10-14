// Opt-in prototype target. Reuse the current production CV/DAC implementation
// without changing the existing production/diagnostic targets or manual copper.
#include <USB.h>
#include <USBMIDI.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <esp_mac.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <atomic>
#include "lite_contract.h"
#include "lite_settings_json.h"
#include "build_identity.h"
#define setup bearcvSetup
#define loop bearcvLoop
#include "../BearCV/BearCV.ino"
#undef setup
#undef loop
#include "lite_runtime.h"
void setup(){lite_runtime::begin();}
void loop(){lite_runtime::loop();}
