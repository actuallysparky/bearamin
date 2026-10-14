#!/usr/bin/env python3
"""Execute the production save/load/boot handlers with storage and UART fakes."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'firmware/libraries/BearaminSidecar/src/BearaminSidecarApp.cpp'
source = SOURCE.read_text()
functions = []
for name in ('handlePresetSave', 'handlePresetLoad', 'pollDefaultPresetBoot'):
    match = re.search(r'^void ' + name + r'\(\) \{.*?^\}', source, re.M | re.S)
    assert match, name
    functions.append(match.group())
preamble = r'''
#include <cassert>
#include <cstdint>
#include <string>
using String = std::string;
namespace bearamin_sidecar { constexpr int STATE_DPT_READY = 1; }
struct Server { String name; String arg(const char *) { return name; } } server;
bool presetSdReady = false;
String storedDefault, storedNamed, response, presetLastError;
bool bootControlSyncComplete = true, bootControlSyncUsedRuntime = false;
bool presetDefaultLoaded = false, presetDefaultAttempted = false;
bool stale = false, dptReady = true, valid = true, failDefaultRead = false;
int sdReads = 0, sdWrites = 0, defaultReads = 0, defaultWrites = 0, applies = 0;
uint8_t presetDefaultAttempts = 0;
constexpr uint8_t kPresetDefaultMaxAttempts = 5;
constexpr uint16_t kPresetDefaultRetryMs = 1500;
uint32_t now = 100, presetDefaultNextAttemptMs = 0;
uint32_t millis() { return now; }
bool matrixStale() { return stale; }
bool stateFlag(int) { return dptReady; }
String sanitizePresetName(const String &name) { return name; }
String serverPresetJson() { return "full-preset-all-mode-mappings"; }
String jsonBool(bool value) { return value ? "true" : "false"; }
String jsonString(const String &value) { return "\"" + value + "\""; }
String stateJson() { return "applied"; }
void sendJson(const String &body) { response = body; }
void sendPresetErrorResponse(const String &error, int = 500) { response = "error:" + error; }
bool writeDefaultPreset(const String &json, String &) { ++defaultWrites; storedDefault = json; return true; }
bool writePresetToSd(const String &, const String &json, String &error) {
  ++sdWrites;
  if (!presetSdReady) { error = "sd"; return false; }
  storedNamed = json; return true;
}
bool readDefaultPreset(String &json, String &error) {
  ++defaultReads;
  if (failDefaultRead) { error = "timeout"; return false; }
  if (storedDefault.empty()) { error = "missing"; return false; }
  json = storedDefault; return true;
}
bool readPresetFromSd(const String &, String &json, String &error) {
  ++sdReads;
  if (!presetSdReady) { error = "sd"; return false; }
  json = storedNamed; return true;
}
bool applySettingsJson(const String &json, String &error) {
  ++applies;
  if (!valid) { error = "invalid_json"; return false; }
  assert(json == serverPresetJson()); return true;
}
void resetBoot() {
  bootControlSyncComplete = true; bootControlSyncUsedRuntime = false;
  presetDefaultLoaded = false; presetDefaultAttempted = false;
  presetDefaultAttempts = 0; presetDefaultNextAttemptMs = 0;
  stale = false; dptReady = true; valid = true; failDefaultRead = false;
  now = 100; applies = 0;
}
'''
assertions = r'''
int main() {
  server.name = "default";
  handlePresetSave();
  assert(defaultWrites == 1 && sdWrites == 0);
  assert(response.find("\"ok\":true") != String::npos);
  assert(response.find("\"sd_ready\":false") != String::npos);
  handlePresetLoad();
  assert(response == "applied" && sdReads == 0);

  server.name = "room";
  handlePresetSave();
  assert(response == "error:sd" && defaultWrites == 1 && sdWrites == 1);
  handlePresetLoad();
  assert(response == "error:sd" && sdReads == 1);
  presetSdReady = true;
  handlePresetSave(); handlePresetLoad();
  assert(response == "applied" && storedNamed == storedDefault);

  // Cold boot uses NVS even with no card and never consults a legacy SD default.
  presetSdReady = false;
  const int previousSdReads = sdReads;
  resetBoot(); pollDefaultPresetBoot();
  assert(presetDefaultLoaded && applies == 1 && sdReads == previousSdReads);
  pollDefaultPresetBoot(); assert(applies == 1);
  // A sidecar-only restart must not replace ongoing runtime settings.
  resetBoot(); bootControlSyncUsedRuntime = true; pollDefaultPresetBoot();
  assert(applies == 0 && !presetDefaultAttempted);
  // Wait for runtime synchronization and DPT readiness, then load once.
  resetBoot(); bootControlSyncComplete = false; pollDefaultPresetBoot();
  assert(!presetDefaultAttempted);
  bootControlSyncComplete = true; dptReady = false; pollDefaultPresetBoot();
  assert(!presetDefaultAttempted);
  dptReady = true; stale = true; pollDefaultPresetBoot(); assert(!presetDefaultAttempted);
  stale = false; pollDefaultPresetBoot(); assert(presetDefaultLoaded);

  resetBoot(); storedDefault.clear(); pollDefaultPresetBoot();
  assert(!presetDefaultLoaded && applies == 0 && presetDefaultAttempts == kPresetDefaultMaxAttempts);
  storedDefault = serverPresetJson();
  resetBoot(); valid = false; pollDefaultPresetBoot();
  assert(!presetDefaultLoaded && presetLastError == "invalid_json");
  resetBoot(); failDefaultRead = true;
  for(int attempt = 0; attempt < kPresetDefaultMaxAttempts; ++attempt) {
    pollDefaultPresetBoot();
    const int reads = defaultReads;
    pollDefaultPresetBoot(); assert(defaultReads == reads);
    now += kPresetDefaultRetryMs;
  }
  failDefaultRead = false; pollDefaultPresetBoot();
  assert(!presetDefaultLoaded && applies == 0);
}
'''
with tempfile.TemporaryDirectory(prefix='bearamin-default-routes-') as temp:
    cpp = Path(temp) / 'routes.cpp'
    binary = Path(temp) / 'routes'
    cpp.write_text(preamble + '\n'.join(functions) + assertions)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('default preset routes: PASS (no SD, named errors, cold boot, restart, corrupt/missing defaults, retries)')
