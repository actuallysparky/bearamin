#include "pico/stdlib.h"
#include "pico/rand.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include "vendor/vl53l7cx/vl53l7cx_api.h"
#include "../common/bearamin_virtual_wire.h"
#include "midi_map.h"
#include "safety.h"
#include "runtime_service.h"
#include "family_identity.h"
#include "editor_settings_json.h"
#include "gesture_processing.h"
#include "preview_cadence.h"
#include "mbedtls/sha256.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/unique_id.h"
#include <cstdlib>
#include <bearfield/bearfield.h>

namespace {
// Waveshare RP2040-Zero perimeter pads 13, 10, 15, and 14, respectively.
constexpr uint kSda = 12, kScl = 9, kLpn = 14, kInt = 13;
constexpr uint32_t kStatusUs = 1000000;
VL53L7CX_Configuration sensor{};
VL53L7CX_ResultsData result{};
bearfield::Processor processor(bearfield::BearCvV1());
char line[1024];
uint32_t bootId = 0, sequence = 0;
bool noteHeld = false;
bool usbSessionChanged = false;
bearamin_safety::SensorHealth health;
bearamin_safety::MidiOutput midiOutput;
#include "editor_runtime.h"
void applyEditorConfig() {
 processor=bearfield::Processor(virtual_editor::feelProfile(config));
 const uint8_t cc[]={uint8_t(config.ccX),uint8_t(config.ccY),uint8_t(config.ccZ),uint8_t(config.ccPresence)};midiOutput.configure(config.channel-1,cc,config.note);noteHeld=false;
}

void midi(uint8_t status, uint8_t data1, uint8_t data2) {
  midiOutput.queue(status, data1, data2);
}
void serial(size_t size) {
  if (!size || !tud_cdc_connected() || tud_cdc_write_available() < size) return;
  tud_cdc_write(line, size);
  tud_cdc_write_flush();
}
void status(bool fault) {
  serial(bearamin_virtual::statusLine(line, sizeof(line), bootId, sequence++, time_us_64(), fault));
}
bool beginSensor() {
  if (!RecoverI2cBus(kSda, kScl)) return false;
  BeginOperation(&sensor.platform, bearamin_safety::kInitBudgetUs);
  gpio_put(kLpn, 0);
  if (WaitMs(&sensor.platform, 10)) return false;
  gpio_put(kLpn, 1);
  if (WaitMs(&sensor.platform, 250)) return false;
  sensor.platform.address = 0x29;
  uint8_t alive = 0;
  if (vl53l7cx_is_alive(&sensor, &alive) || !alive) return false;
  if (vl53l7cx_init(&sensor) ||
      vl53l7cx_set_resolution(&sensor, 64) ||
      vl53l7cx_set_ranging_frequency_hz(&sensor, 15) ||
      vl53l7cx_set_target_order(&sensor, VL53L7CX_TARGET_ORDER_STRONGEST) ||
      vl53l7cx_set_integration_time_ms(&sensor, 10) ||
      vl53l7cx_start_ranging(&sensor)) return false;
  processor.reset();
  return true;
}
void publishFrame() {
  const uint64_t captured = time_us_64();
  health.frame(captured);
  bearfield::DepthFrame8x8 depth{};
  depth.sequence = sequence;
  depth.capturedUs = captured;
  for (unsigned i = 0; i < 64; ++i) {
    if (bearamin_virtual::validZone(result.distance_mm[i], result.target_status[i])) {
      depth.distanceMm[i] = static_cast<uint16_t>(result.distance_mm[i]);
      depth.validMask |= uint64_t{1} << i;
    }
  }
  auto canonical = virtual_editor::orient(depth, config);
  const auto field = processor.process(canonical, captured);
  const auto gesture = virtual_editor::gesture(field, config);
  serial(bearamin_virtual::frameLine(line, sizeof(line), bootId, sequence++, captured,
                                   result.distance_mm, result.target_status, &gesture));
  if(!editorSaving) {
    virtual_editor::performanceMidi(field, config, noteHeld, midi);
  }
  editorPreview(canonical,field);
}
void sensorFault() {
  health.fault(time_us_64());
  previewPending=false;
  if(editorIsPreview&&!editorOffset){editorLength=0;editorIsPreview=false;}
  processor.reset();
  noteHeld = false;
  midiOutput.silence();
  status(true);
  bearamin_service_usb();
}
}

extern "C" void tud_mount_cb(void) { usbSessionChanged = true; }
extern "C" void tud_umount_cb(void) { usbSessionChanged = true; }
void bearamin_service_usb() {
  tud_task();
  static bearamin_identity_probe identity{};
  static bool standardPending=false;
  uint8_t incoming[4];
  // Bound inbound work so discovery cannot monopolize the sensor loop.
  for (unsigned n=0;n<4 && tud_midi_packet_read(incoming);++n) {
    if (bearamin_identity_accept(&identity, incoming, editorPacket)) standardPending=true;
  }
  if (usbSessionChanged) {
    usbSessionChanged = false; midiOutput.disconnect(); noteHeld = false;
    identity.pending = false;standardPending=false;editorDecoder.reset();editorPending=false;editorLength=editorOffset=0;editorIsPreview=false;editorStopPreview();editorSaving=false;
  }
  const bool mounted = tud_midi_mounted() && !tud_suspended();
  if (!mounted) noteHeld = false;
  if(standardPending&&!editorLength){standardPending=false;editorOffset=0;bearamin_identity_reply(2, [](const uint8_t*p){memcpy(editorWire+editorLength,p+1,3);editorLength+=3;});}
  editorService(mounted);
}

int main() {
  restore_interrupts(0); // Resident loader enters with IRQs masked.
  stdio_init_all();
  bootId = get_rand_32();
  i2c_init(i2c0, 400000);
  gpio_set_function(kSda, GPIO_FUNC_I2C);
  gpio_set_function(kScl, GPIO_FUNC_I2C);
  gpio_pull_up(kSda); gpio_pull_up(kScl);
  gpio_init(kLpn); gpio_set_dir(kLpn, GPIO_OUT); gpio_put(kLpn, 1);
  gpio_init(kInt); gpio_set_dir(kInt, GPIO_IN); gpio_pull_up(kInt);
  editorLoad();applyEditorConfig();
  tusb_init();
  uint64_t statusAt = 0;
  bool started = false;
  while (true) {
    bearamin_service_usb();
    editorHandle();midiUpdater.poll();if(midiUpdater.active){sleep_ms(1);continue;}if(editorApply){editorApply=false;applyEditorConfig();}
    const uint64_t now = time_us_64();
    if (health.retryDue(now)) {
      status(started);
      noteHeld = false;
      midiOutput.silence();
      if (beginSensor()) health.started(time_us_64());
      else sensorFault();
      started = true;
      statusAt = time_us_64() + kStatusUs;
    }
    if (health.stale(time_us_64())) sensorFault();
    if (health.ready) {
      BeginOperation(&sensor.platform, bearamin_safety::kReadBudgetUs);
      uint8_t available = 0;
      if (vl53l7cx_check_data_ready(&sensor, &available)) {
        sensorFault();
      } else if (available) {
        if (vl53l7cx_get_ranging_data(&sensor, &result)) {
          sensorFault();
        } else if (health.stale(time_us_64())) sensorFault();
        else publishFrame();
      }
      if (health.stale(time_us_64())) sensorFault();
    } else if (now >= statusAt) {
      status(true);
      statusAt = now + kStatusUs;
    }
    bearamin_service_usb();
    sleep_ms(1);
  }
}
