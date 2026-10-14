#include "tusb.h"
#include "pico/unique_id.h"
#include <string.h>

enum { ITF_CDC, ITF_CDC_DATA, ITF_MIDI_AUDIO, ITF_MIDI_STREAM, ITF_TOTAL };
#define TOTAL_LENGTH (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MIDI_DESC_LEN)

static const tusb_desc_device_t device = {
  .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
  .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
  .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
  .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
  .idVendor = 0xCafe, .idProduct = 0x2041, .bcdDevice = 0x0100,
  .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3, .bNumConfigurations = 1,
};
static const uint8_t configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, TOTAL_LENGTH, 0, 100),
  TUD_CDC_DESCRIPTOR(ITF_CDC, 4, 0x81, 8, 0x02, 0x82, 64),
  TUD_MIDI_DESCRIPTOR(ITF_MIDI_AUDIO, 5, 0x03, 0x83, 64),
};
static const char* strings[] = {
  "", "Bearamin", "Bearamin Virtual", "",
  "Bearamin Serial", "Bearamin MIDI",
};
uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*)&device; }
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) { (void)index; return configuration; }
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  static uint16_t buffer[32];
  if (index >= sizeof(strings)/sizeof(strings[0])) return NULL;
  if (index == 0) { buffer[1] = 0x0409; buffer[0] = (TUSB_DESC_STRING << 8) | 4; return buffer; }
  static char serial[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
  if (index == 3) {
    pico_get_unique_board_id_string(serial, sizeof(serial));
    strings[3] = serial;
  }
  const char* text = strings[index];
  size_t length = strlen(text); if (length > 31) length = 31;
  for (size_t i=0; i<length; ++i) buffer[i+1] = text[i];
  buffer[0] = (TUSB_DESC_STRING << 8) | (2*length+2);
  return buffer;
}
