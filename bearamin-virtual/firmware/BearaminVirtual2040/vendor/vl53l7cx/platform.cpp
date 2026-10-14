#include "platform.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "../../safety.h"
#include "../../runtime_service.h"
#include <initializer_list>

static constexpr uint32_t kChunk = 256;
static constexpr uint8_t kSensorAddress = 0x29;

void BeginOperation(VL53L7CX_Platform* p, uint32_t budget_us) {
  p->failed = false;
  p->deadline_us = time_us_64() + budget_us;
}
static bool healthy(VL53L7CX_Platform* p) {
  bearamin_service_usb();
  if (p->deadline_us && time_us_64() >= p->deadline_us) p->failed = true;
  return !p->failed;
}
static absolute_time_t chunkDeadline(VL53L7CX_Platform* p) {
  uint64_t until = time_us_64() + bearamin_safety::kChunkTimeoutUs;
  if (p->deadline_us && p->deadline_us < until) until = p->deadline_us;
  return from_us_since_boot(until);
}

bool RecoverI2cBus(unsigned sda, unsigned scl) {
  i2c_deinit(i2c0); // Reset FIFO/restart state after an aborted transaction.
  for (unsigned pin : {sda, scl}) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN); // Open drain: release high; never drive high.
    gpio_pull_up(pin);
  }
  auto released = [](unsigned pin) {
    const uint64_t until = time_us_64() + 50;
    do { if (gpio_get(pin)) return true; sleep_us(1); } while (time_us_64() < until);
    return false;
  };
  bool free = released(scl);
  for (unsigned i = 0; free && !gpio_get(sda) && i < 9; ++i) {
    gpio_set_dir(scl, GPIO_OUT); sleep_us(5);
    gpio_set_dir(scl, GPIO_IN); free = released(scl); sleep_us(5);
  }
  if (free) {
    gpio_set_dir(sda, GPIO_OUT); sleep_us(5);
    free = released(scl);
    gpio_set_dir(sda, GPIO_IN); sleep_us(5); // STOP, with SCL released high.
    free = free && gpio_get(sda);
  }
  gpio_set_dir(sda, GPIO_IN); gpio_set_dir(scl, GPIO_IN);
  i2c_init(i2c0, 400000);
  gpio_set_function(sda, GPIO_FUNC_I2C); gpio_set_function(scl, GPIO_FUNC_I2C);
  bearamin_service_usb();
  return free;
}

uint8_t RdMulti(VL53L7CX_Platform* p, uint16_t reg, uint8_t* out, uint32_t size) {
  memset(out, 0, size); // Failed ULD reads must not expose stale/uninitialized data.
  const uint8_t address = p->address ? p->address : kSensorAddress;
  for (uint32_t offset = 0; offset < size; offset += kChunk) {
    if (!healthy(p)) { memset(out, 0, size); return 1; }
    uint8_t command[2] = {static_cast<uint8_t>((reg + offset) >> 8),
                          static_cast<uint8_t>(reg + offset)};
    uint32_t count = size - offset < kChunk ? size - offset : kChunk;
    const auto until = chunkDeadline(p); // Address + repeated-start read share a deadline.
    if (i2c_write_blocking_until(i2c0, address, command, 2, true, until) != 2 ||
        i2c_read_blocking_until(i2c0, address, out + offset, count, false, until) != static_cast<int>(count)) {
      p->failed = true; memset(out, 0, size); bearamin_service_usb(); return 1;
    }
    bearamin_service_usb();
  }
  return 0;
}

uint8_t WrMulti(VL53L7CX_Platform* p, uint16_t reg, uint8_t* data, uint32_t size) {
  const uint8_t address = p->address ? p->address : kSensorAddress;
  uint8_t buffer[kChunk + 2];
  for (uint32_t offset = 0; offset < size; offset += kChunk) {
    if (!healthy(p)) return 1;
    uint32_t count = size - offset < kChunk ? size - offset : kChunk;
    buffer[0] = static_cast<uint8_t>((reg + offset) >> 8);
    buffer[1] = static_cast<uint8_t>(reg + offset);
    memcpy(buffer + 2, data + offset, count);
    if (i2c_write_blocking_until(i2c0, address, buffer, count + 2, false, chunkDeadline(p)) != static_cast<int>(count + 2)) {
      p->failed = true; bearamin_service_usb(); return 1;
    }
    bearamin_service_usb();
  }
  return 0;
}
uint8_t RdByte(VL53L7CX_Platform* p, uint16_t reg, uint8_t* out) { return RdMulti(p, reg, out, 1); }
uint8_t WrByte(VL53L7CX_Platform* p, uint16_t reg, uint8_t value) { return WrMulti(p, reg, &value, 1); }
void SwapBuffer(uint8_t* buffer, uint16_t size) {
  for (uint16_t i = 0; i < size; i += 4) {
    uint32_t value = (uint32_t(buffer[i]) << 24) | (uint32_t(buffer[i+1]) << 16) |
                     (uint32_t(buffer[i+2]) << 8) | buffer[i+3];
    memcpy(buffer + i, &value, 4);
  }
}
uint8_t WaitMs(VL53L7CX_Platform* p, uint32_t ms) {
  for (uint32_t i = 0; i < ms; ++i) {
    if (!healthy(p)) return 1;
    sleep_ms(1);
  }
  return 0;
}
