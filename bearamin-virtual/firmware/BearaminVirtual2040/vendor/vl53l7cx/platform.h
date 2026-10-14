#pragma once
#include <stdint.h>
#include <string.h>

typedef struct {
  uint16_t address;
  bool failed;
  uint64_t deadline_us;
} VL53L7CX_Platform;
#define VL53L7CX_NB_TARGET_PER_ZONE (1U)
#define PROCESSOR_LITTLE_ENDIAN
#define SWAP_UINT16(x) (x)
#define SWAP_UINT32(x) (x)

uint8_t RdByte(VL53L7CX_Platform*, uint16_t, uint8_t*);
uint8_t WrByte(VL53L7CX_Platform*, uint16_t, uint8_t);
uint8_t RdMulti(VL53L7CX_Platform*, uint16_t, uint8_t*, uint32_t);
uint8_t WrMulti(VL53L7CX_Platform*, uint16_t, uint8_t*, uint32_t);
void SwapBuffer(uint8_t*, uint16_t);
uint8_t WaitMs(VL53L7CX_Platform*, uint32_t);
void BeginOperation(VL53L7CX_Platform*, uint32_t budget_us);
bool RecoverI2cBus(unsigned sda, unsigned scl);
