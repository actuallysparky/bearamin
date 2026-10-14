#pragma once

constexpr uint8_t kSensorSdaPin = 1;
constexpr uint8_t kSensorSclPin = 2;
constexpr uint8_t kSensorInterruptPin = 4;
constexpr uint8_t kSensorLprPin = 5;
constexpr uint8_t kRangeAdcPin = 6;
constexpr uint8_t kSlewAdcPin = 7;
constexpr uint8_t kLedPin = 14;
constexpr uint8_t kDacSclPin = 35;
constexpr uint8_t kDacSdaPin = 36;
constexpr uint8_t kPresenceGatePin = 37;
constexpr uint8_t kSpankGatePin = 38;
constexpr uint8_t kBipolarLdacPin = 39;
constexpr uint8_t kUnipolarLdacPin = 40;
constexpr uint8_t kBipolarDacAddress = 0x60;
constexpr uint8_t kUnipolarDacAddress = 0x61;
constexpr uint8_t kMatrixSourceHz = 15;
constexpr uint8_t kOutputHz = 30;
constexpr uint32_t kSensorStaleMs = 250;

constexpr bearfield::SensorMountConfig kSensorMount = {
    {bearfield::QuarterTurns::Deg180, true, false},
    UINT64_MAX,
};
constexpr bearfield::DisplayMountConfig kDisplayMount = {
    {bearfield::QuarterTurns::Deg0, false, false},
    bearfield::PixelOrder::RowMajor,
    nullptr,
};
