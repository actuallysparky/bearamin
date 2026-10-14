// Bearamin Matrix S3 realtime-core target for the VL53L8CX-SATEL prototype.
//
// Matrix GPIO1 -> SATEL MOSI_SDA
// Matrix GPIO2 -> SATEL MCLK_SCL
// Matrix GPIO3 -> SATEL PWREN
// Matrix GPIO4 -> SATEL SPI_I2C_N (held low for I2C)
// Matrix GPIO5 -> SATEL LPn
//
// This target uses genuine 8x8 acquisition at 15 Hz. Sensor and display
// mounting are configured independently before the shared processor.

#include <Arduino.h>

#include <BearaminMatrixCoreApp.h>
