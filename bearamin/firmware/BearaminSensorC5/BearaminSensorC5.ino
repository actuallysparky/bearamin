// Bearamin XIAO ESP32-C5 sidecar target for the standard MatrixCore processor.
//
// MatrixCore GPIO35 TX -> XIAO C5 D7 / GPIO12 RX
// XIAO C5 D6 / GPIO11 TX -> MatrixCore GPIO36 RX
// Common GND is required. D3 / GPIO7 is an optional dedicated debug TX.

#include <BearaminSidecarApp.h>
