#pragma once

// Single-core cooperative background work, including sensor initialization waits.
// Must not initiate sensor/I2C work: this function is called from that work.
void bearamin_service_usb();
