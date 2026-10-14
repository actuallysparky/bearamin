#pragma once

// DPT Sensor Rev C PCB uses an ESP32-S3 XIAO footprint.  Its Matrix UART nets
// therefore land on the installed C5's D3/GPIO7 and D4/GPIO23 pads.
#define BEARAMIN_MATRIX_RX_PIN D3
#define BEARAMIN_MATRIX_TX_PIN D4
#define BEARAMIN_DEBUG_RX_PIN -1
// D3 is now Matrix RX; do not drive a debug UART onto the link.
#define BEARAMIN_DEBUG_TX_PIN -1
// Preserve the RevD board's physical XIAO pad locations while translating
// the original S3-labelled SD nets to the installed C5's GPIO map.
#define BEARAMIN_SD_CS_PIN D2
#define BEARAMIN_SD_SCK_PIN D8
#define BEARAMIN_SD_MISO_PIN D9
#define BEARAMIN_SD_MOSI_PIN D10
#define BEARAMIN_MDNS_HOSTNAME "bearamin"
#define BEARAMIN_SOFTAP_SSID "bearamin"
// The nearby 5 GHz BSSID repeatedly deauthenticates this RevD fixture with
// excessive retries. Prefer its 2.4 GHz peer, retaining 5 GHz as fallback.
#define BEARAMIN_WIFI_PREFER_2GHZ 1
// This level-shifted microSD module verifies writes reliably at the slower
// clock; other sidecar targets retain the 1 MHz default.
#define BEARAMIN_SD_SPI_HZ 400000
#define BEARAMIN_FIRMWARE_VERSION "bearamin-sensor-c5-revc-s3layout"
#define BEARAMIN_DEVICE_ROLE "sensor_c5_revc_s3layout"
