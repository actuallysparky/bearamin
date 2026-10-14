#pragma once

#define BEARAMIN_MATRIX_RX_PIN 4
#define BEARAMIN_MATRIX_TX_PIN 5
#define BEARAMIN_DEBUG_RX_PIN -1
// D8/GPIO7 is now the SD SPI clock.  Use the XIAO USB serial console.
#define BEARAMIN_DEBUG_TX_PIN -1
#define BEARAMIN_SD_CS_PIN 3
#define BEARAMIN_SD_SCK_PIN 7
#define BEARAMIN_SD_MISO_PIN 8
#define BEARAMIN_SD_MOSI_PIN 9
#define BEARAMIN_MDNS_HOSTNAME "bearamin"
#define BEARAMIN_SOFTAP_SSID "bearamin"
#define BEARAMIN_FIRMWARE_VERSION "bearamin-sidecar-matrix-poc"
#define BEARAMIN_DEVICE_ROLE "xiao_sidecar"
#define BEARAMIN_BOOT_LABEL "Bearamin XIAO sidecar POC"
#define BEARAMIN_WIFI_PREFER_5GHZ 0
