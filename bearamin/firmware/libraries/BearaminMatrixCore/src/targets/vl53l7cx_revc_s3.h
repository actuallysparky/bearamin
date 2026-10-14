#pragma once

#define BEARAMIN_MATRIX_SENSOR_VL53L7CX 1
// DPT Sensor Rev C routes the VL53L7CX LPR/reset net to Matrix GPIO3.
#define BEARAMIN_MATRIX_LPR_PIN 3
// Live visual calibration after the independently-corrected LED transform:
// physical left -> displayed down, right -> up, and up -> left revealed one
// extra clockwise sensor turn. Keep only the Rev C vertical mirror.
#define BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS 0
#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X 0
#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y 1
#define BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK UINT64_MAX
#define BEARAMIN_TRUE_GRID_CENTROID_MIN_X 0
#define BEARAMIN_TRUE_GRID_CENTROID_MAX_X 7
#define BEARAMIN_TRUE_GRID_CENTROID_MIN_Y 0
#define BEARAMIN_TRUE_GRID_CENTROID_MAX_Y 7
// The installed LED matrix is physically rotated 180 degrees. Keep this
// display-only so canonical telemetry and DPT coordinates stay unchanged.
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS 2
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X 0
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y 0
#define BEARAMIN_STATUS_USES_DISPLAY_PROFILE 1
#define BEARAMIN_OPTICAL_CLOCK_HZ 400000
#define MATRIXCORE_MDNS_HOSTNAME "bearamin-matrixcore"
