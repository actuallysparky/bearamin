#pragma once

#define BEARAMIN_MATRIX_SENSOR_VL53L7CX 1
// The installed VL53L7CX is rotated relative to the physical soundfield.
// This makes raw (x, y) become true-grid (7 - y, 7 - x).
#define BEARAMIN_RAW_TO_TRUE_GRID_ROTATION_QUARTERS 1
#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_X 0
#define BEARAMIN_RAW_TO_TRUE_GRID_MIRROR_Y 1
#define BEARAMIN_TRUE_GRID_ACTIVE_ZONE_MASK UINT64_MAX
#define BEARAMIN_TRUE_GRID_CENTROID_MIN_X 0
#define BEARAMIN_TRUE_GRID_CENTROID_MAX_X 7
#define BEARAMIN_TRUE_GRID_CENTROID_MIN_Y 0
#define BEARAMIN_TRUE_GRID_CENTROID_MAX_Y 7
// The installed ESPMatrix LED grid is mounted 180 degrees from
// bearamin_true_grid_v1. This is only the local true-grid consumer profile;
// it does not change the raw-to-true transform or BOV1.
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_ROTATION_QUARTERS 2
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_X 0
#define BEARAMIN_TRUE_GRID_TO_DISPLAY_MIRROR_Y 0
#define BEARAMIN_OPTICAL_CLOCK_HZ 400000
#define MATRIXCORE_MDNS_HOSTNAME "bearamin-matrixcore"
