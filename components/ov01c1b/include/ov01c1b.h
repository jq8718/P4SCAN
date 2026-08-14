#pragma once

#include "esp_cam_sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OV01C1B_SCCB_ADDR            0x10
#define OV01C1B_8BIT_WRITE_ADDR      (OV01C1B_SCCB_ADDR << 1)
#define OV01C1B_CHIP_ID              0x56014311
#define OV01C1B_PID                  0x5601
#define OV01C1B_SENSOR_NAME          "OV01C1B"

esp_cam_sensor_device_t *ov01c1b_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
