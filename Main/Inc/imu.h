#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  IMU_INIT_ID_ERROR = 0,
  IMU_INIT_RESET_ERROR,
  IMU_INIT_CONFIG_ERROR,
  IMU_INIT_CALIBRATION_ERROR,
  IMU_INIT_OK
} IMUInitResult;

typedef struct {
  int32_t accel_mg[3];
  int32_t gyro_mdps[3];
  int64_t yaw_mdeg;
  uint32_t sample_count;
  uint32_t error_count;
  uint8_t device_id;
  IMUInitResult init_result;
  bool ready;
} IMUData;

bool IMU_Init(void);
void IMU_Update(uint32_t now_ms);
void IMU_ZeroYaw(void);
IMUData IMU_GetData(void);

#endif
