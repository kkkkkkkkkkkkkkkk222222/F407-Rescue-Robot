#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int32_t accel_mg[3];
  int32_t gyro_mdps[3];
  int32_t yaw_mdeg;
  uint32_t sample_count;
  uint32_t error_count;
  uint8_t device_id;
  bool ready;
} IMUData;

bool IMU_Init(void);
void IMU_Update(uint32_t now_ms);
void IMU_ZeroYaw(void);
IMUData IMU_GetData(void);

#endif
