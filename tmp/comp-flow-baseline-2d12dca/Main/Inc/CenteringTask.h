#ifndef CENTERING_TASK_H
#define CENTERING_TASK_H

#include <stdint.h>

typedef enum {
  CENTERING_WAIT_TARGET = 0,
  CENTERING_TRACKING,
  CENTERING_CENTERED,
  CENTERING_MOTOR_FAULT
} CenteringState;

typedef struct {
  CenteringState state;
  uint8_t camera_angle;
  int16_t rotation_mm_s;
} CenteringTaskStatus;

void CenteringTask_Init(uint32_t now_ms);
void CenteringTask_Process(uint32_t now_ms);
CenteringTaskStatus CenteringTask_GetStatus(void);

#endif
