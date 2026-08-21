#ifndef TASK_H
#define TASK_H

#include <stdint.h>

typedef enum {
  TASK_CAMERA_WIDE = 0,
  TASK_WAIT_TARGET,
  TASK_CLEAR_BUMP,
  TASK_SEARCH_TARGET,
  TASK_APPROACH_TARGET,
  TASK_STOPPED
} TaskState_t;

void Task_Init(uint32_t now_ms);
void Task_Update(uint32_t now_ms);
TaskState_t Task_GetState(void);

#endif
