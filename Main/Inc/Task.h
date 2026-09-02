#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TASK_WAIT_CONFIG = 0,
  TASK_START,
  TASK_FIND_OBJECT,
  TASK_GRAB_OBJECT,
  TASK_RETURN_SAFE,
  TASK_DROP_OBJECT,
  TASK_STOPPED
} TaskState;

typedef enum {
  TASK_DEST_NONE = 0,
  TASK_DEST_MATERIAL = 1,
  TASK_DEST_CASUALTY = 2
} TaskDestination;

typedef enum {
  TASK_DROP_ENTER = 0,
  TASK_DROP_RELEASE,
  TASK_DROP_CAMERA,
  TASK_DROP_VERIFY,
  TASK_DROP_BACK,
  TASK_DROP_RETRY_BACK
} TaskDropPhase;

typedef enum {
  TASK_FAULT_NONE = 0,
  TASK_FAULT_REMOTE_STOP,
  TASK_FAULT_MATCH_TIMEOUT,
  TASK_FAULT_MOTOR,
  TASK_FAULT_START_TIMEOUT,
  TASK_FAULT_RETURN_TIMEOUT,
  TASK_FAULT_DROP_TIMEOUT,
  TASK_FAULT_DROP_VERIFY,
  TASK_FAULT_CARGO,
  TASK_FAULT_INVALID_STATE,
  TASK_FAULT_RESCUE
} TaskFault;

typedef struct {
  TaskState state;
  TaskDestination destination;
  uint16_t remaining_s;
  uint32_t distance_mm;
  uint8_t color;
  uint8_t start_zone;
  uint8_t cargo_counts;
  uint8_t object_count;
  uint8_t nav_direction;
  uint8_t recovery_count;
  TaskDropPhase drop_phase;
  TaskFault fault;
  bool found;
  bool grabbed;
  bool cargo_valid;
  bool normal_delivered;
  bool nav_fresh;
  bool near_safe;
  bool claw_empty;
} TaskStatus;

void Task_Process(uint32_t now_ms);
TaskStatus Task_GetStatus(void);

#endif
