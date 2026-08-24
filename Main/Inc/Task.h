#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TASK_WAIT_CONFIG = 0,
  TASK_START,
  TASK_FIND_OBJECT,
  TASK_CRAB_OBJECT,
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
  TaskDropPhase drop_phase;
  bool found;
  bool grabbed;
  bool cargo_valid;
  bool normal_delivered;
  bool nav_fresh;
  bool near_safe;
  bool claw_empty;
} TaskStatus;

void Task_FindObject(uint32_t now_ms);
TaskStatus Task_GetStatus(void);

#endif
