#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  TASK_WAIT_CONFIG = 0,
  TASK_START,
  TASK_OPEN_CLAW,
  TASK_SEARCH,
  TASK_APPROACH,
  TASK_GRAB_OBSERVE,
  TASK_GRAB_RAISE_WAIT,
  TASK_GRAB_ROTATE,
  TASK_CLOSE_CLAW,
  TASK_WAIT_NAVIGATION,
  TASK_NAVIGATE,
  /* Keep wire mode values stable while omitting removed modes 11, 13, 14. */
  TASK_OPEN_FOR_RAM = 12,
  TASK_RAM_VERIFY = 15,
  /* Encoder/IMU backoff after unloading, before remote H/D return. */
  TASK_EXIT_SAFE_ZONE = 16,
  /* Turn to the remote H direction, then drive forward using remote D. */
  TASK_FACE_FIELD_CENTER = 17,
  TASK_STOPPED = 18,
  /* Values 19..23 belonged to the removed blind start-scatter sequence. */
  TASK_APPROACH_RECOVER = 24,
  TASK_REMOTE_ACTION = 25
} TaskState;

typedef enum {
  TASK_FAULT_NONE = 0,
  TASK_FAULT_REMOTE_STOP,
  TASK_FAULT_MATCH_TIMEOUT,
  TASK_FAULT_MOTOR,
  TASK_FAULT_START_TIMEOUT,
  TASK_FAULT_POSE_TIMEOUT,
  TASK_FAULT_COMMAND_TIMEOUT,
  TASK_FAULT_RAM,
  TASK_FAULT_INVALID_STATE,
  TASK_FAULT_TARGET_LOST
} TaskFault;

typedef struct {
  TaskState state;
  TaskFault fault;
  uint16_t remaining_s;
  uint8_t acknowledged_sequence;
  uint8_t last_command;
  uint8_t camera_angle;
  uint8_t audit_left_class;
  uint8_t audit_right_class;
  uint8_t audit_total_count;
  uint16_t nav_locked_heading_deg;
  bool command_received;
  bool found;
  bool claw_visible;
  bool gripper_closed;
  bool motors_active;
  bool auto_approach;
  bool audit_ready;
  bool audit_valid;
  bool nav_stale;
  bool nav_done;
  bool nav_final_push;
  bool nav_heading_locked;
} TaskStatus;

void Task_Process(uint32_t now_ms);
TaskStatus Task_GetStatus(void);

#endif
