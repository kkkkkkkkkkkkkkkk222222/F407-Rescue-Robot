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
  TASK_ALIGN_SAFE_ZONE,
  TASK_OPEN_FOR_RAM,
  TASK_RAM_BACK,
  TASK_RAM_FORWARD,
  TASK_RAM_VERIFY,
  TASK_EXIT_SAFE_ZONE,
  TASK_FACE_FIELD_CENTER,
  TASK_STOPPED,
  /* Appended to preserve every protocol-visible state number above. */
  TASK_PILE_APPROACH,
  TASK_SCATTER_POSITIVE,
  TASK_SCATTER_PAUSE,
  TASK_SCATTER_NEGATIVE,
  TASK_SCATTER_EXIT,
  TASK_APPROACH_RECOVER
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
  bool command_received;
  bool found;
  bool claw_visible;
  bool gripper_closed;
  bool motors_active;
  bool auto_approach;
} TaskStatus;

void Task_Process(uint32_t now_ms);
TaskStatus Task_GetStatus(void);

#endif
