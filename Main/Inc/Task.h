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
  /* Safe-zone pose/visual alignment. Wire mode 11 is reported only after the
   * current alignment motion has settled. */
  TASK_ALIGN_SAFE_ZONE = 11,
  /* Keep the remaining wire mode values stable while omitting modes 13, 14. */
  TASK_OPEN_FOR_RAM = 12,
  TASK_RAM_VERIFY = 15,
  /* Encoder/IMU backoff after unloading, before remote H/D return. */
  TASK_EXIT_SAFE_ZONE = 16,
  /* Turn to the remote H direction, then drive forward using remote D. */
  TASK_FACE_FIELD_CENTER = 17,
  TASK_STOPPED = 18,
  /* Values 19..22 belonged to removed states; 23 is the explicit post-grab
   * visual audit and must not shift any existing wire mode. */
  TASK_POST_GRAB_AUDIT = 23,
  TASK_APPROACH_RECOVER = 24,
  TASK_REMOTE_ACTION = 25,
  TASK_DISPERSE_READY = 26,
  /* Values 27..38 are existing public action modes. Keep new wire modes
   * explicit so none of those established values can shift. */
  TASK_SAFE_SWEEP = 39,
  TASK_SAFE_SWEEP_DONE = 40,
  TASK_BOUNDARY_RECOVER = 41,
  TASK_SAFE_SWEEP_APPROACH = 42,
  TASK_SAFE_SWEEP_AUDIT = 43,
  TASK_SAFE_SWEEP_RETRIEVE = 44,
  TASK_SAFE_SWEEP_RETRIEVE_AUDIT = 45,
  TASK_SAFE_SWEEP_RETRIEVE_FAILED = 46
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

typedef enum {
  TASK_COMMAND_REJECT_NONE = 0,
  TASK_COMMAND_REJECT_STATE,
  TASK_COMMAND_REJECT_AUDIT,
  TASK_COMMAND_REJECT_EMPTY,
  TASK_COMMAND_REJECT_SIDE
} TaskCommandReject;

typedef struct {
  TaskState state;
  TaskFault fault;
  uint16_t remaining_s;
  uint8_t acknowledged_sequence;
  uint8_t last_command;
  uint8_t camera_angle;
  uint8_t action_command;
  uint8_t action_phase;
  uint8_t audit_left_class;
  uint8_t audit_right_class;
  uint8_t audit_total_count;
  uint8_t command_reject_reason;
  uint16_t nav_locked_heading_deg;
  bool command_received;
  bool found;
  bool claw_visible;
  bool gripper_closed;
  bool motors_active;
  bool auto_approach;
  bool audit_ready;
  bool audit_valid;
  bool audit_recheck_pending;
  bool action_done;
  bool action_disambiguate;
  bool action_green_bump;
  bool nav_stale;
  bool nav_done;
  bool nav_final_push;
  bool nav_heading_locked;
  /* Latched boundary trigger; retained after recovery for debugger/LCD QA. */
  int32_t boundary_x_mm;
  int32_t boundary_y_mm;
  int32_t boundary_yaw_mdeg;
  int32_t boundary_edge_mm;
  uint16_t boundary_heading_deg;
  bool boundary_turn_done;
  uint16_t sweep_forward_used_mm;
  uint8_t sweep_original_left_class;
  uint8_t sweep_original_right_class;
  uint8_t sweep_original_total_count;
} TaskStatus;

void Task_Process(uint32_t now_ms);
TaskStatus Task_GetStatus(void);

#endif
