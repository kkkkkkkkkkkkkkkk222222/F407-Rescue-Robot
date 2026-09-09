#ifndef DEBUG_H
#define DEBUG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  DEBUG_MOTION_IDLE = 0,
  DEBUG_MOTION_RUNNING,
  DEBUG_MOTION_DONE,
  DEBUG_MOTION_FAULT,
  DEBUG_MOTION_STOPPED
} DebugMotionState;

typedef enum {
  DEBUG_MOTION_FAULT_NONE = 0,
  DEBUG_MOTION_FAULT_INVALID = 1,
  DEBUG_MOTION_FAULT_IMU = 2,
  DEBUG_MOTION_FAULT_ODOM = 3,
  DEBUG_MOTION_FAULT_TIMEOUT = 4,
  DEBUG_MOTION_FAULT_MOTOR = 5
} DebugMotionFault;

typedef struct {
  DebugMotionState state;
  uint8_t command;
  uint8_t command_sequence;
  int16_t progress;
  uint16_t remaining;
  uint8_t flags;
  DebugMotionFault fault;
} DebugStatus;

void Debug_Init(uint32_t now_ms);
void Debug_Process(uint32_t now_ms);
DebugStatus Debug_GetStatus(void);

#endif
