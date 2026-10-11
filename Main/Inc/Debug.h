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

typedef enum {
  DEBUG_SOURCE_NONE = 0,
  DEBUG_SOURCE_UPPER,
  DEBUG_SOURCE_CONSOLE,
  DEBUG_SOURCE_WHEELS
} DebugCommandSource;

typedef enum {
  DEBUG_CONSOLE_ACCEPTED = 0,
  DEBUG_CONSOLE_BUSY,
  DEBUG_CONSOLE_INVALID
} DebugConsoleResult;

typedef struct {
  DebugMotionState state;
  uint8_t command;
  uint8_t command_sequence;
  int16_t progress;
  uint16_t remaining;
  uint8_t flags;
  DebugMotionFault fault;
  DebugCommandSource source;
  bool console_pending;
} DebugStatus;

void Debug_Init(uint32_t now_ms);
void Debug_Process(uint32_t now_ms);
DebugStatus Debug_GetStatus(void);
/* Cancel the current motion and discard commands received during a bench test. */
void Debug_SetWheelCheckActive(bool enabled);
DebugConsoleResult Debug_SubmitConsoleCommand(uint8_t opcode, int16_t arg1,
    int16_t arg2, uint16_t speed, uint8_t *sequence);

#endif
