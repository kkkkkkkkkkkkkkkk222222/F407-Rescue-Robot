#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t servo_id;
  uint8_t servo_angle;
  bool active;
} DebugConsoleStatus;

void DebugConsole_Init(void);
void DebugConsole_Process(void);
bool DebugConsole_IsActive(void);
DebugConsoleStatus DebugConsole_GetStatus(void);

#endif
