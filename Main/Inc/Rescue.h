#ifndef RESCUE_H
#define RESCUE_H

#include <stdint.h>

typedef enum {
  RESCUE_IDLE = 0,
  RESCUE_RUNNING,
  RESCUE_DONE,
  RESCUE_FAILED
} RescueStatus;

void Rescue_Init(void);
void Rescue_Start(uint32_t now_ms);
RescueStatus Rescue_Process(uint32_t now_ms);

#endif
