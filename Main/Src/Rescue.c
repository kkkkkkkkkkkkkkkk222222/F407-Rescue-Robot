#include "Rescue.h"

#include "app_config.h"
#include "motor.h"

static RescueStatus status;
static uint32_t started_ms;

void Rescue_Init(void)
{
  status = RESCUE_IDLE;
  started_ms = 0U;
}

void Rescue_Start(uint32_t now_ms)
{
  Motor_Stop();
  started_ms = now_ms;
  status = RESCUE_RUNNING;
}

RescueStatus Rescue_Process(uint32_t now_ms)
{
  if (status != RESCUE_RUNNING) {
    return status;
  }

  if ((uint32_t)(now_ms - started_ms) < APP_RESCUE_REVERSE_TIME_MS) {
    Motor_Move(-APP_RESCUE_REVERSE_SPEED_MM_S, 0.0f, 0.0f);
  } else {
    Motor_Stop();
    status = RESCUE_DONE;
  }
  return status;
}
