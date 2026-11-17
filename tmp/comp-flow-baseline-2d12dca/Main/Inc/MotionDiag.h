#ifndef MOTION_DIAG_H
#define MOTION_DIAG_H

#include "app_config.h"
#include <stdint.h>

#if APP_ENABLE_MOTION_DEBUG_TASK
typedef enum {
  MOTION_DIAG_BOOT = 0,
  MOTION_DIAG_MAIN,
  MOTION_DIAG_IMU,
  MOTION_DIAG_CONSOLE,
  MOTION_DIAG_UART3,
  MOTION_DIAG_VISION,
  MOTION_DIAG_LCD,
  MOTION_DIAG_IDLE
} MotionDiagStage;

void MotionDiag_Init(void);
void MotionDiag_SetStage(MotionDiagStage stage, uint32_t app_ms);
void MotionDiag_TaskTick(void);
void MotionDiag_MotorTick(uint32_t app_ms);
void MotionDiag_Print(void (*send)(const char *), const char *prefix);
void MotionDiag_PrintPrevious(void (*send)(const char *));
void MotionDiag_FaultEntry(void) __attribute__((naked, noreturn));
void MotionDiag_Error(uint32_t caller) __attribute__((noreturn));
#endif

#endif
