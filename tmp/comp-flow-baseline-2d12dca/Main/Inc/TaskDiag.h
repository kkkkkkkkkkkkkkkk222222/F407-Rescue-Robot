#ifndef TASK_DIAG_H
#define TASK_DIAG_H
#include "app_config.h"
#include <stdint.h>
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
void TaskDiag_Event(const char *kind, uint32_t a, uint32_t b, uint32_t c,
                    uint32_t d, uint32_t e, uint32_t f);
void TaskDiag_Process(void);
void TaskDiag_Send(const char *text);
void TaskDiag_TxComplete(void);
void TaskDiag_MotorTick(void);
void TaskDiag_TaskTick(void);
void TaskDiag_Enable(uint8_t enabled);
void TaskDiag_RequestStatus(void);
void TaskDiag_Dump(void);
#else
#define TaskDiag_Event(...) ((void)0)
#define TaskDiag_Process() ((void)0)
#define TaskDiag_MotorTick() ((void)0)
#define TaskDiag_TaskTick() ((void)0)
#endif
#endif
