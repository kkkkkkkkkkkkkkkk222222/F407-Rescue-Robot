#ifndef WHEEL_CHECK_H
#define WHEEL_CHECK_H

#include <stdint.h>

void WheelCheck_Init(void);
void WheelCheck_Process(void);
void WheelCheck_Stop(void);
void WheelCheck_OnUartError(uint32_t error);
void WheelCheck_OnTxComplete(void);
void WheelCheck_OnRxEvent(void);

#endif
