#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>

void Robot_Init(void);
void Robot_Process(void);
void Robot_RunDeferredTask(void);
uint32_t Robot_GetMilliseconds(void);

#endif
