#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdint.h>

void Mechanism_Init(void);
void Camera_SetAngle(uint8_t angle);
uint8_t Camera_GetAngle(void);
void Claw_SetAngle(uint8_t angle);

#endif
