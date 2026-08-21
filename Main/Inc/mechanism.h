#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdint.h>

void Mechanism_Init(void);
void Camera_SetAngle(uint8_t angle);
void Camera_ChangeAngle(int16_t change);
uint8_t Camera_GetAngle(void);
void Camera_Wide(void);
void Claw_SetAngle(uint8_t angle);
void Claw_Open(void);
void Claw_Close(void);

#endif
