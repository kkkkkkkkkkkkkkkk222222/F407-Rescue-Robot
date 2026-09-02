#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include <stdint.h>

void Mechanism_Init(void);
void Camera_SetAngle(uint8_t angle);
uint8_t Camera_GetAngle(void);
bool Claw_Open(uint32_t now_ms);
bool Claw_Touch(uint32_t now_ms);

#endif
