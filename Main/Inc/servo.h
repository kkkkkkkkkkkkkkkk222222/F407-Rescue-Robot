#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void Servo_Init(void);
void Servo_Set(uint8_t id, uint8_t angle);
uint8_t Servo_GetAngle(uint8_t id);

#endif
