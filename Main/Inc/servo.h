#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void Servo_Init(void);
void Servo_SetAngle(uint8_t id, uint8_t angle);
uint8_t Servo_GetAngle(uint8_t id);
/* Resolve a logical target to the angle used to generate its PWM pulse. */
uint8_t Servo_OutputAngle(uint8_t id, uint8_t angle);
/* F405 maintenance adjustment. The logical angle stays available to callers. */
void Servo_SetOffset(uint8_t id, int16_t offset_deg);

#endif
