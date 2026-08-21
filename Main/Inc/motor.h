#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_MAX_SPEED 1000

void Motor_Init(void);
void Motor_Control(uint8_t id, int16_t speed);
void Motor_StopAll(void);
int16_t Motor_GetCommand(uint8_t id);
void Motor_Update(void);
void Motor_Stop(void);
void Motor_Forward(int16_t speed);
void Motor_Back(int16_t speed);
void Motor_RotateLeft(int16_t speed);
void Motor_RotateRight(int16_t speed);
void Motor_Move(int16_t forward, int16_t lateral, int16_t rotate);
void Motor_Follow(int16_t speed, int16_t turn);
int16_t Motor_GetTarget(uint8_t id);

#endif
