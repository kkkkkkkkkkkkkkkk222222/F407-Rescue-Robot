#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int16_t command;
  int16_t target;
  int16_t measured_speed_mm_s;
  int16_t target_speed_mm_s;
  bool direction_fault;
  bool stall_fault;
} MotorStatus;

typedef enum {
  MOTOR_DISTANCE_IDLE = 0,
  MOTOR_DISTANCE_RUNNING,
  MOTOR_DISTANCE_DONE,
  MOTOR_DISTANCE_FAULT,
  MOTOR_DISTANCE_INVALID
} MotorDistanceStatus;

typedef enum {
  MOTOR_TURN_IDLE = 0,
  MOTOR_TURN_RUNNING,
  MOTOR_TURN_DONE,
  MOTOR_TURN_FAULT,
  MOTOR_TURN_INVALID
} MotorTurnStatus;

void Motor_Init(void);
void Motor_SetSpeed(float target_speed, uint8_t id);
MotorDistanceStatus Go_distance(float distance_m);
MotorTurnStatus Motor_TurnAngle(float angle_deg);
void Motor_Move(float forward_mm_s, float lateral_mm_s, float rotate_mm_s);
void Motor_MoveAngle(float speed_mm_s, float angle_deg);
void Motor_Stop(void);
void Motor_Update(void);
MotorStatus Motor_GetStatus(uint8_t id);

#endif
