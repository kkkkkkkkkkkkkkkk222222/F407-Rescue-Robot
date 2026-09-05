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
/* Encoder distance control with IMU heading hold; positive is forward. */
MotorDistanceStatus Motor_MoveDistance(float distance_m,
                                       float max_speed_mm_s);
/* Encoder distance control with a linear terminal speed profile. */
MotorDistanceStatus Motor_MoveDistanceLinear(float distance_m,
                                             float max_speed_mm_s,
                                             float slowdown_mm,
                                             float end_speed_mm_s);
MotorTurnStatus Motor_TurnAngle(float angle_deg);
/* yaw_tangent_mm_s is R*omega, so all three inputs use mm/s. */
void Motor_Move(float forward_mm_s, float lateral_mm_s,
                float yaw_tangent_mm_s);
/* Keep translation on one Location-assisted floor line while the chassis
 * spins. 0 deg=initial forward, 90 deg=initial left. */
bool Motor_MoveSpin(float speed_mm_s, float direction_deg,
                    float yaw_tangent_mm_s);
/* 0 deg=forward, 90 deg=left, 180 deg=backward, 270 deg=right. */
bool Motor_MoveAngle(float speed_mm_s, float angle_deg);
void Motor_Stop(void);
void Motor_Update(void);
MotorStatus Motor_GetStatus(uint8_t id);

#endif
