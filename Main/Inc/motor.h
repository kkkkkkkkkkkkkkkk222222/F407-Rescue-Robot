#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

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
/* Distance progress along the latched request; wrong-way travel is negative. */
float Motor_GetDistanceTravelledMm(void);
/* Encoder distance control with a linear terminal speed profile. */
MotorDistanceStatus Motor_MoveDistanceLinear(float distance_m,
                                             float max_speed_mm_s,
                                             float slowdown_mm,
                                             float end_speed_mm_s);
/* Encoder distance control without terminal deceleration; acceleration,
 * heading hold, target braking and distance tolerance remain active. */
MotorDistanceStatus Motor_MoveDistanceConstant(float distance_m,
                                               float speed_mm_s);
MotorTurnStatus Motor_TurnAngle(float angle_deg);
/* Caller-specific rotation speed; MotionDebug profiles and confirms rest. */
MotorTurnStatus Motor_TurnAngleSpeed(float angle_deg, float speed_mm_s);
/* Travel along the request; MotionDebug preserves signed, unwrapped progress. */
float Motor_GetTurnTravelledDeg(void);
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
/* Suspend an existing finite move without changing its encoder/yaw origin.
 * Only an explicit resume continues it; Stop still cancels it completely. */
void Motor_PauseFiniteMotion(void);
void Motor_ResumeFiniteMotion(void);
void Motor_Update(void);
MotorStatus Motor_GetStatus(uint8_t id);

#if APP_ENABLE_MOTION_DEBUG_TASK
typedef enum {
  MOTOR_TURN_PHASE_IDLE = 0, MOTOR_TURN_PHASE_DRIVE,
  MOTOR_TURN_PHASE_BRAKE, MOTOR_TURN_PHASE_SETTLE,
  MOTOR_TURN_PHASE_TRIM, MOTOR_TURN_PHASE_DONE, MOTOR_TURN_PHASE_FAULT
} MotorTurnPhase;
typedef enum {
  MOTOR_TURN_FAIL_NONE = 0, MOTOR_TURN_FAIL_IMU_STALE,
  MOTOR_TURN_FAIL_TIMEOUT, MOTOR_TURN_FAIL_OVERRUN,
  MOTOR_TURN_FAIL_WRONG_WAY, MOTOR_TURN_FAIL_TRIM_LIMIT,
  MOTOR_TURN_FAIL_IMU_NOT_READY, MOTOR_TURN_FAIL_NO_PROGRESS
} MotorTurnFailure;
/* Fixed raw-IMU goal for a debug circle batch; each leg is bounded to 368 deg. */
MotorTurnStatus Motor_DebugTurnToYaw(int64_t raw_goal_mdeg, float speed_mm_s);
typedef struct {
  int32_t raw_count[3], feedback_count[3];
  int32_t error_c100[3], p_pwm100[3], i_pwm100[3], correction_pwm100[3];
  int16_t target_mm_s[3], actual_mm_s[3], feedforward[3], output[3];
  int32_t turn_target_mdeg;
  int64_t turn_delta_mdeg;
  uint32_t turn_elapsed_ms;
  int32_t turn_error_mdeg, turn_reference_mdps, turn_command_mdps;
  uint32_t turn_imu_age_ms;
  uint16_t turn_settle_ms, turn_rest_ms, turn_boost_ms;
  int32_t turn_brake_distance_mdeg;
  uint8_t turn_boost_active, turn_boost_count, turn_boost_exit;
  uint8_t turn_phase, turn_corrections, turn_failure;
  uint8_t turn_state, slowing, brake_cycles, paused, pi_active_mask;
} MotorDebugTrace;
MotorDebugTrace Motor_GetDebugTrace(void);
/* Logical wheel polarity, open loop; bounded by the 10 ms motor scheduler. */
bool Motor_WheelCheckPulse(uint8_t id, int16_t pwm, uint16_t duration_ms);
bool Motor_WheelCheckRunning(void);
/* Air-only same-wheel/same-direction steps; 25%, 1.5 s lease, 30 s total. */
bool Motor_CalibrationStep(uint8_t id, int16_t pwm, uint16_t duration_ms);
#endif

#endif
