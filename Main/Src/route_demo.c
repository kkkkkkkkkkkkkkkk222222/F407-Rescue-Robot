#include "route_demo.h"

#include <math.h>

#include "app_config.h"
#include "imu.h"
#include "Location.h"
#include "motor.h"
#include "Rescue.h"

typedef enum {
  ROUTE_DEMO_BACK_OUT = 0,
  ROUTE_DEMO_TO_MATERIAL,
  ROUTE_DEMO_TO_DROP,
  ROUTE_DEMO_RESCUE,
  ROUTE_DEMO_RETURN_MATERIAL,
  ROUTE_DEMO_FINISHED
} RouteDemoStage;

static RouteDemoStage route_stage;
static uint32_t route_stage_tick;
static uint32_t route_next_control_ms;
static bool route_back_ready;
static bool route_heading_aligned;
static bool route_fault;
static uint8_t route_arrival_count;

static bool route_motor_has_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static float route_wrap_degrees(float degrees)
{
  while (degrees > 180.0f) {
    degrees -= 360.0f;
  }
  while (degrees < -180.0f) {
    degrees += 360.0f;
  }
  return degrees;
}

static void route_set_stage(RouteDemoStage stage, uint32_t now_ms)
{
  route_stage = stage;
  route_stage_tick = now_ms;
  route_heading_aligned = false;
  route_arrival_count = 0U;
}

static void route_stop_with_fault(uint32_t now_ms)
{
  Motor_Stop();
  route_fault = true;
  route_stage_tick = now_ms;
  route_heading_aligned = false;
  route_arrival_count = 0U;
}

static bool route_align_to(const LocationPose *pose,
                           float target_x_mm, float target_y_mm,
                           uint32_t now_ms)
{
  if (route_heading_aligned) {
    return true;
  }

  const float dx = target_x_mm - (float)pose->x_mm;
  const float dy = target_y_mm - (float)pose->y_mm;
  const float target_heading_deg = atan2f(dy, dx) * 57.2957795f;
  const float current_heading_deg = (float)pose->heading_mdeg * 0.001f;
  const float turn_deg = route_wrap_degrees(
      target_heading_deg - current_heading_deg);
  const MotorTurnStatus result = Motor_TurnAngle(turn_deg);

  if (result == MOTOR_TURN_DONE) {
    route_heading_aligned = true;
    return true;
  }
  if ((result == MOTOR_TURN_FAULT) ||
      (result == MOTOR_TURN_INVALID) ||
      ((uint32_t)(now_ms - route_stage_tick) >=
       APP_LOCATION_DEMO_MOVE_TIMEOUT_MS)) {
    route_stop_with_fault(now_ms);
  }
  return false;
}

static bool route_drive_to(const LocationPose *pose,
                           float target_x_mm, float target_y_mm,
                           float maximum_speed_mm_s)
{
  const float dx = target_x_mm - (float)pose->x_mm;
  const float dy = target_y_mm - (float)pose->y_mm;
  const float distance_mm = sqrtf(dx * dx + dy * dy);

  if (distance_mm <= APP_LOCATION_DEMO_TOLERANCE_MM) {
    Motor_Stop();
    if (route_arrival_count < APP_LOCATION_DEMO_CONFIRM_CYCLES) {
      ++route_arrival_count;
    }
    return route_arrival_count >= APP_LOCATION_DEMO_CONFIRM_CYCLES;
  }
  route_arrival_count = 0U;

  float speed_mm_s = maximum_speed_mm_s;
  if (distance_mm < APP_LOCATION_DEMO_SLOWDOWN_MM) {
    const float ratio = distance_mm / APP_LOCATION_DEMO_SLOWDOWN_MM;
    speed_mm_s = APP_LOCATION_DEMO_TRAVEL_MIN_MM_S +
        (maximum_speed_mm_s - APP_LOCATION_DEMO_TRAVEL_MIN_MM_S) * ratio;
  }

  const float field_angle_deg = atan2f(dy, dx) * 57.2957795f;
  const float current_heading_deg = (float)pose->heading_mdeg * 0.001f;
  const float body_angle_deg = route_wrap_degrees(
      field_angle_deg - current_heading_deg);
  (void)Motor_MoveAngle(speed_mm_s, body_angle_deg);
  return false;
}

void RouteDemo_Init(void)
{
  route_stage = ROUTE_DEMO_BACK_OUT;
  route_stage_tick = 0U;
  route_next_control_ms = 0U;
  route_back_ready = false;
  route_heading_aligned = false;
  route_fault = false;
  route_arrival_count = 0U;
  Rescue_Init();
}

void RouteDemo_Process(uint32_t now_ms)
{
  if ((int32_t)(now_ms - route_next_control_ms) < 0) {
    return;
  }
  route_next_control_ms = now_ms + APP_LOCATION_DEMO_CONTROL_MS;

  const LocationPose pose = Location_GetPose();
  if (route_fault) {
    Motor_Stop();
    return;
  }
  if (!IMU_GetData().ready || !pose.valid) {
    route_stop_with_fault(now_ms);
    return;
  }
  if (route_stage == ROUTE_DEMO_RESCUE) {
    const RescueStatus status = Rescue_Process(now_ms);
    if (status == RESCUE_DONE) {
      route_set_stage(ROUTE_DEMO_RETURN_MATERIAL, now_ms);
    } else if (status == RESCUE_FAILED) {
      route_stop_with_fault(now_ms);
    }
    return;
  }
  if (route_motor_has_fault()) {
    route_stop_with_fault(now_ms);
    return;
  }

  switch (route_stage) {
    case ROUTE_DEMO_BACK_OUT:
      if (!route_back_ready) {
        if (Motor_MoveAngle(APP_LOCATION_DEMO_SPEED_MM_S, 180.0f)) {
          route_back_ready = true;
          route_stage_tick = now_ms;
        } else if ((uint32_t)(now_ms - route_stage_tick) >=
                   APP_LOCATION_DEMO_TIMEOUT_MS) {
          route_stop_with_fault(now_ms);
        }
      } else if ((uint32_t)(now_ms - route_stage_tick) >=
                 APP_LOCATION_DEMO_TIME_MS) {
        Motor_Stop();
        route_set_stage(ROUTE_DEMO_TO_MATERIAL, now_ms);
      }
      break;

    case ROUTE_DEMO_TO_MATERIAL:
      if ((uint32_t)(now_ms - route_stage_tick) <
          APP_LOCATION_DEMO_BRAKE_WAIT_MS) {
        Motor_Stop();
        break;
      }
      if (!route_align_to(&pose,
                          APP_LOCATION_DEMO_MATERIAL_X_MM,
                          APP_LOCATION_DEMO_MATERIAL_Y_MM,
                          now_ms)) {
        break;
      }
      if (route_drive_to(&pose,
                         APP_LOCATION_DEMO_MATERIAL_X_MM,
                         APP_LOCATION_DEMO_MATERIAL_Y_MM,
                         APP_LOCATION_DEMO_TRAVEL_MAX_MM_S)) {
        Motor_Stop();
        route_set_stage(ROUTE_DEMO_TO_DROP, now_ms);
      } else if ((uint32_t)(now_ms - route_stage_tick) >=
                 APP_LOCATION_DEMO_MOVE_TIMEOUT_MS) {
        route_stop_with_fault(now_ms);
      }
      break;

    case ROUTE_DEMO_TO_DROP:
      if (!route_align_to(&pose,
                          APP_LOCATION_DEMO_DROP_X_MM,
                          APP_LOCATION_DEMO_DROP_Y_MM,
                          now_ms)) {
        break;
      }
      if (route_drive_to(&pose,
                         APP_LOCATION_DEMO_DROP_X_MM,
                         APP_LOCATION_DEMO_DROP_Y_MM,
                         APP_LOCATION_DEMO_TRAVEL_MAX_MM_S)) {
        Motor_Stop();
        Rescue_Start(now_ms);
        route_set_stage(ROUTE_DEMO_RESCUE, now_ms);
      } else if ((uint32_t)(now_ms - route_stage_tick) >=
                 APP_LOCATION_DEMO_MOVE_TIMEOUT_MS) {
        route_stop_with_fault(now_ms);
      }
      break;

    case ROUTE_DEMO_RETURN_MATERIAL:
      if (!route_align_to(&pose,
                          APP_LOCATION_DEMO_MATERIAL_X_MM,
                          APP_LOCATION_DEMO_MATERIAL_Y_MM,
                          now_ms)) {
        break;
      }
      if (route_drive_to(&pose,
                         APP_LOCATION_DEMO_MATERIAL_X_MM,
                         APP_LOCATION_DEMO_MATERIAL_Y_MM,
                         APP_LOCATION_DEMO_RETURN_MAX_MM_S)) {
        Motor_Stop();
        route_set_stage(ROUTE_DEMO_FINISHED, now_ms);
      } else if ((uint32_t)(now_ms - route_stage_tick) >=
                 APP_LOCATION_DEMO_MOVE_TIMEOUT_MS) {
        route_stop_with_fault(now_ms);
      }
      break;

    default:
      Motor_Stop();
      break;
  }
}

bool RouteDemo_IsRunning(void)
{
  return (route_stage != ROUTE_DEMO_FINISHED) && !route_fault;
}
