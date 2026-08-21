#include "mechanism.h"

#include "app_config.h"
#include "servo.h"

static uint8_t camera_angle = APP_CAMERA_WIDE_ANGLE;

void Mechanism_Init(void)
{
  Camera_SetAngle(APP_CAMERA_WIDE_ANGLE);
}

void Camera_SetAngle(uint8_t angle)
{
  if (angle < APP_CAMERA_MIN_ANGLE) {
    angle = APP_CAMERA_MIN_ANGLE;
  } else if (angle > APP_CAMERA_MAX_ANGLE) {
    angle = APP_CAMERA_MAX_ANGLE;
  }
  camera_angle = angle;
  Servo_Set(APP_CAMERA_SERVO_ID, camera_angle);
}

uint8_t Camera_GetAngle(void)
{
  return camera_angle;
}

void Claw_SetAngle(uint8_t angle)
{
  Servo_Set(APP_CLAW_SERVO_ID, angle);
}
