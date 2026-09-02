#include "servo.h"

#include "main.h"

extern TIM_HandleTypeDef htim8;

static const uint32_t channels[4] = {
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};
static const uint8_t initial_angles[4] = {90U, 52U, 90U, 128U};
static volatile uint8_t angles[4];

void Servo_Init(void)
{
  for (uint32_t i = 0; i < 4U; ++i) {
    if (HAL_TIM_PWM_Start(&htim8, channels[i]) != HAL_OK) {
      Error_Handler();
    }
    Servo_SetAngle((uint8_t)(i + 1U), initial_angles[i]);
  }
}

void Servo_SetAngle(uint8_t id, uint8_t angle)
{
  if ((id < 1U) || (id > 4U)) {
    return;
  }
  if (angle > 180U) {
    angle = 180U;
  }

  const uint32_t pulse_us = 500U + (((uint32_t)angle * 2000U) / 180U);
  __HAL_TIM_SET_COMPARE(&htim8, channels[id - 1U], pulse_us);
  angles[id - 1U] = angle;
}

uint8_t Servo_GetAngle(uint8_t id)
{
  return ((id >= 1U) && (id <= 4U)) ? angles[id - 1U] : 0U;
}
