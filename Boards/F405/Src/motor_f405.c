#include "main.h"

void BoardF405_MotorWrite(uint8_t id, int16_t pwm, bool brake)
{
  if (id < 1U || id > 3U) { return; }
  if (pwm > APP_MOTOR_PWM_MAX_PERMILLE) { pwm = APP_MOTOR_PWM_MAX_PERMILLE; }
  if (pwm < -APP_MOTOR_PWM_MAX_PERMILLE) { pwm = -APP_MOTOR_PWM_MAX_PERMILLE; }
  TIM_TypeDef *timer = board_motor_timers[id - 1U].Instance;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  /* Gate PWM while changing direction. Each motor owns its entire timer. */
  timer->CCER &= ~TIM_CCER_CC1E;
  const bool in1 = brake || pwm > 0;
  const bool in2 = brake || pwm < 0;
  if (id == 1U) {
    GPIOA->BSRR = (in1 ? GPIO_PIN_1 : GPIO_PIN_1 << 16U) |
                  (in2 ? GPIO_PIN_2 : GPIO_PIN_2 << 16U);
  } else if (id == 2U) {
    GPIOA->BSRR = (in1 ? GPIO_PIN_3 : GPIO_PIN_3 << 16U) |
                  (in2 ? GPIO_PIN_4 : GPIO_PIN_4 << 16U);
  } else {
    GPIOA->BSRR = in1 ? GPIO_PIN_7 : GPIO_PIN_7 << 16U;
    GPIOC->BSRR = in2 ? GPIO_PIN_4 : GPIO_PIN_4 << 16U;
  }
  const uint32_t duty = (uint32_t)(pwm < 0 ? -pwm : pwm);
  /* TB6612: PWM low is short brake. IN1=IN2=0, PWM high is coast. */
  timer->CCR1 = (!brake && pwm == 0) ? 4200U : 4200U * duty / 1000U;
  timer->EGR = TIM_EGR_UG;
  timer->CCER |= TIM_CCER_CC1E;
  __set_PRIMASK(mask);
}

void BoardF405_MotorStart(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    if (HAL_TIM_PWM_Start(&board_motor_timers[id - 1U], TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    BoardF405_MotorWrite(id, 0, false);
  }
}
