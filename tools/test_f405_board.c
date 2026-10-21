#include <assert.h>
#include <stdio.h>
#include "../Boards/F405/Src/motor_f405.c"
#include "../Boards/F405/Src/encoder_f405.c"
#include "../Boards/F405/Src/servo_f405.c"

TIM_TypeDef test_tim4;
static TIM_TypeDef test_motors[3], test_enc1, test_enc2;
GPIO_TypeDef test_gpioa, test_gpioc;
EXTI_TypeDef test_exti;
TIM_HandleTypeDef htim8 = {&test_enc2}, htim12 = {&test_enc1};
TIM_HandleTypeDef board_motor_timers[3] = {
  {&test_motors[0]}, {&test_motors[1]}, {&test_motors[2]}
};

static void edge(uint8_t ab)
{
  GPIOC->IDR = (uint32_t)ab << 8U;
  EXTI->PR = GPIO_PIN_8 | GPIO_PIN_9;
  BoardF405_EncoderIRQ();
}

int main(void)
{
  BoardF405_MotorStart();
  for (uint8_t i = 0; i < 3; ++i) { assert(test_motors[i].CCR1 == 4200U); }
  BoardF405_MotorWrite(1, 250, false);
  assert(test_motors[0].CCR1 == 1050U);
  assert(GPIOA->BSRR == (GPIO_PIN_1 | (GPIO_PIN_2 << 16U)));
  BoardF405_MotorWrite(2, -900, false);
  assert(test_motors[1].CCR1 == 2520U && test_motors[0].CCR1 == 1050U);
  assert(GPIOA->BSRR == (GPIO_PIN_4 | (GPIO_PIN_3 << 16U)));
  BoardF405_MotorWrite(3, -250, false);
  assert(test_motors[2].CCR1 == 1050U);
  assert(GPIOA->BSRR == (GPIO_PIN_7 << 16U) && GPIOC->BSRR == GPIO_PIN_4);
  BoardF405_MotorWrite(1, 0, true);
  assert(test_motors[0].CCR1 == 0U && GPIOA->BSRR == (GPIO_PIN_1 | GPIO_PIN_2));
  BoardF405_MotorWrite(1, 0, false);
  assert(test_motors[0].CCR1 == 4200U);
  assert(GPIOA->BSRR == ((GPIO_PIN_1 | GPIO_PIN_2) << 16U));
  BoardF405_MotorWrite(0, 500, false);
  assert(test_motors[0].CCR1 == 4200U);

  Encoder_Init();
  edge(1); edge(3); edge(2); edge(0);
  Encoder_Sample10ms();
  EncoderStatus enc[3];
  Encoder_GetAll(enc);
  assert(enc[2].position == 4 && enc[2].delta_10ms == 4);
  edge(2); edge(3); edge(1); edge(0);
  test_enc1.CNT = 65535U;
  test_enc2.CNT = 2U;
  Encoder_Sample10ms();
  Encoder_GetAll(enc);
  assert(enc[2].position == 0 && enc[2].delta_10ms == -4);
  assert(enc[0].delta_10ms == -1 && enc[1].delta_10ms == 2);
  test_enc1.CNT = 0U;
  Encoder_Sample10ms();
  Encoder_GetAll(enc);
  assert(enc[0].delta_10ms == 1 && enc[0].position == 0);
  edge(3);
  assert(BoardF405_EncoderInvalidTransitions() == 1U);
  Encoder_Sample10ms();
  Encoder_GetAll(enc);
  assert(enc[2].delta_10ms == 0);

  Servo_Init();
  assert(TIM4->PSC == 83U && TIM4->ARR == 19999U && TIM4->CCR3 == 1500U);
  Servo_SetAngle(3, 105);
  assert(Servo_GetAngle(3) == 105U && TIM4->CCR3 == 1666U);
  Servo_SetAngle(4, 255);
  assert(Servo_GetAngle(4) == 180U && TIM4->CCR4 == 2500U);
  TIM4->SR = TIM_SR_UIF;
  BoardF405_ServoIRQ();
  assert(GPIOC->BSRR == F405_SERVO_PINS);
  TIM4->SR = TIM_SR_CC3IF;
  BoardF405_ServoIRQ();
  assert(GPIOC->BSRR == (GPIO_PIN_1 << 16U)); /* Camera is JP308 PC1. */
  TIM4->SR = TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC4IF;
  BoardF405_ServoIRQ();
  const uint32_t expired = GPIO_PIN_13 | GPIO_PIN_2;
  assert(GPIOC->BSRR == ((F405_SERVO_PINS & ~expired) | (expired << 16U)));
  puts("PASS F405: independent PWM, TB6612 directions/coast/brake/cap, "
       "encoder x4/sign/wrap/invalid edges, servo mapping/pulse/late IRQ");
  return 0;
}
