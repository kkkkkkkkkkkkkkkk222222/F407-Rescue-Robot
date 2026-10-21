#include "servo.h"
#include "main.h"

/* Lift/right claw/camera/left claw -> JP304/JP307/JP308/JP309. */
static const uint16_t pins[4] = {GPIO_PIN_13, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2};
static volatile uint8_t angles[4];
static bool started;

void BoardF405_ServoIRQ(void)
{
  const uint32_t events = TIM4->SR & TIM4->DIER &
      (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
  TIM4->SR = ~events;
  uint32_t set = (events & TIM_SR_UIF) ? F405_SERVO_PINS : 0U;
  uint32_t clear = 0U;
  for (uint8_t i = 0; i < 4U; ++i) {
    if (events & (TIM_SR_CC1IF << i)) { clear |= pins[i]; }
  }
  /* If delayed past a falling edge, never raise an already-expired pulse. */
  GPIOC->BSRR = (set & ~clear) | (clear << 16U);
}

void Servo_Init(void)
{
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
  TIM4->CR1 = 0U;
  TIM4->DIER = 0U;
  TIM4->PSC = 83U; /* APB1 timer 84 MHz -> 1 us per tick. */
  TIM4->ARR = 19999U;
  TIM4->CNT = 0U;
  TIM4->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
  TIM4->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
  TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = 1500U;
  for (uint8_t i = 0; i < 4U; ++i) { angles[i] = 90U; }
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR = 0U;
  started = true;
  HAL_NVIC_SetPriority(TIM4_IRQn, 1U, 0U);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
  TIM4->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE |
               TIM_DIER_CC3IE | TIM_DIER_CC4IE;
  GPIOC->BSRR = F405_SERVO_PINS;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Servo_SetAngle(uint8_t id, uint8_t angle)
{
  if (id < 1U || id > 4U) { return; }
  if (angle > 180U) { angle = 180U; }
  const uint32_t pulse = 500U + (uint32_t)angle * 2000U / 180U;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  angles[id - 1U] = angle;
  if (started) {
    /* Preload transfers at the next 20 ms frame, never midway through a pulse. */
    switch (id) {
      case 1U: TIM4->CCR1 = pulse; break;
      case 2U: TIM4->CCR2 = pulse; break;
      case 3U: TIM4->CCR3 = pulse; break;
      default: TIM4->CCR4 = pulse; break;
    }
  }
  __set_PRIMASK(mask);
}

uint8_t Servo_GetAngle(uint8_t id)
{
  return id >= 1U && id <= 4U ? angles[id - 1U] : 0U;
}
