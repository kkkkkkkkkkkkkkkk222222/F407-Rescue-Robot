#include "servo.h"
#include "main.h"
#include "TaskDiag.h"

/* Lift/right claw/camera/left claw -> JP304/JP307/JP308/JP309. */
static const uint16_t pins[4] = {GPIO_PIN_13, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2};
static volatile uint8_t angles[4];
static bool started;
static volatile uint16_t enabled_pins, pending_pins;
#if APP_ENABLE_MOTION_DEBUG_TASK
static volatile BoardF405_ServoTiming timing[4];
static uint16_t rise_tick[4];
static uint32_t rise_cycle[4];
static bool pulse_started[4], period_valid[4];
static uint8_t timing_skip[4];

static void servo_timing_reset(uint8_t i)
{
  timing[i] = (BoardF405_ServoTiming){.high_min_us = UINT16_MAX,
                                    .period_min_us = UINT16_MAX};
  pulse_started[i] = period_valid[i] = false;
  timing_skip[i] = 2U;
}

void BoardF405_ServoTimingRead(BoardF405_ServoTiming result[4], bool reset)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < 4U; ++i) {
    if (reset) { servo_timing_reset(i); }
    result[i] = timing[i];
    result[i].angle = angles[i];
    result[i].enabled = ((enabled_pins | pending_pins) & pins[i]) != 0U;
  }
  __set_PRIMASK(mask);
}

static void servo_record_edges(uint16_t set, uint16_t clear, uint16_t tick,
                               uint32_t cycle)
{
  for (uint8_t i = 0U; i < 4U; ++i) {
    if ((set & clear & pins[i]) != 0U) { ++timing[i].late_edges; }
    if ((set & (uint16_t)~clear & pins[i]) != 0U) {
      if (timing_skip[i]) {
        --timing_skip[i];
        period_valid[i] = false;
      } else {
        if (tick > timing[i].rise_delay_max_us) { timing[i].rise_delay_max_us = tick; }
        if (period_valid[i]) {
          const uint32_t us = (cycle - rise_cycle[i]) / (SystemCoreClock / 1000000U);
          const uint16_t period = us > UINT16_MAX ? UINT16_MAX : (uint16_t)us;
          if (period < timing[i].period_min_us) { timing[i].period_min_us = period; }
          if (period > timing[i].period_max_us) { timing[i].period_max_us = period; }
        }
        period_valid[i] = true;
      }
      rise_tick[i] = tick;
      rise_cycle[i] = cycle;
      pulse_started[i] = timing_skip[i] == 0U && period_valid[i];
    }
    if ((clear & pins[i]) != 0U && pulse_started[i]) {
      const uint16_t width = (tick + 20000U - rise_tick[i]) % 20000U;
      const uint16_t expected = 500U + (uint32_t)angles[i] * 2000U / 180U;
      const uint16_t delay = (tick + 20000U - expected) % 20000U;
      if (width < timing[i].high_min_us) { timing[i].high_min_us = width; }
      if (width > timing[i].high_max_us) { timing[i].high_max_us = width; }
      if (delay > timing[i].fall_delay_max_us) { timing[i].fall_delay_max_us = delay; }
      ++timing[i].pulses;
      pulse_started[i] = false;
    }
  }
}
#endif

void BoardF405_ServoIRQ(void)
{
  const uint32_t events = TIM4->SR & TIM4->DIER &
      (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
  TIM4->SR = ~events;
  uint32_t set = (events & TIM_SR_UIF) ? enabled_pins : 0U;
  if (events & TIM_SR_UIF) {
    /* Delay a newly enabled output until its preloaded compare is active. */
    enabled_pins |= pending_pins;
    pending_pins = 0U;
  }
  uint32_t clear = 0U;
  for (uint8_t i = 0; i < 4U; ++i) {
    if (events & (TIM_SR_CC1IF << i)) { clear |= pins[i]; }
  }
  /* If delayed past a falling edge, never raise an already-expired pulse. */
#if APP_ENABLE_MOTION_DEBUG_TASK
  const uint32_t cycle = DWT->CYCCNT;
  const uint16_t tick = (uint16_t)TIM4->CNT;
#endif
  GPIOC->BSRR = (set & ~clear) | (clear << 16U);
#if APP_ENABLE_MOTION_DEBUG_TASK
  servo_record_edges((uint16_t)set, (uint16_t)clear, tick, cycle);
#endif
}

static void servo_start(uint16_t outputs)
{
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
  enabled_pins = outputs;
  pending_pins = 0U;
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
#if APP_ENABLE_MOTION_DEBUG_TASK
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  for (uint8_t i = 0U; i < 4U; ++i) { servo_timing_reset(i); }
#endif
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR = 0U;
  started = true;
  HAL_NVIC_SetPriority(TIM4_IRQn, 1U, 0U);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
  TIM4->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE |
               TIM_DIER_CC3IE | TIM_DIER_CC4IE;
  GPIOC->BSRR = outputs | ((uint32_t)(F405_SERVO_PINS & ~outputs) << 16U);
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Servo_Init(void) { servo_start(F405_SERVO_PINS); }

void BoardF405_ServoTestSet(uint8_t id, uint8_t angle)
{
  if (id < 1U || id > 4U || angle > 180U) { return; }
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (!started) { servo_start(0U); }
  enabled_pins &= (uint16_t)~pins[id - 1U];
  GPIOC->BSRR = (uint32_t)pins[id - 1U] << 16U;
  Servo_SetAngle(id, angle);
  pending_pins |= pins[id - 1U];
  __set_PRIMASK(mask);
}

void BoardF405_ServoTestOff(uint8_t id)
{
  if (id > 4U) { return; }
  const uint16_t off = id == 0U ? F405_SERVO_PINS : pins[id - 1U];
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  enabled_pins &= (uint16_t)~off;
  pending_pins &= (uint16_t)~off;
  GPIOC->BSRR = (uint32_t)off << 16U;
  __set_PRIMASK(mask);
}

void Servo_SetAngle(uint8_t id, uint8_t angle)
{
  if (id < 1U || id > 4U) { return; }
  if (angle > 180U) { angle = 180U; }
  TaskDiag_Event("SERVO_TARGET", id, angle, 0, 0, 0, 0);
  const uint32_t pulse = 500U + (uint32_t)angle * 2000U / 180U;
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  angles[id - 1U] = angle;
#if APP_ENABLE_MOTION_DEBUG_TASK
  servo_timing_reset(id - 1U);
#endif
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
