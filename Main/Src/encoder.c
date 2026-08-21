#include "encoder.h"

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static TIM_HandleTypeDef *const encoder_timers[3] = {&htim1, &htim3, &htim4};
static uint16_t previous_counter[3];
static volatile int32_t position[4];
static volatile int32_t delta_10ms[4];
static volatile int32_t control_pending_delta[4];
static volatile int32_t m4_pending_delta;

void Encoder_Init(void)
{
  for (uint32_t i = 0; i < 3U; ++i) {
    __HAL_TIM_SET_COUNTER(encoder_timers[i], 0U);
    previous_counter[i] = 0U;
    (void)HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL);
  }
  position[3] = 0;
  delta_10ms[3] = 0;
  m4_pending_delta = 0;
  for (uint32_t i = 0; i < 4U; ++i) {
    control_pending_delta[i] = 0;
  }
}

void Encoder_Sample10ms(void)
{
  for (uint32_t i = 0; i < 3U; ++i) {
    const uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timers[i]);
    const int16_t wrapped_delta = (int16_t)(current - previous_counter[i]);
    previous_counter[i] = current;
    delta_10ms[i] = wrapped_delta;
    position[i] += wrapped_delta;
    control_pending_delta[i] += wrapped_delta;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const int32_t m4_delta = m4_pending_delta;
  m4_pending_delta = 0;
  if (primask == 0U) {
    __enable_irq();
  }
  delta_10ms[3] = m4_delta;
  control_pending_delta[3] += m4_delta;
}

int32_t Encoder_Get(uint8_t id)
{
  return ((id >= 1U) && (id <= 4U)) ? position[id - 1U] : 0;
}

int32_t Encoder_GetDelta10ms(uint8_t id)
{
  return ((id >= 1U) && (id <= 4U)) ? delta_10ms[id - 1U] : 0;
}

int32_t Encoder_TakeControlDelta(uint8_t id)
{
  int32_t delta;
  uint32_t primask;

  if ((id < 1U) || (id > 4U)) {
    return 0;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  delta = control_pending_delta[id - 1U];
  control_pending_delta[id - 1U] = 0;
  if (primask == 0U) {
    __enable_irq();
  }
  return delta;
}

void Encoder_Reset(uint8_t id)
{
  uint32_t primask;

  if ((id < 1U) || (id > 4U)) {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (id <= 3U) {
    __HAL_TIM_SET_COUNTER(encoder_timers[id - 1U], 0U);
    previous_counter[id - 1U] = 0U;
  } else {
    m4_pending_delta = 0;
  }
  position[id - 1U] = 0;
  delta_10ms[id - 1U] = 0;
  control_pending_delta[id - 1U] = 0;
  if (primask == 0U) {
    __enable_irq();
  }
}

void Encoder_OnExti(uint16_t gpio_pin)
{
  if (gpio_pin != M4AEXTI_Pin) {
    return;
  }

  /* One count per rising edge on channel A; swap the signs if direction is reversed. */
  const int32_t step = (HAL_GPIO_ReadPin(M4BI_GPIO_Port, M4BI_Pin) == GPIO_PIN_SET) ? -1 : 1;
  position[3] += step;
  m4_pending_delta += step;
}
