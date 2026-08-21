#include "encoder.h"

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static TIM_HandleTypeDef *const encoder_timers[3] = {&htim1, &htim3, &htim4};
static uint16_t previous_counter[3];
static volatile int64_t position[3];
static volatile int32_t delta_10ms[3];
static volatile int32_t control_pending_delta[3];

void Encoder_Init(void)
{
  for (uint32_t i = 0; i < 3U; ++i) {
    __HAL_TIM_SET_COUNTER(encoder_timers[i], 0U);
    previous_counter[i] = 0U;
    position[i] = 0;
    delta_10ms[i] = 0;
    control_pending_delta[i] = 0;
    if (HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL) != HAL_OK) {
      Error_Handler();
    }
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

}

EncoderStatus Encoder_GetStatus(uint8_t id)
{
  EncoderStatus status = {0, 0};
  uint32_t primask;

  if ((id < 1U) || (id > 3U)) {
    return status;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  status.position = position[id - 1U];
  status.delta_10ms = delta_10ms[id - 1U];
  if (primask == 0U) {
    __enable_irq();
  }
  return status;
}

int32_t Encoder_TakeControlDelta(uint8_t id)
{
  int32_t delta;
  uint32_t primask;

  if ((id < 1U) || (id > 3U)) {
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
