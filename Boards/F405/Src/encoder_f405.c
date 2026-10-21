#include "encoder.h"
#include "main.h"

static TIM_HandleTypeDef *const hardware[2] = {&htim12, &htim8};
static uint16_t previous_hw[2];
static volatile uint32_t third_count, invalid_transitions;
static uint32_t previous_third;
static uint8_t previous_ab;
static volatile int64_t position[3];
static volatile int32_t delta_10ms[3];

static uint8_t read_ab(void)
{
  return (uint8_t)((GPIOC->IDR >> 8U) & 3U); /* bit0=A, bit1=B. */
}

void BoardF405_EncoderIRQ(void)
{
  const uint32_t pending = EXTI->PR & (GPIO_PIN_8 | GPIO_PIN_9);
  if (pending == 0U) { return; }
  EXTI->PR = pending;
  const uint8_t ab = read_ab();
  /* STM32 TI12 convention: TI1/A rising while TI2/B is low counts UP. */
  static const int8_t step[16] = {
     0,  1, -1,  0, -1,  0,  0,  1,
     1,  0,  0, -1,  0, -1,  1,  0
  };
  if ((previous_ab ^ ab) == 3U) { ++invalid_transitions; }
  third_count += (uint32_t)step[(previous_ab << 2U) | ab];
  previous_ab = ab;
}

uint32_t BoardF405_EncoderInvalidTransitions(void) { return invalid_transitions; }

void Encoder_Init(void)
{
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  third_count = previous_third = invalid_transitions = 0U;
  previous_ab = read_ab();
  for (uint8_t i = 0; i < 3U; ++i) { position[i] = delta_10ms[i] = 0; }
  for (uint8_t i = 0; i < 2U; ++i) {
    previous_hw[i] = 0U;
    __HAL_TIM_SET_COUNTER(hardware[i], 0U);
    if (HAL_TIM_Encoder_Start(hardware[i], TIM_CHANNEL_ALL) != HAL_OK) { Error_Handler(); }
  }
  EXTI->PR = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Encoder_Sample10ms(void)
{
  for (uint8_t i = 0; i < 2U; ++i) {
    const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(hardware[i]);
    delta_10ms[i] = (int16_t)(now - previous_hw[i]);
    previous_hw[i] = now;
    position[i] += delta_10ms[i];
  }
  const uint32_t now = third_count;
  delta_10ms[2] = (int32_t)(now - previous_third);
  previous_third = now;
  position[2] += delta_10ms[2];
}

void Encoder_GetAll(EncoderStatus status[3])
{
  if (status == 0) { return; }
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0; i < 3U; ++i) {
    status[i].position = position[i];
    status[i].delta_10ms = delta_10ms[i];
  }
  __set_PRIMASK(mask);
}
