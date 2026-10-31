#include "DebugTelemetry.h"
#if DEBUG_TIMED_LINK
#include "main.h"
static volatile uint32_t clock_high;

void DebugTelemetry_ClockIRQ(void)
{
  if (TIM5->SR & TIM_SR_UIF) { TIM5->SR = ~TIM_SR_UIF; ++clock_high; }
}

uint64_t DebugTelemetry_NowUs(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  uint32_t high = clock_high, low = TIM5->CNT;
  if (TIM5->SR & TIM_SR_UIF) { ++high; low = TIM5->CNT; }
  __set_PRIMASK(mask);
  return ((uint64_t)high << 32) | low;
}

void DebugClock_Init(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1 = 0U; TIM5->DIER = 0U;
  TIM5->PSC = 83U; TIM5->ARR = UINT32_MAX; TIM5->CNT = 0U;
  TIM5->EGR = TIM_EGR_UG; TIM5->SR = 0U; clock_high = 0U;
  HAL_NVIC_SetPriority(TIM5_IRQn, 0U, 0U);
  HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
  TIM5->DIER = TIM_DIER_UIE; TIM5->CR1 = TIM_CR1_CEN;
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

bool DebugClock_NewEpoch(uint8_t epoch[16])
{
  __HAL_RCC_RNG_CLK_ENABLE();
  RNG->CR=RNG_CR_RNGEN;
  bool valid=true;
  uint32_t previous=0U;
  for(uint8_t i=0;i<4U;++i) {
    const uint64_t deadline=DebugTelemetry_NowUs()+10000U;
    uint32_t polls=0U;
    while(!(RNG->SR & (RNG_SR_DRDY|RNG_SR_CECS|RNG_SR_SECS)) &&
          DebugTelemetry_NowUs()<deadline && ++polls<200000U) {}
    if(!(RNG->SR&RNG_SR_DRDY) || (RNG->SR&(RNG_SR_CECS|RNG_SR_SECS))) {
      valid=false; break;
    }
    const uint32_t word=RNG->DR;
    if(i && word==previous) { valid=false; break; }
    previous=word;
    for(uint8_t b=0;b<4U;++b) epoch[i*4U+b]=(uint8_t)(word>>(24U-8U*b));
  }
  RNG->CR=0U;
  if(!valid) { for(uint8_t i=0;i<16U;++i) epoch[i]=0U; }
  return valid;
}
#endif
