#include "DebugTelemetry.h"
#if DEBUG_TIMED_LINK
#include "main.h"
static volatile uint32_t clock_high;
static uint64_t last_read;
static volatile uint32_t clock_faults, regressions, wraps;

void DebugTelemetry_ClockIRQ(void)
{
  if (TIM5->SR & TIM_SR_UIF) {
    TIM5->SR = ~TIM_SR_UIF; ++clock_high; ++wraps;
  }
}

uint64_t DebugTelemetry_NowUs(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  uint32_t high = clock_high, low = TIM5->CNT;
  if (TIM5->SR & TIM_SR_UIF) { ++high; low = TIM5->CNT; }
  const uint64_t now = ((uint64_t)high << 32) | low;
  if (now < last_read) { clock_faults |= 1U; ++regressions; }
  last_read = now;
  __set_PRIMASK(mask);
  /* Never clamp a discontinuity into a seemingly healthy continuous clock. */
  return now;
}

bool DebugClock_IsHealthy(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (TIM5->PSC != 83U || TIM5->ARR != UINT32_MAX ||
      !(TIM5->CR1 & TIM_CR1_CEN) || !(TIM5->DIER & TIM_DIER_UIE)) {
    clock_faults |= 2U;
  }
  if ((RCC->CR & (RCC_CR_HSERDY | RCC_CR_PLLRDY)) !=
      (RCC_CR_HSERDY | RCC_CR_PLLRDY) ||
      (RCC->CFGR & (RCC_CFGR_SWS | RCC_CFGR_HPRE | RCC_CFGR_PPRE1)) !=
      (RCC_CFGR_SWS_PLL | RCC_CFGR_PPRE1_DIV4) ||
      (RCC->PLLCFGR & 0x0F437FFFU) != (8U | (336U << 6U) | RCC_PLLCFGR_PLLSRC_HSE | (7U << 24U))) {
    clock_faults |= 4U;
  }
  const bool healthy = clock_faults == 0U;
  __set_PRIMASK(mask);
  return healthy;
}

void DebugClock_GetQuality(uint32_t fields[12])
{
  (void)DebugClock_IsHealthy();
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  fields[0]=clock_faults; fields[1]=regressions; fields[2]=wraps;
  fields[3]=RCC->CR; fields[4]=RCC->CFGR; fields[5]=RCC->PLLCFGR;
  fields[6]=TIM5->PSC; fields[7]=TIM5->ARR; fields[8]=TIM5->CR1;
  fields[9]=TIM5->DIER; fields[10]=DBGMCU->APB1FZ;
  fields[11]=UINT32_MAX; /* Board crystal total error is not specified. */
  __set_PRIMASK(mask);
}

void DebugClock_Init(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1 = 0U; TIM5->DIER = 0U;
  TIM5->PSC = 83U; TIM5->ARR = UINT32_MAX; TIM5->CNT = 0U;
  TIM5->EGR = TIM_EGR_UG; TIM5->SR = 0U; clock_high = 0U;
  last_read=0U; clock_faults=regressions=wraps=0U;
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
