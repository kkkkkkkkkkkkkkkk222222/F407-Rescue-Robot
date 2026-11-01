#include "main.h"
#include "DebugTelemetry.h"
#include <assert.h>
#include <stdio.h>
TestTimer test_timer;
TestRcc test_rcc={.CR=RCC_CR_HSERDY|RCC_CR_PLLRDY,
  .CFGR=RCC_CFGR_SWS_PLL|RCC_CFGR_PPRE1_DIV4,
  .PLLCFGR=8U|(336U<<6U)|RCC_PLLCFGR_PLLSRC_HSE|(7U<<24U)};
TestRng test_rng;
TestDebug test_debug;
void DebugClock_Init(void);
int main(void)
{
  DebugClock_Init();assert(DebugClock_IsHealthy());
  test_timer.CNT=UINT32_MAX-10U;assert(DebugTelemetry_NowUs()==UINT32_MAX-10U);
  test_timer.CNT=20U;test_timer.SR=TIM_SR_UIF;
  assert(DebugTelemetry_NowUs()==((uint64_t)1U<<32U)+20U);
  DebugTelemetry_ClockIRQ();test_timer.CNT=30U;
  assert(DebugTelemetry_NowUs()==((uint64_t)1U<<32U)+30U);
  uint32_t fields[12];DebugClock_GetQuality(fields);
  assert(fields[0]==0 && fields[2]==1 && fields[11]==UINT32_MAX);
  test_timer.CNT=10U;(void)DebugTelemetry_NowUs();assert(!DebugClock_IsHealthy());
  DebugClock_GetQuality(fields);assert(fields[0]&1U);assert(fields[1]==1);
  test_timer.CNT=100U;(void)DebugTelemetry_NowUs();assert(!DebugClock_IsHealthy());
  DebugClock_Init();test_timer.PSC=84U;assert(!DebugClock_IsHealthy());
  DebugClock_Init();test_rcc.CFGR=0;assert(!DebugClock_IsHealthy());
  puts("HOST MODEL ONLY: actual DebugClock.c nominal divider, pending/serviced low32 wrap, regression quarantine, timer/clock-tree change checks passed; drift remains unknown.");
}
