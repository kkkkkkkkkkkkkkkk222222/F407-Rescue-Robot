#ifndef CLOCK_TEST_MAIN_H
#define CLOCK_TEST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t CR1,DIER,PSC,ARR,CNT,EGR,SR; } TestTimer;
typedef struct { uint32_t CR,CFGR,PLLCFGR; } TestRcc;
typedef struct { uint32_t CR,SR,DR; } TestRng;
typedef struct { uint32_t APB1FZ; } TestDebug;
extern TestTimer test_timer;
extern TestRcc test_rcc;
extern TestRng test_rng;
extern TestDebug test_debug;
#define TIM5 (&test_timer)
#define RCC (&test_rcc)
#define RNG (&test_rng)
#define DBGMCU (&test_debug)
#define TIM_SR_UIF 1U
#define TIM_CR1_CEN 1U
#define TIM_DIER_UIE 1U
#define TIM_EGR_UG 1U
#define RCC_CR_HSERDY 0x00020000U
#define RCC_CR_PLLRDY 0x02000000U
#define RCC_CFGR_SWS 0x0000000cU
#define RCC_CFGR_SWS_PLL 0x00000008U
#define RCC_CFGR_HPRE 0x000000f0U
#define RCC_CFGR_PPRE1 0x00001c00U
#define RCC_CFGR_PPRE1_DIV4 0x00001400U
#define RCC_PLLCFGR_PLLSRC_HSE 0x00400000U
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define TIM5_IRQn 50U
#define __HAL_RCC_TIM5_CLK_ENABLE() ((void)0)
#define __HAL_RCC_RNG_CLK_ENABLE() ((void)0)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
#define HAL_NVIC_SetPriority(a,b,c) ((void)(a),(void)(b),(void)(c))
#define HAL_NVIC_ClearPendingIRQ(a) ((void)(a))
#define HAL_NVIC_EnableIRQ(a) ((void)(a))
#endif
