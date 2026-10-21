#ifndef F405_TEST_MAIN_H
#define F405_TEST_MAIN_H
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "app_config.h"
typedef struct {
  uint32_t CR1, DIER, PSC, ARR, CNT, CCMR1, CCMR2, CCER, CCR1, CCR2, CCR3, CCR4, EGR, SR;
} TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t IDR, BSRR; } GPIO_TypeDef;
typedef struct { uint32_t PR; } EXTI_TypeDef;
extern TIM_TypeDef test_tim4;
extern GPIO_TypeDef test_gpioa, test_gpioc;
extern EXTI_TypeDef test_exti;
extern TIM_HandleTypeDef htim8, htim12, board_motor_timers[3];
#define TIM4 (&test_tim4)
#define GPIOA (&test_gpioa)
#define GPIOC (&test_gpioc)
#define EXTI (&test_exti)
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_13 (1U << 13)
#define F405_SERVO_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13)
#define TIM_CCER_CC1E (1U << 0)
#define TIM_CCER_CC2E (1U << 4)
#define TIM_CCER_CC3E (1U << 8)
#define TIM_CCER_CC4E (1U << 12)
#define TIM_EGR_UG 1U
#define TIM_CR1_CEN 1U
#define TIM_CR1_ARPE (1U << 7)
#define TIM_CCMR1_OC1PE (1U << 3)
#define TIM_CCMR1_OC2PE (1U << 11)
#define TIM_CCMR2_OC3PE (1U << 3)
#define TIM_CCMR2_OC4PE (1U << 11)
#define TIM_SR_UIF 1U
#define TIM_SR_CC1IF (1U << 1)
#define TIM_SR_CC2IF (1U << 2)
#define TIM_SR_CC3IF (1U << 3)
#define TIM_SR_CC4IF (1U << 4)
#define TIM_DIER_UIE TIM_SR_UIF
#define TIM_DIER_CC1IE TIM_SR_CC1IF
#define TIM_DIER_CC2IE TIM_SR_CC2IF
#define TIM_DIER_CC3IE TIM_SR_CC3IF
#define TIM_DIER_CC4IE TIM_SR_CC4IF
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_ALL 0x3CU
#define TIM4_IRQn 30
#define EXTI9_5_IRQn 23
#define HAL_OK 0
#define __HAL_TIM_SET_COUNTER(t, x) ((t)->Instance->CNT = (x))
#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t m) { (void)m; }
static inline void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_SetPriority(int irq, unsigned p, unsigned s) {
  (void)irq; (void)p; (void)s;
}
static inline int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t c) {
  (void)t; (void)c; return HAL_OK;
}
static inline int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t, uint32_t c) {
  (void)t; (void)c; return HAL_OK;
}
static inline void Error_Handler(void) { abort(); }
#endif
