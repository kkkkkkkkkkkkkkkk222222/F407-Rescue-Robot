#ifndef MOTOR_TEST_MAIN_H
#define MOTOR_TEST_MAIN_H
#include <stdint.h>
#include <stdlib.h>
typedef struct {
  volatile uint32_t CR1;
} TIM_TypeDef;
typedef struct {
  TIM_TypeDef *Instance;
  struct { uint32_t Period; } Init;
  uint32_t compare[4];
} TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
#define HAL_OK 0
#define TIM_CR1_UDIS (1U << 1)
void MotorTest_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
#define __HAL_TIM_SET_COMPARE(timer, channel, value) MotorTest_SetCompare(timer, channel, value)
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }
static inline void Error_Handler(void) { abort(); }
uint32_t HAL_GetTick(void);
#endif
