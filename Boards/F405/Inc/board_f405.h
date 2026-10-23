#ifndef BOARD_F405_H
#define BOARD_F405_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#if !defined(STM32F405xx)
#error "F405_ROBO must be built with the STM32F405xx device header"
#endif

#define MOTOR_PWM_KEY_Pin GPIO_PIN_12
#define MOTOR_PWM_KEY_GPIO_Port GPIOB
#define F405_SERVO_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13)

/* Logical roles retain the existing application handle names. */
extern UART_HandleTypeDef huart1; /* USART1: PB6 TX, PB7 RX, service console. */
extern UART_HandleTypeDef huart3; /* UART4: PC10 TX, PC11 RX, RDK. */
extern DMA_HandleTypeDef hdma_usart3_rx; /* UART4 RX: DMA1 Stream2 Channel4. */
extern DMA_HandleTypeDef hdma_console_rx; /* USART1 RX: DMA2 Stream2 Channel4. */
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef board_motor_timers[3];
extern const uint32_t board_motor_channels[3];

void BoardF405_MotorStart(void);
void BoardF405_MotorWrite(uint8_t id, int16_t pwm, bool brake);
void BoardF405_EncoderIRQ(void);
uint32_t BoardF405_EncoderInvalidTransitions(void);
void BoardF405_ServoIRQ(void);
void BoardF405_ServoTestSet(uint8_t id, uint8_t angle);
void BoardF405_ServoTestOff(uint8_t id);
void BoardF405_Report(void);
void BoardF405_ReportVersion(void);

#endif
