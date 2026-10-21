#ifndef IMU_TEST_MAIN_H
#define IMU_TEST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define IMU_CS_GPIO_Port 0
#define IMU_CS_Pin 0
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
static inline void HAL_GPIO_WritePin(int p, int n, int v) { (void)p; (void)n; (void)v; }
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#if APP_IMU_USE_UART
typedef struct { uint32_t SR, DR; } USART_TypeDef;
extern USART_TypeDef test_usart2;
#define USART2 (&test_usart2)
#define USART2_IRQn 38
#define USART_SR_RXNE (1U << 5)
#define USART_SR_ORE (1U << 3)
#define USART_SR_FE (1U << 1)
#define USART_SR_NE (1U << 2)
#define USART_SR_PE 1U
#define UART_WORDLENGTH_8B 0U
#define UART_STOPBITS_1 0U
#define UART_PARITY_NONE 0U
#define UART_MODE_TX_RX 0U
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 0U
#define UART_IT_RXNE 0U
#define UART_IT_ERR 0U
#define UART_IT_PE 0U
typedef struct {
  USART_TypeDef *Instance;
  struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
  } Init;
} UART_HandleTypeDef;
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
#define __HAL_UART_ENABLE_IT(uart, it) ((void)(uart))
static inline void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static inline void HAL_NVIC_SetPriority(int irq, unsigned p, unsigned s) {
  (void)irq; (void)p; (void)s;
}
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
#endif
#endif
