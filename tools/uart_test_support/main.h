#ifndef UART_TEST_MAIN_H
#define UART_TEST_MAIN_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
enum { HAL_UART_STATE_READY=1, HAL_UART_STATE_BUSY_TX=2,
       HAL_DMA_STATE_READY=1, HAL_DMA_STATE_BUSY=2 };
enum { USART_CR1_RXNEIE=1, USART_CR1_PEIE=2, USART_CR1_IDLEIE=4,
       USART_CR1_TXEIE=8, USART_CR1_TCIE=16, USART_CR3_EIE=1,
       USART_CR3_DMAT=2, USART_SR_TC=1, DMA_SxCR_EN=1, DMA_IT_HT=1,
       HAL_UART_ERROR_NONE=0, HAL_UART_ERROR_DMA=16, HAL_DMA_ERROR_NONE=0 };
typedef struct { uint32_t SR, CR1, CR3; } USART_TypeDef;
typedef struct { uint32_t CR, NDTR; } DMA_Stream_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; unsigned State, ErrorCode; } DMA_HandleTypeDef;
typedef struct {
  USART_TypeDef *Instance;
  DMA_HandleTypeDef *hdmatx, *hdmarx;
  unsigned gState, ErrorCode;
  uint16_t TxXferCount, TxXferSize;
} UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart3;
extern USART_TypeDef console_regs;
#define USART1 (&console_regs)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
static inline void __DMB(void) {}
#define __HAL_DMA_DISABLE_IT(h, i) ((void)(h),(void)(i))
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
#define ATOMIC_CLEAR_BIT(reg, bits) ((reg)&=~(bits))
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *,const uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *);
#endif
