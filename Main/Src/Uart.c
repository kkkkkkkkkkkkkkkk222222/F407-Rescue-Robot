#include "Uart.h"

#include <string.h>

#include "Robot.h"
#include "main.h"
#include "vision.h"

extern UART_HandleTypeDef huart3;

#define UART_RX_DMA_SIZE    64U
#define UART_TX_MAX_SIZE    64U
#define UART_TX_QUEUE_SIZE   8U
#define UART_RETRY_MS      100U

static uint8_t rx_dma[UART_RX_DMA_SIZE];
static uint16_t rx_position;
static uint32_t retry_ms;
static bool rx_active;

typedef struct {
  uint8_t data[UART_TX_MAX_SIZE];
  uint16_t size;
  uint32_t order;
  bool used;
} UartTxSlot;

static UartTxSlot tx_queue[UART_TX_QUEUE_SIZE];
static int8_t tx_active_slot;
static uint32_t tx_order;
static uint32_t tx_dropped;

static bool uart_is_ack(const uint8_t *data, uint16_t size)
{
  return (size == 4U) && (data[0] == 0xA3U) && (data[1] == 0xB3U) &&
         (data[2] == 0x01U) && (data[3] == 0xC3U);
}

static bool uart_is_odom(const UartTxSlot *slot)
{
  return slot->used && (slot->size == 15U) &&
         (slot->data[0] == 0xA3U) && (slot->data[1] == 0xB3U) &&
         (slot->data[2] == 0x15U) && (slot->data[14] == 0xC3U);
}

static bool uart_data_is_odom(const uint8_t *data, uint16_t size)
{
  return (size == VISION_FRAME_SIZE) &&
         (data[0] == VISION_FRAME_HEAD_1) &&
         (data[1] == VISION_FRAME_HEAD_2) &&
         (data[2] == VISION_MSG_ODOM) &&
         (data[VISION_FRAME_SIZE - 1U] == VISION_FRAME_TAIL);
}

static bool uart_data_is_status(const uint8_t *data, uint16_t size)
{
  return (size == VISION_FRAME_SIZE) &&
         (data[0] == VISION_FRAME_HEAD_1) &&
         (data[1] == VISION_FRAME_HEAD_2) &&
         (data[2] == VISION_MSG_STM_STATUS) &&
         (data[VISION_FRAME_SIZE - 1U] == VISION_FRAME_TAIL);
}

static void uart_start_tx(void)
{
  if ((tx_active_slot >= 0) ||
      (huart3.gState != HAL_UART_STATE_READY)) {
    return;
  }

  int8_t selected = -1;
  bool selected_ack = false;
  uint32_t selected_order = UINT32_MAX;
  for (uint8_t i = 0U; i < UART_TX_QUEUE_SIZE; ++i) {
    if (!tx_queue[i].used) {
      continue;
    }
    const bool is_ack = uart_is_ack(tx_queue[i].data, tx_queue[i].size);
    if ((selected < 0) || (is_ack && !selected_ack) ||
        ((is_ack == selected_ack) &&
         (tx_queue[i].order < selected_order))) {
      selected = (int8_t)i;
      selected_ack = is_ack;
      selected_order = tx_queue[i].order;
    }
  }
  if (selected < 0) {
    return;
  }

  if (HAL_UART_Transmit_IT(&huart3,
                           tx_queue[(uint8_t)selected].data,
                           tx_queue[(uint8_t)selected].size) == HAL_OK) {
    tx_active_slot = selected;
  }
}

static bool uart_start_rx(void)
{
  if (huart3.hdmarx == 0) {
    return false;
  }

  rx_position = 0U;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
                                  rx_dma,
                                  UART_RX_DMA_SIZE) != HAL_OK) {
    rx_active = false;
    retry_ms = Robot_GetMilliseconds() + UART_RETRY_MS;
    (void)HAL_UART_AbortReceive(&huart3);
    return false;
  }

  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  rx_active = true;
  return true;
}

static void uart_parse(uint16_t size)
{
  const uint32_t now_ms = Robot_GetMilliseconds();
  if (size == UART_RX_DMA_SIZE) {
    Vision_ParseBytes(&rx_dma[rx_position],
                      UART_RX_DMA_SIZE - rx_position, now_ms);
    rx_position = 0U;
    return;
  }

  if (size > rx_position) {
    Vision_ParseBytes(&rx_dma[rx_position], size - rx_position, now_ms);
  } else if (size < rx_position) {
    Vision_ParseBytes(&rx_dma[rx_position],
                      UART_RX_DMA_SIZE - rx_position, now_ms);
    if (size != 0U) {
      Vision_ParseBytes(rx_dma, size, now_ms);
    }
  }
  rx_position = size;
}

bool Uart_Init(void)
{
  memset(rx_dma, 0, sizeof(rx_dma));
  memset(tx_queue, 0, sizeof(tx_queue));
  rx_position = 0U;
  retry_ms = 0U;
  rx_active = false;
  tx_active_slot = -1;
  tx_order = 0U;
  tx_dropped = 0U;
  return uart_start_rx();
}

bool Uart_Send(const uint8_t *data, uint16_t size)
{
  if ((data == 0) || (size == 0U) || (size > UART_TX_MAX_SIZE)) {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  int8_t slot = -1;
  for (uint8_t i = 0U; i < UART_TX_QUEUE_SIZE; ++i) {
    if (!tx_queue[i].used) {
      slot = (int8_t)i;
      break;
    }
  }

  /* Keep control ACKs and the newest cumulative odometry snapshot. */
  if ((slot < 0) &&
      (uart_is_ack(data, size) || uart_data_is_odom(data, size) ||
       uart_data_is_status(data, size))) {
    uint32_t oldest_order = UINT32_MAX;
    for (uint8_t i = 0U; i < UART_TX_QUEUE_SIZE; ++i) {
      if (((int8_t)i != tx_active_slot) && uart_is_odom(&tx_queue[i]) &&
          (tx_queue[i].order < oldest_order)) {
        slot = (int8_t)i;
        oldest_order = tx_queue[i].order;
      }
    }
  }

  if (slot < 0) {
    ++tx_dropped;
    if (primask == 0U) {
      __enable_irq();
    }
    return false;
  }

  if (tx_queue[(uint8_t)slot].used) {
    ++tx_dropped;
  }
  memcpy(tx_queue[(uint8_t)slot].data, data, size);
  tx_queue[(uint8_t)slot].size = size;
  tx_queue[(uint8_t)slot].order = tx_order++;
  tx_queue[(uint8_t)slot].used = true;
  uart_start_tx();
  if (primask == 0U) {
    __enable_irq();
  }
  return true;
}

bool Uart_Receive(uint16_t size)
{
  if (size == 0U) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart_start_tx();
    if (primask == 0U) {
      __enable_irq();
    }
    if (!rx_active &&
        ((int32_t)(Robot_GetMilliseconds() - retry_ms) >= 0)) {
      (void)uart_start_rx();
    }
    return rx_active;
  }
  if (size > UART_RX_DMA_SIZE) {
    return false;
  }

  rx_active = true;
  uart_parse(size);
  return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  if (uart == &huart3) {
    (void)Uart_Receive(size);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if ((uart != &huart3) || (tx_active_slot < 0)) {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  tx_queue[(uint8_t)tx_active_slot].used = false;
  tx_active_slot = -1;
  uart_start_tx();
  if (primask == 0U) {
    __enable_irq();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart != &huart3) {
    return;
  }

  rx_active = false;
  retry_ms = Robot_GetMilliseconds() + UART_RETRY_MS;
  (void)HAL_UART_AbortReceive(uart);
  rx_position = 0U;
  Vision_ResetParser();
}
