#include "Uart.h"
#include "DebugTelemetry.h"
#include "TaskDiag.h"
#include "DebugConsole.h"

#include <string.h>

#include "app_config.h"
#include "Robot.h"
#include "main.h"
#include "vision.h"
#include "WheelCheck.h"

extern UART_HandleTypeDef huart3;

#define UART_RX_DMA_SIZE    64U
#if DEBUG_TIMED_LINK
#define UART_TX_MAX_SIZE   192U
#else
#define UART_TX_MAX_SIZE    64U
#endif
#define UART_TX_QUEUE_SIZE   8U
#define UART_RETRY_MS      100U

static uint8_t rx_dma[UART_RX_DMA_SIZE];
static uint16_t rx_position;
static volatile uint32_t retry_ms;
static volatile bool rx_active;
static volatile bool rx_recovery_pending;
static volatile uint32_t rx_errors, last_rx_error, rx_restarts;

typedef struct {
  uint8_t data[UART_TX_MAX_SIZE];
  uint16_t size;
  uint32_t order;
  bool used;
#if DEBUG_TIMED_LINK
  uint64_t queued_us;
#endif
} UartTxSlot;

static UartTxSlot tx_queue[UART_TX_QUEUE_SIZE];
static int8_t tx_active_slot;
static uint32_t tx_order;
static uint32_t tx_dropped;
static uint32_t odom_tx_frames, imu_tx_frames;

static bool uart_is_ack(const uint8_t *data, uint16_t size)
{
  return (size == 4U) && (data[0] == 0xA3U) && (data[1] == 0xB3U) &&
         (data[2] == 0x01U) && (data[3] == 0xC3U);
}

#if DEBUG_TIMED_LINK
static bool uart_is_extended(const uint8_t *data, uint16_t size)
{ return size>=11U && data[0]==0xd4U && data[1]==0x4dU; }
static bool uart_is_timed_sample(const uint8_t *data, uint16_t size)
{ return uart_is_extended(data,size) && (data[3]==7U || data[3]==8U); }
#endif

static bool uart_is_telemetry(const UartTxSlot *slot)
{
#if DEBUG_TIMED_LINK
  if(slot->used && uart_is_timed_sample(slot->data,slot->size)) return true;
#endif
  return slot->used && (slot->size == 15U) &&
         (slot->data[0] == 0xA3U) && (slot->data[1] == 0xB3U) &&
         (slot->data[2] == VISION_MSG_ODOM || slot->data[2] == VISION_MSG_IMU) &&
         (slot->data[14] == 0xC3U);
}

static bool uart_data_is_telemetry(const uint8_t *data, uint16_t size)
{
#if DEBUG_TIMED_LINK
  if(uart_is_timed_sample(data,size)) return true;
#endif
  return (size == VISION_FRAME_SIZE) &&
         (data[0] == VISION_FRAME_HEAD_1) &&
         (data[1] == VISION_FRAME_HEAD_2) &&
         (data[2] == VISION_MSG_ODOM || data[2] == VISION_MSG_IMU) &&
         (data[VISION_FRAME_SIZE - 1U] == VISION_FRAME_TAIL);
}

static bool uart_data_is_status(const uint8_t *data, uint16_t size)
{
  if (size == 2U * VISION_FRAME_SIZE) {
    return data[2] == VISION_MSG_STATUS_CONTEXT &&
           data[VISION_FRAME_SIZE + 2U] == VISION_MSG_STM_STATUS;
  }
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
#if DEBUG_TIMED_LINK
    if(uart_is_timed_sample(tx_queue[i].data,tx_queue[i].size) &&
        DebugTelemetry_NowUs()-tx_queue[i].queued_us>8000U) {
      tx_queue[i].used=false; ++tx_dropped; DebugTelemetry_TxDropped(); continue;
    }
#endif
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

#if DEBUG_TIMED_LINK
  if (HAL_UART_Transmit_DMA(&huart3,
#else
  if (HAL_UART_Transmit_IT(&huart3,
#endif
                           tx_queue[(uint8_t)selected].data,
                           tx_queue[(uint8_t)selected].size) == HAL_OK) {
    tx_active_slot = selected;
#if DEBUG_TIMED_LINK
    DebugTelemetry_TxStarted(tx_queue[(uint8_t)selected].data,
        tx_queue[(uint8_t)selected].size,tx_queue[(uint8_t)selected].queued_us);
#endif
  }
}

static bool uart_start_rx(void)
{
  if (huart3.hdmarx == 0) {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  rx_position = 0U;
  const HAL_StatusTypeDef result = HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
                                  rx_dma,
                                  UART_RX_DMA_SIZE);
  if (result != HAL_OK) {
    rx_active = false;
    retry_ms = Robot_GetMilliseconds() + UART_RETRY_MS;
    if (primask == 0U) { __enable_irq(); }
    (void)HAL_UART_AbortReceive(&huart3);
    return false;
  }

  __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  rx_active = true;
  ++rx_restarts;
  if (primask == 0U) { __enable_irq(); }
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
  rx_recovery_pending = false;
  rx_errors = 0U;
  last_rx_error = 0U;
  rx_restarts = 0U;
  tx_active_slot = -1;
  tx_order = 0U;
  tx_dropped = 0U;
  odom_tx_frames = imu_tx_frames = 0U;
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
#if DEBUG_TIMED_LINK
  if(uart_is_timed_sample(data,size)) {
    for(uint8_t i=0;i<UART_TX_QUEUE_SIZE;++i) {
      if((int8_t)i!=tx_active_slot && tx_queue[i].used &&
          uart_is_timed_sample(tx_queue[i].data,tx_queue[i].size) &&
          tx_queue[i].data[3]==data[3]) { slot=(int8_t)i; break; }
    }
  }
#endif
  for (uint8_t i = 0U; i < UART_TX_QUEUE_SIZE; ++i) {
    if (slot < 0 && !tx_queue[i].used) {
      slot = (int8_t)i;
      break;
    }
  }

  /* Control messages can replace queued telemetry, never the active transfer. */
  if ((slot < 0) &&
      (uart_is_ack(data, size) || uart_data_is_telemetry(data, size) ||
       uart_data_is_status(data, size)
#if DEBUG_TIMED_LINK
       || uart_is_extended(data,size)
#endif
       )) {
    uint32_t oldest_order = UINT32_MAX;
    for (uint8_t i = 0U; i < UART_TX_QUEUE_SIZE; ++i) {
      if (((int8_t)i != tx_active_slot) && uart_is_telemetry(&tx_queue[i]) &&
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
#if DEBUG_TIMED_LINK
    if(uart_is_timed_sample(tx_queue[(uint8_t)slot].data,tx_queue[(uint8_t)slot].size)) {
      DebugTelemetry_TxDropped();
    }
#endif
  }
  memcpy(tx_queue[(uint8_t)slot].data, data, size);
  tx_queue[(uint8_t)slot].size = size;
  tx_queue[(uint8_t)slot].order = tx_order++;
  tx_queue[(uint8_t)slot].used = true;
#if DEBUG_TIMED_LINK
  tx_queue[(uint8_t)slot].queued_us=DebugTelemetry_NowUs();
#endif
  uart_start_tx();
  if (primask == 0U) {
    __enable_irq();
  }
  return true;
}

bool Uart_Receive(uint16_t size)
{
  if (size == 0U) {
    if (rx_recovery_pending) {
      /* HAL DMA abort may wait; keep SysTick running during that operation. */
      (void)HAL_UART_AbortReceive(&huart3);
      const uint32_t mask = __get_PRIMASK();
      __disable_irq();
      __HAL_UART_CLEAR_OREFLAG(&huart3);
      huart3.ErrorCode = HAL_UART_ERROR_NONE;
      rx_position = 0U;
      Vision_ResetParser();
      rx_recovery_pending = false;
      if (mask == 0U) { __enable_irq(); }
    }
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
  if ((size > UART_RX_DMA_SIZE) || rx_recovery_pending) {
    return false;
  }

  rx_active = true;
  uart_parse(size);
  return true;
}

UartLinkStatus Uart_GetLinkStatus(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const UartLinkStatus status = {
    .rx_errors = rx_errors, .last_rx_error = last_rx_error,
    .rx_restarts = rx_restarts, .tx_dropped = tx_dropped,
    .odom_tx_frames = odom_tx_frames, .imu_tx_frames = imu_tx_frames,
    .tx_remaining = huart3.TxXferCount, .tx_size = huart3.TxXferSize,
    .rx_active = rx_active, .rx_pending = rx_recovery_pending
  };
  if (mask == 0U) { __enable_irq(); }
  return status;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
#if APP_ENABLE_MOTION_DEBUG_TASK && APP_BOARD_F405_ROBO
  if (uart == &huart1) {
    WheelCheck_OnRxEvent();
    return;
  }
#endif
  if (uart == &huart3) {
    (void)Uart_Receive(size);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
  if (uart == &huart1) { TaskDiag_TxComplete(); return; }
#endif
#if APP_ENABLE_MOTION_DEBUG_TASK
  if (uart->Instance == USART1) {
    WheelCheck_OnTxComplete();
    return;
  }
#endif
  if ((uart != &huart3) || (tx_active_slot < 0)) {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const UartTxSlot *completed = &tx_queue[(uint8_t)tx_active_slot];
#if DEBUG_TIMED_LINK
  DebugTelemetry_TxCompleted(completed->data,completed->size);
#endif
  if (completed->size == VISION_FRAME_SIZE) {
    if (completed->data[2] == VISION_MSG_ODOM) { ++odom_tx_frames; }
    if (completed->data[2] == VISION_MSG_IMU) { ++imu_tx_frames; }
  }
  tx_queue[(uint8_t)tx_active_slot].used = false;
  tx_active_slot = -1;
  uart_start_tx();
  if (primask == 0U) {
    __enable_irq();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
  if (uart == &huart1) { DebugConsole_OnUartError(uart->ErrorCode); return; }
#endif
#if APP_ENABLE_MOTION_DEBUG_TASK
  if (uart->Instance == USART1) {
    WheelCheck_OnUartError(uart->ErrorCode);
    return;
  }
#endif
  if (uart != &huart3) {
    return;
  }

  ++rx_errors;
  last_rx_error = uart->ErrorCode;
  rx_active = false;
  rx_recovery_pending = true;
  retry_ms = Robot_GetMilliseconds() + UART_RETRY_MS;
  ATOMIC_CLEAR_BIT(uart->Instance->CR1,
      USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE);
  ATOMIC_CLEAR_BIT(uart->Instance->CR3, USART_CR3_EIE);
  __HAL_UART_CLEAR_OREFLAG(uart);
}
