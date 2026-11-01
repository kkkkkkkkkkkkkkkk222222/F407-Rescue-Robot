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
  uint32_t seal;
#endif
} UartTxSlot;

static UartTxSlot tx_queue[UART_TX_QUEUE_SIZE];
static int8_t tx_active_slot;
static uint32_t tx_order;
static uint32_t tx_dropped;
static uint32_t odom_tx_frames, imu_tx_frames;
#if DEBUG_TIMED_LINK
static volatile bool tx_recovery_pending;
static uint64_t tx_started_us;
static uint32_t tx_retry_ms;
static uint8_t tx_dma_buffer[UART_TX_MAX_SIZE];
static uint32_t tx_dma_seal;
static bool tx_draining;
static uint64_t tx_drain_started_us;
static uint32_t tx_quality[30];
static uint64_t last_timed_complete_us[2];
enum {
  Q_STARTED, Q_COMPLETED, Q_HAL_BUSY, Q_START_ERRORS, Q_RECOVERIES,
  Q_ABORT_ERRORS, Q_UNEXPECTED_TC, Q_SEAL_ERRORS, Q_PRECONDITION,
  Q_STUCK, Q_EXPIRED, Q_REPLACED, Q_QUEUE_HIGH, Q_BYTES_STARTED,
  Q_BYTES_COMPLETED, Q_MAX_QUEUE_US, Q_MAX_TRANSFER_US,
  Q_MAX_WHEEL_ENQUEUE_US, Q_MAX_GYRO_ENQUEUE_US,
  Q_MAX_WHEEL_COMPLETE_AGE_US, Q_MAX_GYRO_COMPLETE_AGE_US,
  Q_MAX_WHEEL_COMPLETE_GAP_US, Q_MAX_GYRO_COMPLETE_GAP_US,
  Q_DMA_ENABLED, Q_UART_DMAT, Q_UART_TX_IRQS, Q_DMA_REMAINING,
  Q_LAST_ERROR, Q_DRAIN_WAITS, Q_TC_DURING_RECOVERY
};
static uint32_t uart_seal(const uint8_t *p, uint16_t n)
{
  /* RAM lifetime guard, not the wire CRC. Keep masked work short for rear EXTI. */
  uint32_t seal=2166136261U;
  while(n--) seal=(seal^*p++)*16777619U;
  return seal;
}
static void uart_max(uint8_t index, uint64_t value)
{
  if(value>tx_quality[index]) tx_quality[index]=value>UINT32_MAX?UINT32_MAX:(uint32_t)value;
}
static uint64_t uart_read_be64(const uint8_t *p)
{ uint64_t value=0U; for(uint8_t i=0;i<8U;++i) value=(value<<8U)|p[i]; return value; }
static void uart_source_age(const uint8_t *data, uint16_t size, bool completed)
{
  if(size!=95U || data[0]!=0xd4U || data[1]!=0x4dU || (data[3]!=7U && data[3]!=8U)) return;
  const uint64_t now=DebugTelemetry_NowUs();
  uint64_t source=uart_read_be64(data+40U);
  if(data[3]==8U) {
    const uint64_t rate=uart_read_be64(data+64U);
    if(!source || !rate) return;
    if(rate<source) source=rate;
  }
  if(!source || source>now) return;
  const uint8_t type=(uint8_t)(data[3]-7U);
  uart_max((completed?Q_MAX_WHEEL_COMPLETE_AGE_US:Q_MAX_WHEEL_ENQUEUE_US)+type,now-source);
  if(completed) {
    if(last_timed_complete_us[type]) uart_max(Q_MAX_WHEEL_COMPLETE_GAP_US+type,now-last_timed_complete_us[type]);
    last_timed_complete_us[type]=now;
  }
}
static void uart_request_tx_recovery(void)
{
  if(!tx_recovery_pending) {
    tx_recovery_pending=true;
    tx_retry_ms=Robot_GetMilliseconds();
  }
}
#endif

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
#if DEBUG_TIMED_LINK
  /* One start owner: enqueue and completion IRQs never start another DMA. */
  if(tx_recovery_pending || tx_draining) { return; }
#endif
  if ((tx_active_slot >= 0)
#if !DEBUG_TIMED_LINK
      || (huart3.gState != HAL_UART_STATE_READY)
#endif
      ) {
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
      tx_queue[i].used=false; ++tx_dropped; ++tx_quality[Q_EXPIRED];
      DebugTelemetry_TxDropped(); continue;
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
  if(huart3.gState!=HAL_UART_STATE_READY || huart3.hdmatx==0 ||
      huart3.hdmatx->State!=HAL_DMA_STATE_READY ||
      (huart3.hdmatx->Instance->CR & DMA_SxCR_EN) ||
      (huart3.Instance->CR3 & USART_CR3_DMAT) ||
      (huart3.Instance->CR1 & (USART_CR1_TXEIE|USART_CR1_TCIE)) ||
      !(huart3.Instance->SR & USART_SR_TC)) {
    ++tx_quality[Q_PRECONDITION]; uart_request_tx_recovery(); return;
  }
  UartTxSlot *active=&tx_queue[(uint8_t)selected];
  if(uart_seal(active->data,active->size)!=active->seal) {
    ++tx_quality[Q_SEAL_ERRORS]; ++tx_dropped;
    if(uart_is_timed_sample(active->data,active->size)) DebugTelemetry_TxDropped();
    active->used=false; return;
  }
  memcpy(tx_dma_buffer,active->data,active->size);
  tx_dma_seal=active->seal;
  tx_active_slot=selected; /* Reserve before the peripheral can consume bytes. */
  tx_started_us=DebugTelemetry_NowUs();
  __DMB();
  const HAL_StatusTypeDef result=HAL_UART_Transmit_DMA(&huart3,tx_dma_buffer,active->size);
  if(result!=HAL_OK || huart3.hdmatx->State!=HAL_DMA_STATE_BUSY) {
    ++tx_quality[result==HAL_BUSY?Q_HAL_BUSY:Q_START_ERRORS];
    uart_request_tx_recovery(); return;
  }
  ++tx_quality[Q_STARTED]; tx_quality[Q_BYTES_STARTED]+=active->size;
  uart_max(Q_MAX_QUEUE_US,tx_started_us-active->queued_us);
  DebugTelemetry_TxStarted(tx_dma_buffer,active->size,active->queued_us);
#else
  if (HAL_UART_Transmit_IT(&huart3,
                           tx_queue[(uint8_t)selected].data,
                           tx_queue[(uint8_t)selected].size) == HAL_OK) {
    tx_active_slot = selected;
  }
#endif
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
#if DEBUG_TIMED_LINK
  tx_recovery_pending=false; tx_started_us=0U; tx_retry_ms=0U;
  tx_draining=false; tx_drain_started_us=0U;
  memset(tx_quality,0,sizeof(tx_quality));
  memset(last_timed_complete_us,0,sizeof(last_timed_complete_us));
#endif
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
    ++tx_quality[Q_REPLACED];
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
  tx_queue[(uint8_t)slot].seal=uart_seal(data,size);
  uart_source_age(data,size,false);
  uint32_t depth=0U;
  for(uint8_t i=0;i<UART_TX_QUEUE_SIZE;++i) depth+=tx_queue[i].used?1U:0U;
  uart_max(Q_QUEUE_HIGH,depth);
#endif
#if !DEBUG_TIMED_LINK
  uart_start_tx();
#endif
  if (primask == 0U) {
    __enable_irq();
  }
  return true;
}

bool Uart_Receive(uint16_t size)
{
  if (size == 0U) {
#if DEBUG_TIMED_LINK
    if(!tx_recovery_pending && tx_active_slot>=0 && DebugTelemetry_NowUs()-tx_started_us>100000U) {
      ++tx_quality[Q_STUCK]; uart_request_tx_recovery();
    }
    if(tx_recovery_pending && (int32_t)(Robot_GetMilliseconds()-tx_retry_ms)>=0) {
      /* Keep SysTick and DMA IRQs live while HAL aborts. The pending flag
       * prevents a completion callback from starting another transfer. */
      if(HAL_UART_AbortTransmit(&huart3)==HAL_OK && huart3.hdmatx &&
          (huart3.hdmatx->State==HAL_DMA_STATE_READY || HAL_DMA_Abort(huart3.hdmatx)==HAL_OK)) {
        const uint32_t mask=__get_PRIMASK(); __disable_irq();
        if(tx_active_slot>=0) {
          if(uart_is_timed_sample(tx_queue[(uint8_t)tx_active_slot].data,
                                 tx_queue[(uint8_t)tx_active_slot].size)) {
            DebugTelemetry_TxDropped();
          }
          tx_queue[(uint8_t)tx_active_slot].used=false;
          tx_active_slot=-1; ++tx_dropped;
        }
        ++tx_quality[Q_RECOVERIES];
        tx_draining=true; tx_drain_started_us=DebugTelemetry_NowUs();
        tx_recovery_pending=false; __set_PRIMASK(mask);
      } else { ++tx_quality[Q_ABORT_ERRORS]; }
      tx_retry_ms=Robot_GetMilliseconds()+100U;
    }
    /* Abort does not drain USART DR/shift registers. Let residual bytes finish
     * before reusing the dedicated DMA buffer or starting a new frame. */
    if(tx_draining) {
      if((huart3.Instance->SR & USART_SR_TC) &&
          !(huart3.hdmatx->Instance->CR & DMA_SxCR_EN)) {
        tx_draining=false;
      } else if(DebugTelemetry_NowUs()-tx_drain_started_us>=2000U) {
        ++tx_quality[Q_DRAIN_WAITS]; tx_drain_started_us=DebugTelemetry_NowUs();
      }
    }
#endif
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

void Uart_GetTimedQuality(uint32_t fields[30])
{
  const uint32_t mask=__get_PRIMASK(); __disable_irq();
#if DEBUG_TIMED_LINK
  memcpy(fields,tx_quality,sizeof(tx_quality));
  fields[Q_DMA_ENABLED]=huart3.hdmatx && (huart3.hdmatx->Instance->CR & DMA_SxCR_EN)?1U:0U;
  fields[Q_UART_DMAT]=(huart3.Instance->CR3 & USART_CR3_DMAT)?1U:0U;
  fields[Q_UART_TX_IRQS]=huart3.Instance->CR1 & (USART_CR1_TXEIE|USART_CR1_TCIE);
  fields[Q_DMA_REMAINING]=huart3.hdmatx?huart3.hdmatx->Instance->NDTR:0U;
#else
  memset(fields,0,30U*sizeof(*fields));
#endif
  __set_PRIMASK(mask);
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
  if (uart != &huart3) {
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
#if DEBUG_TIMED_LINK
  if(tx_recovery_pending || tx_draining) {
    ++tx_quality[Q_TC_DURING_RECOVERY]; __set_PRIMASK(primask); return;
  }
#endif
  if(tx_active_slot<0) {
#if DEBUG_TIMED_LINK
    ++tx_quality[Q_UNEXPECTED_TC];
#endif
    __set_PRIMASK(primask); return;
  }
  const UartTxSlot *completed = &tx_queue[(uint8_t)tx_active_slot];
#if DEBUG_TIMED_LINK
  if(!(uart->Instance->SR & USART_SR_TC) ||
      (uart->Instance->CR3 & USART_CR3_DMAT) ||
      (uart->hdmatx->Instance->CR & DMA_SxCR_EN)) {
    ++tx_quality[Q_UNEXPECTED_TC]; uart_request_tx_recovery();
    __set_PRIMASK(primask); return;
  }
  if(uart_seal(tx_dma_buffer,completed->size)!=tx_dma_seal ||
      uart_seal(completed->data,completed->size)!=completed->seal) {
    ++tx_quality[Q_SEAL_ERRORS]; uart_request_tx_recovery();
    __set_PRIMASK(primask); return;
  }
  ++tx_quality[Q_COMPLETED]; tx_quality[Q_BYTES_COMPLETED]+=completed->size;
  uart_max(Q_MAX_TRANSFER_US,DebugTelemetry_NowUs()-tx_started_us);
  uart_source_age(completed->data,completed->size,true);
  DebugTelemetry_TxCompleted(completed->data,completed->size);
#endif
  if (completed->size == VISION_FRAME_SIZE) {
    if (completed->data[2] == VISION_MSG_ODOM) { ++odom_tx_frames; }
    if (completed->data[2] == VISION_MSG_IMU) { ++imu_tx_frames; }
  }
  tx_queue[(uint8_t)tx_active_slot].used = false;
  tx_active_slot = -1;
#if !DEBUG_TIMED_LINK
  uart_start_tx();
#endif
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

#if DEBUG_TIMED_LINK
  tx_quality[Q_LAST_ERROR]=uart->ErrorCode;
  if((uart->ErrorCode & HAL_UART_ERROR_DMA) &&
      (tx_active_slot>=0 || (uart->hdmatx && uart->hdmatx->ErrorCode!=HAL_DMA_ERROR_NONE))) {
    uart_request_tx_recovery();
  }
#endif
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
