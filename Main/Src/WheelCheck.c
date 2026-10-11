#include "WheelCheck.h"

#include "app_config.h"

#if APP_ENABLE_MOTION_DEBUG_TASK

#include <stdio.h>
#include <string.h>

#include "Debug.h"
#include "encoder.h"
#include "imu.h"
#include "main.h"
#include "motor.h"

extern UART_HandleTypeDef huart1;

#define RX_SIZE 64U
#define TX_SIZE 512U
#define LINE_SIZE 16U

static uint8_t rx_byte;
static volatile uint8_t rx_ring[RX_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile bool rx_overflow;
static char line[LINE_SIZE];
static uint8_t line_length;
static bool discard_line;
static uint8_t tx_ring[TX_SIZE];
static uint16_t tx_head;
static uint16_t tx_tail;
static uint8_t tx_chunk[128];
static EncoderStatus start_encoder[3];
static bool result_pending;
static bool settling;
static uint32_t settle_start_ms;
static uint8_t pulse_id;
static int pulse_sign;
static bool sequence_active;
static volatile bool cancel_requested;
static uint8_t next_id;
static uint32_t sequence_start_ms;
static uint32_t next_pulse_ms;
static bool gyro_reference_valid;
static int64_t previous_gyro_yaw_mdeg;

static const int8_t encoder_signs[3] = {
  APP_OMNI_M1_ENCODER_SIGN,
  APP_OMNI_M2_ENCODER_SIGN,
  APP_OMNI_M3_ENCODER_SIGN
};

static void wheel_send(const char *text)
{
  const size_t length = strlen(text);
  const uint16_t available = (uint16_t)((tx_tail + TX_SIZE - tx_head - 1U) % TX_SIZE);
  if (length > available) {
    return;
  }
  for (size_t i = 0U; i < length; ++i) {
    tx_ring[tx_head] = (uint8_t)text[i];
    tx_head = (uint16_t)((tx_head + 1U) % TX_SIZE);
  }
}

static void wheel_start_rx(void)
{
  if (huart1.RxState == HAL_UART_STATE_READY) {
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

static void wheel_pump_tx(void)
{
  if ((huart1.gState != HAL_UART_STATE_READY) || (tx_tail == tx_head)) {
    return;
  }
  uint16_t tail = tx_tail;
  uint16_t size = 0U;
  while ((tail != tx_head) && (size < sizeof(tx_chunk))) {
    tx_chunk[size++] = tx_ring[tail];
    tail = (uint16_t)((tail + 1U) % TX_SIZE);
  }
  if (HAL_UART_Transmit_IT(&huart1, tx_chunk, size) == HAL_OK) {
    tx_tail = tail;
  }
}

static void wheel_report(void)
{
  EncoderStatus encoder[3];
  int32_t raw[3];
  Encoder_GetAll(encoder);
  for (uint8_t i = 0U; i < 3U; ++i) {
    raw[i] = (int32_t)(encoder[i].position - start_encoder[i].position);
  }
  char text[192];
  (void)snprintf(text, sizeof(text),
      "RESULT W%u%c RAW=%ld,%ld,%ld FB=%ld,%ld,%ld\r\n",
      pulse_id, (pulse_sign > 0) ? '+' : '-',
      (long)raw[0], (long)raw[1], (long)raw[2],
      (long)(raw[0] * encoder_signs[0]),
      (long)(raw[1] * encoder_signs[1]),
      (long)(raw[2] * encoder_signs[2]));
  wheel_send(text);
  result_pending = false;
  settling = false;
}

void WheelCheck_Stop(void)
{
  cancel_requested = true;
  Motor_Stop();
}

static void wheel_finish(bool cancelled)
{
  if (result_pending) {
    wheel_report();
  }
  Debug_SetWheelCheckActive(false);
  sequence_active = false;
  cancel_requested = false;
  next_id = 0U;
  wheel_send(cancelled ? "WHEELS CANCELLED\r\n" : "WHEELS DONE\r\n");
}

static bool wheel_start_pulse(uint8_t id)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool started = false;
  if (!cancel_requested) {
    Encoder_GetAll(start_encoder);
    started = Motor_WheelCheckPulse(id, 250, 400U);
    if (started) {
      pulse_id = id;
      pulse_sign = 1;
      result_pending = true;
      settling = false;
    }
  }
  if (primask == 0U) {
    __enable_irq();
  }
  if (started) {
    char text[64];
    (void)snprintf(text, sizeof(text), "START W%u+ PWM=25%% TIME=400ms\r\n", id);
    wheel_send(text);
  }
  return started;
}

static void wheel_report_gyro(void)
{
  const IMUData imu = IMU_GetData();
  const int64_t delta = gyro_reference_valid ?
      imu.yaw_mdeg - previous_gyro_yaw_mdeg : 0LL;
  const bool delta_valid = imu.ready && gyro_reference_valid &&
      (delta >= INT32_MIN) && (delta <= INT32_MAX);
  char text[192];
  (void)snprintf(text, sizeof(text),
      "GYRO READY=%u SAMPLE=%lu YAW_MOD_MDEG=%ld DELTA_VALID=%u "
      "DELTA_MDEG=%ld HEADING_SIGN=%d\r\n",
      (unsigned int)imu.ready, (unsigned long)imu.sample_count,
      (long)(imu.yaw_mdeg % 360000LL), (unsigned int)delta_valid,
      delta_valid ? (long)delta : 0L, (int)APP_MOTOR_HEADING_OUTPUT_SIGN);
  wheel_send(text);
  previous_gyro_yaw_mdeg = imu.yaw_mdeg;
  gyro_reference_valid = imu.ready;
}

static void wheel_execute(void)
{
  if (strcmp(line, "STOP") == 0) {
    WheelCheck_Stop();
    wheel_send("STOP OK\r\n");
    return;
  }
  if (strcmp(line, "STAT") == 0) {
    wheel_send("MOTIONDEBUG: WHEELS / GYRO / STOP / STAT\r\n"
               "WHEELS: 1+ -> 2+ -> 3+, 25% 400ms EACH, 2s GAP\r\n");
    return;
  }
  if (strcmp(line, "GYRO") == 0) {
    wheel_report_gyro();
    return;
  }
  if (strcmp(line, "WHEELS") != 0) {
    wheel_send("ERR COMMAND\r\n");
    return;
  }
  if (sequence_active || cancel_requested) {
    wheel_send("BUSY: WAIT WHEELS DONE\r\n");
    return;
  }
  Debug_SetWheelCheckActive(true);
  sequence_active = true;
  sequence_start_ms = HAL_GetTick();
  next_pulse_ms = sequence_start_ms + 1000U;
  next_id = 1U;
  result_pending = false;
  settling = false;
  wheel_send("WHEELS BEGIN: FIRST WHEEL IN 1s\r\n");
}

void WheelCheck_Init(void)
{
  Motor_Stop();
  wheel_start_rx();
  wheel_send("MOTIONDEBUG USART1 115200 8N1; WHEELS / GYRO / STOP / STAT\r\n");
}

void WheelCheck_Process(void)
{
  if (rx_overflow || (huart1.ErrorCode != HAL_UART_ERROR_NONE)) {
    WheelCheck_Stop();
    if (huart1.ErrorCode != HAL_UART_ERROR_NONE) {
      (void)HAL_UART_AbortReceive(&huart1);
    }
    rx_tail = rx_head;
    rx_overflow = false;
    line_length = 0U;
    discard_line = true;
    wheel_send("ERR RX; STOPPED; SEND NEWLINE THEN RETRY\r\n");
  }
  wheel_start_rx();
  while (rx_tail != rx_head) {
    char value = (char)rx_ring[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) % RX_SIZE);
    if ((value == '\r') || (value == '\n')) {
      if (!discard_line && (line_length != 0U)) {
        line[line_length] = '\0';
        wheel_execute();
      }
      line_length = 0U;
      discard_line = false;
    } else if (!discard_line) {
      if ((value < ' ') || (value > '~') ||
          (line_length >= LINE_SIZE - 1U)) {
        WheelCheck_Stop();
        discard_line = true;
        line_length = 0U;
        wheel_send("ERR LINE; STOPPED\r\n");
      } else {
        if ((value >= 'a') && (value <= 'z')) {
          value = (char)(value - ('a' - 'A'));
        }
        line[line_length++] = value;
      }
    }
  }
  const uint32_t now_ms = HAL_GetTick();
  if (cancel_requested ||
      (sequence_active && ((uint32_t)(now_ms - sequence_start_ms) >= 10000U))) {
    wheel_finish(true);
  } else if (result_pending && !Motor_WheelCheckRunning()) {
    if (!settling) {
      settling = true;
      settle_start_ms = now_ms;
    } else if ((uint32_t)(now_ms - settle_start_ms) >= 100U) {
      wheel_report();
      if (pulse_id == 3U) {
        wheel_finish(false);
      } else {
        next_id = (uint8_t)(pulse_id + 1U);
        next_pulse_ms = now_ms + 2000U;
      }
    }
  } else if (sequence_active && !result_pending &&
             ((int32_t)(now_ms - next_pulse_ms) >= 0)) {
    (void)wheel_start_pulse(next_id);
  }
  wheel_pump_tx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1) {
    const uint8_t next = (uint8_t)((rx_head + 1U) % RX_SIZE);
    if (next == rx_tail) {
      rx_overflow = true;
    } else {
      rx_ring[rx_head] = rx_byte;
      rx_head = next;
    }
    wheel_start_rx();
  }
}

#endif
