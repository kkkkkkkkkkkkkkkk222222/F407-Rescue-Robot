#include "WheelCheck.h"

#include "app_config.h"

#if APP_ENABLE_WHEEL_CHECK

#include <stdio.h>
#include <string.h>

#include "encoder.h"
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

static void wheel_execute(void)
{
  if (strcmp(line, "STOP") == 0) {
    Motor_Stop();
    wheel_send("STOP OK\r\n");
    return;
  }
  if (strcmp(line, "STAT") == 0) {
    wheel_send("WHEELCHECK ONLY: W1+ W1- W2+ W2- W3+ W3- STOP\r\n"
               "OPEN LOOP 25% 400ms; NO PID/HEADING; USART3 MOTION DISABLED\r\n");
    return;
  }
  if ((strlen(line) != 3U) || (line[0] != 'W') ||
      (line[1] < '1') || (line[1] > '3') ||
      ((line[2] != '+') && (line[2] != '-'))) {
    wheel_send("ERR COMMAND\r\n");
    return;
  }
  if (result_pending) {
    wheel_send("BUSY: WAIT RESULT\r\n");
    return;
  }
  const uint8_t id = (uint8_t)(line[1] - '0');
  const int sign = (line[2] == '+') ? 1 : -1;
  Encoder_GetAll(start_encoder);
  if (!Motor_WheelCheckPulse(id, (int16_t)(sign * 250), 400U)) {
    wheel_send("BUSY: WAIT STOP\r\n");
    return;
  }
  pulse_id = id;
  pulse_sign = sign;
  result_pending = true;
  settling = false;
  wheel_send("PULSE START 25% 400ms\r\n");
}

void WheelCheck_Init(void)
{
  Motor_Stop();
  wheel_start_rx();
  wheel_send("WHEELCHECK READY USART1 115200 8N1; SEND STAT\r\n");
}

void WheelCheck_Process(void)
{
  if (rx_overflow || (huart1.ErrorCode != HAL_UART_ERROR_NONE)) {
    Motor_Stop();
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
        Motor_Stop();
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
  if (result_pending && !Motor_WheelCheckRunning()) {
    if (!settling) {
      settling = true;
      settle_start_ms = HAL_GetTick();
    } else if ((uint32_t)(HAL_GetTick() - settle_start_ms) >= 100U) {
      wheel_report();
    }
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
