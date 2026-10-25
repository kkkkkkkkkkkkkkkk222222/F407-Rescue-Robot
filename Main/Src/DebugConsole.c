#include "DebugConsole.h"
#include "TaskDiag.h"

#include <string.h>

#include "app_config.h"
#include "main.h"
#include "mechanism.h"
#include "motor.h"
#include "servo.h"

#if APP_ENABLE_RUNTIME_SERVO_DEBUG

extern UART_HandleTypeDef huart1;

#define DEBUG_RX_RING_SIZE  32U
#define DEBUG_LINE_SIZE     24U

static volatile uint8_t rx_byte;
static volatile uint8_t rx_ring[DEBUG_RX_RING_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static char line[DEBUG_LINE_SIZE];
static uint8_t line_length;
static bool line_overflow;
static volatile DebugConsoleStatus status;
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
static volatile uint32_t rx_error;
static volatile bool rx_recover;

void DebugConsole_OnUartError(uint32_t error)
{
  rx_error |= error;
  rx_recover = true;
  ATOMIC_CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE);
  ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
}
#endif

static void debug_send(const char *text)
{
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
  TaskDiag_Send(text);
#else
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text,
                          (uint16_t)strlen(text), 20U);
#endif
}

static void debug_start_rx(void)
{
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
  if (rx_recover) return;
#endif
  if (huart1.RxState == HAL_UART_STATE_READY) {
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_byte, 1U);
  }
}

static void debug_set_servo(unsigned int id, unsigned int angle)
{
  if (!status.active || (id < 1U) || (id > 4U) || (angle > 180U)) {
    debug_send("ERR\r\n");
    return;
  }

  if (id == 3U) {
    Camera_SetAngle((uint8_t)angle);
    angle = Camera_GetAngle();
  } else {
    Servo_SetAngle((uint8_t)id, (uint8_t)angle);
  }
  status.servo_id = (uint8_t)id;
  status.servo_angle = (uint8_t)angle;
  debug_send("OK\r\n");
}

static bool debug_parse_uint(const char **cursor, unsigned int *value)
{
  unsigned int parsed = 0U;
  bool has_digit = false;

  while (**cursor == ' ') {
    ++(*cursor);
  }
  while ((**cursor >= '0') && (**cursor <= '9')) {
    has_digit = true;
    parsed = parsed * 10U + (unsigned int)(**cursor - '0');
    ++(*cursor);
  }
  *value = parsed;
  return has_digit;
}

static void debug_execute(char *command)
{
  unsigned int id;
  unsigned int angle;

  for (uint8_t i = 0U; command[i] != '\0'; ++i) {
    if ((command[i] >= 'a') && (command[i] <= 'z')) {
      command[i] = (char)(command[i] - ('a' - 'A'));
    }
  }

#if APP_BOARD_F405_ROBO
#if APP_ENABLE_TASK
  if (strcmp(command, "TLOG 1") == 0 || strcmp(command, "TLOG 0") == 0) {
    TaskDiag_Enable(command[5] == '1'); debug_send("TLOG OK\r\n"); return;
  }
  if (strcmp(command, "TSTAT") == 0) { TaskDiag_RequestStatus(); return; }
  if (strcmp(command, "TDUMP") == 0) { TaskDiag_Dump(); return; }
#endif
  if (strcmp(command, "PING") == 0) {
    debug_send("PONG NORMALDEBUG F405\r\n");
    return;
  }
  if (strcmp(command, "HELP") == 0) {
    debug_send("NORMALDEBUG: PING / STAT / IMU / ENC / MOTOR / DEBUG / S id deg / RUN\r\n");
    debug_send("TASK LOG: TLOG 1 / TLOG 0 / TSTAT / TDUMP; NO MOTION\r\n");
    return;
  }
  if (strcmp(command, "STAT") == 0 || strcmp(command, "IMU") == 0 ||
      strcmp(command, "ENC") == 0 || strcmp(command, "MOTOR") == 0) {
    BoardF405_Report();
    return;
  }
#endif
  if (strcmp(command, "DEBUG") == 0) {
    status.active = true;
    __DMB();
    Motor_Stop();
    status.servo_id = 3U;
    status.servo_angle = Servo_GetAngle(3U);
    debug_send("DEBUG OK\r\n");
  } else if (status.active && (strcmp(command, "RUN") == 0)) {
    Motor_Stop();
    debug_send("RUN RESET\r\n");
    NVIC_SystemReset();
  } else if ((command[0] == 'S') && (command[1] == ' ')) {
    const char *cursor = &command[1];
    const bool id_valid = debug_parse_uint(&cursor, &id);
    const bool angle_valid = debug_parse_uint(&cursor, &angle);
    while (*cursor == ' ') {
      ++cursor;
    }
    if (id_valid && angle_valid && (*cursor == '\0')) {
      debug_set_servo(id, angle);
    } else {
      debug_send("ERR\r\n");
    }
  } else if (command[0] != '\0') {
    debug_send("ERR\r\n");
  }
}

void DebugConsole_Init(void)
{
  rx_head = 0U;
  rx_tail = 0U;
  line_length = 0U;
  line_overflow = false;
  status.active = false;
  status.servo_id = 3U;
  status.servo_angle = Servo_GetAngle(3U);
#if APP_ENABLE_RUNTIME_SERVO_DEBUG
  debug_start_rx();
#endif
}

void DebugConsole_Process(void)
{
#if APP_ENABLE_RUNTIME_SERVO_DEBUG
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
  if (rx_recover) {
    (void)HAL_UART_AbortReceive(&huart1);
    const uint32_t mask=__get_PRIMASK(); __disable_irq();
    const uint32_t error=rx_error;
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    huart1.ErrorCode=HAL_UART_ERROR_NONE;
    rx_tail=rx_head; line_length=0U; line_overflow=true;
    rx_error=0U; rx_recover=false;
    __set_PRIMASK(mask);
    TaskDiag_Event("CONSOLE_RX_ERROR",error,0,0,0,0,0);
    debug_send("ERR CONSOLE RX; SEND NEWLINE THEN RETRY; TASK UNCHANGED\r\n");
  }
#endif
  debug_start_rx();
  while (rx_tail != rx_head) {
    const char value = (char)rx_ring[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) % DEBUG_RX_RING_SIZE);

    if ((value == '\r') || (value == '\n')) {
      if (line_overflow) {
        line_overflow = false;
        line_length = 0U;
      } else if (line_length != 0U) {
        line[line_length] = '\0';
        debug_execute(line);
        line_length = 0U;
      }
    } else if (!line_overflow && (value >= ' ') && (value <= '~')) {
      if (line_length < (DEBUG_LINE_SIZE - 1U)) {
        line[line_length++] = value;
      } else {
        line_length = 0U;
        line_overflow = true;
        debug_send("ERR LONG\r\n");
      }
    }
  }
#endif
}

bool DebugConsole_IsActive(void)
{
  return status.active;
}

DebugConsoleStatus DebugConsole_GetStatus(void)
{
  const DebugConsoleStatus snapshot = {
    .servo_id = status.servo_id,
    .servo_angle = status.servo_angle,
    .active = status.active
  };
  return snapshot;
}

#endif /* APP_ENABLE_RUNTIME_SERVO_DEBUG */

#if APP_ENABLE_RUNTIME_SERVO_DEBUG
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1) {
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
    if (uart->ErrorCode != HAL_UART_ERROR_NONE) {
      DebugConsole_OnUartError(uart->ErrorCode); return;
    }
#endif
    const uint8_t next = (uint8_t)((rx_head + 1U) % DEBUG_RX_RING_SIZE);
    if (next != rx_tail) {
      rx_ring[rx_head] = rx_byte;
      rx_head = next;
    }
#if APP_BOARD_F405_ROBO && APP_ENABLE_TASK
    else { DebugConsole_OnUartError(0x80000000UL); } /* Software ring full, not hardware ORE. */
#endif
    debug_start_rx();
  }
}
#endif
