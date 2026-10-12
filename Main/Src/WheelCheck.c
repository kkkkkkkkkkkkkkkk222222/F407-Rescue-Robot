#include "WheelCheck.h"

#include "app_config.h"

#if APP_ENABLE_MOTION_DEBUG_TASK

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Debug.h"
#include "encoder.h"
#include "imu.h"
#include "Location.h"
#include "main.h"
#include "motor.h"
#include "vision.h"

extern UART_HandleTypeDef huart1;

#define RX_SIZE 128U
#define TX_SIZE 1024U
#define LINE_SIZE 64U
#define TX_TIMEOUT_MS 250U
#define LINE_TIMEOUT_MS 1000U

static uint8_t rx_byte;
static volatile uint8_t rx_ring[RX_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile bool rx_overflow;
static volatile uint32_t rx_error_pending;
static volatile bool rx_recovery_pending;
static volatile uint32_t rx_bytes;
static volatile uint32_t last_rx_byte_ms;
static uint32_t rx_lines;
static uint32_t rx_errors;
static uint32_t rx_last_error;
static uint32_t rx_overflows;
static volatile uint32_t rx_start_failures;
static uint32_t line_timeouts;
static char line[LINE_SIZE];
static uint8_t line_length;
static bool discard_line;
static uint8_t tx_ring[TX_SIZE];
static uint16_t tx_head;
static uint16_t tx_tail;
static uint8_t tx_chunk[128];
static volatile bool tx_active;
static uint32_t tx_start_ms;
static uint32_t tx_timeouts;
static uint32_t tx_dropped;
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
static bool motion_tracking;
static uint8_t tracked_sequence;
static int64_t motion_start_yaw_mdeg;
static uint32_t last_motion_report_ms;

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
    ++tx_dropped;
    return;
  }
  for (size_t i = 0U; i < length; ++i) {
    tx_ring[tx_head] = (uint8_t)text[i];
    tx_head = (uint16_t)((tx_head + 1U) % TX_SIZE);
  }
}

static void wheel_start_rx(void)
{
  if (rx_recovery_pending || rx_overflow) {
    return;
  }
  if (huart1.RxState == HAL_UART_STATE_READY) {
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK) {
      ++rx_start_failures;
    }
  }
}

void WheelCheck_OnUartError(uint32_t error)
{
  rx_error_pending |= error;
  rx_recovery_pending = true;
  /* AbortReceive alone does not clear F4 SR error flags. Quiesce RX now to
   * avoid an interrupt storm; defer parser reset and rearming to main. */
  ATOMIC_CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE);
  ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
}

void WheelCheck_OnTxComplete(void)
{
  tx_active = false;
}

static void wheel_pump_tx(void)
{
  if (tx_active && ((uint32_t)(HAL_GetTick() - tx_start_ms) >= TX_TIMEOUT_MS)) {
    bool recovered = false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (tx_active) {
      (void)HAL_UART_AbortTransmit(&huart1);
      tx_active = false;
      tx_head = tx_tail;
      ++tx_timeouts;
      recovered = true;
      WheelCheck_Stop();
      (void)Debug_SubmitConsoleCommand(VISION_MOTION_CMD_STOP, 0, 0, 0, NULL);
    }
    if (primask == 0U) {
      __enable_irq();
    }
    if (recovered) {
      wheel_send("\r\nERR TX TIMEOUT; STOPPED; RETRY COMMAND\r\n");
    }
  }
  if (tx_active || (huart1.gState != HAL_UART_STATE_READY) || (tx_tail == tx_head)) {
    return;
  }
  uint16_t tail = tx_tail;
  uint16_t size = 0U;
  while ((tail != tx_head) && (size < sizeof(tx_chunk))) {
    tx_chunk[size++] = tx_ring[tail];
    tail = (uint16_t)((tail + 1U) % TX_SIZE);
  }
  tx_start_ms = HAL_GetTick();
  tx_active = true;
  if (HAL_UART_Transmit_IT(&huart1, tx_chunk, size) == HAL_OK) {
    tx_tail = tail;
  } else {
    tx_active = false;
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
  const bool was_active = sequence_active;
  if (result_pending) {
    wheel_report();
  }
  if (was_active) {
    Debug_SetWheelCheckActive(false);
  }
  sequence_active = false;
  cancel_requested = false;
  next_id = 0U;
  if (was_active) {
    wheel_send(cancelled ? "WHEELS CANCELLED\r\n" : "WHEELS DONE\r\n");
  }
}

static bool wheel_start_pulse(uint8_t id)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool started = false;
  if (!cancel_requested && !rx_recovery_pending && !rx_overflow) {
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
  const long total = (imu.yaw_mdeg > INT32_MAX) ? INT32_MAX :
      ((imu.yaw_mdeg < INT32_MIN) ? INT32_MIN : (long)imu.yaw_mdeg);
  char text[256];
  (void)snprintf(text, sizeof(text),
      "GYRO READY=%u SAMPLE=%lu TOTAL_MDEG=%ld YAW_MOD_MDEG=%ld DELTA_VALID=%u "
      "DELTA_MDEG=%ld HEADING_SIGN=%d\r\n",
      (unsigned int)imu.ready, (unsigned long)imu.sample_count,
      total, (long)(imu.yaw_mdeg % 360000LL), (unsigned int)delta_valid,
      delta_valid ? (long)delta : 0L, (int)APP_MOTOR_HEADING_OUTPUT_SIGN);
  wheel_send(text);
  previous_gyro_yaw_mdeg = imu.yaw_mdeg;
  gyro_reference_valid = imu.ready;
}

static const char *wheel_state_name(const DebugStatus *status)
{
  if (status->console_pending) {
    return "QUEUED";
  }
  switch (status->state) {
    case DEBUG_MOTION_IDLE: return "IDLE";
    case DEBUG_MOTION_RUNNING: return "RUNNING";
    case DEBUG_MOTION_DONE: return "DONE";
    case DEBUG_MOTION_FAULT: return "FAULT";
    case DEBUG_MOTION_STOPPED: return "STOPPED";
    default: return "UNKNOWN";
  }
}

static const char *wheel_source_name(DebugCommandSource source)
{
  switch (source) {
    case DEBUG_SOURCE_UPPER: return "UPPER";
    case DEBUG_SOURCE_CONSOLE: return "SERIAL";
    case DEBUG_SOURCE_WHEELS: return "WHEELS";
    default: return "NONE";
  }
}

static const char *wheel_fault_name(DebugMotionFault fault)
{
  switch (fault) {
    case DEBUG_MOTION_FAULT_NONE: return "NONE";
    case DEBUG_MOTION_FAULT_INVALID: return "INVALID";
    case DEBUG_MOTION_FAULT_IMU: return "IMU_NOT_READY";
    case DEBUG_MOTION_FAULT_ODOM: return "ODOM_INVALID";
    case DEBUG_MOTION_FAULT_TIMEOUT: return "TIMEOUT";
    case DEBUG_MOTION_FAULT_MOTOR: return "MOTOR";
    default: return "UNKNOWN";
  }
}

static void wheel_report_motors(void)
{
  MotorStatus motor[3];
  for (uint8_t i = 0U; i < 3U; ++i) {
    motor[i] = Motor_GetStatus((uint8_t)(i + 1U));
  }
  char text[192];
  (void)snprintf(text, sizeof(text),
      "DRIVE TARGET_MM_S=%d,%d,%d ACTUAL_MM_S=%d,%d,%d PWM_PERMILLE=%d,%d,%d\r\n",
      motor[0].target_speed_mm_s, motor[1].target_speed_mm_s,
      motor[2].target_speed_mm_s, motor[0].measured_speed_mm_s,
      motor[1].measured_speed_mm_s, motor[2].measured_speed_mm_s,
      motor[0].command, motor[1].command, motor[2].command);
  wheel_send(text);
}

static void wheel_report_state(const char *prefix)
{
  const DebugStatus status = Debug_GetStatus();
  const IMUData imu = IMU_GetData();
  const LocationPose pose = Location_GetPose();
  const int64_t delta = motion_tracking ? imu.yaw_mdeg - motion_start_yaw_mdeg : 0LL;
  const long bounded_delta = (delta > INT32_MAX) ? INT32_MAX :
      ((delta < INT32_MIN) ? INT32_MIN : (long)delta);
  char text[224];
  (void)snprintf(text, sizeof(text),
      "%s STATE=%s SRC=%s CMD=%u SEQ=%u PROGRESS=%d LEFT=%u FAULT=%s(%u) "
      "IMU=%u ODOM=%u DYAW_MDEG=%ld\r\n",
      prefix, wheel_state_name(&status), wheel_source_name(status.source),
      status.command, status.command_sequence, status.progress, status.remaining,
      wheel_fault_name(status.fault), (unsigned int)status.fault,
      (unsigned int)imu.ready, (unsigned int)pose.valid, bounded_delta);
  wheel_send(text);
}

static void wheel_report_version(void)
{
  char text[240];
  (void)snprintf(text, sizeof(text),
      "MOTIONDEBUG SERIAL_V3 RX_GUARD=1 MODE=%u FORWARD_SIGN=%d HEADING_SIGN=%d "
      "HOLD_P_X100=%ld HOLD_I_X10000=%ld STEER_P_X1000=%ld STEER_D_X10000=%ld BUILT="
      __DATE__ " " __TIME__ "\r\n", (unsigned int)APP_ACTIVE_MODE,
      APP_OMNI_FORWARD_SIGN, (int)APP_MOTOR_HEADING_OUTPUT_SIGN,
      (long)(APP_MOTOR_HEADING_KP * 100.0f + 0.5f),
      (long)(APP_MOTOR_HEADING_KI * 10000.0f + 0.5f),
      (long)(APP_STEERING_KP_MM_S * 1000.0f + 0.5f),
      (long)(APP_STEERING_KD_MM * 10000.0f + 0.5f));
  wheel_send(text);
}

static bool wheel_parse_args(const char *cursor, int32_t args[3], uint8_t *count)
{
  *count = 0U;
  while (*cursor != '\0') {
    while (*cursor == ' ') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    if (*count == 3U) {
      return false;
    }
    char *end;
    const long value = strtol(cursor, &end, 10);
    if ((end == cursor) || ((*end != '\0') && (*end != ' ')) ||
        (value < INT16_MIN) || (value > INT16_MAX)) {
      return false;
    }
    args[(*count)++] = (int32_t)value;
    cursor = end;
  }
  return true;
}

static void wheel_submit_motion(uint8_t opcode, int16_t arg1, int16_t arg2,
                                uint16_t speed)
{
  if (rx_recovery_pending || rx_overflow) {
    wheel_send("ERR RX RECOVERY PENDING; RETRY\r\n");
    return;
  }
  if (cancel_requested || sequence_active) {
    if (opcode != VISION_MOTION_CMD_HOLD) {
      wheel_send("BUSY: WAIT WHEELS DONE\r\n");
      return;
    }
  }
  if (((opcode == VISION_MOTION_CMD_MOVE_BODY) ||
       (opcode == VISION_MOTION_CMD_MOVE_FIELD)) &&
      ((speed < APP_MOTION_DEBUG_MIN_SPEED_MM_S) ||
       (speed > APP_MOTION_DEBUG_MAX_SPEED_MM_S))) {
    char text[80];
    (void)snprintf(text, sizeof(text), "ERR SPEED=%u: ALLOWED 50..700 MM/S\r\n", speed);
    wheel_send(text);
    return;
  }
  uint8_t sequence;
  const int64_t start_yaw = IMU_GetData().yaw_mdeg;
  const DebugConsoleResult result = Debug_SubmitConsoleCommand(
      opcode, arg1, arg2, speed, &sequence);
  if (result != DEBUG_CONSOLE_ACCEPTED) {
    wheel_send((result == DEBUG_CONSOLE_BUSY) ?
        "BUSY: MOTION ACTIVE; STOP FIRST\r\n" : "ERR ARGUMENT RANGE\r\n");
    return;
  }
  motion_tracking = true;
  tracked_sequence = sequence;
  motion_start_yaw_mdeg = start_yaw;
  last_motion_report_ms = HAL_GetTick() - 250U;
  char text[80];
  (void)snprintf(text, sizeof(text), "ACCEPT SERIAL CMD=%u SEQ=%u\r\n", opcode, sequence);
  wheel_send(text);
}

static void wheel_execute(void)
{
  ++rx_lines;
  char received[LINE_SIZE + 12U];
  (void)snprintf(received, sizeof(received), "RXCMD %s\r\n", line);
  wheel_send(received);
  char *command = line;
  while (*command == ' ') {
    ++command;
  }
  char *args_text = strchr(command, ' ');
  if (args_text != NULL) {
    *args_text++ = '\0';
  } else {
    args_text = command + strlen(command);
  }
  int32_t args[3] = {0, 0, 0};
  uint8_t count;
  if (!wheel_parse_args(args_text, args, &count)) {
    wheel_send("ERR ARGUMENTS; SEND HELP\r\n");
    return;
  }
  if ((strcmp(command, "TURN") == 0) && ((count == 1U) || (count == 2U))) {
    const int32_t rate = (count == 2U) ? args[1] : 30;
    if ((args[0] < -360) || (args[0] > 360) || (rate < 1) || (rate > 360)) {
      wheel_send("ERR TURN RANGE: ANGLE -360..360, RATE 1..360 DEG/S\r\n");
      return;
    }
    wheel_submit_motion(VISION_MOTION_CMD_TURN_REL,
        (int16_t)(args[0] * 10), 0, (uint16_t)(rate * 10));
    return;
  }
  if ((strcmp(command, "MOVE") == 0) && ((count == 1U) || (count == 2U))) {
    const int32_t speed = (count == 2U) ? args[1] : 150;
    wheel_submit_motion(VISION_MOTION_CMD_MOVE_BODY, (int16_t)args[0], 0,
                         (uint16_t)speed);
    return;
  }
  if (((strcmp(command, "BODY") == 0) || (strcmp(command, "FIELD") == 0)) &&
      ((count == 2U) || (count == 3U))) {
    const int32_t speed = (count == 3U) ? args[2] : 150;
    wheel_submit_motion((strcmp(command, "BODY") == 0) ?
        VISION_MOTION_CMD_MOVE_BODY : VISION_MOTION_CMD_MOVE_FIELD,
        (int16_t)args[0], (int16_t)args[1], (uint16_t)speed);
    return;
  }
  if (count != 0U) {
    wheel_send("ERR COMMAND ARGUMENTS; SEND HELP\r\n");
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    WheelCheck_Stop();
    (void)Debug_SubmitConsoleCommand(VISION_MOTION_CMD_STOP, 0, 0, 0, NULL);
    wheel_send("STOP OK\r\n");
    return;
  }
  if ((strcmp(command, "HOLD") == 0) || (strcmp(command, "RESET") == 0)) {
    wheel_submit_motion((strcmp(command, "HOLD") == 0) ?
        VISION_MOTION_CMD_HOLD : VISION_MOTION_CMD_RESET_ODOM, 0, 0, 0);
    return;
  }
  if ((strcmp(command, "STAT") == 0) || (strcmp(command, "STATE") == 0)) {
    wheel_report_version();
    wheel_report_state("STATUS");
    wheel_report_motors();
    return;
  }
  if ((strcmp(command, "HELP") == 0) || (strcmp(command, "VER") == 0)) {
    wheel_report_version();
    wheel_send("TURN deg [deg/s=30]; MOVE mm [mm/s=150]\r\n"
               "BODY forward_mm left_mm [mm/s=150]\r\n"
               "FIELD x_mm y_mm [mm/s=150]\r\n"
               "STOP HOLD RESET STAT GYRO IMU MOTOR ENC WHEELS PING\r\n"
               "WHEELS: AIR ONLY, 1+ -> 2+ -> 3+, 25% 400ms EACH\r\n");
    return;
  }
  if (strcmp(command, "PING") == 0) {
    char text[224];
    (void)snprintf(text, sizeof(text),
        "PONG MS=%lu RX_BYTES=%lu LINES=%lu RX_ERRORS=%lu LAST_RX_ERROR=0x%08lX "
        "OVERFLOWS=%lu REARM_FAIL=%lu LINE_TIMEOUTS=%lu TX_TIMEOUTS=%lu TX_DROPS=%lu\r\n",
        (unsigned long)HAL_GetTick(), (unsigned long)rx_bytes,
        (unsigned long)rx_lines, (unsigned long)rx_errors,
        (unsigned long)rx_last_error, (unsigned long)rx_overflows,
        (unsigned long)rx_start_failures, (unsigned long)line_timeouts,
        (unsigned long)tx_timeouts, (unsigned long)tx_dropped);
    wheel_send(text);
    return;
  }
  if (strcmp(command, "MOTOR") == 0) {
    wheel_report_motors();
    return;
  }
  if (strcmp(command, "ENC") == 0) {
    EncoderStatus encoder[3];
    Encoder_GetAll(encoder);
    char text[160];
    (void)snprintf(text, sizeof(text),
        "ENC RAW32=%ld,%ld,%ld DELTA10=%ld,%ld,%ld\r\n",
        (long)(int32_t)encoder[0].position, (long)(int32_t)encoder[1].position,
        (long)(int32_t)encoder[2].position, (long)encoder[0].delta_10ms,
        (long)encoder[1].delta_10ms, (long)encoder[2].delta_10ms);
    wheel_send(text);
    return;
  }
  if (strcmp(command, "GYRO") == 0) {
    wheel_report_gyro();
    return;
  }
  if (strcmp(command, "IMU") == 0) {
    const IMUData imu = IMU_GetData();
    char text[288];
    (void)snprintf(text, sizeof(text),
        "IMU ALG=TS_TRAPZ READY=%u INIT=%u ID=0x%02X SAMPLE=%lu ERRORS=%lu "
        "DT_US=%lu MAX_DT_US=%lu GAPS=%lu CLOCK_FINE=%d "
        "BIAS_MDPS=%ld,%ld,%ld STILL=%u BIAS_UPDATES=%lu Z_SCALE_PPM=%ld\r\n",
        (unsigned int)imu.ready, (unsigned int)imu.init_result,
        imu.device_id, (unsigned long)imu.sample_count,
        (unsigned long)imu.error_count, (unsigned long)imu.sample_interval_us,
        (unsigned long)imu.max_sample_interval_us,
        (unsigned long)imu.integration_gap_count, (int)imu.clock_fine,
        (long)imu.gyro_bias_mdps[0], (long)imu.gyro_bias_mdps[1],
        (long)imu.gyro_bias_mdps[2], (unsigned int)imu.stationary,
        (unsigned long)imu.bias_update_count, (long)APP_IMU_GYRO_Z_SCALE_PPM);
    wheel_send(text);
    return;
  }
  if (strcmp(command, "WHEELS") != 0) {
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
  wheel_report_version();
  wheel_send("USART1 115200 8N1; SEND HELP; NO AUTOMATIC MOTION\r\n");
}

void WheelCheck_Process(void)
{
  if (rx_recovery_pending || rx_overflow || (huart1.ErrorCode != HAL_UART_ERROR_NONE)) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t error = rx_error_pending | huart1.ErrorCode;
    if (rx_recovery_pending || (error != HAL_UART_ERROR_NONE)) {
      ++rx_errors;
      rx_last_error = error;
    }
    if (rx_overflow) {
      ++rx_overflows;
    }
    (void)HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    rx_tail = rx_head;
    rx_overflow = false;
    rx_error_pending = HAL_UART_ERROR_NONE;
    rx_recovery_pending = false;
    line_length = 0U;
    discard_line = true;
    last_rx_byte_ms = HAL_GetTick();
    if (primask == 0U) {
      __enable_irq();
    }
    WheelCheck_Stop();
    (void)Debug_SubmitConsoleCommand(VISION_MOTION_CMD_STOP, 0, 0, 0, NULL);
    wheel_send("ERR RX; STOPPED; SEND NEWLINE THEN RETRY\r\n");
  }
  wheel_start_rx();
  while (rx_tail != rx_head) {
    char value = (char)rx_ring[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) % RX_SIZE);
    if (value == '\t') {
      value = ' ';
    }
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
        (void)Debug_SubmitConsoleCommand(VISION_MOTION_CMD_STOP, 0, 0, 0, NULL);
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
  const uint32_t rx_tick_ms = last_rx_byte_ms;
  const uint32_t now_ms = HAL_GetTick();
  if (((line_length != 0U) || discard_line) &&
      ((uint32_t)(now_ms - rx_tick_ms) >= LINE_TIMEOUT_MS)) {
    line_length = 0U;
    discard_line = false;
    ++line_timeouts;
    WheelCheck_Stop();
    (void)Debug_SubmitConsoleCommand(VISION_MOTION_CMD_STOP, 0, 0, 0, NULL);
    wheel_send("ERR LINE TIMEOUT; STOPPED; PARTIAL INPUT DISCARDED\r\n");
  }
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
  if (motion_tracking) {
    const DebugStatus status = Debug_GetStatus();
    const bool matching = (status.source == DEBUG_SOURCE_CONSOLE) &&
                         (status.command_sequence == tracked_sequence);
    const bool terminal = !matching || (!status.console_pending &&
        (status.state != DEBUG_MOTION_RUNNING) && (status.state != DEBUG_MOTION_IDLE));
    if (terminal || ((uint32_t)(now_ms - last_motion_report_ms) >= 250U)) {
      wheel_report_state(terminal ? "FINAL" : "MOTION");
      wheel_report_motors();
      last_motion_report_ms = now_ms;
    }
    if (terminal) {
      motion_tracking = false;
    }
  }
  wheel_pump_tx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1) {
    ++rx_bytes;
    last_rx_byte_ms = HAL_GetTick();
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
