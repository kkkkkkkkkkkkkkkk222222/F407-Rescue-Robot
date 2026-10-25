#include "WheelCheck.h"
#include "Uart.h"

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
#include "MotionDiag.h"

extern UART_HandleTypeDef huart1;

#if APP_BOARD_F405_ROBO
#define RX_SIZE 1024U
#define RX_DMA_SIZE 512U
static uint8_t rx_dma[RX_DMA_SIZE];
static uint16_t rx_dma_cursor;
#else
#define RX_SIZE 128U
static uint8_t rx_byte;
#endif
#define TX_SIZE 4096U
#define LINE_SIZE 64U
#define TX_TIMEOUT_MS 250U
#define LINE_TIMEOUT_MS 1000U

static volatile uint8_t rx_ring[RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
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
static bool trace_enabled = true;
#if APP_BOARD_F405_ROBO
static bool move_log_enabled, move_log_saved_trace;
static uint32_t move_log_last_ms, move_log_index;
static int64_t move_log_yaw_origin;
#endif
static bool post_motion_trace;
static uint32_t post_motion_started_ms;
static bool calibration_active;
static uint8_t calibration_id;
static int16_t calibration_pwm;
static uint32_t calibration_step, calibration_step_ms, calibration_sample_ms;
static int64_t calibration_position[3];

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

#if APP_BOARD_F405_ROBO
static void wheel_report_move_log(void)
{
  const uint32_t now = HAL_GetTick();
  const DebugStatus status = Debug_GetStatus();
  const MotorDebugTrace motor = Motor_GetDebugTrace();
  EncoderStatus enc[3];
  Encoder_GetAll(enc);
  const IMUData imu = IMU_GetData();
  const LocationPose pose = Location_GetPose();
  const int64_t yaw = imu.yaw_mdeg - move_log_yaw_origin;
  char text[512];
  (void)snprintf(text, sizeof(text),
      "MOVE_LOG N=%lu MS=%lu SEQ=%u CMD=%u STATE=%u PENDING=%u FAULT=%u "
      "RAW32=%ld,%ld,%ld TARGET=%d,%d,%d ACTUAL=%d,%d,%d PWM=%d,%d,%d "
      "FF=%d,%d,%d YAW_MDEG=%ld RATE_MDPS=%ld XY=%ld,%ld CROSS=%ld VALID=%u "
      "YAW_CMD100=%ld IMU=%u AGE=%lu UERR=%lu ENC_BAD=%lu DROP=%lu\r\n",
      (unsigned long)++move_log_index, (unsigned long)now, status.command_sequence,
      status.command, status.state, status.console_pending, status.fault,
      (long)(int32_t)enc[0].position, (long)(int32_t)enc[1].position, (long)(int32_t)enc[2].position,
      motor.target_mm_s[0], motor.target_mm_s[1], motor.target_mm_s[2],
      motor.actual_mm_s[0], motor.actual_mm_s[1], motor.actual_mm_s[2],
      motor.output[0], motor.output[1], motor.output[2],
      motor.feedforward[0], motor.feedforward[1], motor.feedforward[2],
      (long)(yaw > INT32_MAX ? INT32_MAX : yaw < INT32_MIN ? INT32_MIN : yaw),
      (long)imu.gyro_mdps[2], (long)pose.x_mm, (long)pose.y_mm,
      (long)motor.move_cross_mm, motor.move_path_valid,
      (long)motor.move_yaw_c100, imu.ready, (unsigned long)imu.uart_angle_age_ms,
      (unsigned long)imu.uart_rx_errors,
      (unsigned long)BoardF405_EncoderInvalidTransitions(), (unsigned long)tx_dropped);
  wheel_send(text);
  move_log_last_ms = now;
}
#endif

static void wheel_start_rx(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (rx_recovery_pending || rx_overflow) {
    if (mask == 0U) { __enable_irq(); }
    return;
  }
  if (huart1.RxState == HAL_UART_STATE_READY) {
#if APP_BOARD_F405_ROBO
    rx_dma_cursor = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma, RX_DMA_SIZE) != HAL_OK) {
#else
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK) {
#endif
      ++rx_start_failures;
    }
  }
  if (mask == 0U) { __enable_irq(); }
}

static void wheel_rx_byte(uint8_t value)
{
  ++rx_bytes;
  last_rx_byte_ms = HAL_GetTick();
  const uint16_t next = (uint16_t)((rx_head + 1U) % RX_SIZE);
  if (next == rx_tail) { rx_overflow = true; }
  else { rx_ring[rx_head] = value; rx_head = next; }
}

void WheelCheck_OnRxEvent(void)
{
#if APP_BOARD_F405_ROBO
  if (rx_recovery_pending || rx_overflow) { return; }
  if (huart1.ErrorCode != HAL_UART_ERROR_NONE) {
    WheelCheck_OnUartError(huart1.ErrorCode);
    return;
  }
  /* HT/TC/IDLE can be pending together. Read the live cursor instead of an
   * older callback Size, so delayed events never duplicate a command. */
  const uint16_t end = (uint16_t)((RX_DMA_SIZE -
      __HAL_DMA_GET_COUNTER(huart1.hdmarx)) % RX_DMA_SIZE);
  while (rx_dma_cursor != end) {
    wheel_rx_byte(rx_dma[rx_dma_cursor]);
    rx_dma_cursor = (uint16_t)((rx_dma_cursor + 1U) % RX_DMA_SIZE);
  }
#endif
}

void WheelCheck_OnUartError(uint32_t error)
{
  rx_error_pending |= error;
  rx_recovery_pending = true;
  /* AbortReceive alone does not clear F4 SR error flags. Quiesce RX now to
   * avoid an interrupt storm; defer parser reset and rearming to main. */
  ATOMIC_CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE);
  ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
#if APP_BOARD_F405_ROBO
  ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
#endif
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
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  tx_start_ms = HAL_GetTick();
  tx_active = true;
  if (HAL_UART_Transmit_IT(&huart1, tx_chunk, size) == HAL_OK) {
    tx_tail = tail;
  } else {
    tx_active = false;
  }
  if (mask == 0U) { __enable_irq(); }
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

static void calibration_report(void)
{
  const uint32_t now = HAL_GetTick();
  const uint32_t dt = now - calibration_sample_ms;
  if (!calibration_active || dt == 0U) { return; }
  EncoderStatus encoder[3];
  Encoder_GetAll(encoder);
  int32_t delta[3];
  for (uint8_t i = 0U; i < 3U; ++i) {
    delta[i] = (int32_t)(encoder[i].position - calibration_position[i]);
    calibration_position[i] = encoder[i].position;
  }
  char text[208];
  (void)snprintf(text, sizeof(text),
      "CAL_DATA STEP=%lu ID=%u PWM=%d MS=%lu AGE_MS=%lu DT_MS=%lu RAW=%ld,%ld,%ld FB=%ld ACTIVE=%u\r\n",
      (unsigned long)calibration_step, calibration_id, calibration_pwm,
      (unsigned long)now, (unsigned long)(now - calibration_step_ms),
      (unsigned long)dt, (long)delta[0], (long)delta[1], (long)delta[2],
      (long)(delta[calibration_id - 1U] * encoder_signs[calibration_id - 1U]),
      Motor_WheelCheckRunning());
  wheel_send(text);
  calibration_sample_ms = now;
}

static void wheel_finish(bool cancelled)
{
  const bool was_active = sequence_active || calibration_active;
  const bool was_calibration = calibration_active;
  if (was_calibration) {
    calibration_report();
    calibration_active = false;
  }
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
    wheel_send(was_calibration ? (cancelled ? "CAL_END CANCELLED\r\n" : "CAL_END EXPIRED\r\n") :
        (cancelled ? "WHEELS CANCELLED\r\n" : "WHEELS DONE\r\n"));
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

static void wheel_report_circles(void)
{
  static uint8_t last_sequence;
  static uint8_t last_completed;
  const DebugCircleStatus circle = Debug_GetCircleStatus();
  if (!circle.valid) {
    last_completed = 0U;
    return;
  }
  if (circle.sequence != last_sequence) {
    last_sequence = circle.sequence;
    last_completed = 0U;
  }
  const char *event = !circle.active ?
      ((circle.completed == circle.total) ? "DONE" : "CANCELLED") :
      ((circle.completed != last_completed) ? "LAP_DONE" :
       (circle.dwelling ? "DWELL" : "RUNNING"));
  char text[256];
  (void)snprintf(text, sizeof(text),
      "CIRCLE EVENT=%s SEQ=%u ACTIVE=%u CURRENT=%u COMPLETED=%u TOTAL=%u DIR=%d "
      "TOTAL_MDEG=%ld GOAL_MDEG=%ld ERR_MDEG=%ld PROGRESS_SCOPE=LAP\r\n",
      event, circle.sequence, circle.active, circle.current, circle.completed,
      circle.total, circle.direction, (long)circle.total_delta_mdeg,
      (long)circle.direction * circle.current * 360000L,
      (long)circle.goal_error_mdeg);
  wheel_send(text);
  last_completed = circle.completed;
}

static void wheel_report_state(const char *prefix)
{
  const DebugStatus status = Debug_GetStatus();
  const IMUData imu = IMU_GetData();
  const LocationPose pose = Location_GetPose();
  const int64_t delta = (motion_tracking || post_motion_trace) ?
      imu.yaw_mdeg - motion_start_yaw_mdeg : 0LL;
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
  wheel_report_circles();
}

static void wheel_report_version(void)
{
  char text[320];
  (void)snprintf(text, sizeof(text),
#if APP_BOARD_F405_ROBO
      "NORMALDEBUG BOARD=F405_ROBO SERVICE=MOTIONDEBUG "
#else
      "MOTIONDEBUG "
#endif
      "SERIAL_V3 RX_GUARD=1 DIAG=1 TURN_PROFILE=2 UART_FIX=1 CAL=1 MODE=%u FORWARD_SIGN=%d HEADING_SIGN=%d "
      "HOLD_P_X100=%ld HOLD_I_X10000=%ld STEER_P_X1000=%ld STEER_D_X10000=%ld BUILT="
      __DATE__ " " __TIME__ "\r\n", (unsigned int)APP_ACTIVE_MODE,
      APP_OMNI_FORWARD_SIGN, (int)APP_MOTOR_HEADING_OUTPUT_SIGN,
      (long)(APP_MOTOR_HEADING_KP * 100.0f + 0.5f),
      (long)(APP_MOTOR_HEADING_KI * 10000.0f + 0.5f),
      (long)(APP_STEERING_KP_MM_S * 1000.0f + 0.5f),
      (long)(APP_STEERING_KD_MM * 10000.0f + 0.5f));
  wheel_send(text);
#if APP_BOARD_F405_ROBO
  wheel_send("CONSOLE_RX=DMA_IDLE DMA_BYTES=512 RING_BYTES=1024\r\n");
  wheel_send("AXIS=LEFT_POSITIVE TURN_MODE=MAIN_THEN_TRIM FINAL_ENTRY_MDEG=3000\r\n");
#endif
  (void)snprintf(text, sizeof(text),
      "DIAG_CFG TRACE=%u PERIOD_MS=250 POST_MS=2000 SPEED_P100=%ld SPEED_I10000=%ld TURN_TIMEOUT_MS=%lu\r\n",
      trace_enabled, (long)(APP_MOTOR_SPEED_KP * 100.0f),
      (long)(APP_MOTOR_SPEED_KI * 10000.0f),
      (unsigned long)APP_MOTOR_TURN_TIMEOUT_MS);
  wheel_send(text);
  (void)snprintf(text, sizeof(text),
      "TURN_CFG POS_P100=%ld RATE_P100=%ld ACCEL_DPS2=%ld DECEL_DPS2=%ld STATIC_PWM=%ld ENTER_MDEG=%ld EXIT_MDEG=%ld STOP_MDPS=%ld SETTLE_MS=%u MAX_TRIMS=%u STALE_MS=%u\r\n",
      (long)(APP_DEBUG_TURN_POSITION_KP * 100.0f), (long)(APP_DEBUG_TURN_RATE_KP * 100.0f),
      (long)APP_DEBUG_TURN_ACCEL_DPS2, (long)APP_DEBUG_TURN_DECEL_DPS2,
      (long)APP_DEBUG_TURN_STATIC_PWM, (long)APP_DEBUG_TURN_ENTER_MDEG,
      (long)APP_DEBUG_TURN_EXIT_MDEG,
      (long)APP_DEBUG_TURN_STOP_RATE_MDPS, APP_DEBUG_TURN_SETTLE_MS,
      APP_DEBUG_TURN_MAX_CORRECTIONS, APP_DEBUG_TURN_IMU_STALE_MS);
  wheel_send(text);
  (void)snprintf(text, sizeof(text),
      "BOOST_CFG PWM=%u MAX_MS=%u WAIT_MS=%u COOLDOWN_MS=%u MAX_COUNT=%u TRAVEL_MDEG=%ld\r\n",
      APP_DEBUG_TURN_BOOST_PWM, APP_DEBUG_TURN_BOOST_MAX_MS,
      APP_DEBUG_TURN_BOOST_WAIT_MS, APP_DEBUG_TURN_BOOST_COOLDOWN_MS,
      APP_DEBUG_TURN_BOOST_MAX_COUNT, (long)APP_DEBUG_TURN_BOOST_TRAVEL_MDEG);
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
  post_motion_trace = false;
  tracked_sequence = sequence;
  motion_start_yaw_mdeg = start_yaw;
  last_motion_report_ms = HAL_GetTick() - 250U;
  char text[80];
  (void)snprintf(text, sizeof(text), "ACCEPT SERIAL CMD=%u SEQ=%u\r\n", opcode, sequence);
  wheel_send(text);
}

static void wheel_submit_circles(int8_t count, uint16_t rate_dps)
{
  if (rx_recovery_pending || rx_overflow) {
    wheel_send("ERR RX RECOVERY PENDING; RETRY\r\n");
    return;
  }
  if (cancel_requested || sequence_active) {
    wheel_send("BUSY: WAIT WHEELS DONE\r\n");
    return;
  }
  const int64_t start_yaw = IMU_GetData().yaw_mdeg;
  uint8_t sequence;
  const DebugConsoleResult result = Debug_SubmitConsoleCircles(count, rate_dps, &sequence);
  if (result != DEBUG_CONSOLE_ACCEPTED) {
    wheel_send((result == DEBUG_CONSOLE_BUSY) ?
        "BUSY: MOTION ACTIVE; STOP FIRST\r\n" : "ERR CIRCLES RANGE OR IMU NOT READY\r\n");
    return;
  }
  motion_tracking = true;
  post_motion_trace = false;
  tracked_sequence = sequence;
  motion_start_yaw_mdeg = start_yaw;
  last_motion_report_ms = HAL_GetTick() - 250U;
  char text[144];
  (void)snprintf(text, sizeof(text),
      "ACCEPT SERIAL CMD=%u SEQ=%u CIRCLES=%d RATE_DPS=%u DWELL_MS=%u FIXED_REFERENCE=1\r\n",
      VISION_MOTION_CMD_TURN_REL, sequence, count, rate_dps, APP_DEBUG_CIRCLES_DWELL_MS);
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
#if APP_BOARD_F405_ROBO
  if (strcmp(command, "S") == 0) {
    if (count != 2U || args[0] < 1 || args[0] > 4 || args[1] < 0 || args[1] > 180) {
      wheel_send("ERR S: ID 1..4 ANGLE 0..180\r\n"); return;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const DebugStatus status = Debug_GetStatus();
    if (sequence_active || calibration_active || status.console_pending ||
        status.state == DEBUG_MOTION_RUNNING || Motor_WheelCheckRunning()) {
      __set_PRIMASK(mask);
      wheel_send("BUSY: STOP BEFORE SERVO\r\n"); return;
    }
    BoardF405_ServoTestSet((uint8_t)args[0], (uint8_t)args[1]);
    __set_PRIMASK(mask);
    char reply[80];
    (void)snprintf(reply, sizeof(reply), "SERVO ID=%ld TARGET_DEG=%ld PWM=ON\r\n",
                   (long)args[0], (long)args[1]);
    wheel_send(reply); return;
  }
  if (strcmp(command, "SOFF") == 0) {
    if (count > 1U || (count == 1U && (args[0] < 1 || args[0] > 4))) {
      wheel_send("ERR SOFF: OPTIONAL ID 1..4\r\n"); return;
    }
    BoardF405_ServoTestOff(count == 0U ? 0U : (uint8_t)args[0]);
    wheel_send("SERVO PWM OFF\r\n"); return;
  }
#endif
  if (strcmp(command, "CAL") == 0) {
    if (count != 3U || args[0] < 1 || args[0] > 3 || args[1] == 0 ||
        args[1] < -250 || args[1] > 250 || args[2] < 100 || args[2] > 1500) {
      wheel_send("ERR CAL: ID 1..3, PWM -250..250 NONZERO PERMILLE, MS 100..1500\r\n");
      return;
    }
    if (calibration_active) { calibration_report(); }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const DebugStatus status = Debug_GetStatus();
    const bool busy = sequence_active || cancel_requested || rx_recovery_pending || rx_overflow ||
        (calibration_active && !Motor_WheelCheckRunning()) ||
        (!calibration_active && (status.console_pending || status.state == DEBUG_MOTION_RUNNING));
    if (busy) {
      if (mask == 0U) { __enable_irq(); }
      wheel_send("BUSY: STOP BEFORE CAL\r\n");
      return;
    }
    const bool first = !calibration_active;
    if (first) { Debug_SetWheelCheckActive(true); }
    const bool started = Motor_CalibrationStep((uint8_t)args[0], (int16_t)args[1], (uint16_t)args[2]);
    if (!started && first) { Debug_SetWheelCheckActive(false); }
    if (mask == 0U) { __enable_irq(); }
    if (!started) { wheel_send("ERR CAL: STOP AND WAIT; SAME WHEEL/DIRECTION ONLY\r\n"); return; }
    EncoderStatus encoder[3];
    Encoder_GetAll(encoder);
    for (uint8_t i = 0U; i < 3U; ++i) { calibration_position[i] = encoder[i].position; }
    calibration_id = (uint8_t)args[0];
    calibration_pwm = (int16_t)args[1];
    calibration_active = true;
    calibration_step_ms = calibration_sample_ms = HAL_GetTick();
    ++calibration_step;
    motion_tracking = post_motion_trace = false;
    char text[128];
    (void)snprintf(text, sizeof(text), "CAL_BEGIN STEP=%lu ID=%u PWM=%d LEASE_MS=%u CONTINUE=%u MAX_TOTAL_MS=30000\r\n",
        (unsigned long)calibration_step, calibration_id, calibration_pwm, (unsigned int)args[2], !first);
    wheel_send(text);
    return;
  }
  if (strcmp(command, "CIRCLES") == 0) {
    const int32_t rate = (count == 2U) ? args[1] : 60;
    if (((count != 1U) && (count != 2U)) || (args[0] == 0) ||
        (args[0] < -(int32_t)APP_DEBUG_CIRCLES_MAX_COUNT) ||
        (args[0] > (int32_t)APP_DEBUG_CIRCLES_MAX_COUNT) ||
        (rate < (int32_t)APP_DEBUG_CIRCLES_MIN_RATE_DPS) ||
        (rate > (int32_t)APP_DEBUG_CIRCLES_MAX_RATE_DPS)) {
      wheel_send("ERR CIRCLES RANGE: COUNT -10..-1 OR 1..10, RATE 60..120 DEG/S\r\n");
      return;
    }
    wheel_submit_circles((int8_t)args[0], (uint16_t)rate);
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
#if APP_BOARD_F405_ROBO
    if (strcmp(command, "MLOG") == 0 && count == 1U && (args[0] == 0 || args[0] == 1)) {
      if (args[0] == 1) {
        if (!move_log_enabled) { move_log_saved_trace = trace_enabled; }
        trace_enabled = false;
        move_log_enabled = true;
        move_log_index = 0U;
        move_log_yaw_origin = IMU_GetData().yaw_mdeg;
        wheel_send("MLOG ON PERIOD_MS=100; ORDER=RIGHT,LEFT,REAR; SPEED=MM/S PWM=PERMILLE; NO MOTION\r\n");
        wheel_report_move_log();
      } else {
        if (move_log_enabled) { trace_enabled = move_log_saved_trace; }
        move_log_enabled = false;
        wheel_send("MLOG OFF\r\n");
      }
      return;
    }
#endif
    if (strcmp(command, "TRACE") == 0 && count == 1U && (args[0] == 0 || args[0] == 1)) {
      trace_enabled = args[0] != 0;
      wheel_send(trace_enabled ? "TRACE ON\r\n" : "TRACE OFF\r\n");
      return;
    }
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
#if APP_BOARD_F405_ROBO
    wheel_send("S id deg: SERVO 1..4 0..180; SOFF [id]: DISABLE PULSES\r\n"
               "SERVO 1=LIFT 2=RIGHT_CLAW 3=CAMERA 4=LEFT_CLAW; NO AUTO SWEEP\r\n");
#endif
    wheel_report_state("STATUS");
    wheel_report_motors();
    return;
  }
  if (strcmp(command, "DIAG") == 0) {
    MotionDiag_Print(wheel_send, "DIAG");
    return;
  }
  if (strcmp(command, "CRASH") == 0) {
    MotionDiag_PrintPrevious(wheel_send);
    return;
  }
  if ((strcmp(command, "HELP") == 0) || (strcmp(command, "VER") == 0)) {
    wheel_report_version();
#if APP_BOARD_F405_ROBO
    wheel_send("MLOG 1/0: COMPACT 100ms DRIVE LOG, NO MOTION, CONTINUES AFTER STOP\r\n");
#endif
    wheel_send("DIAG CRASH TRACE 0/1; MOTION TRACE + 2s POST-STOP; NO AUTO MOTION\r\n");
    wheel_send("TURN deg [deg/s=30]; MOVE mm [mm/s=150]\r\n"
               "CIRCLES signed_count [deg/s=60]: 1..10 LAPS, RATE 60..120, 1s DWELL\r\n"
               "CIRCLES: FIXED REFERENCE; PROGRESS IS PER LAP; STOP CANCELS BATCH\r\n"
               "BODY forward_mm left_mm [mm/s=150]\r\n"
               "FIELD x_mm y_mm [mm/s=150]\r\n"
               "STOP HOLD RESET STAT GYRO IMU MOTOR ENC WHEELS PING\r\n"
               "CAL wheel signed_permille lease_ms: AIR ONLY, MAX 250/1500ms/30s, STOP ENDS\r\n"
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
    const UartLinkStatus link = Uart_GetLinkStatus();
    (void)snprintf(text, sizeof(text),
#if APP_BOARD_F405_ROBO
        "UART4 "
#else
        "UART3 "
#endif
        "RX_ERRORS=%lu LAST_RX_ERROR=0x%08lX RX_RESTARTS=%lu TX_DROPS=%lu TX_LEFT=%u TX_SIZE=%u RX_ACTIVE=%u RECOVERY=%u\r\n",
        (unsigned long)link.rx_errors, (unsigned long)link.last_rx_error,
        (unsigned long)link.rx_restarts, (unsigned long)link.tx_dropped,
        link.tx_remaining, link.tx_size, link.rx_active, link.rx_pending);
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
        "IMU ALG="
#if APP_IMU_USE_UART
        "UART_YAW_UNWRAP"
#else
        "FIFO_TS_TRAPZ"
#endif
        " READY=%u INIT=%u ID=0x%02X SAMPLE=%lu ERRORS=%lu "
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
#if APP_IMU_USE_UART
    (void)snprintf(text, sizeof(text),
#if APP_BOARD_F405_ROBO
        "IMU_UART USART=5 TX=PC12 RX=PD2 "
#else
        "IMU_UART USART=2 TX=PD5 RX=PD6 "
#endif
        "BAUD=%lu SIGN=%d RX_BYTES=%lu "
        "CRC_ERRORS=%lu RX_ERRORS=%lu RATE_FRAMES=%lu ANGLE_AGE_MS=%lu "
        "RATE_AGE_MS=%lu PERIOD_MS=%lu CFG=%u CFG_ERRORS=%lu\r\n",
        (unsigned long)imu.uart_baud, APP_IMU_UART_YAW_SIGN,
        (unsigned long)imu.uart_rx_bytes, (unsigned long)imu.uart_checksum_errors,
        (unsigned long)imu.uart_rx_errors, (unsigned long)imu.uart_rate_frames,
        (unsigned long)imu.uart_angle_age_ms, (unsigned long)imu.uart_rate_age_ms,
        (unsigned long)imu.uart_output_period_ms, imu.uart_config_state,
        (unsigned long)imu.uart_config_errors);
    wheel_send(text);
#else
    (void)snprintf(text, sizeof(text),
        "IMU_FIFO PEAK=%u OVERRUN=%lu FAULTS=%lu MAX_POLL_MS=%lu TEMP_MC=%ld\r\n",
        (unsigned int)imu.fifo_peak_words, (unsigned long)imu.fifo_overrun_count,
        (unsigned long)imu.fifo_fault_count, (unsigned long)imu.max_poll_interval_ms,
        (long)imu.temperature_mdeg_c);
    wheel_send(text);
#endif
    (void)snprintf(text, sizeof(text), "IMU_RECOVERY ACTIVE=%u UNCERTAIN=%u COUNT=%lu\r\n",
        imu.recovering, imu.heading_uncertain, (unsigned long)imu.recovery_count);
    wheel_send(text);
    return;
  }
  if (strcmp(command, "WHEELS") != 0) {
    wheel_send("ERR COMMAND\r\n");
    return;
  }
  if (sequence_active || calibration_active || cancel_requested) {
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
#if APP_BOARD_F405_ROBO
  wheel_send("USART1 PB6_TX/PB7_RX 115200 8N1; SEND HELP; NO AUTOMATIC MOTION\r\n"
             "NOTICE: ONBOARD_IMU_I2C=SHARED_NOT_DISABLED; REQUEST/REPLY CHECK FIRST\r\n");
#else
  wheel_send("USART1 115200 8N1; SEND HELP; NO AUTOMATIC MOTION\r\n");
#endif
  MotionDiag_PrintPrevious(wheel_send);
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
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
    if (value == '\t') {
      value = ' ';
    }
    if ((value == '\r') || (value == '\n')) {
      if (discard_line) { wheel_send("RX READY; RESEND COMMAND\r\n"); }
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
  if (calibration_active) {
    if ((uint32_t)(now_ms - calibration_sample_ms) >= 50U) { calibration_report(); }
    if (!Motor_WheelCheckRunning()) { wheel_finish(false); }
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
      if (trace_enabled) { MotionDiag_Print(wheel_send, "DIAG"); }
      last_motion_report_ms = now_ms;
    }
    if (terminal) {
      motion_tracking = false;
      post_motion_trace = true;
      post_motion_started_ms = now_ms;
    }
  }
  if (post_motion_trace && !motion_tracking &&
      (uint32_t)(now_ms - last_motion_report_ms) >= 250U) {
    if (trace_enabled) {
      wheel_report_state("SETTLE");
      MotionDiag_Print(wheel_send, "DIAG");
    }
    last_motion_report_ms = now_ms;
    if ((uint32_t)(now_ms - post_motion_started_ms) >= 2000U) { post_motion_trace = false; }
  }
#if APP_BOARD_F405_ROBO
  if (move_log_enabled && (uint32_t)(now_ms - move_log_last_ms) >= 100U) {
    wheel_report_move_log();
  }
#endif
  wheel_pump_tx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
#if APP_BOARD_F405_ROBO
  (void)uart; /* Circular RX uses HAL_UARTEx_RxEventCallback, not byte rearming. */
#else
  if (uart == &huart1) {
    /* HAL calls RxCplt before its error callback. Rearming here would erase
     * ErrorCode and admit a damaged byte as a valid command character. */
    if (uart->ErrorCode != HAL_UART_ERROR_NONE) {
      WheelCheck_OnUartError(uart->ErrorCode);
      return;
    }
    wheel_rx_byte(rx_byte);
    wheel_start_rx();
  }
#endif
}

#endif
