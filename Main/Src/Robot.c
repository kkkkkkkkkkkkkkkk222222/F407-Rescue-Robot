#include "Robot.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "encoder.h"
#include "imu.h"
#include "main.h"
#include "motor.h"
#include "Task.h"
#include "servo.h"
#include "Lcd.h"
#include "vision.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;

#define UART_DMA_RX_SIZE  64U

static volatile uint32_t app_milliseconds;
static uint8_t usart3_rx_dma[UART_DMA_RX_SIZE];
static uint16_t usart3_rx_position;
static volatile uint8_t usart3_last_byte;
static volatile bool usart3_byte_received;
static volatile uint32_t usart3_last_rx_ms;
static volatile bool usart3_rx_active;
static volatile uint32_t usart3_rx_next_retry_ms;
static volatile uint32_t lcd_release_sequence;
static uint32_t lcd_consumed_sequence;
static volatile uint32_t imu_release_sequence;
static uint32_t imu_consumed_sequence;
static uint8_t motor_control_period_ms;
static uint8_t lcd_period_ms;
#if APP_ENABLE_TASK
static uint8_t task_period_ms;
static volatile uint32_t task_release_sequence;
static volatile uint32_t task_release_ms;
static uint32_t task_consumed_sequence;
#endif
static bool lcd_ready;
#if APP_ENABLE_TASK
static uint8_t lcd_task_layout = 0xFFU;
#endif
#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
static bool motor_test_running;
static bool motor_key_sample;
static bool motor_key_stable;
static uint32_t motor_key_change_ms;
#endif

static bool vision_start_receive_dma(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_dma, UART_DMA_RX_SIZE) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    usart3_rx_active = true;
    return true;
  }
  usart3_rx_active = false;
  (void)HAL_UART_AbortReceive(&huart3);
  return false;
}

static void vision_parse_dma_range(uint16_t size)
{
  /*
   * HAL reports Size == UART_DMA_RX_SIZE at every circular-DMA transfer
   * completion. Treat that event as the end of the current revolution and
   * store position 0 for the next revolution. This also makes consecutive
   * full-buffer callbacks consumable when the UART stream never becomes idle.
   */
  if (size == UART_DMA_RX_SIZE) {
    Vision_ParseBytes(&usart3_rx_dma[usart3_rx_position],
                      UART_DMA_RX_SIZE - usart3_rx_position,
                      app_milliseconds);
    usart3_rx_position = 0U;
    return;
  }

  if (size > usart3_rx_position) {
    Vision_ParseBytes(&usart3_rx_dma[usart3_rx_position],
                      size - usart3_rx_position,
                      app_milliseconds);
  } else if (size < usart3_rx_position) {
    Vision_ParseBytes(&usart3_rx_dma[usart3_rx_position],
                      UART_DMA_RX_SIZE - usart3_rx_position,
                      app_milliseconds);
    if (size != 0U) {
      Vision_ParseBytes(usart3_rx_dma, size, app_milliseconds);
    }
  }
  usart3_rx_position = size;
}

#if APP_ENABLE_TASK
static void lcd_draw_task_layout(TaskState state)
{
  LCD_FillScreen(LCD_BLACK);
  LCD_DrawText(15U, 4U, "RESCUE TASK", LCD_YELLOW, LCD_BLACK);
  LCD_DrawText(0U, 20U, "STATE:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 34U, "RX0:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 48U, "RX1:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 62U, "RX2:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 76U, "RX3:", LCD_CYAN, LCD_BLACK);

  switch (state) {
    case TASK_WAIT_CONFIG:
      LCD_DrawText(0U, 94U, "COLOR:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "ZONE:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "CFG:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "UART:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "MOTOR:", LCD_GREEN, LCD_BLACK);
      break;
    case TASK_START:
      LCD_DrawText(0U, 94U, "TIME:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "DIST:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "SPEED:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "ZONE:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "MOTOR:", LCD_GREEN, LCD_BLACK);
      break;
    case TASK_FIND_OBJECT:
      LCD_DrawText(0U, 94U, "TIME:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "FOUND:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "DIST:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "TYPE:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "MOTOR:", LCD_GREEN, LCD_BLACK);
      break;
    case TASK_CRAB_OBJECT:
      LCD_DrawText(0U, 94U, "TIME:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "GRAB:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "COUNT:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "LOAD:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "MOTOR:", LCD_GREEN, LCD_BLACK);
      break;
    case TASK_RETURN_SAFE:
      LCD_DrawText(0U, 94U, "TIME:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "CARGO:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "DEST:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "NEAR:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "NAV:", LCD_GREEN, LCD_BLACK);
      break;
    case TASK_DROP_OBJECT:
      LCD_DrawText(0U, 94U, "TIME:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "PHASE:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "CHECK:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "CARGO:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "MOTOR:", LCD_GREEN, LCD_BLACK);
      break;
    default:
      LCD_DrawText(0U, 94U, "TIME:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 108U, "FOUND:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 122U, "GRAB:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 136U, "COUNT:", LCD_GREEN, LCD_BLACK);
      LCD_DrawText(0U, 150U, "MOTOR:", LCD_GREEN, LCD_BLACK);
      break;
  }
  lcd_task_layout = (uint8_t)state;
}
#endif

static void lcd_draw_static(void)
{
#if APP_ENABLE_TASK
  lcd_draw_task_layout(Task_GetStatus().state);
#else
  LCD_DrawText(6U, 4U, "LCD WHEEL TEST", LCD_YELLOW, LCD_BLACK);
  LCD_DrawText(0U, 24U, "M1:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 38U, "M2:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 52U, "M3:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 86U, "UART3:", LCD_GREEN, LCD_BLACK);
  LCD_DrawText(0U, 104U, "SERVO:", LCD_GREEN, LCD_BLACK);
  LCD_DrawText(0U, 122U, "WHEEL:", LCD_GREEN, LCD_BLACK);
#endif
}

#if APP_ENABLE_TASK
static void lcd_write_value(uint16_t y, const char *text)
{
  LCD_FillRect(42U, y, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, y, text, LCD_WHITE, LCD_BLACK);
}

static void lcd_write_rx(uint16_t y, const char *text)
{
  LCD_FillRect(30U, y, 98U, 8U, LCD_BLACK);
  LCD_DrawText(30U, y, text, LCD_WHITE, LCD_BLACK);
}

static void lcd_format_counts(char *text, size_t size, uint8_t counts)
{
  (void)snprintf(text, size, "N%u C%u H%u D%u",
                 VISION_COUNT_NORMAL(counts), VISION_COUNT_CORE(counts),
                 VISION_COUNT_CASUALTY(counts), VISION_COUNT_DANGER(counts));
}

static const char *lcd_nav_text(uint8_t direction)
{
  switch (direction) {
    case VISION_NAV_FORWARD:
      return "FORWARD";
    case VISION_NAV_TURN_LEFT:
      return "LEFT";
    case VISION_NAV_TURN_RIGHT:
      return "RIGHT";
    case VISION_NAV_BACKWARD:
      return "BACK";
    default:
      return "HOLD";
  }
}

static const char *lcd_drop_text(TaskDropPhase phase)
{
  switch (phase) {
    case TASK_DROP_ENTER:
      return "ENTER";
    case TASK_DROP_RELEASE:
      return "RELEASE";
    case TASK_DROP_CAMERA:
      return "CAMERA";
    case TASK_DROP_VERIFY:
      return "VERIFY";
    case TASK_DROP_BACK:
      return "LEAVE";
    case TASK_DROP_RETRY_BACK:
      return "RETRY";
    default:
      return "ERROR";
  }
}
#endif

static void lcd_draw_dynamic(void)
{
  char text[24];
  bool motor_fault = false;

#if APP_ENABLE_TASK
  const TaskStatus task = Task_GetStatus();
  const VisionData vision = Vision_GetSnapshot();
  if (lcd_task_layout != (uint8_t)task.state) {
    lcd_draw_task_layout(task.state);
  }
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    motor_fault = motor_fault || motor.direction_fault || motor.stall_fault;
  }

  switch (task.state) {
    case TASK_WAIT_CONFIG:
      (void)strcpy(text, "WAIT_CONFIG");
      break;
    case TASK_START:
      (void)strcpy(text, "START");
      break;
    case TASK_FIND_OBJECT:
      (void)strcpy(text, "FIND_OBJECT");
      break;
    case TASK_CRAB_OBJECT:
      (void)strcpy(text, "CRAB_OBJECT");
      break;
    case TASK_RETURN_SAFE:
      (void)strcpy(text, "RETURN_SAFE");
      break;
    case TASK_DROP_OBJECT:
      (void)strcpy(text, "DROP_OBJECT");
      break;
    default:
      (void)strcpy(text, "STOPPED");
      break;
  }
  lcd_write_value(20U, text);

  if ((vision.last_frame[0] == VISION_FRAME_HEAD_1) &&
      (vision.last_frame[1] == VISION_FRAME_HEAD_2) &&
      (vision.last_frame[14] == VISION_FRAME_TAIL)) {
    (void)snprintf(text, sizeof(text), "%02X %02X %02X %02X",
                   vision.last_frame[0], vision.last_frame[1],
                   vision.last_frame[2], vision.last_frame[3]);
    lcd_write_rx(34U, text);
    (void)snprintf(text, sizeof(text), "%02X %02X %02X %02X",
                   vision.last_frame[4], vision.last_frame[5],
                   vision.last_frame[6], vision.last_frame[7]);
    lcd_write_rx(48U, text);
    (void)snprintf(text, sizeof(text), "%02X %02X %02X %02X",
                   vision.last_frame[8], vision.last_frame[9],
                   vision.last_frame[10], vision.last_frame[11]);
    lcd_write_rx(62U, text);
    (void)snprintf(text, sizeof(text), "%02X %02X %02X",
                   vision.last_frame[12], vision.last_frame[13],
                   vision.last_frame[14]);
    lcd_write_rx(76U, text);
  } else {
    lcd_write_rx(34U, "-- -- -- --");
    lcd_write_rx(48U, "-- -- -- --");
    lcd_write_rx(62U, "-- -- -- --");
    lcd_write_rx(76U, "-- -- --");
  }

  if (!usart3_rx_active) {
    (void)strcpy(text, "DMA ERR");
  } else if (usart3_byte_received &&
             ((uint32_t)(app_milliseconds - usart3_last_rx_ms) <=
              APP_VISION_TIMEOUT_MS)) {
    (void)strcpy(text, "RX OK");
  } else if (usart3_byte_received) {
    (void)strcpy(text, "TIMEOUT");
  } else {
    (void)strcpy(text, "WAIT");
  }
  char uart_text[sizeof(text)];
  (void)strcpy(uart_text, text);

  const char *color_text = task.color == VISION_COLOR_RED ? "RED" :
                           (task.color == VISION_COLOR_BLUE ? "BLUE" : "--");
  switch (task.state) {
    case TASK_WAIT_CONFIG:
      lcd_write_value(94U, color_text);
      if (task.start_zone == 0U) {
        lcd_write_value(108U, "--");
      } else {
        (void)snprintf(text, sizeof(text), "%u", task.start_zone);
        lcd_write_value(108U, text);
      }
      lcd_write_value(122U, vision.config_ready ? "OK" : "WAIT 3");
      lcd_write_value(136U, uart_text);
      lcd_write_value(150U, motor_fault ? "FAULT" : "OK");
      break;
    case TASK_START:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      lcd_write_value(94U, text);
      (void)snprintf(text, sizeof(text), "%umm", task.distance_mm);
      lcd_write_value(108U, text);
      (void)snprintf(text, sizeof(text), "%ldMM/S",
                     (long)APP_GO_DISTANCE_SPEED_MM_S);
      lcd_write_value(122U, text);
      (void)snprintf(text, sizeof(text), "%u", task.start_zone);
      lcd_write_value(136U, text);
      lcd_write_value(150U, motor_fault ? "FAULT" : "OK");
      break;
    case TASK_FIND_OBJECT:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      lcd_write_value(94U, text);
      lcd_write_value(108U, task.found ? "YES" : "NO");
      (void)snprintf(text, sizeof(text), "%umm", task.distance_mm);
      lcd_write_value(122U, text);
      lcd_format_counts(text, sizeof(text), task.cargo_counts);
      lcd_write_value(136U, text);
      lcd_write_value(150U, motor_fault ? "FAULT" : "OK");
      break;
    case TASK_CRAB_OBJECT:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      lcd_write_value(94U, text);
      lcd_write_value(108U, task.grabbed ? "YES" : "NO");
      (void)snprintf(text, sizeof(text), "%u", task.object_count);
      lcd_write_value(122U, text);
      if (vision.unknown) {
        lcd_write_value(136U, "UNKNOWN");
      } else if (VISION_COUNT_DANGER(task.cargo_counts) != 0U) {
        lcd_write_value(136U, "DANGER");
      } else if (task.cargo_valid) {
        lcd_write_value(136U, "VALID");
      } else if (task.grabbed) {
        lcd_write_value(136U, "VERIFY");
      } else {
        lcd_write_value(136U, "APPROACH");
      }
      lcd_write_value(150U, motor_fault ? "FAULT" : "OK");
      break;
    case TASK_RETURN_SAFE:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      lcd_write_value(94U, text);
      lcd_format_counts(text, sizeof(text), task.cargo_counts);
      lcd_write_value(108U, text);
      lcd_write_value(122U,
                      task.destination == TASK_DEST_MATERIAL ? "MATERIAL" :
                      (task.destination == TASK_DEST_CASUALTY ? "CASUALTY" : "--"));
      lcd_write_value(136U, task.near_safe ? "YES" : "NO");
      lcd_write_value(150U,
                      task.nav_fresh ? lcd_nav_text(task.nav_direction) :
                                       "TIMEOUT");
      break;
    case TASK_DROP_OBJECT:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      lcd_write_value(94U, text);
      lcd_write_value(108U, lcd_drop_text(task.drop_phase));
      if (task.drop_phase < TASK_DROP_VERIFY) {
        lcd_write_value(122U, "WAIT");
      } else if (task.claw_empty) {
        lcd_write_value(122U, "EMPTY");
      } else if (task.drop_phase == TASK_DROP_RETRY_BACK) {
        lcd_write_value(122U, "LOADED");
      } else {
        lcd_write_value(122U, "VISION");
      }
      lcd_format_counts(text, sizeof(text), task.cargo_counts);
      lcd_write_value(136U, text);
      lcd_write_value(150U, motor_fault ? "FAULT" : "OK");
      break;
    default:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      lcd_write_value(94U, text);
      lcd_write_value(108U, task.found ? "YES" : "NO");
      lcd_write_value(122U, task.grabbed ? "YES" : "NO");
      (void)snprintf(text, sizeof(text), "%u", task.object_count);
      lcd_write_value(136U, text);
      lcd_write_value(150U, motor_fault ? "FAULT" : "OK");
      break;
  }
#else
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const EncoderStatus encoder = Encoder_GetStatus(id);
    const MotorStatus motor = Motor_GetStatus(id);
    motor_fault = motor_fault || motor.direction_fault || motor.stall_fault;
    (void)snprintf(text, sizeof(text), "%lld", (long long)encoder.position);
    const uint16_t y = (uint16_t)(24U + ((id - 1U) * 14U));
    LCD_FillRect(24U, y, 104U, 8U, LCD_BLACK);
    LCD_DrawText(24U, y, text, LCD_WHITE, LCD_BLACK);
  }

  if (!usart3_rx_active) {
    (void)strcpy(text, "DMA ERR");
  } else if (usart3_byte_received &&
             ((uint32_t)(app_milliseconds - usart3_last_rx_ms) <=
              APP_VISION_TIMEOUT_MS)) {
    (void)snprintf(text, sizeof(text), "RX %02X", usart3_last_byte);
  } else if (usart3_byte_received) {
    (void)strcpy(text, "TIMEOUT");
  } else {
    (void)strcpy(text, "WAIT");
  }
  LCD_FillRect(42U, 86U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 86U, text, LCD_WHITE, LCD_BLACK);
  LCD_FillRect(42U, 104U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 104U, "READY", LCD_WHITE, LCD_BLACK);

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST
  if (motor_fault) {
    (void)strcpy(text, "FAULT RESET");
  } else if (motor_test_running) {
    (void)snprintf(text, sizeof(text), "PID %ldMM",
                   (long)APP_MOTOR_SPEED_TEST_TARGET_MM_S);
  } else {
    (void)strcpy(text, "KEY START");
  }
#else
  (void)strcpy(text, "STOP");
#endif
  LCD_FillRect(42U, 122U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 122U, text, LCD_WHITE, LCD_BLACK);
#endif
}

#if APP_ENABLE_SERVO_SWEEP_TEST && !APP_ENABLE_TASK
static void run_servo_sweep(uint32_t now)
{
  static uint32_t next_change;
  static uint8_t stage;
  static const uint8_t angles[] = {0U, 90U, 180U, 90U};
  if ((int32_t)(now - next_change) < 0) {
    return;
  }
  for (uint8_t id = 1U; id <= 4U; ++id) {
    Servo_Set(id, angles[stage]);
  }
  stage = (uint8_t)((stage + 1U) % 4U);
  next_change = now + 1500U;
}
#endif

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
static bool automatic_motor_test_has_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static void process_motor_test_key(uint32_t now)
{
  const bool sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;

  if (sample != motor_key_sample) {
    motor_key_sample = sample;
    motor_key_change_ms = now;
  }
  if ((sample == motor_key_stable) ||
      ((uint32_t)(now - motor_key_change_ms) < APP_MOTOR_KEY_DEBOUNCE_MS)) {
    return;
  }

  motor_key_stable = sample;
  if (!motor_key_stable) {
    return;
  }

  if (automatic_motor_test_has_fault()) {
    motor_test_running = false;
    return;
  }
  if (motor_test_running) {
    Motor_Stop();
    motor_test_running = false;
    return;
  }
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t id = 1U; id <= 3U; ++id) {
    Motor_SetSpeed(APP_MOTOR_SPEED_TEST_TARGET_MM_S, id);
  }
  if (primask == 0U) {
    __enable_irq();
  }
  motor_test_running = true;
}
#endif

void Robot_Init(void)
{
  app_milliseconds = 0U;
  usart3_last_byte = 0U;
  usart3_byte_received = false;
  usart3_last_rx_ms = 0U;
  usart3_rx_active = false;
  usart3_rx_next_retry_ms = 0U;
  usart3_rx_position = 0U;
  lcd_release_sequence = 0U;
  lcd_consumed_sequence = 0U;
  imu_release_sequence = 0U;
  imu_consumed_sequence = 0U;
  motor_control_period_ms = 0U;
  lcd_period_ms = 0U;
#if APP_ENABLE_TASK
  task_period_ms = 0U;
  task_release_sequence = 0U;
  task_release_ms = 0U;
  task_consumed_sequence = 0U;
#endif
#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  motor_test_running = false;
  motor_key_sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;
  motor_key_stable = motor_key_sample;
  motor_key_change_ms = 0U;
#endif
  Vision_Init();

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
#if APP_ENABLE_TASK
  HAL_NVIC_SetPriority(PendSV_IRQn, 15U, 0U);
#endif

  Motor_Init();
  Encoder_Init();
  const bool imu_ready = IMU_Init();
  /* Start servos after gyro calibration so their startup motion cannot bias it. */
  Servo_Init();
  if (!vision_start_receive_dma()) {
    usart3_rx_next_retry_ms = 100U;
  }

#if APP_ENABLE_TASK
  /* Initialize the non-blocking task before drawing its first LCD snapshot. */
  Task_FindObject(app_milliseconds);
#endif

  lcd_ready = LCD_Init();
  if (lcd_ready) {
    LCD_FillScreen(LCD_BLACK);
    if (imu_ready) {
      LCD_DrawText(25U, 76U, "IMU660RC: OK", LCD_GREEN, LCD_BLACK);
    } else {
      LCD_DrawText(19U, 76U, "IMU660RC: ERROR", LCD_RED, LCD_BLACK);
    }
    HAL_Delay(1000U);
    LCD_FillScreen(LCD_BLACK);
    lcd_draw_static();
    lcd_draw_dynamic();
  }

  /* Start the real-time schedule only after all blocking startup work. */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }
}

void Robot_Process(void)
{
#if (!APP_ENABLE_TASK && APP_ENABLE_SERVO_SWEEP_TEST) || \
    (APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK)
  const uint32_t now = app_milliseconds;
#endif

  if (!usart3_rx_active &&
      ((int32_t)(app_milliseconds - usart3_rx_next_retry_ms) >= 0)) {
    usart3_rx_position = 0U;
    Vision_ResetParser();
    if (!vision_start_receive_dma()) {
      usart3_rx_next_retry_ms = app_milliseconds + 100U;
    }
  }
  Vision_Process();

  const uint32_t imu_released = imu_release_sequence;
  if (imu_released != imu_consumed_sequence) {
    imu_consumed_sequence = imu_released;
    IMU_Update(app_milliseconds);
  }

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  process_motor_test_key(now);
#endif

  /* Coalesce delayed low-priority work instead of replaying stale periods. */
  const uint32_t released = lcd_release_sequence;
  if (released != lcd_consumed_sequence) {
    lcd_consumed_sequence = released;
    if (lcd_ready) {
      lcd_draw_dynamic();
    }
  }

#if APP_ENABLE_SERVO_SWEEP_TEST && !APP_ENABLE_TASK
  run_servo_sweep(now);
#endif
}

void Robot_RunDeferredTask(void)
{
#if APP_ENABLE_TASK
  const uint32_t released = task_release_sequence;
  if (released != task_consumed_sequence) {
    const uint32_t now_ms = task_release_ms;
    task_consumed_sequence = released;
    Task_FindObject(now_ms);
  }

  /* A TIM6 release arriving while the task ran must remain pending. */
  if (task_release_sequence != task_consumed_sequence) {
    __DMB();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  }
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM6) {
    bool motor_update_due = false;

    ++app_milliseconds;

    if (++motor_control_period_ms >= APP_MOTOR_CONTROL_PERIOD_MS) {
      motor_control_period_ms = 0U;
      Encoder_Sample10ms();
      motor_update_due = true;
    }

#if APP_ENABLE_TASK
    if (++task_period_ms >= APP_TASK_PERIOD_MS) {
      task_period_ms = 0U;
      task_release_ms = app_milliseconds;
      ++task_release_sequence;
      __DMB();
      SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
#endif

    if (motor_update_due) {
      Motor_Update();
      ++imu_release_sequence;
    }

    if (++lcd_period_ms >= 100U) {
      lcd_period_ms = 0U;
      ++lcd_release_sequence;
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  if ((size == 0U) || (size > UART_DMA_RX_SIZE)) {
    return;
  }

  if (uart->Instance == USART3) {
    usart3_rx_active = true;
    usart3_last_byte = usart3_rx_dma[size - 1U];
    usart3_byte_received = true;
    usart3_last_rx_ms = app_milliseconds;
    vision_parse_dma_range(size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART3) {
    Vision_OnTxError();
    usart3_rx_active = false;
    (void)HAL_UART_AbortReceive(&huart3);
    usart3_rx_position = 0U;
    Vision_ResetParser();
    if (!vision_start_receive_dma()) {
      usart3_rx_next_retry_ms = app_milliseconds + 100U;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART3) {
    Vision_OnTxComplete();
  }
}
