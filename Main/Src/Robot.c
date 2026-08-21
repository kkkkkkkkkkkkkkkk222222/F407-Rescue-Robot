#include "Robot.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "encoder.h"
#include "main.h"
#include "motor.h"
#include "Task.h"
#include "servo.h"
#include "Lcd.h"
#include "vision.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

#define UART_DMA_RX_SIZE  64U

static volatile uint32_t app_milliseconds;
static uint8_t usart3_rx_dma[UART_DMA_RX_SIZE];
static uint16_t usart3_rx_position;
static volatile uint8_t usart3_last_byte;
static volatile bool usart3_byte_received;
static volatile bool usart3_rx_active;
static volatile uint32_t usart3_rx_next_retry_ms;
static volatile bool usart1_tx_busy;
static volatile uint32_t report_release_sequence;
static uint32_t report_consumed_sequence;
static uint8_t motor_control_period_ms;
static uint8_t report_period_ms;
#if APP_ENABLE_TASK
static uint8_t task_period_ms;
#endif
static char telemetry_message[512];
static bool lcd_ready;
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

static void lcd_draw_static(void)
{
  LCD_DrawText(6U, 4U, "LCD WHEEL TEST", LCD_YELLOW, LCD_BLACK);
  LCD_DrawText(0U, 24U, "M1:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 38U, "M2:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 52U, "M3:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 86U, "UART3:", LCD_GREEN, LCD_BLACK);
  LCD_DrawText(0U, 104U, "SERVO:", LCD_GREEN, LCD_BLACK);
  LCD_DrawText(0U, 122U, "WHEEL:", LCD_GREEN, LCD_BLACK);
}

static void lcd_draw_dynamic(void)
{
  char text[24];
  bool motor_fault = false;
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
  } else if (usart3_byte_received) {
    (void)snprintf(text, sizeof(text), "RX %02X", usart3_last_byte);
  } else {
    (void)strcpy(text, "WAIT");
  }
  LCD_FillRect(42U, 86U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 86U, text, LCD_WHITE, LCD_BLACK);

  (void)strcpy(text, "READY");
  LCD_FillRect(42U, 104U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 104U, text, LCD_WHITE, LCD_BLACK);

#if APP_ENABLE_TASK
  (void)strcpy(text, motor_fault ? "FAULT RESET" : "TASK ACTIVE");
#elif APP_ENABLE_AUTOMATIC_MOTOR_TEST
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
}

static void send_telemetry(void)
{
  const VisionData vision = Vision_GetSnapshot();
  const MotorStatus motor1 = Motor_GetStatus(1U);
  const MotorStatus motor2 = Motor_GetStatus(2U);
  const MotorStatus motor3 = Motor_GetStatus(3U);
  const EncoderStatus encoder1 = Encoder_GetStatus(1U);
  const EncoderStatus encoder2 = Encoder_GetStatus(2U);
  const EncoderStatus encoder3 = Encoder_GetStatus(3U);
  const char *vision_state = vision.stop ? "STOP" :
                             (vision.valid ? "TARGET" : "NONE");

  if (usart1_tx_busy) {
    return;
  }

  const int length = snprintf(
      telemetry_message, sizeof(telemetry_message),
      "Encoder: M1=%lld(%ld) M2=%lld(%ld) M3=%lld(%ld)\r\n"
      "Servo: S1=%u S2=%u S3=%u S4=%u\r\n"
      "PWM: motor=20kHz servo=50Hz\r\n"
      "Motor: M1=%d/%d/%s M2=%d/%d/%s M3=%d/%d/%s\r\n"
      "Speed(mm/s actual/target): M1=%d/%d M2=%d/%d M3=%d/%d\r\n"
      "Vision3: %s X=%u Y=%u age=%lums task=%s\r\n"
      "USART3 RX: %s 0x%02X\r\n\r\n",
      (long long)encoder1.position, (long)encoder1.delta_10ms,
      (long long)encoder2.position, (long)encoder2.delta_10ms,
      (long long)encoder3.position, (long)encoder3.delta_10ms,
      Servo_GetAngle(1U), Servo_GetAngle(2U), Servo_GetAngle(3U), Servo_GetAngle(4U),
      motor1.command, motor1.target,
      motor1.direction_fault ? "DIR" : (motor1.stall_fault ? "STALL" : "OK"),
      motor2.command, motor2.target,
      motor2.direction_fault ? "DIR" : (motor2.stall_fault ? "STALL" : "OK"),
      motor3.command, motor3.target,
      motor3.direction_fault ? "DIR" : (motor3.stall_fault ? "STALL" : "OK"),
      motor1.measured_speed_mm_s, motor1.target_speed_mm_s,
      motor2.measured_speed_mm_s, motor2.target_speed_mm_s,
      motor3.measured_speed_mm_s, motor3.target_speed_mm_s,
      vision_state, vision.x, vision.y,
      (unsigned long)(app_milliseconds - vision.tick_ms),
#if APP_ENABLE_TASK
      "ON",
#else
      "OFF",
#endif
      !usart3_rx_active ? "ERR" : (usart3_byte_received ? "RX" : "WAIT"),
      usart3_last_byte);
  if (length > 0) {
    const uint16_t size = (length < (int)sizeof(telemetry_message))
                              ? (uint16_t)length
                              : (uint16_t)(sizeof(telemetry_message) - 1U);
    usart1_tx_busy = true;
    if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)telemetry_message, size) != HAL_OK) {
      usart1_tx_busy = false;
    }
  }
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
  usart3_rx_active = false;
  usart3_rx_next_retry_ms = 0U;
  usart3_rx_position = 0U;
  usart1_tx_busy = false;
  report_release_sequence = 0U;
  report_consumed_sequence = 0U;
  motor_control_period_ms = 0U;
  report_period_ms = 0U;
#if APP_ENABLE_TASK
  task_period_ms = 0U;
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
  HAL_NVIC_SetPriority(USART1_IRQn, 7U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  Motor_Init();
  Encoder_Init();
  Servo_Init();
  if (!vision_start_receive_dma()) {
    usart3_rx_next_retry_ms = 100U;
  }

#if APP_ENABLE_TASK
  Task_Init(app_milliseconds);
#endif

  lcd_ready = LCD_Init();
  if (lcd_ready) {
    LCD_FillScreen(LCD_BLACK);
    lcd_draw_static();
    lcd_draw_dynamic();
  }

  static const char banner[] =
      "\r\nSTM32F407 board test ready. USART1 TX and USART3 RX are active. "
#if APP_ENABLE_TASK
      "LED_3-derived robot task is enabled; USART3 receives vision frames. "
#elif APP_ENABLE_AUTOMATIC_MOTOR_TEST
      "Press S1 to start or stop the M1-M3 110 mm/s PID wheel-speed test. "
#else
      "Automatic motor motion is disabled. "
#endif
      "Keep wheels lifted.\r\n";
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1U), 100U);

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

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  process_motor_test_key(now);
#endif

  /* Coalesce delayed low-priority work instead of replaying stale periods. */
  const uint32_t released = report_release_sequence;
  if (released != report_consumed_sequence) {
    report_consumed_sequence = released;
    send_telemetry();
    if (lcd_ready) {
      lcd_draw_dynamic();
    }
  }

#if APP_ENABLE_SERVO_SWEEP_TEST && !APP_ENABLE_TASK
  run_servo_sweep(now);
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
      /* The 10 ms encoder sample above is visible before targets are updated. */
      Task_Update(app_milliseconds);
    }
#endif

    if (motor_update_due) {
      Motor_Update();
    }

    if (++report_period_ms >= 100U) {
      report_period_ms = 0U;
      ++report_release_sequence;
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
    vision_parse_dma_range(size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART3) {
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
  if (uart->Instance == USART1) {
    usart1_tx_busy = false;
  }
}
