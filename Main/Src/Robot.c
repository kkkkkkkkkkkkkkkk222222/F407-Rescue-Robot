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

#define UART1_TX_MASK  (1U << 0)
#define UART3_TX_MASK  (1U << 1)
#define UART_DMA_RX_SIZE  64U

static volatile uint32_t app_milliseconds;
static uint8_t usart1_rx_dma[UART_DMA_RX_SIZE];
static uint8_t usart3_rx_dma[UART_DMA_RX_SIZE];
static uint16_t usart3_rx_position;
static volatile uint8_t usart1_last_byte;
static volatile uint8_t usart3_last_byte;
static volatile bool usart1_byte_received;
static volatile bool usart3_byte_received;
static volatile uint8_t uart_tx_busy_mask;
static volatile uint32_t report_release_sequence;
static uint32_t report_consumed_sequence;
static uint8_t encoder_period_ms;
static uint8_t report_period_ms;
#if APP_ENABLE_TASK
static uint8_t task_period_ms;
#endif
static char telemetry_message[512];
static bool lcd_ready;

static void uart_start_receive_dma(UART_HandleTypeDef *uart, uint8_t *buffer)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(uart, buffer, UART_DMA_RX_SIZE) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
  }
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
  LCD_DrawText(6U, 4U, "STM32F407 TEST", LCD_YELLOW, LCD_BLACK);
  LCD_DrawText(0U, 24U, "M1:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 38U, "M2:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 52U, "M3:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 66U, "M4:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 86U, "UART:", LCD_GREEN, LCD_BLACK);
  LCD_DrawText(0U, 104U, "SERVO:", LCD_GREEN, LCD_BLACK);
  LCD_DrawText(0U, 122U, "MOTOR:", LCD_GREEN, LCD_BLACK);
}

static void lcd_draw_dynamic(void)
{
  char text[20];
  char usart1_text[3];
  char usart3_text[3];
  for (uint8_t id = 1U; id <= 4U; ++id) {
    (void)snprintf(text, sizeof(text), "%ld", (long)Encoder_Get(id));
    const uint16_t y = (uint16_t)(24U + ((id - 1U) * 14U));
    LCD_FillRect(24U, y, 104U, 8U, LCD_BLACK);
    LCD_DrawText(24U, y, text, LCD_WHITE, LCD_BLACK);
  }

  if (usart1_byte_received) {
    (void)snprintf(usart1_text, sizeof(usart1_text), "%02X", usart1_last_byte);
  } else {
    (void)strcpy(usart1_text, "--");
  }
  if (usart3_byte_received) {
    (void)snprintf(usart3_text, sizeof(usart3_text), "%02X", usart3_last_byte);
  } else {
    (void)strcpy(usart3_text, "--");
  }
  (void)snprintf(text, sizeof(text), "1:%s 3:%s", usart1_text, usart3_text);
  LCD_FillRect(36U, 86U, 92U, 8U, LCD_BLACK);
  LCD_DrawText(36U, 86U, text, LCD_WHITE, LCD_BLACK);

  (void)strcpy(text, "READY");
  LCD_FillRect(42U, 104U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 104U, text, LCD_WHITE, LCD_BLACK);

#if APP_ENABLE_TASK
  (void)strcpy(text, "TASK ACTIVE");
#else
  const int16_t active_command = Motor_GetCommand(1U);
  if (active_command == 0) {
    (void)strcpy(text, "STOP");
  } else {
    (void)snprintf(text, sizeof(text), "ALL %+d%%", active_command / 10);
  }
#endif
  LCD_FillRect(42U, 122U, 86U, 8U, LCD_BLACK);
  LCD_DrawText(42U, 122U, text, LCD_WHITE, LCD_BLACK);
}

static void send_telemetry(void)
{
  const VisionData vision = Vision_GetSnapshot();
  const char *vision_state = vision.stop ? "STOP" :
                             (vision.valid ? "TARGET" : "NONE");

  if (uart_tx_busy_mask != 0U) {
    return;
  }

  const int length = snprintf(
      telemetry_message, sizeof(telemetry_message),
      "Encoder: M1=%ld(%ld) M2=%ld(%ld) M3=%ld(%ld) M4=%ld(%ld)\r\n"
      "Servo: S1=%u S2=%u S3=%u S4=%u\r\n"
      "PWM: motor=20kHz servo=50Hz\r\n"
      "Motor: M1=%d/%d M2=%d/%d M3=%d/%d M4=%d/RES\r\n"
      "Vision3: %s X=%u Y=%u age=%lums task=%s\r\n"
      "USART1: %s 0x%02X  USART3: %s 0x%02X\r\n\r\n",
      (long)Encoder_Get(1U), (long)Encoder_GetDelta10ms(1U),
      (long)Encoder_Get(2U), (long)Encoder_GetDelta10ms(2U),
      (long)Encoder_Get(3U), (long)Encoder_GetDelta10ms(3U),
      (long)Encoder_Get(4U), (long)Encoder_GetDelta10ms(4U),
      Servo_GetAngle(1U), Servo_GetAngle(2U), Servo_GetAngle(3U), Servo_GetAngle(4U),
      Motor_GetCommand(1U), Motor_GetTarget(1U),
      Motor_GetCommand(2U), Motor_GetTarget(2U),
      Motor_GetCommand(3U), Motor_GetTarget(3U),
      Motor_GetCommand(4U),
      vision_state, vision.x, vision.y,
      (unsigned long)(app_milliseconds - vision.tick_ms),
#if APP_ENABLE_TASK
      "ON",
#else
      "OFF",
#endif
      usart1_byte_received ? "RX" : "WAIT", usart1_last_byte,
      usart3_byte_received ? "RX" : "WAIT", usart3_last_byte);
  if (length > 0) {
    const uint16_t size = (length < (int)sizeof(telemetry_message))
                              ? (uint16_t)length
                              : (uint16_t)(sizeof(telemetry_message) - 1U);
    uart_tx_busy_mask = UART1_TX_MASK | UART3_TX_MASK;
    if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)telemetry_message, size) != HAL_OK) {
      uart_tx_busy_mask &= (uint8_t)~UART1_TX_MASK;
    }
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)telemetry_message, size) != HAL_OK) {
      uart_tx_busy_mask &= (uint8_t)~UART3_TX_MASK;
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
static uint16_t motor_test_command = APP_MOTOR_TEST_DEFAULT_COMMAND;
static int8_t motor_test_direction;

static void motor_set_all(int16_t command)
{
  for (uint8_t id = 1U; id <= 4U; ++id) {
    Motor_Control(id, command);
  }
}

static void process_motor_key(uint32_t now)
{
  static GPIO_PinState previous_raw = GPIO_PIN_RESET;
  static GPIO_PinState stable_state = GPIO_PIN_RESET;
  static uint32_t raw_changed_at;
  const GPIO_PinState raw_state = HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin);

  if (raw_state != previous_raw) {
    previous_raw = raw_state;
    raw_changed_at = now;
  }
  if ((raw_state != stable_state) &&
      ((uint32_t)(now - raw_changed_at) >= APP_MOTOR_KEY_DEBOUNCE_MS)) {
    stable_state = raw_state;
    if (stable_state == GPIO_PIN_SET) {
      motor_test_command += APP_MOTOR_TEST_COMMAND_STEP;
      if (motor_test_command > 1000U) {
        motor_test_command = APP_MOTOR_TEST_MIN_COMMAND;
      }
      if (motor_test_direction != 0) {
        motor_set_all((int16_t)(motor_test_direction * (int16_t)motor_test_command));
      }
    }
  }
}

static void run_motor_test(uint32_t now)
{
  static uint32_t next_change;
  static uint8_t stage;
  if ((int32_t)(now - next_change) < 0) {
    return;
  }

  switch (stage) {
    case 0U:
      motor_test_direction = 1;
      motor_set_all((int16_t)motor_test_command);
      next_change = now + APP_MOTOR_TEST_DIRECTION_MS;
      stage = 1U;
      break;
    case 1U:
      motor_test_direction = 0;
      Motor_StopAll();
      next_change = now + APP_MOTOR_TEST_REVERSAL_PAUSE_MS;
      stage = 2U;
      break;
    case 2U:
      motor_test_direction = -1;
      motor_set_all(-(int16_t)motor_test_command);
      next_change = now + APP_MOTOR_TEST_DIRECTION_MS;
      stage = 3U;
      break;
    default:
      motor_test_direction = 0;
      Motor_StopAll();
      next_change = now + APP_MOTOR_TEST_REVERSAL_PAUSE_MS;
      stage = 0U;
      break;
  }
}
#endif

void Robot_Init(void)
{
  app_milliseconds = 0U;
  usart1_last_byte = 0U;
  usart3_last_byte = 0U;
  usart1_byte_received = false;
  usart3_byte_received = false;
  usart3_rx_position = 0U;
  uart_tx_busy_mask = 0U;
  report_release_sequence = 0U;
  report_consumed_sequence = 0U;
  encoder_period_ms = 0U;
  report_period_ms = 0U;
#if APP_ENABLE_TASK
  task_period_ms = 0U;
#endif
  Vision_Init();

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4U, 0U);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 7U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  Motor_Init();
  Encoder_Init();
  Servo_Init();
  uart_start_receive_dma(&huart1, usart1_rx_dma);
  uart_start_receive_dma(&huart3, usart3_rx_dma);

#if APP_ENABLE_TASK
  Task_Init(app_milliseconds);
#endif

  lcd_ready = LCD_Init();
  if (lcd_ready) {
#if APP_LCD_STARTUP_COLOR_TEST
    LCD_FillScreen(LCD_RED);
    HAL_Delay(250U);
    LCD_FillScreen(LCD_GREEN);
    HAL_Delay(250U);
    LCD_FillScreen(LCD_BLUE);
    HAL_Delay(250U);
    LCD_FillScreen(LCD_BLACK);
#endif
    lcd_draw_static();
    lcd_draw_dynamic();
  }

  static const char banner[] =
      "\r\nSTM32F407 board test ready. USART1 and USART3 are active. "
#if APP_ENABLE_TASK
      "LED_3-derived robot task is enabled; USART3 receives vision frames. "
#elif APP_ENABLE_AUTOMATIC_MOTOR_TEST
      "All-motor reversing test starts at 50%; PA0 cycles duty in 10% steps. "
#else
      "Automatic motor motion is disabled. "
#endif
      "Keep wheels lifted.\r\n";
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1U), 100U);
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)banner, (uint16_t)(sizeof(banner) - 1U), 100U);

  /* Start the real-time schedule only after all blocking startup work. */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }
}

void Robot_Process(void)
{
#if !APP_ENABLE_TASK && (APP_ENABLE_AUTOMATIC_MOTOR_TEST || APP_ENABLE_SERVO_SWEEP_TEST)
  const uint32_t now = app_milliseconds;
#endif

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  process_motor_key(now);
  run_motor_test(now);
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
    ++app_milliseconds;

    if (++encoder_period_ms >= 10U) {
      encoder_period_ms = 0U;
      Encoder_Sample10ms();
    }

#if APP_ENABLE_TASK
    if (++task_period_ms >= APP_TASK_PERIOD_MS) {
      task_period_ms = 0U;
      /* Encoder sampling above runs first on common 20 ms boundaries. */
      Task_Update(app_milliseconds);
    }
#endif

    if (++report_period_ms >= 100U) {
      report_period_ms = 0U;
      ++report_release_sequence;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  Encoder_OnExti(gpio_pin);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  if ((size == 0U) || (size > UART_DMA_RX_SIZE)) {
    return;
  }

  if (uart->Instance == USART1) {
    usart1_last_byte = usart1_rx_dma[size - 1U];
    usart1_byte_received = true;
  } else if (uart->Instance == USART3) {
    usart3_last_byte = usart3_rx_dma[size - 1U];
    usart3_byte_received = true;
    vision_parse_dma_range(size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART1) {
    (void)HAL_UART_AbortReceive(&huart1);
    uart_start_receive_dma(&huart1, usart1_rx_dma);
  } else if (uart->Instance == USART3) {
    (void)HAL_UART_AbortReceive(&huart3);
    usart3_rx_position = 0U;
    Vision_ResetParser();
    uart_start_receive_dma(&huart3, usart3_rx_dma);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART1) {
    uart_tx_busy_mask &= (uint8_t)~UART1_TX_MASK;
  } else if (uart->Instance == USART3) {
    uart_tx_busy_mask &= (uint8_t)~UART3_TX_MASK;
  }
}
