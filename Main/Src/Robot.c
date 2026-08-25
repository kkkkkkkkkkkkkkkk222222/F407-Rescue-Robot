#include "Robot.h"

#include <stdbool.h>

#include "app_config.h"
#include "encoder.h"
#include "imu.h"
#include "Lcd.h"
#include "main.h"
#include "motor.h"
#include "servo.h"
#include "Task.h"
#include "vision.h"

extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;

#define UART_DMA_RX_SIZE  64U
#define LCD_PERIOD_MS     100U
#define UART_RETRY_MS     100U

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
#if !APP_ENABLE_MOTION_TEST
static volatile uint32_t imu_release_sequence;
static uint32_t imu_consumed_sequence;
#endif
static uint8_t motor_control_period_ms;
static uint8_t lcd_period_ms;
static bool lcd_ready;

#if APP_ENABLE_TASK
static volatile uint32_t task_release_sequence;
static volatile uint32_t task_release_ms;
static uint32_t task_consumed_sequence;
static uint8_t task_period_ms;
#endif

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
static bool motor_test_running;
static bool motor_key_sample;
static bool motor_key_stable;
static uint32_t motor_key_change_ms;
#endif

#if APP_ENABLE_MOTION_TEST
static bool motion_test_running;
static bool motion_test_rotate;
static bool motion_test_done;
static bool motion_key_sample;
static bool motion_key_stable;
static uint32_t motion_key_change_ms;
static uint32_t motion_phase_start_ms;
#endif

static bool vision_start_receive_dma(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_rx_dma,
                                   UART_DMA_RX_SIZE) == HAL_OK) {
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
  /* Size == buffer size marks the end of one circular-DMA revolution. */
  if (size == UART_DMA_RX_SIZE) {
    Vision_ParseBytes(&usart3_rx_dma[usart3_rx_position],
                      UART_DMA_RX_SIZE - usart3_rx_position,
                      app_milliseconds);
    usart3_rx_position = 0U;
    return;
  }

  if (size > usart3_rx_position) {
    Vision_ParseBytes(&usart3_rx_dma[usart3_rx_position],
                      size - usart3_rx_position, app_milliseconds);
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

static void draw_dashboard(void)
{
  LCDDashboard dashboard = {
    .now_ms = app_milliseconds,
    .uart_last_rx_ms = usart3_last_rx_ms,
    .uart_last_byte = usart3_last_byte,
    .uart_active = usart3_rx_active,
    .uart_received = usart3_byte_received,
    .motor_test_running = false,
    .motion_test_running = false,
    .motion_test_rotate = false,
    .motion_test_done = false
  };
#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  dashboard.motor_test_running = motor_test_running;
#endif
#if APP_ENABLE_MOTION_TEST
  dashboard.motion_test_running = motion_test_running;
  dashboard.motion_test_rotate = motion_test_rotate;
  dashboard.motion_test_done = motion_test_done;
#endif
  LCD_DrawDashboard(&dashboard);
}

#if !APP_ENABLE_TASK && \
    (APP_ENABLE_MOTION_TEST || APP_ENABLE_AUTOMATIC_MOTOR_TEST)
static bool robot_motor_has_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}
#endif

#if APP_ENABLE_SERVO_SWEEP_TEST && !APP_ENABLE_TASK
static void run_servo_sweep(uint32_t now_ms)
{
  static uint32_t next_change_ms;
  static uint8_t stage;
  static const uint8_t angles[] = {0U, 90U, 180U, 90U};

  if ((int32_t)(now_ms - next_change_ms) < 0) {
    return;
  }
  for (uint8_t id = 1U; id <= 4U; ++id) {
    Servo_Set(id, angles[stage]);
  }
  stage = (uint8_t)((stage + 1U) % 4U);
  next_change_ms = now_ms + 1500U;
}
#endif

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
static void process_motor_test_key(uint32_t now_ms)
{
  const bool sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;

  if (sample != motor_key_sample) {
    motor_key_sample = sample;
    motor_key_change_ms = now_ms;
  }
  if ((sample == motor_key_stable) ||
      ((uint32_t)(now_ms - motor_key_change_ms) < APP_MOTOR_KEY_DEBOUNCE_MS)) {
    return;
  }

  motor_key_stable = sample;
  if (!sample) {
    return;
  }
  if (robot_motor_has_fault()) {
    motor_test_running = false;
  } else if (motor_test_running) {
    Motor_Stop();
    motor_test_running = false;
  } else {
    for (uint8_t id = 1U; id <= 3U; ++id) {
      Motor_SetSpeed(APP_MOTOR_SPEED_TEST_TARGET_MM_S, id);
    }
    motor_test_running = true;
  }
}
#endif

#if APP_ENABLE_MOTION_TEST
static bool motion_key_pressed(uint32_t now_ms)
{
  const bool sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;

  if (sample != motion_key_sample) {
    motion_key_sample = sample;
    motion_key_change_ms = now_ms;
  }
  if ((sample == motion_key_stable) ||
      ((uint32_t)(now_ms - motion_key_change_ms) <
       APP_MOTOR_KEY_DEBOUNCE_MS)) {
    return false;
  }
  motion_key_stable = sample;
  return sample;
}

static void start_motion_test(uint32_t now_ms)
{
  motion_test_running = true;
  motion_test_rotate = false;
  motion_test_done = false;
  motion_phase_start_ms = now_ms;
  Motor_Move(-APP_MOTION_TEST_BACKWARD_MM_S, 0.0f, 0.0f);
}

static void process_motion_test(uint32_t now_ms)
{
  const bool pressed = motion_key_pressed(now_ms);

  if (robot_motor_has_fault()) {
    if (motion_test_running) {
      Motor_Stop();
    }
    motion_test_running = false;
    motion_test_rotate = false;
    return;
  }

  if (pressed) {
    if (motion_test_running) {
      Motor_Stop();
      motion_test_running = false;
      motion_test_rotate = false;
      motion_test_done = false;
    } else {
      start_motion_test(now_ms);
    }
    return;
  }
  if (!motion_test_running) {
    return;
  }

  const uint32_t elapsed_ms = now_ms - motion_phase_start_ms;
  if (!motion_test_rotate &&
      (elapsed_ms >= APP_MOTION_TEST_BACKWARD_MS)) {
    motion_test_rotate = true;
    motion_phase_start_ms = now_ms;
    Motor_Move(0.0f, 0.0f, APP_MOTION_TEST_ROTATE_MM_S);
  } else if (motion_test_rotate &&
             (elapsed_ms >= APP_MOTION_TEST_ROTATE_MS)) {
    Motor_Stop();
    motion_test_running = false;
    motion_test_rotate = false;
    motion_test_done = true;
  }
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
#if !APP_ENABLE_MOTION_TEST
  imu_release_sequence = 0U;
  imu_consumed_sequence = 0U;
#endif
  motor_control_period_ms = 0U;
  lcd_period_ms = 0U;

#if APP_ENABLE_TASK
  task_release_sequence = 0U;
  task_release_ms = 0U;
  task_consumed_sequence = 0U;
  task_period_ms = 0U;
#endif

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  motor_test_running = false;
  motor_key_sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;
  motor_key_stable = motor_key_sample;
  motor_key_change_ms = 0U;
#endif

#if APP_ENABLE_MOTION_TEST
  motion_test_running = false;
  motion_test_rotate = false;
  motion_test_done = false;
  motion_key_sample =
      HAL_GPIO_ReadPin(MOTOR_PWM_KEY_GPIO_Port, MOTOR_PWM_KEY_Pin) == GPIO_PIN_SET;
  motion_key_stable = motion_key_sample;
  motion_key_change_ms = 0U;
  motion_phase_start_ms = 0U;
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
#if !APP_ENABLE_MOTION_TEST
  const bool imu_ready = IMU_Init();
#endif
  /* Initialize the existing servo outputs after the motor and sensor setup. */
  Servo_Init();

  if (!vision_start_receive_dma()) {
    usart3_rx_next_retry_ms = UART_RETRY_MS;
  }
#if APP_ENABLE_TASK
  Task_FindObject(app_milliseconds);
#endif

  lcd_ready = LCD_Init();
  if (lcd_ready) {
    LCD_FillScreen(LCD_BLACK);
#if APP_ENABLE_MOTION_TEST
    LCD_DrawText(19U, 76U, "MOTOR TEST READY", LCD_GREEN, LCD_BLACK);
#else
    LCD_DrawText(imu_ready ? 25U : 19U, 76U,
                 imu_ready ? "IMU660RC: OK" : "IMU660RC: ERROR",
                 imu_ready ? LCD_GREEN : LCD_RED, LCD_BLACK);
#endif
    HAL_Delay(1000U);
    draw_dashboard();
  }

  /* Blocking startup is complete; the 1 ms real-time schedule starts here. */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }
}

void Robot_Process(void)
{
  if (!usart3_rx_active &&
      ((int32_t)(app_milliseconds - usart3_rx_next_retry_ms) >= 0)) {
    usart3_rx_position = 0U;
    Vision_ResetParser();
    if (!vision_start_receive_dma()) {
      usart3_rx_next_retry_ms = app_milliseconds + UART_RETRY_MS;
    }
  }
  Vision_Process();

#if !APP_ENABLE_MOTION_TEST
  const uint32_t imu_released = imu_release_sequence;
  if (imu_released != imu_consumed_sequence) {
    imu_consumed_sequence = imu_released;
    IMU_Update(app_milliseconds);
  }
#endif

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  process_motor_test_key(app_milliseconds);
#endif
#if APP_ENABLE_MOTION_TEST
  process_motion_test(app_milliseconds);
#endif

  const uint32_t lcd_released = lcd_release_sequence;
  if (lcd_released != lcd_consumed_sequence) {
    lcd_consumed_sequence = lcd_released;
    if (lcd_ready) {
      draw_dashboard();
    }
  }

#if APP_ENABLE_SERVO_SWEEP_TEST && !APP_ENABLE_TASK
  run_servo_sweep(app_milliseconds);
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

  /* Keep PendSV pending if TIM6 released another task period meanwhile. */
  if (task_release_sequence != task_consumed_sequence) {
    __DMB();
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  }
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance != TIM6) {
    return;
  }

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
#if !APP_ENABLE_MOTION_TEST
    ++imu_release_sequence;
#endif
  }
  if (++lcd_period_ms >= LCD_PERIOD_MS) {
    lcd_period_ms = 0U;
    ++lcd_release_sequence;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  if ((uart->Instance != USART3) || (size == 0U) ||
      (size > UART_DMA_RX_SIZE)) {
    return;
  }

  usart3_rx_active = true;
  usart3_last_byte = usart3_rx_dma[size - 1U];
  usart3_byte_received = true;
  usart3_last_rx_ms = app_milliseconds;
  vision_parse_dma_range(size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance != USART3) {
    return;
  }

  Vision_OnTxError();
  usart3_rx_active = false;
  (void)HAL_UART_AbortReceive(&huart3);
  usart3_rx_position = 0U;
  Vision_ResetParser();
  if (!vision_start_receive_dma()) {
    usart3_rx_next_retry_ms = app_milliseconds + UART_RETRY_MS;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART3) {
    Vision_OnTxComplete();
  }
}
