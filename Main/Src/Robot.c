#include "Robot.h"

#include <stdbool.h>

#include "app_config.h"
#include "encoder.h"
#include "imu.h"
#include "Lcd.h"
#include "Location.h"
#include "main.h"
#include "motor.h"
#include "route_demo.h"
#include "servo.h"
#include "Task.h"
#include "Uart.h"
#include "vision.h"

extern TIM_HandleTypeDef htim6;

#define LCD_PERIOD_MS     100U

static volatile uint32_t app_milliseconds;

static volatile uint32_t lcd_release_sequence;
static uint32_t lcd_consumed_sequence;
static volatile uint32_t imu_release_sequence;
static uint32_t imu_consumed_sequence;
static volatile uint32_t odom_release_sequence;
static uint32_t odom_consumed_sequence;
static bool odom_reset_pending;
static uint8_t imu_period_ms;
static uint8_t motor_control_period_ms;
static uint8_t lcd_period_ms;
static bool lcd_ready;
static bool uart_active;

#if APP_ENABLE_MOTION_TEST
typedef enum {
  MOTION_TEST_START = 0,
  MOTION_TEST_ACCELERATING,
  MOTION_TEST_MOVING,
  MOTION_TEST_BRAKING,
  MOTION_TEST_TURNING,
  MOTION_TEST_RESTART_WAIT,
  MOTION_TEST_STOPPED
} MotionTestStage;

static MotionTestStage motion_test_stage;
static uint32_t motion_test_stop_ms;
static uint8_t motion_test_round;
#endif

#if APP_ENABLE_MOVE_SPIN_TEST
static bool move_spin_test_running;
static uint32_t move_spin_test_end_ms;
static uint32_t move_spin_test_next_ms;
#endif

static void format_hex_byte(char text[5], uint8_t value)
{
  static const char digits[] = "0123456789ABCDEF";
  text[0] = '0';
  text[1] = 'x';
  text[2] = digits[(value >> 4U) & 0x0FU];
  text[3] = digits[value & 0x0FU];
  text[4] = '\0';
}

static const char *imu_init_result_text(IMUInitResult result)
{
  switch (result) {
    case IMU_INIT_RESET_ERROR:
      return "FAIL: RESET";
    case IMU_INIT_CONFIG_ERROR:
      return "FAIL: CONFIG";
    case IMU_INIT_CALIBRATION_ERROR:
      return "FAIL: CALIB";
    case IMU_INIT_OK:
      return "READY";
    default:
      return "FAIL: SPI ID";
  }
}

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

static void draw_dashboard(void)
{
  const VisionData uart = Vision_GetSnapshot();
  LCDDashboard dashboard = {
    .now_ms = app_milliseconds,
    .uart_last_rx_ms = uart.last_frame_tick_ms,
    .uart_last_byte = uart.last_frame[VISION_FRAME_SIZE - 1U],
    .uart_active = uart_active,
    .uart_received = uart.frame_received,
    .motor_test_running = false,
    .imu_ready = false,
    .imu_yaw_mdeg = 0,
    .location_demo_running = false
  };
#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  dashboard.motor_test_running = motor_test_running;
#endif
#if APP_ENABLE_MOTION_TEST || APP_ENABLE_MOVE_SPIN_TEST
  const IMUData imu = IMU_GetData();
  dashboard.imu_ready = imu.ready;
  dashboard.imu_yaw_mdeg = imu.yaw_mdeg;
#endif
#if APP_ENABLE_LOCATION_DEMO || APP_ENABLE_TASK
  const IMUData imu = IMU_GetData();
  dashboard.imu_ready = imu.ready;
  dashboard.imu_yaw_mdeg = imu.yaw_mdeg;
  dashboard.location = Location_GetPose();
#endif
#if APP_ENABLE_LOCATION_DEMO
  dashboard.location_demo_running = RouteDemo_IsRunning();
#endif
  LCD_DrawDashboard(&dashboard);
}

#if APP_ENABLE_MOTION_TEST || APP_ENABLE_MOVE_SPIN_TEST || \
    (APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK)
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

#if APP_ENABLE_MOVE_SPIN_TEST
static void run_move_spin_test(uint32_t now_ms)
{
  if (!move_spin_test_running) {
    return;
  }
  if (!IMU_GetData().ready || robot_motor_has_fault() ||
      ((int32_t)(now_ms - move_spin_test_end_ms) >= 0)) {
    Motor_Stop();
    move_spin_test_running = false;
    return;
  }
  if ((int32_t)(now_ms - move_spin_test_next_ms) < 0) {
    return;
  }
  move_spin_test_next_ms = now_ms + APP_MOVE_SPIN_TEST_CONTROL_MS;
  if (!Motor_MoveSpin(APP_MOVE_SPIN_TEST_SPEED_MM_S,
                      APP_MOVE_SPIN_TEST_ANGLE_DEG,
                      APP_MOVE_SPIN_TEST_YAW_MM_S)) {
    Motor_Stop();
    move_spin_test_running = false;
  }
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
    Servo_SetAngle(id, angles[stage]);
  }
  stage = (uint8_t)((stage + 1U) % 4U);
  next_change_ms = now_ms + 1500U;
}
#endif

#if APP_ENABLE_MOTION_TEST
static void motion_test_finish_turn(uint32_t now_ms)
{
  ++motion_test_round;
  if (motion_test_round < APP_MOTION_TEST_REPEAT_COUNT) {
    Motor_Stop();
    motion_test_stop_ms = now_ms + APP_MOTION_TEST_BRAKE_TIME_MS;
    motion_test_stage = MOTION_TEST_RESTART_WAIT;
  } else {
    motion_test_stage = MOTION_TEST_STOPPED;
  }
}

static void run_motion_test(uint32_t now_ms)
{
  if (!IMU_GetData().ready || robot_motor_has_fault()) {
    Motor_Stop();
    motion_test_stage = MOTION_TEST_STOPPED;
    return;
  }

  switch (motion_test_stage) {
    case MOTION_TEST_START:
      (void)Motor_MoveAngle(APP_MOTION_TEST_SPEED_MM_S,
                            APP_MOTION_TEST_MOVE_ANGLE_DEG);
      motion_test_stop_ms =
          now_ms + APP_MOTION_TEST_TRANSITION_TIMEOUT_MS;
      motion_test_stage = MOTION_TEST_ACCELERATING;
      break;

    case MOTION_TEST_ACCELERATING:
      if (Motor_MoveAngle(APP_MOTION_TEST_SPEED_MM_S,
                          APP_MOTION_TEST_MOVE_ANGLE_DEG)) {
        motion_test_stop_ms = now_ms + APP_MOTION_TEST_MOVE_TIME_MS;
        motion_test_stage = MOTION_TEST_MOVING;
      } else if ((int32_t)(now_ms - motion_test_stop_ms) >= 0) {
        Motor_Stop();
        motion_test_stage = MOTION_TEST_STOPPED;
      }
      break;

    case MOTION_TEST_MOVING:
      if ((int32_t)(now_ms - motion_test_stop_ms) >= 0) {
        Motor_Stop();
        motion_test_stop_ms = now_ms + APP_MOTION_TEST_BRAKE_TIME_MS;
        motion_test_stage = MOTION_TEST_BRAKING;
      }
      break;

    case MOTION_TEST_BRAKING:
      if ((int32_t)(now_ms - motion_test_stop_ms) >= 0) {
        const MotorTurnStatus result =
            Motor_TurnAngle(APP_MOTION_TEST_TURN_DEG);
        if (result == MOTOR_TURN_RUNNING) {
          motion_test_stage = MOTION_TEST_TURNING;
        } else if (result == MOTOR_TURN_DONE) {
          motion_test_finish_turn(now_ms);
        } else {
          Motor_Stop();
          motion_test_stage = MOTION_TEST_STOPPED;
        }
      }
      break;

    case MOTION_TEST_TURNING: {
      const MotorTurnStatus result =
          Motor_TurnAngle(APP_MOTION_TEST_TURN_DEG);
      if (result == MOTOR_TURN_DONE) {
        motion_test_finish_turn(now_ms);
      } else if ((result == MOTOR_TURN_FAULT) ||
                 (result == MOTOR_TURN_INVALID)) {
        Motor_Stop();
        motion_test_stage = MOTION_TEST_STOPPED;
      }
      break;
    }

    case MOTION_TEST_RESTART_WAIT:
      if ((int32_t)(now_ms - motion_test_stop_ms) >= 0) {
        motion_test_stage = MOTION_TEST_START;
      }
      break;

    default:
      break;
  }
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

void Robot_Init(void)
{
  app_milliseconds = 0U;
  lcd_release_sequence = 0U;
  lcd_consumed_sequence = 0U;
  imu_release_sequence = 0U;
  imu_consumed_sequence = 0U;
  odom_release_sequence = 0U;
  odom_consumed_sequence = 0U;
  odom_reset_pending = true;
  imu_period_ms = 0U;
  motor_control_period_ms = 0U;
  lcd_period_ms = 0U;

#if APP_ENABLE_MOTION_TEST
  motion_test_stage = MOTION_TEST_START;
  motion_test_stop_ms = 0U;
  motion_test_round = 0U;
#endif
#if APP_ENABLE_MOVE_SPIN_TEST
  move_spin_test_running = true;
  move_spin_test_end_ms = APP_MOVE_SPIN_TEST_TIME_MS;
  move_spin_test_next_ms = 0U;
#endif
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

  Vision_Init();
  uart_active = Uart_Init();
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
#if APP_ENABLE_TASK
  /* The real start zone arrives in the validated configuration frame. */
  Location_Init(LOCATION_START_UNKNOWN);
#elif APP_ENABLE_LOCATION_DEMO || APP_ENABLE_MOVE_SPIN_TEST
  Location_Init((LocationStart)APP_LOCATION_DEMO_START_ZONE);
#endif
#if APP_ENABLE_LOCATION_DEMO
  RouteDemo_Init();
#endif
#if APP_ENABLE_TASK
  Task_Process(app_milliseconds);
#endif

  lcd_ready = LCD_Init();
  if (lcd_ready) {
    const IMUData imu = IMU_GetData();
    char device_id_text[5];
    format_hex_byte(device_id_text, imu.device_id);

    LCD_FillScreen(LCD_BLACK);
    LCD_DrawText(imu_ready ? 25U : 19U, 48U,
                 imu_ready ? "IMU660RC: OK" : "IMU660RC: ERROR",
                 imu_ready ? LCD_GREEN : LCD_RED, LCD_BLACK);
    LCD_DrawText(13U, 68U, "WHO_AM_I:", LCD_CYAN, LCD_BLACK);
    LCD_DrawText(79U, 68U, device_id_text,
                 (imu.device_id == 0x70U) ? LCD_GREEN : LCD_RED, LCD_BLACK);
    LCD_DrawText(31U, 88U, imu_init_result_text(imu.init_result),
                 imu_ready ? LCD_GREEN : LCD_RED, LCD_BLACK);
    HAL_Delay(imu_ready ? 1000U : 3000U);
    draw_dashboard();
  }

  /* Blocking startup is complete; the 1 ms real-time schedule starts here. */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }
}

void Robot_Process(void)
{
  uart_active = Uart_Receive(0U);

  const uint32_t odom_released = odom_release_sequence;
  if (odom_released != odom_consumed_sequence) {
    EncoderStatus encoder[3];
    Encoder_GetAll(encoder);
    VisionOdom odom = {
      .sample_period_ms = APP_MOTOR_CONTROL_PERIOD_MS,
      .status = (uint8_t)(0x07U | (odom_reset_pending ? 0x08U : 0U))
    };
    for (uint8_t i = 0U; i < 3U; ++i) {
      odom.position[i] = (uint16_t)encoder[i].position;
    }
    odom_consumed_sequence = odom_released;
    Vision_QueueOdom(&odom);
    odom_reset_pending = false;
  }

  Vision_Process();
  const uint32_t imu_released = imu_release_sequence;
  if (imu_released != imu_consumed_sequence) {
    imu_consumed_sequence = imu_released;
    IMU_Update(app_milliseconds);
  }

#if APP_ENABLE_AUTOMATIC_MOTOR_TEST && !APP_ENABLE_TASK
  process_motor_test_key(app_milliseconds);
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
#if APP_ENABLE_MOTION_TEST
  run_motion_test(app_milliseconds);
#endif
#if APP_ENABLE_MOVE_SPIN_TEST
  run_move_spin_test(app_milliseconds);
#endif
#if APP_ENABLE_LOCATION_DEMO
  RouteDemo_Process(app_milliseconds);
#endif
}

void Robot_RunDeferredTask(void)
{
#if APP_ENABLE_TASK
  const uint32_t released = task_release_sequence;
  if (released != task_consumed_sequence) {
    const uint32_t now_ms = task_release_ms;
    task_consumed_sequence = released;
    Task_Process(now_ms);
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
    ++odom_release_sequence;
#if APP_ENABLE_LOCATION_DEMO || APP_ENABLE_MOVE_SPIN_TEST || APP_ENABLE_TASK
    Location_Update10ms();
#endif
    motor_update_due = true;
  }

  if (++imu_period_ms >= APP_IMU_UPDATE_PERIOD_MS) {
    imu_period_ms = 0U;
    ++imu_release_sequence;
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
  }
  if (++lcd_period_ms >= LCD_PERIOD_MS) {
    lcd_period_ms = 0U;
    ++lcd_release_sequence;
  }
}

uint32_t Robot_GetMilliseconds(void)
{
  return app_milliseconds;
}
