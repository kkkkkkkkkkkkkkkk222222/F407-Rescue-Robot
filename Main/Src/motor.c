#include "motor.h"

#include <math.h>

#include "app_config.h"
#include "encoder.h"
#include "imu.h"
#include "main.h"
#include "pid.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

#define MOTOR_COUNT 3U
#define MOTOR_MAX_SPEED 1000
#define MOTOR_DEG_TO_RAD 0.01745329252f
#define MOTOR_SQRT3_OVER_2 0.86602540378f
#define MOTOR_ONE_HALF 0.5f
/* Calibrated on the real chassis: API +90 deg is physical left. */
#define MOTOR_API_LEFT_TO_BODY_Y -1.0f
#define MOTOR_DEFAULT_OUTPUT_LIMIT \
  (APP_MOTOR_BASE_PWM + (int32_t)APP_MOTOR_PID_LIMIT)

typedef struct {
  TIM_HandleTypeDef *in1_timer;
  uint32_t in1_channel;
  TIM_HandleTypeDef *in2_timer;
  uint32_t in2_channel;
} MotorPwm;

typedef struct {
  MotorDistanceStatus status;
  bool slowing;
  float target_mm;
  float slowdown_start_mm;
  float last_progress_mm;
  int64_t start_m1_count;
  int64_t start_m3_count;
  uint16_t no_progress_cycles;
} DistanceMove;

typedef struct {
  MotorTurnStatus status;
  bool slowing;
  int8_t direction;
  int32_t target_mdeg;
  uint32_t start_ms;
} AngleTurn;

typedef struct {
  bool active;
  float forward_mm_s;
  float lateral_mm_s;
  int32_t target_yaw_mdeg;
} DirectionMove;

static const MotorPwm motors[MOTOR_COUNT] = {
  {&htim5, TIM_CHANNEL_3, &htim5, TIM_CHANNEL_4},
  {&htim9, TIM_CHANNEL_1, &htim9, TIM_CHANNEL_2},
  {&htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4},
};

static volatile int16_t commands[MOTOR_COUNT];
static volatile int16_t targets[MOTOR_COUNT];
static float target_counts_per_update[MOTOR_COUNT];
static volatile int16_t measured_speeds_mm_s[MOTOR_COUNT];
static volatile int16_t target_speeds_mm_s[MOTOR_COUNT];
static uint8_t direction_mismatch_cycles[MOTOR_COUNT];
static uint8_t direction_grace_cycles[MOTOR_COUNT];
static uint8_t stall_cycles[MOTOR_COUNT];
static uint8_t stall_grace_cycles[MOTOR_COUNT];
static volatile bool direction_faults[MOTOR_COUNT];
static volatile bool stall_faults[MOTOR_COUNT];
static Pid_t speed_pids[MOTOR_COUNT];
static Pid_t heading_pid;
static volatile DistanceMove distance_move;
static volatile AngleTurn angle_turn;
static volatile DirectionMove direction_move;
static uint8_t brake_cycles_remaining;

static const int8_t motor_signs[MOTOR_COUNT] = {
  APP_OMNI_M1_MOTOR_SIGN,
  APP_OMNI_M2_MOTOR_SIGN,
  APP_OMNI_M3_MOTOR_SIGN
};
static const int8_t encoder_signs[MOTOR_COUNT] = {
  APP_OMNI_M1_ENCODER_SIGN,
  APP_OMNI_M2_ENCODER_SIGN,
  APP_OMNI_M3_ENCODER_SIGN
};

static float motor_counts_to_mm_s(float counts);
static void motor_set_speed_target(float target_speed, uint8_t id);
static void motor_set_omni_speed(float forward_mm_s, float lateral_mm_s,
                                 float yaw_tangent_mm_s);

static uint32_t motor_enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void motor_leave_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static int32_t motor_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static float motor_abs_float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int16_t motor_limit(int32_t value)
{
  if (value > MOTOR_MAX_SPEED) {
    return MOTOR_MAX_SPEED;
  }
  if (value < -MOTOR_MAX_SPEED) {
    return -MOTOR_MAX_SPEED;
  }
  return (int16_t)value;
}

static int16_t motor_round_to_int16(float value)
{
  if (value > 32767.0f) {
    return INT16_MAX;
  }
  if (value < -32768.0f) {
    return INT16_MIN;
  }
  return (int16_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static bool motor_has_fault(void)
{
  for (uint32_t i = 0U; i < MOTOR_COUNT; ++i) {
    if (direction_faults[i] || stall_faults[i]) {
      return true;
    }
  }
  return false;
}

static int16_t motor_default_output_limit(void)
{
  return motor_limit(MOTOR_DEFAULT_OUTPUT_LIMIT);
}

static void motor_set_target(uint8_t id, int32_t target)
{
  int16_t limited;

  if ((id < 1U) || (id > MOTOR_COUNT)) {
    return;
  }
  limited = motor_limit(target);
  if (motor_has_fault() && (limited != 0)) {
    return;
  }

  const uint32_t index = id - 1U;
  const bool starting = (targets[index] == 0) && (limited != 0);
  const bool reversing = ((targets[index] > 0) && (limited < 0)) ||
                         ((targets[index] < 0) && (limited > 0));
  if ((limited == 0) || starting || reversing) {
    Pid_Reset(&speed_pids[index]);
    direction_mismatch_cycles[index] = 0U;
    stall_cycles[index] = 0U;
    direction_grace_cycles[index] =
        (limited == 0) ? 0U : APP_MOTOR_DIRECTION_GRACE_CYCLES;
    stall_grace_cycles[index] =
        (limited == 0) ? 0U : APP_MOTOR_STALL_GRACE_CYCLES;
  }

  targets[index] = limited;
  target_counts_per_update[index] =
      (float)limited * (float)APP_MOTOR_MAX_COUNT_10MS / (float)MOTOR_MAX_SPEED;
  target_speeds_mm_s[index] =
      motor_round_to_int16(motor_counts_to_mm_s(target_counts_per_update[index]));
}

static void motor_set_target_counts(uint8_t id, float target_counts)
{
  float limited_counts = target_counts;
  int32_t normalized;

  if ((id < 1U) || (id > MOTOR_COUNT) || !isfinite(limited_counts)) {
    return;
  }
  if (motor_has_fault() && (limited_counts != 0.0f)) {
    return;
  }
  if (limited_counts > (float)APP_MOTOR_MAX_COUNT_10MS) {
    limited_counts = (float)APP_MOTOR_MAX_COUNT_10MS;
  } else if (limited_counts < -(float)APP_MOTOR_MAX_COUNT_10MS) {
    limited_counts = -(float)APP_MOTOR_MAX_COUNT_10MS;
  }

  normalized = (int32_t)(limited_counts * (float)MOTOR_MAX_SPEED /
                         (float)APP_MOTOR_MAX_COUNT_10MS +
                         ((limited_counts >= 0.0f) ? 0.5f : -0.5f));
  if ((normalized == 0) && (limited_counts != 0.0f)) {
    normalized = (limited_counts > 0.0f) ? 1 : -1;
  }
  motor_set_target(id, normalized);
  target_counts_per_update[id - 1U] = limited_counts;
  target_speeds_mm_s[id - 1U] =
      motor_round_to_int16(motor_counts_to_mm_s(limited_counts));
}

static uint32_t speed_to_compare(TIM_HandleTypeDef *timer, int16_t speed)
{
  uint32_t magnitude = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
  if (magnitude > 1000U) {
    magnitude = 1000U;
  }
  return ((timer->Init.Period + 1U) * magnitude) / 1000U;
}

static float motor_wheel_circumference_mm(void)
{
  return 3.14159265358979323846f * (float)APP_WHEEL_DIAMETER_MM;
}

static float motor_counts_to_mm_s(float counts)
{
  return counts * motor_wheel_circumference_mm() * 1000.0f /
         ((float)APP_ENCODER_COUNTS_PER_WHEEL_REV *
          (float)APP_MOTOR_CONTROL_PERIOD_MS);
}

static float motor_counts_to_mm(float counts)
{
  return counts * motor_wheel_circumference_mm() /
         (float)APP_ENCODER_COUNTS_PER_WHEEL_REV;
}

static void motor_set_speed_target(float target_speed, uint8_t id)
{
  const float target_counts =
      target_speed / motor_wheel_circumference_mm() *
      (float)APP_ENCODER_COUNTS_PER_WHEEL_REV *
      (float)APP_MOTOR_CONTROL_PERIOD_MS / 1000.0f;
  motor_set_target_counts(id, target_counts);
}

static void motor_set_forward_speed(float speed_mm_s)
{
  const float wheel_speed = speed_mm_s * MOTOR_SQRT3_OVER_2;
  motor_set_speed_target(-wheel_speed, 1U);
  motor_set_speed_target(0.0f, 2U);
  motor_set_speed_target( wheel_speed, 3U);
}

static void motor_set_rotate_speed(float speed_mm_s)
{
  for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
    motor_set_speed_target(speed_mm_s, id);
  }
}

static void motor_set_omni_speed(float forward_mm_s, float lateral_mm_s,
                                 float yaw_tangent_mm_s)
{
  /*
   * Standard 120-degree three-wheel inverse kinematics:
   *   v1 =  Vy                 + R*w
   *   v2 = -sqrt(3)/2*Vx-Vy/2 + R*w
   *   v3 =  sqrt(3)/2*Vx-Vy/2 + R*w
   * Physical mapping on this chassis is M1=v2, M2=v1, M3=v3. The third
   * input is therefore R*w expressed as wheel tangential speed in mm/s.
   */
  float wheel[MOTOR_COUNT] = {
    -MOTOR_SQRT3_OVER_2 * forward_mm_s -
        MOTOR_ONE_HALF * lateral_mm_s + yaw_tangent_mm_s,
     lateral_mm_s + yaw_tangent_mm_s,
     MOTOR_SQRT3_OVER_2 * forward_mm_s -
        MOTOR_ONE_HALF * lateral_mm_s + yaw_tangent_mm_s
  };
  float maximum = motor_abs_float(wheel[0]);
  for (uint32_t i = 1U; i < MOTOR_COUNT; ++i) {
    if (motor_abs_float(wheel[i]) > maximum) {
      maximum = motor_abs_float(wheel[i]);
    }
  }

  const float maximum_wheel_speed =
      motor_counts_to_mm_s((float)APP_MOTOR_MAX_COUNT_10MS);
  if (maximum > maximum_wheel_speed) {
    const float scale = maximum_wheel_speed / maximum;
    for (uint32_t i = 0U; i < MOTOR_COUNT; ++i) {
      wheel[i] *= scale;
    }
  }

  for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
    motor_set_speed_target(wheel[id - 1U], id);
  }
}

static void motor_apply_pwm(uint8_t id, int16_t speed)
{
  if ((id < 1U) || (id > MOTOR_COUNT)) {
    return;
  }
  speed = motor_limit(speed);

  const MotorPwm *motor = &motors[id - 1U];
  const uint32_t in1 = (speed > 0) ? speed_to_compare(motor->in1_timer, speed) : 0U;
  const uint32_t in2 = (speed < 0) ? speed_to_compare(motor->in2_timer, speed) : 0U;
  __HAL_TIM_SET_COMPARE(motor->in1_timer, motor->in1_channel, in1);
  __HAL_TIM_SET_COMPARE(motor->in2_timer, motor->in2_channel, in2);
  commands[id - 1U] = speed;
}

static void motor_apply_brake(uint8_t id)
{
  if ((id < 1U) || (id > MOTOR_COUNT)) {
    return;
  }
  const MotorPwm *motor = &motors[id - 1U];
  const uint32_t in1 = speed_to_compare(motor->in1_timer, MOTOR_MAX_SPEED);
  const uint32_t in2 = speed_to_compare(motor->in2_timer, MOTOR_MAX_SPEED);
  __HAL_TIM_SET_COMPARE(motor->in1_timer, motor->in1_channel, in1);
  __HAL_TIM_SET_COMPARE(motor->in2_timer, motor->in2_channel, in2);
  commands[id - 1U] = 0;
}

static void motor_reset_targets(void)
{
  for (uint32_t i = 0U; i < MOTOR_COUNT; ++i) {
    targets[i] = 0;
    target_counts_per_update[i] = 0.0f;
    target_speeds_mm_s[i] = 0;
    direction_mismatch_cycles[i] = 0U;
    direction_grace_cycles[i] = 0U;
    stall_cycles[i] = 0U;
    stall_grace_cycles[i] = 0U;
    Pid_Reset(&speed_pids[i]);
  }
}

static void motor_stop_outputs(bool use_brake)
{
  bool was_moving = false;

  for (uint32_t i = 0U; i < MOTOR_COUNT; ++i) {
    if ((commands[i] != 0) || (measured_speeds_mm_s[i] != 0)) {
      was_moving = true;
    }
  }
  direction_move.active = false;
  Pid_Reset(&heading_pid);
  motor_reset_targets();

  if (use_brake && was_moving && (brake_cycles_remaining == 0U) &&
      (APP_MOTOR_BRAKE_CYCLES > 0U)) {
    brake_cycles_remaining = APP_MOTOR_BRAKE_CYCLES;
    for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
      motor_apply_brake(id);
    }
  } else if (brake_cycles_remaining == 0U) {
    for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
      motor_apply_pwm(id, 0);
    }
  }
}

static void motor_finish_distance(MotorDistanceStatus status, bool use_brake)
{
  distance_move.status = status;
  distance_move.slowing = false;
  distance_move.no_progress_cycles = 0U;
  motor_stop_outputs(use_brake);
}

static void motor_abort_all(MotorDistanceStatus distance_status)
{
  if (distance_move.status == MOTOR_DISTANCE_RUNNING) {
    distance_move.status = distance_status;
  }
  if (angle_turn.status == MOTOR_TURN_RUNNING) {
    angle_turn.status = MOTOR_TURN_FAULT;
    angle_turn.slowing = false;
  }
  motor_stop_outputs(true);
}

static void motor_update_distance_move(const EncoderStatus encoder[MOTOR_COUNT])
{
  if (distance_move.status != MOTOR_DISTANCE_RUNNING) {
    return;
  }

  const float m1_mm = motor_counts_to_mm(
      (float)(encoder[0].position - distance_move.start_m1_count) *
      (float)encoder_signs[0]);
  const float m3_mm = motor_counts_to_mm(
      (float)(encoder[2].position - distance_move.start_m3_count) *
      (float)encoder_signs[2]);
  const float forward_mm = (m3_mm - m1_mm) / 1.7320508f;
  const float direction = (distance_move.target_mm >= 0.0f) ? 1.0f : -1.0f;
  const float target_mm = distance_move.target_mm * direction;
  const float travelled_mm = forward_mm * direction;
  const float remaining_mm = target_mm - travelled_mm;

  if (remaining_mm <= APP_GO_DISTANCE_TOLERANCE_MM) {
    motor_finish_distance(MOTOR_DISTANCE_DONE, true);
    return;
  }

  if ((targets[0] != 0) || (targets[2] != 0)) {
    const float progress = travelled_mm - distance_move.last_progress_mm;
    if (progress >= APP_GO_DISTANCE_PROGRESS_MM) {
      distance_move.last_progress_mm = travelled_mm;
      distance_move.no_progress_cycles = 0U;
    } else if (distance_move.no_progress_cycles < UINT16_MAX) {
      ++distance_move.no_progress_cycles;
    }
    if (distance_move.no_progress_cycles >= APP_GO_DISTANCE_NO_PROGRESS_CYCLES) {
      stall_faults[0] = true;
      stall_faults[2] = true;
      motor_abort_all(MOTOR_DISTANCE_FAULT);
      return;
    }
  }

  float speed_mm_s = APP_GO_DISTANCE_SPEED_MM_S;

  if (remaining_mm < distance_move.slowdown_start_mm) {
    if (!distance_move.slowing) {
      distance_move.slowing = true;
      Pid_Reset(&speed_pids[0]);
      Pid_Reset(&speed_pids[2]);
    }

    const float usable_slowdown =
        distance_move.slowdown_start_mm - APP_GO_DISTANCE_TOLERANCE_MM;
    float ratio = (remaining_mm - APP_GO_DISTANCE_TOLERANCE_MM) / usable_slowdown;
    if (ratio < 0.0f) {
      ratio = 0.0f;
    } else if (ratio > 1.0f) {
      ratio = 1.0f;
    }
    speed_mm_s = APP_GO_DISTANCE_MIN_SPEED_MM_S +
        (APP_GO_DISTANCE_SPEED_MM_S - APP_GO_DISTANCE_MIN_SPEED_MM_S) * ratio;
  }

  motor_set_forward_speed(speed_mm_s * direction);
}

static void motor_update_angle_turn(void)
{
  if (angle_turn.status != MOTOR_TURN_RUNNING) {
    return;
  }

  const IMUData imu = IMU_GetData();
  if (!imu.ready) {
    angle_turn.status = MOTOR_TURN_FAULT;
    angle_turn.slowing = false;
    motor_stop_outputs(true);
    return;
  }

  if ((uint32_t)(HAL_GetTick() - angle_turn.start_ms) >=
      APP_MOTOR_TURN_TIMEOUT_MS) {
    angle_turn.status = MOTOR_TURN_FAULT;
    angle_turn.slowing = false;
    motor_stop_outputs(true);
    return;
  }

  const int64_t signed_yaw_mdeg = imu.yaw_mdeg;
  const int64_t yaw_mdeg =
      (signed_yaw_mdeg < 0LL) ? -signed_yaw_mdeg : signed_yaw_mdeg;
  const int64_t remaining_mdeg =
      (int64_t)angle_turn.target_mdeg - yaw_mdeg;

  if (remaining_mdeg <= APP_MOTOR_TURN_TOLERANCE_MDEG) {
    angle_turn.status = MOTOR_TURN_DONE;
    angle_turn.slowing = false;
    motor_stop_outputs(true);
    return;
  }

  if (!angle_turn.slowing &&
      (remaining_mdeg <= APP_MOTOR_TURN_SLOWDOWN_MDEG)) {
    angle_turn.slowing = true;
    for (uint32_t i = 0U; i < MOTOR_COUNT; ++i) {
      Pid_Reset(&speed_pids[i]);
    }
    motor_set_rotate_speed(APP_MOTOR_TURN_SLOW_MM_S *
                           (float)angle_turn.direction);
  }
}

static int32_t motor_wrap_heading_error(int64_t error_mdeg)
{
  error_mdeg %= 360000LL;
  if (error_mdeg > 180000LL) {
    error_mdeg -= 360000LL;
  } else if (error_mdeg < -180000LL) {
    error_mdeg += 360000LL;
  }
  return (int32_t)error_mdeg;
}

static void motor_update_direction_move(void)
{
  if (!direction_move.active) {
    return;
  }

  const IMUData imu = IMU_GetData();
  if (!imu.ready) {
    motor_stop_outputs(true);
    return;
  }

  const int32_t error_mdeg = motor_wrap_heading_error(
      (int64_t)direction_move.target_yaw_mdeg - imu.yaw_mdeg);
  const float rotate_correction =
      Pid_Update(&heading_pid, (float)error_mdeg / 1000.0f, 0.0f) *
      APP_MOTOR_HEADING_OUTPUT_SIGN;
  motor_set_omni_speed(direction_move.forward_mm_s,
                       direction_move.lateral_mm_s,
                       rotate_correction);
}

void Motor_Init(void)
{
  brake_cycles_remaining = 0U;
  for (uint32_t i = 0U; i < MOTOR_COUNT; ++i) {
    __HAL_TIM_SET_COMPARE(motors[i].in1_timer, motors[i].in1_channel, 0U);
    __HAL_TIM_SET_COMPARE(motors[i].in2_timer, motors[i].in2_channel, 0U);
    if (HAL_TIM_PWM_Start(motors[i].in1_timer, motors[i].in1_channel) != HAL_OK) {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Start(motors[i].in2_timer, motors[i].in2_channel) != HAL_OK) {
      Error_Handler();
    }
    Pid_Init(&speed_pids[i],
             APP_MOTOR_SPEED_KP,
             APP_MOTOR_SPEED_KI,
             APP_MOTOR_SPEED_KD,
             -APP_MOTOR_PID_LIMIT,
             APP_MOTOR_PID_LIMIT,
             -APP_MOTOR_INTEGRAL_LIMIT,
             APP_MOTOR_INTEGRAL_LIMIT);
    commands[i] = 0;
    targets[i] = 0;
    target_counts_per_update[i] = 0.0f;
    measured_speeds_mm_s[i] = 0;
    target_speeds_mm_s[i] = 0;
    direction_mismatch_cycles[i] = 0U;
    direction_grace_cycles[i] = 0U;
    stall_cycles[i] = 0U;
    stall_grace_cycles[i] = 0U;
    direction_faults[i] = false;
    stall_faults[i] = false;
  }
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
  distance_move.target_mm = 0.0f;
  distance_move.slowdown_start_mm = 0.0f;
  distance_move.last_progress_mm = 0.0f;
  distance_move.start_m1_count = 0;
  distance_move.start_m3_count = 0;
  distance_move.no_progress_cycles = 0U;
  angle_turn.status = MOTOR_TURN_IDLE;
  angle_turn.slowing = false;
  angle_turn.direction = 1;
  angle_turn.target_mdeg = 0;
  angle_turn.start_ms = 0U;
  direction_move.active = false;
  direction_move.forward_mm_s = 0.0f;
  direction_move.lateral_mm_s = 0.0f;
  direction_move.target_yaw_mdeg = 0;
  Pid_Init(&heading_pid,
           APP_MOTOR_HEADING_KP,
           APP_MOTOR_HEADING_KI,
           APP_MOTOR_HEADING_KD,
           -APP_MOTOR_HEADING_LIMIT_MM_S,
           APP_MOTOR_HEADING_LIMIT_MM_S,
           -APP_MOTOR_HEADING_INTEGRAL_LIMIT,
           APP_MOTOR_HEADING_INTEGRAL_LIMIT);
}

void Motor_Stop(void)
{
  const uint32_t primask = motor_enter_critical();
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
  distance_move.no_progress_cycles = 0U;
  angle_turn.status = MOTOR_TURN_IDLE;
  angle_turn.slowing = false;
  motor_stop_outputs(true);
  motor_leave_critical(primask);
}

void Motor_Update(void)
{
  int32_t measured_counts[MOTOR_COUNT];
  bool fault_detected = false;
  EncoderStatus encoder[MOTOR_COUNT];

  Encoder_GetAll(encoder);
  motor_update_distance_move(encoder);
  motor_update_angle_turn();
  motor_update_direction_move();

  for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
    const uint32_t index = id - 1U;
    measured_counts[index] = encoder[index].delta_10ms * encoder_signs[index];
    measured_speeds_mm_s[index] =
        motor_round_to_int16(motor_counts_to_mm_s((float)measured_counts[index]));
  }

  if (brake_cycles_remaining > 0U) {
    for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
      motor_apply_brake(id);
    }
    --brake_cycles_remaining;
    if (brake_cycles_remaining == 0U) {
      for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
        motor_apply_pwm(id, 0);
      }
    }
    return;
  }

  if (motor_has_fault()) {
    motor_abort_all(MOTOR_DISTANCE_FAULT);
    return;
  }

  for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
    const uint32_t index = id - 1U;
    const int32_t measured = measured_counts[index];
    float correction;

    if (targets[index] == 0) {
      Pid_Reset(&speed_pids[index]);
      motor_apply_pwm(id, 0);
      continue;
    }

    if (direction_grace_cycles[index] > 0U) {
      --direction_grace_cycles[index];
    }
    if (stall_grace_cycles[index] > 0U) {
      --stall_grace_cycles[index];
    }

    const bool wrong_direction = (direction_grace_cycles[index] == 0U) &&
        (((targets[index] > 0) && (measured <= -APP_MOTOR_DIRECTION_FAULT_COUNT)) ||
         ((targets[index] < 0) && (measured >= APP_MOTOR_DIRECTION_FAULT_COUNT)));
    if (wrong_direction) {
      if (++direction_mismatch_cycles[index] >= APP_MOTOR_DIRECTION_FAULT_CYCLES) {
        direction_faults[index] = true;
        fault_detected = true;
      }
    } else {
      direction_mismatch_cycles[index] = 0U;
    }

    const bool stalled = (stall_grace_cycles[index] == 0U) &&
        (motor_abs(commands[index]) >= APP_MOTOR_STALL_MIN_PWM) &&
        (motor_abs(measured) <= APP_MOTOR_STALL_MAX_COUNT_10MS);
    if (stalled) {
      if (++stall_cycles[index] >= APP_MOTOR_STALL_CYCLES) {
        stall_faults[index] = true;
        fault_detected = true;
      }
    } else {
      stall_cycles[index] = 0U;
    }

    if (fault_detected) {
      continue;
    }

    correction = Pid_Update(&speed_pids[index],
                            target_counts_per_update[index],
                            (float)measured);
    const int32_t base_pwm =
        (targets[index] > 0) ? APP_MOTOR_BASE_PWM : -APP_MOTOR_BASE_PWM;
    int32_t output = base_pwm + (int32_t)correction;
    const int32_t output_limit = motor_default_output_limit();
    if (output > output_limit) {
      output = output_limit;
    } else if (output < -output_limit) {
      output = -output_limit;
    }
    motor_apply_pwm(id, motor_limit(output * motor_signs[index]));
  }

  if (fault_detected) {
    motor_abort_all(MOTOR_DISTANCE_FAULT);
  }
}

void Motor_SetSpeed(float target_speed, uint8_t id)
{
  if ((id < 1U) || (id > MOTOR_COUNT) || !isfinite(target_speed)) {
    return;
  }

  const uint32_t primask = motor_enter_critical();
  if (motor_has_fault()) {
    motor_leave_critical(primask);
    return;
  }
  if ((distance_move.status != MOTOR_DISTANCE_IDLE) ||
      (angle_turn.status != MOTOR_TURN_IDLE)) {
    motor_stop_outputs(false);
  }
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
  angle_turn.status = MOTOR_TURN_IDLE;
  angle_turn.slowing = false;
  direction_move.active = false;
  Pid_Reset(&heading_pid);
  motor_set_speed_target(target_speed, id);
  motor_leave_critical(primask);
}

MotorDistanceStatus Go_distance(float distance_m)
{
  const uint32_t primask = motor_enter_critical();
  if (distance_move.status != MOTOR_DISTANCE_IDLE) {
    const MotorDistanceStatus status = distance_move.status;
    motor_leave_critical(primask);
    return status;
  }
  if (angle_turn.status != MOTOR_TURN_IDLE) {
    motor_leave_critical(primask);
    return MOTOR_DISTANCE_INVALID;
  }
  if (!isfinite(distance_m)) {
    motor_leave_critical(primask);
    return MOTOR_DISTANCE_INVALID;
  }
  if (motor_has_fault()) {
    motor_leave_critical(primask);
    return MOTOR_DISTANCE_FAULT;
  }

  const float distance_mm = distance_m * 1000.0f;
  if (!isfinite(distance_mm)) {
    motor_leave_critical(primask);
    return MOTOR_DISTANCE_INVALID;
  }

  motor_stop_outputs(false);
  if ((distance_mm >= -APP_GO_DISTANCE_TOLERANCE_MM) &&
      (distance_mm <= APP_GO_DISTANCE_TOLERANCE_MM)) {
    distance_move.status = MOTOR_DISTANCE_DONE;
    motor_leave_critical(primask);
    return MOTOR_DISTANCE_DONE;
  }

  EncoderStatus encoder[MOTOR_COUNT];
  Encoder_GetAll(encoder);
  const float absolute_distance = motor_abs_float(distance_mm);
  distance_move.target_mm = distance_mm;
  distance_move.start_m1_count = encoder[0].position;
  distance_move.start_m3_count = encoder[2].position;
  distance_move.slowdown_start_mm = APP_GO_DISTANCE_SLOWDOWN_MM;
  if (distance_move.slowdown_start_mm > (absolute_distance * 0.5f)) {
    distance_move.slowdown_start_mm = absolute_distance * 0.5f;
  }
  if (distance_move.slowdown_start_mm <= APP_GO_DISTANCE_TOLERANCE_MM) {
    distance_move.slowdown_start_mm = APP_GO_DISTANCE_TOLERANCE_MM + 1.0f;
  }
  distance_move.last_progress_mm = 0.0f;
  distance_move.no_progress_cycles = 0U;
  distance_move.slowing = false;
  distance_move.status = MOTOR_DISTANCE_RUNNING;
  motor_update_distance_move(encoder);
  motor_leave_critical(primask);
  return MOTOR_DISTANCE_RUNNING;
}

MotorTurnStatus Motor_TurnAngle(float angle_deg)
{
  const uint32_t primask = motor_enter_critical();
  if (angle_turn.status != MOTOR_TURN_IDLE) {
    const MotorTurnStatus status = angle_turn.status;
    motor_leave_critical(primask);
    return status;
  }
  if (!isfinite(angle_deg) ||
      (angle_deg < -APP_MOTOR_TURN_MAX_DEG) ||
      (angle_deg > APP_MOTOR_TURN_MAX_DEG)) {
    motor_leave_critical(primask);
    return MOTOR_TURN_INVALID;
  }
  if (motor_has_fault() || !IMU_GetData().ready) {
    angle_turn.status = MOTOR_TURN_FAULT;
    angle_turn.slowing = false;
    motor_stop_outputs(true);
    motor_leave_critical(primask);
    return MOTOR_TURN_FAULT;
  }
  if (distance_move.status != MOTOR_DISTANCE_IDLE) {
    motor_leave_critical(primask);
    return MOTOR_TURN_INVALID;
  }

  const float absolute_angle = motor_abs_float(angle_deg);
  const int32_t target_mdeg =
      (int32_t)(absolute_angle * 1000.0f + 0.5f);
  motor_stop_outputs(false);
  if (target_mdeg <= APP_MOTOR_TURN_TOLERANCE_MDEG) {
    angle_turn.status = MOTOR_TURN_DONE;
    motor_leave_critical(primask);
    return MOTOR_TURN_DONE;
  }

  IMU_ZeroYaw();
  angle_turn.status = MOTOR_TURN_RUNNING;
  angle_turn.slowing = false;
  angle_turn.direction = (angle_deg >= 0.0f) ? 1 : -1;
  angle_turn.target_mdeg = target_mdeg;
  angle_turn.start_ms = HAL_GetTick();
  motor_set_rotate_speed(APP_MOTOR_TURN_FAST_MM_S *
                         (float)angle_turn.direction);
  motor_leave_critical(primask);
  return MOTOR_TURN_RUNNING;
}

void Motor_Move(float forward_mm_s, float lateral_mm_s,
                float yaw_tangent_mm_s)
{
  const uint32_t primask = motor_enter_critical();

  if (motor_has_fault() || !isfinite(forward_mm_s) ||
      !isfinite(lateral_mm_s) || !isfinite(yaw_tangent_mm_s)) {
    motor_leave_critical(primask);
    return;
  }
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
  angle_turn.status = MOTOR_TURN_IDLE;
  angle_turn.slowing = false;
  direction_move.active = false;
  Pid_Reset(&heading_pid);
  /* All three inputs and all wheel targets use mm/s. */
  motor_set_omni_speed(forward_mm_s, lateral_mm_s, yaw_tangent_mm_s);
  motor_leave_critical(primask);
}

void Motor_MoveAngle(float speed_mm_s, float angle_deg)
{
  if (!isfinite(speed_mm_s) || !isfinite(angle_deg)) {
    return;
  }

  const uint32_t primask = motor_enter_critical();
  const IMUData imu = IMU_GetData();
  if (motor_has_fault() || !imu.ready) {
    motor_stop_outputs(true);
    motor_leave_critical(primask);
    return;
  }

  if (speed_mm_s < 0.0f) {
    speed_mm_s = -speed_mm_s;
    angle_deg += 180.0f;
  }
  if (speed_mm_s == 0.0f) {
    distance_move.status = MOTOR_DISTANCE_IDLE;
    angle_turn.status = MOTOR_TURN_IDLE;
    motor_stop_outputs(true);
    motor_leave_critical(primask);
    return;
  }

  angle_deg = fmodf(angle_deg, 360.0f);
  const float angle_rad = angle_deg * MOTOR_DEG_TO_RAD;
  const bool starting = !direction_move.active;

  if (starting) {
    distance_move.status = MOTOR_DISTANCE_IDLE;
    distance_move.slowing = false;
    distance_move.no_progress_cycles = 0U;
    angle_turn.status = MOTOR_TURN_IDLE;
    angle_turn.slowing = false;
    motor_stop_outputs(false);
    direction_move.target_yaw_mdeg = imu.yaw_mdeg;
    Pid_Reset(&heading_pid);
    direction_move.active = true;
  }

  direction_move.forward_mm_s = speed_mm_s * cosf(angle_rad);
  direction_move.lateral_mm_s =
      speed_mm_s * sinf(angle_rad) * MOTOR_API_LEFT_TO_BODY_Y;
  motor_set_omni_speed(direction_move.forward_mm_s,
                       direction_move.lateral_mm_s, 0.0f);
  motor_leave_critical(primask);
}

MotorStatus Motor_GetStatus(uint8_t id)
{
  MotorStatus status = {0, 0, 0, 0, false, false};

  if ((id >= 1U) && (id <= MOTOR_COUNT)) {
    const uint32_t primask = motor_enter_critical();
    status.command = commands[id - 1U];
    status.target = targets[id - 1U];
    status.measured_speed_mm_s = measured_speeds_mm_s[id - 1U];
    status.target_speed_mm_s = target_speeds_mm_s[id - 1U];
    status.direction_fault = direction_faults[id - 1U];
    status.stall_fault = stall_faults[id - 1U];
    motor_leave_critical(primask);
  }
  return status;
}
