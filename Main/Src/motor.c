#include "motor.h"

#include <math.h>

#include "app_config.h"
#include "encoder.h"
#include "main.h"
#include "pid.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;

#define MOTOR_COUNT 3U
#define MOTOR_MAX_SPEED 1000
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
  int64_t start_m2_count;
  uint16_t no_progress_cycles;
  int16_t slowdown_start_pwm[MOTOR_COUNT];
} DistanceMove;

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
static int16_t output_limits[MOTOR_COUNT];
static uint8_t direction_mismatch_cycles[MOTOR_COUNT];
static uint8_t direction_grace_cycles[MOTOR_COUNT];
static uint8_t stall_cycles[MOTOR_COUNT];
static uint8_t stall_grace_cycles[MOTOR_COUNT];
static volatile bool direction_faults[MOTOR_COUNT];
static volatile bool stall_faults[MOTOR_COUNT];
static Pid_t speed_pids[MOTOR_COUNT];
static volatile DistanceMove distance_move;
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
  output_limits[index] = motor_default_output_limit();
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

static void motor_set_three_targets(int32_t m1, int32_t m2, int32_t m3)
{
  motor_set_target(1U, m1);
  motor_set_target(2U, m2);
  motor_set_target(3U, m3);
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

static void motor_set_output_limit(uint8_t id, int32_t limit)
{
  if ((id < 1U) || (id > MOTOR_COUNT)) {
    return;
  }
  if (limit < 0) {
    limit = -limit;
  }
  output_limits[id - 1U] = motor_limit(limit);
}

static void motor_set_forward_speed(float speed_mm_s, int16_t m1_limit, int16_t m2_limit)
{
  const float wheel_speed = speed_mm_s * 0.8660254f;
  motor_set_speed_target(-wheel_speed, 1U);
  motor_set_speed_target( wheel_speed, 2U);
  motor_set_speed_target(0.0f, 3U);
  motor_set_output_limit(1U, m1_limit);
  motor_set_output_limit(2U, m2_limit);
  motor_set_output_limit(3U, 0);
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
    output_limits[i] = motor_default_output_limit();
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
  motor_stop_outputs(true);
}

static void motor_update_distance_move(void)
{
  if (distance_move.status != MOTOR_DISTANCE_RUNNING) {
    return;
  }

  const EncoderStatus m1 = Encoder_GetStatus(1U);
  const EncoderStatus m2 = Encoder_GetStatus(2U);
  const float m1_mm = motor_counts_to_mm(
      (float)(m1.position - distance_move.start_m1_count) * (float)encoder_signs[0]);
  const float m2_mm = motor_counts_to_mm(
      (float)(m2.position - distance_move.start_m2_count) * (float)encoder_signs[1]);
  const float forward_mm = (m2_mm - m1_mm) / 1.7320508f;
  const float direction = (distance_move.target_mm >= 0.0f) ? 1.0f : -1.0f;
  const float target_mm = distance_move.target_mm * direction;
  const float travelled_mm = forward_mm * direction;
  const float remaining_mm = target_mm - travelled_mm;

  if (remaining_mm <= APP_GO_DISTANCE_TOLERANCE_MM) {
    motor_finish_distance(MOTOR_DISTANCE_DONE, true);
    return;
  }

  if ((targets[0] != 0) || (targets[1] != 0)) {
    const float progress = travelled_mm - distance_move.last_progress_mm;
    if (progress >= APP_GO_DISTANCE_PROGRESS_MM) {
      distance_move.last_progress_mm = travelled_mm;
      distance_move.no_progress_cycles = 0U;
    } else if (distance_move.no_progress_cycles < UINT16_MAX) {
      ++distance_move.no_progress_cycles;
    }
    if (distance_move.no_progress_cycles >= APP_GO_DISTANCE_NO_PROGRESS_CYCLES) {
      stall_faults[0] = true;
      stall_faults[1] = true;
      motor_abort_all(MOTOR_DISTANCE_FAULT);
      return;
    }
  }

  float speed_mm_s = APP_GO_DISTANCE_SPEED_MM_S;
  int16_t m1_limit = motor_default_output_limit();
  int16_t m2_limit = motor_default_output_limit();

  if (remaining_mm < distance_move.slowdown_start_mm) {
    if (!distance_move.slowing) {
      distance_move.slowing = true;
      distance_move.slowdown_start_pwm[0] = (int16_t)motor_abs(commands[0]);
      distance_move.slowdown_start_pwm[1] = (int16_t)motor_abs(commands[1]);
      if (distance_move.slowdown_start_pwm[0] < APP_MOTOR_BASE_PWM) {
        distance_move.slowdown_start_pwm[0] = APP_MOTOR_BASE_PWM;
      }
      if (distance_move.slowdown_start_pwm[1] < APP_MOTOR_BASE_PWM) {
        distance_move.slowdown_start_pwm[1] = APP_MOTOR_BASE_PWM;
      }
      Pid_Reset(&speed_pids[0]);
      Pid_Reset(&speed_pids[1]);
    }

    const float usable_slowdown =
        distance_move.slowdown_start_mm - APP_GO_DISTANCE_TOLERANCE_MM;
    float ratio = (remaining_mm - APP_GO_DISTANCE_TOLERANCE_MM) / usable_slowdown;
    if (ratio < 0.0f) {
      ratio = 0.0f;
    } else if (ratio > 1.0f) {
      ratio = 1.0f;
    }
    speed_mm_s *= ratio;
    m1_limit = motor_round_to_int16((float)distance_move.slowdown_start_pwm[0] * ratio);
    m2_limit = motor_round_to_int16((float)distance_move.slowdown_start_pwm[1] * ratio);
  }

  motor_set_forward_speed(speed_mm_s * direction, m1_limit, m2_limit);
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
    output_limits[i] = motor_default_output_limit();
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
  distance_move.start_m2_count = 0;
  distance_move.no_progress_cycles = 0U;
}

void Motor_Stop(void)
{
  const uint32_t primask = motor_enter_critical();
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
  distance_move.no_progress_cycles = 0U;
  motor_stop_outputs(true);
  motor_leave_critical(primask);
}

void Motor_Update(void)
{
  int32_t measured_counts[MOTOR_COUNT];
  bool fault_detected = false;

  motor_update_distance_move();

  for (uint8_t id = 1U; id <= MOTOR_COUNT; ++id) {
    const uint32_t index = id - 1U;
    measured_counts[index] = Encoder_TakeControlDelta(id) * encoder_signs[index];
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
    const int32_t output_limit = output_limits[index];
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
  if (distance_move.status != MOTOR_DISTANCE_IDLE) {
    motor_stop_outputs(false);
  }
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
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

  const EncoderStatus m1 = Encoder_GetStatus(1U);
  const EncoderStatus m2 = Encoder_GetStatus(2U);
  const float absolute_distance = motor_abs_float(distance_mm);
  distance_move.target_mm = distance_mm;
  distance_move.start_m1_count = m1.position;
  distance_move.start_m2_count = m2.position;
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
  motor_update_distance_move();
  motor_leave_critical(primask);
  return MOTOR_DISTANCE_RUNNING;
}

void Motor_Move(int16_t forward, int16_t lateral, int16_t rotate)
{
  int32_t m1;
  int32_t m2;
  int32_t m3;
  int32_t maximum;
  const uint32_t primask = motor_enter_critical();

  if (motor_has_fault()) {
    motor_leave_critical(primask);
    return;
  }
  distance_move.status = MOTOR_DISTANCE_IDLE;
  distance_move.slowing = false;
  forward = motor_limit(forward);
  lateral = motor_limit(lateral);
  rotate = motor_limit(rotate);

  /* LED_3 three-wheel omni inverse kinematics (sqrt(3)/2 ~= 0.866). */
  m1 = -(int32_t)forward * 866 / 1000 - (int32_t)lateral / 2 + rotate;
  m2 =  (int32_t)forward * 866 / 1000 - (int32_t)lateral / 2 + rotate;
  m3 =  (int32_t)lateral + rotate;

  maximum = motor_abs(m1);
  if (motor_abs(m2) > maximum) {
    maximum = motor_abs(m2);
  }
  if (motor_abs(m3) > maximum) {
    maximum = motor_abs(m3);
  }
  if (maximum > MOTOR_MAX_SPEED) {
    m1 = m1 * MOTOR_MAX_SPEED / maximum;
    m2 = m2 * MOTOR_MAX_SPEED / maximum;
    m3 = m3 * MOTOR_MAX_SPEED / maximum;
  }

  motor_set_three_targets(m1, m2, m3);
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
