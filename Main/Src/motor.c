#include "motor.h"

#include "app_config.h"
#include "encoder.h"
#include "main.h"
#include "pid.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

typedef struct {
  TIM_HandleTypeDef *in1_timer;
  uint32_t in1_channel;
  TIM_HandleTypeDef *in2_timer;
  uint32_t in2_channel;
} MotorPwm;

static const MotorPwm motors[4] = {
  {&htim5, TIM_CHANNEL_3, &htim5, TIM_CHANNEL_4},
  {&htim9, TIM_CHANNEL_1, &htim9, TIM_CHANNEL_2},
  {&htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4},
  {&htim10, TIM_CHANNEL_1, &htim11, TIM_CHANNEL_1},
};

static int16_t commands[4];
static int16_t targets[3];
static Pid_t speed_pids[3];
static const int8_t motor_signs[3] = {
  APP_OMNI_M1_MOTOR_SIGN,
  APP_OMNI_M2_MOTOR_SIGN,
  APP_OMNI_M3_MOTOR_SIGN
};
static const int8_t encoder_signs[3] = {
  APP_OMNI_M1_ENCODER_SIGN,
  APP_OMNI_M2_ENCODER_SIGN,
  APP_OMNI_M3_ENCODER_SIGN
};

static int32_t motor_abs(int32_t value)
{
  return (value < 0) ? -value : value;
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

static void motor_set_target(uint8_t id, int32_t target)
{
  int16_t limited;

  if ((id < 1U) || (id > 3U)) {
    return;
  }
  limited = motor_limit(target);
  if ((limited == 0) ||
      ((targets[id - 1U] > 0) && (limited < 0)) ||
      ((targets[id - 1U] < 0) && (limited > 0))) {
    Pid_Reset(&speed_pids[id - 1U]);
  }
  targets[id - 1U] = limited;
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

void Motor_Init(void)
{
  for (uint32_t i = 0; i < 4U; ++i) {
    (void)HAL_TIM_PWM_Start(motors[i].in1_timer, motors[i].in1_channel);
    (void)HAL_TIM_PWM_Start(motors[i].in2_timer, motors[i].in2_channel);
  }
  for (uint32_t i = 0; i < 3U; ++i) {
    Pid_Init(&speed_pids[i],
             APP_MOTOR_SPEED_KP,
             APP_MOTOR_SPEED_KI,
             APP_MOTOR_SPEED_KD,
             -APP_MOTOR_PID_LIMIT,
             APP_MOTOR_PID_LIMIT,
             -APP_MOTOR_INTEGRAL_LIMIT,
             APP_MOTOR_INTEGRAL_LIMIT);
    targets[i] = 0;
  }
  Motor_StopAll();
}

void Motor_Control(uint8_t id, int16_t speed)
{
  if ((id < 1U) || (id > 4U)) {
    return;
  }
  if (speed > 1000) {
    speed = 1000;
  } else if (speed < -1000) {
    speed = -1000;
  }

  const MotorPwm *motor = &motors[id - 1U];
  const uint32_t in1 = (speed > 0) ? speed_to_compare(motor->in1_timer, speed) : 0U;
  const uint32_t in2 = (speed < 0) ? speed_to_compare(motor->in2_timer, speed) : 0U;
  __HAL_TIM_SET_COMPARE(motor->in1_timer, motor->in1_channel, in1);
  __HAL_TIM_SET_COMPARE(motor->in2_timer, motor->in2_channel, in2);
  commands[id - 1U] = speed;
}

void Motor_StopAll(void)
{
  for (uint32_t i = 0; i < 3U; ++i) {
    targets[i] = 0;
    Pid_Reset(&speed_pids[i]);
  }
  for (uint8_t id = 1U; id <= 4U; ++id) {
    Motor_Control(id, 0);
  }
}

int16_t Motor_GetCommand(uint8_t id)
{
  return ((id >= 1U) && (id <= 4U)) ? commands[id - 1U] : 0;
}

void Motor_Update(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const uint32_t index = id - 1U;
    const int32_t measured = Encoder_TakeControlDelta(id) * encoder_signs[index];
    int32_t target_count;
    int32_t feed_forward;
    int32_t output;
    float correction;

    if (targets[index] == 0) {
      Pid_Reset(&speed_pids[index]);
      Motor_Control(id, 0);
      continue;
    }

    target_count = (int32_t)targets[index] * APP_MOTOR_MAX_COUNT_20MS /
                   MOTOR_MAX_SPEED;
    correction = Pid_Update(&speed_pids[index],
                            (float)target_count,
                            (float)measured);
    feed_forward = (int32_t)targets[index] *
                   APP_MOTOR_FEED_FORWARD_PERCENT / 100;
    output = feed_forward + (int32_t)correction;
    Motor_Control(id, motor_limit(output * motor_signs[index]));
  }

  /* M4 is fitted on the PCB but is not part of the three-wheel chassis. */
  Motor_Control(4U, 0);
}

void Motor_Stop(void)
{
  Motor_StopAll();
}

void Motor_Move(int16_t forward, int16_t lateral, int16_t rotate)
{
  int32_t m1;
  int32_t m2;
  int32_t m3;
  int32_t maximum;

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
}

void Motor_Forward(int16_t speed)
{
  Motor_Move((int16_t)motor_abs(speed), 0, 0);
}

void Motor_Back(int16_t speed)
{
  Motor_Move((int16_t)-motor_abs(speed), 0, 0);
}

void Motor_RotateLeft(int16_t speed)
{
  Motor_Move(0, 0, (int16_t)motor_abs(speed));
}

void Motor_RotateRight(int16_t speed)
{
  Motor_Move(0, 0, (int16_t)-motor_abs(speed));
}

void Motor_Follow(int16_t speed, int16_t turn)
{
  Motor_Move(speed, 0, turn);
}

int16_t Motor_GetTarget(uint8_t id)
{
  return ((id >= 1U) && (id <= 3U)) ? targets[id - 1U] : 0;
}
