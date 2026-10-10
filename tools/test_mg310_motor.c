/* Production motor controller with mocked timers and encoder feedback. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../Main/Src/motor.c"

TIM_HandleTypeDef htim2 = {.Init.Period = 4199U};
TIM_HandleTypeDef htim5 = {.Init.Period = 4199U};
TIM_HandleTypeDef htim9 = {.Init.Period = 4199U};
static EncoderStatus test_encoder[3];
static uint32_t test_ms;
uint32_t HAL_GetTick(void) { return test_ms; }
void Encoder_GetAll(EncoderStatus status[3]) { memcpy(status, test_encoder, sizeof(test_encoder)); }
IMUData IMU_GetData(void) { return (IMUData){.ready = true}; }
LocationPose Location_GetPose(void) { return (LocationPose){.valid = true}; }

static void reset(void)
{
  memset(test_encoder, 0, sizeof(test_encoder));
  test_ms = 0U;
  Motor_Init();
}

static void update(void)
{
  test_ms += APP_MOTOR_CONTROL_PERIOD_MS;
  Motor_Update();
}

static void assert_drive_cap(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(abs(Motor_GetStatus(id).command) <= 550);
    const MotorPwm *pwm = &motors[id - 1U];
    assert(pwm->in1_timer->compare[pwm->in1_channel] <= 2310U);
    assert(pwm->in2_timer->compare[pwm->in2_channel] <= 2310U);
  }
}

static void test_units_and_encoder_scale(void)
{
  reset();
  assert(fabsf(APP_ENCODER_COUNTS_PER_WHEEL_REV - 1061.268f) < 0.001f);
  assert(fabsf(motor_counts_to_mm(APP_ENCODER_COUNTS_PER_WHEEL_REV) -
               70.0f * 3.14159265359f) < 0.001f);
  Motor_SetSpeed(110.0f, 1U);
  assert(fabsf(target_counts_per_update[0] - 5.30844f) < 0.001f);
  assert(Motor_GetStatus(1U).target_speed_mm_s == 110);
}

static void test_hard_cap_and_brake_are_distinct(void)
{
  reset();
  motor_apply_pwm(1U, 1000); assert_drive_cap();
  assert(Motor_GetStatus(1U).command == 550);
  motor_apply_pwm(1U, -1000); assert_drive_cap();
  assert(Motor_GetStatus(1U).command == -550);
  motor_apply_brake(1U);
  assert(htim5.compare[TIM_CHANNEL_3] == 4200U);
  assert(htim5.compare[TIM_CHANNEL_4] == 4200U);
  assert(Motor_GetStatus(1U).command == 0);
}

static void test_physical_speed_limit_preserves_mixer(void)
{
  reset();
  Motor_Move(100000.0f, 0.0f, 0.0f);
  assert(fabsf(target_counts_per_update[0] - APP_MOTOR_MAX_COUNT_10MS) < 0.001f);
  assert(fabsf(target_counts_per_update[0] + target_counts_per_update[1]) < 0.001f);
  assert(target_counts_per_update[2] == 0.0f);
  assert(Motor_GetStatus(1U).target_speed_mm_s == 1100);
}

static void test_normal_and_spin_caps(void)
{
  for (int spin = 0; spin <= 1; ++spin) {
    for (int sign = -1; sign <= 1; sign += 2) {
      reset(); Motor_SetSpeed(sign * 1000.0f, 1U);
      spin_move.active = spin != 0;
      for (int i = 0; i < 10; ++i) { update(); assert_drive_cap(); }
      assert(!motor_has_fault());
    }
  }
}

static void test_low_speed_and_overspeed(void)
{
  reset(); Motor_SetSpeed(110.0f, 1U); update();
  assert(Motor_GetStatus(1U).command > 100 && Motor_GetStatus(1U).command < 200);
  test_encoder[0].delta_10ms = -100;
  update();
  assert(Motor_GetStatus(1U).command == 0);
}

static void test_original_pid_parameters_preserved(void)
{
  reset(); Motor_SetSpeed(1000.0f, 1U);
  for (int i = 0; i < 10; ++i) { update(); }
  assert(Motor_GetStatus(1U).command == 550);
  assert(speed_pids[0].kp == 6.0f && speed_pids[0].ki == 0.15f && speed_pids[0].kd == 0.0f);
  assert(speed_pids[0].output_min == -450.0f && speed_pids[0].output_max == 450.0f);
  assert(speed_pids[0].integral_min == -2500.0f && speed_pids[0].integral_max == 2500.0f);
  assert(APP_MOVE_SPIN_PID_BOOST == 1.8f);
}

static void test_no_stall_or_direction_interlock(void)
{
  assert(APP_ENABLE_TASK && !APP_ENABLE_LOCAL_MOTOR_FAULT_STOP);
  reset(); Motor_Move(1000.0f, 0.0f, 0.0f);
  for (int i = 0; i < 150; ++i) { update(); assert_drive_cap(); }
  assert(!motor_has_fault());
  assert(Motor_GetStatus(1U).target != 0);
  test_encoder[0].delta_10ms = 10; /* opposite to the positive M1 command */
  for (int i = 0; i < 30; ++i) { update(); }
  assert(!motor_has_fault() && Motor_GetStatus(1U).target != 0);
  Motor_Stop();
  assert(Motor_GetStatus(1U).target == 0);
  reset();
  assert(Motor_MoveDistance(1.0f, 300.0f) == MOTOR_DISTANCE_RUNNING);
  for (int i = 0; i < 150; ++i) { update(); }
  assert(distance_move.status == MOTOR_DISTANCE_RUNNING && !motor_has_fault());
}

int main(void)
{
  test_units_and_encoder_scale();
  test_hard_cap_and_brake_are_distinct();
  test_physical_speed_limit_preserves_mixer();
  test_normal_and_spin_caps();
  test_low_speed_and_overspeed();
  test_original_pid_parameters_preserved();
  test_no_stall_or_direction_interlock();
  puts("MG310 motor: 7 scenarios PASS");
  return 0;
}
