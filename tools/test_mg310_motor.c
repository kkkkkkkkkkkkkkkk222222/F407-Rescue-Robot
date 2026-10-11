/* Production motor controller with mocked timers and encoder feedback. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../Main/Src/motor.c"
#include "../Main/Src/Debug.c"

static TIM_TypeDef tim2_regs, tim5_regs, tim9_regs;
TIM_HandleTypeDef htim2 = {.Instance = &tim2_regs, .Init.Period = 4199U};
TIM_HandleTypeDef htim5 = {.Instance = &tim5_regs, .Init.Period = 4199U};
TIM_HandleTypeDef htim9 = {.Instance = &tim9_regs, .Init.Period = 4199U};
static EncoderStatus test_encoder[3];
static uint32_t test_ms;
static IMUData test_imu;
static LocationPose test_pose;
static VisionData test_vision;
uint32_t HAL_GetTick(void) { return test_ms; }
void Encoder_GetAll(EncoderStatus status[3]) { memcpy(status, test_encoder, sizeof(test_encoder)); }
IMUData IMU_GetData(void) { return test_imu; }
LocationPose Location_GetPose(void) { return test_pose; }
void Location_ResetReference(int32_t x, int32_t y, int32_t heading)
{ test_pose.x_mm = x; test_pose.y_mm = y; test_pose.heading_mdeg = heading; }
VisionData Vision_GetSnapshot(void) { return test_vision; }
void Vision_QueueMotionStatus(const VisionMotionStatus *status) { (void)status; }

void MotorTest_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
  assert((timer->Instance->CR1 & TIM_CR1_UDIS) != 0U);
  assert(value <= timer->Init.Period + 1U);
  timer->compare[channel] = value;
}

static void reset(void)
{
  memset(test_encoder, 0, sizeof(test_encoder));
  test_imu = (IMUData){.ready = true};
  test_pose = (LocationPose){.valid = true};
  test_vision = (VisionData){0};
  tim2_regs.CR1 = tim5_regs.CR1 = tim9_regs.CR1 = 0U;
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
    const int16_t command = Motor_GetStatus(id).command;
    assert(abs(command) <= APP_MOTOR_PWM_MAX_PERMILLE);
    const MotorPwm *pwm = &motors[id - 1U];
    assert(pwm->in1_timer == pwm->in2_timer);
    const uint32_t period = pwm->in1_timer->Init.Period + 1U;
    const uint32_t in1 = pwm->in1_timer->compare[pwm->in1_channel];
    const uint32_t in2 = pwm->in2_timer->compare[pwm->in2_channel];
    const uint32_t drive = period * (uint32_t)abs(command) / 1000U;
    if (command > 0) {
      assert(in1 == period && in2 == period - drive);
    } else if (command < 0) {
      assert(in1 == period - drive && in2 == period);
    } else {
      assert(in1 == 0U && in2 == 0U);
    }
    assert(drive <= period * APP_MOTOR_PWM_MAX_PERMILLE / 1000U);
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
  assert(APP_MOTOR_PWM_MAX_PERMILLE == 600);
  motor_apply_pwm(1U, 1000); assert_drive_cap();
  assert(Motor_GetStatus(1U).command == APP_MOTOR_PWM_MAX_PERMILLE);
  motor_apply_pwm(1U, -1000); assert_drive_cap();
  assert(Motor_GetStatus(1U).command == -APP_MOTOR_PWM_MAX_PERMILLE);
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

static void test_slow_decay_mapping_and_update_guard(void)
{
  static const int16_t requests[] = {
    0, 1, 100, 289, 600, 1000, -1000, -600, -289, -100, -1, 0
  };
  reset();
  for (uint8_t id = 1U; id <= 3U; ++id) {
    TIM_HandleTypeDef *timer = motors[id - 1U].in1_timer;
    for (uint32_t i = 0U; i < sizeof(requests) / sizeof(requests[0]); ++i) {
      timer->Instance->CR1 = 0x100U;
      motor_apply_pwm(id, requests[i]);
      assert_drive_cap();
      assert(timer->Instance->CR1 == 0x100U);
      const int16_t expected = requests[i] > 600 ? 600 :
          (requests[i] < -600 ? -600 : requests[i]);
      assert(Motor_GetStatus(id).command == expected);
    }
    timer->Instance->CR1 = 0x100U | TIM_CR1_UDIS;
    motor_apply_pwm(id, 100);
    assert(timer->Instance->CR1 == (0x100U | TIM_CR1_UDIS));
    timer->Instance->CR1 = 0U;
    motor_apply_pwm(id, 0);
  }
}

static void test_stop_brake_release_and_restart(void)
{
  reset();
  Motor_Move(0.0f, 0.0f, 300.0f);
  update();
  Motor_Stop();
  assert(brake_cycles_remaining == APP_MOTOR_BRAKE_CYCLES);
  for (uint32_t i = 1U; i <= APP_MOTOR_BRAKE_CYCLES; ++i) {
    Motor_Stop();
    update();
    assert(brake_cycles_remaining == APP_MOTOR_BRAKE_CYCLES - i);
    for (uint8_t id = 1U; id <= 3U; ++id) {
      const MotorPwm *pwm = &motors[id - 1U];
      const uint32_t expected = brake_cycles_remaining > 0U ? 4200U : 0U;
      assert(pwm->in1_timer->compare[pwm->in1_channel] == expected);
      assert(pwm->in2_timer->compare[pwm->in2_channel] == expected);
    }
  }
  assert_drive_cap();
  Motor_Move(0.0f, 0.0f, 300.0f);
  update();
  Motor_Stop();
  Motor_Move(0.0f, 0.0f, -300.0f);
  for (uint32_t i = 0U; i < APP_MOTOR_BRAKE_CYCLES; ++i) {
    update();
  }
  assert_drive_cap();
  update();
  assert_drive_cap();
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).command < 0);
  }
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
  assert(Motor_GetStatus(1U).command == APP_MOTOR_PWM_MAX_PERMILLE);
  assert(speed_pids[0].kp == 6.0f && speed_pids[0].ki == 0.15f && speed_pids[0].kd == 0.0f);
  assert(speed_pids[0].output_min == -450.0f && speed_pids[0].output_max == 450.0f);
  assert(speed_pids[0].integral_min == -2500.0f && speed_pids[0].integral_max == 2500.0f);
  assert(APP_MOVE_SPIN_PID_BOOST == 1.8f);
}

static void test_no_stall_or_direction_interlock(void)
{
  assert(!APP_ENABLE_LOCAL_MOTOR_FAULT_STOP);
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

static void test_straight_idle_wheel_rejects_small_commands(void)
{
  reset();
  Motor_Move(300.0f, 0.0f, 0.0f);
  update();
  assert(Motor_GetStatus(3U).target == 0);
  for (int i = 0; i < 200; ++i) {
    Motor_Move(300.0f, 0.0f, (i & 1) ? 1.0f : -1.0f);
    test_encoder[2].delta_10ms = (i & 1) ? 1 : -1;
    update();
    assert(Motor_GetStatus(3U).target == 0);
    assert(Motor_GetStatus(3U).command == 0);
    assert(speed_pids[2].integral == 0.0f);
    assert(Motor_GetStatus(1U).target > 0);
    assert(Motor_GetStatus(2U).target < 0);
    assert_drive_cap();
  }
  reset();
  Motor_Move(0.0f, 0.0f, 120.0f);
  update();
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target > 0 && Motor_GetStatus(id).command > 0);
  }
}

static void test_wheel_zero_band_hysteresis_and_reversal(void)
{
  reset();
  Motor_SetSpeed(5.9f, 3U);
  assert(Motor_GetStatus(3U).target == 0);
  Motor_SetSpeed(6.1f, 3U);
  assert(Motor_GetStatus(3U).target > 0);
  Motor_SetSpeed(3.1f, 3U);
  assert(Motor_GetStatus(3U).target > 0);
  Motor_SetSpeed(2.9f, 3U);
  assert(Motor_GetStatus(3U).target == 0);
  Motor_SetSpeed(5.9f, 3U);
  assert(Motor_GetStatus(3U).target == 0);
  Motor_SetSpeed(6.1f, 3U);
  Motor_SetSpeed(-3.1f, 3U);
  assert(Motor_GetStatus(3U).target == 0);
  Motor_SetSpeed(-5.9f, 3U);
  assert(Motor_GetStatus(3U).target == 0);
  Motor_SetSpeed(-6.1f, 3U);
  update();
  assert(Motor_GetStatus(3U).target < 0 && Motor_GetStatus(3U).command < 0);
  assert_drive_cap();
}

static void test_debug_straight_noise_and_real_correction(void)
{
  for (int direction = -1; direction <= 1; direction += 2) {
    reset();
    Debug_Init(test_ms);
    test_vision = (VisionData){
      .motion_valid = true, .motion_sequence = 1U,
      .motion_opcode = VISION_MOTION_CMD_MOVE_BODY,
      .motion_arg1 = (int16_t)(direction * 1000), .motion_speed = 300U
    };
    Debug_Process(test_ms);
    update();
    for (int i = 0; i < 100; ++i) {
      test_pose.x_mm = direction * 100;
      test_pose.y_mm = (i & 1) ? 1 : -1;
      test_pose.heading_mdeg = (i & 1) ? 200 : 359800;
      Debug_Process(test_ms);
      update();
      assert(Debug_GetStatus().state == DEBUG_MOTION_RUNNING);
      assert(Motor_GetStatus(3U).target == 0 && Motor_GetStatus(3U).command == 0);
      assert_drive_cap();
    }
    test_pose.heading_mdeg = 0;
    test_pose.y_mm = 12;
    Debug_Process(test_ms);
    update();
    assert(Motor_GetStatus(3U).target != 0 && Motor_GetStatus(3U).command != 0);
    test_pose.y_mm = 0;
    Debug_Process(test_ms);
    update();
    assert(Motor_GetStatus(3U).target == 0);
    test_pose.heading_mdeg = 5000;
    Debug_Process(test_ms);
    update();
    assert(Motor_GetStatus(3U).target != 0);
  }
}

int main(void)
{
  test_units_and_encoder_scale();
  test_hard_cap_and_brake_are_distinct();
  test_slow_decay_mapping_and_update_guard();
  test_stop_brake_release_and_restart();
  test_physical_speed_limit_preserves_mixer();
  test_normal_and_spin_caps();
  test_low_speed_and_overspeed();
  test_original_pid_parameters_preserved();
  test_no_stall_or_direction_interlock();
  test_straight_idle_wheel_rejects_small_commands();
  test_wheel_zero_band_hysteresis_and_reversal();
  test_debug_straight_noise_and_real_correction();
  puts("MG310 motor: 12 scenarios PASS");
  return 0;
}
