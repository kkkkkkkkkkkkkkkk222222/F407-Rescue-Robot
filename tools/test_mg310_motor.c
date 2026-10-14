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
#if defined(MOTOR_TRACE_CHECK_ONLY) || defined(MOTOR_TURN_PROFILE_CHECK_ONLY)
void WheelCheck_Stop(void) { Motor_Stop(); }
#endif

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
#if defined(MOTOR_TRACE_CHECK_ONLY) || defined(MOTOR_TURN_PROFILE_CHECK_ONLY)
  ++test_imu.sample_count;
#endif
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

static void test_debug_field_noise_and_real_correction(void)
{
  for (int direction = -1; direction <= 1; direction += 2) {
    reset();
    Debug_Init(test_ms);
    test_vision = (VisionData){
      .motion_valid = true, .motion_sequence = 1U,
      .motion_opcode = VISION_MOTION_CMD_MOVE_FIELD,
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
    assert(Motor_GetStatus(3U).target == 0);
    test_imu.yaw_mdeg = 10000LL;
    update();
    assert(Motor_GetStatus(3U).target != 0 && Motor_GetStatus(3U).command != 0);
    test_imu.yaw_mdeg = 0LL;
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

static void start_debug_straight(int16_t distance, uint16_t speed)
{
  Debug_Init(test_ms);
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 1U,
    .motion_opcode = VISION_MOTION_CMD_MOVE_BODY,
    .motion_flags = VISION_MOTION_VALID | VISION_MOTION_KEEP_HEADING |
                    VISION_MOTION_ACK_REQUIRED,
    .motion_arg1 = distance, .motion_speed = speed
  };
  Debug_Process(test_ms);
  update();
  for (int i = 0; i < 100 &&
       (distance_move.waiting_for_stop ||
        distance_move.current_speed_mm_s < active_move_speed_mm_s); ++i) {
    update();
    Debug_Process(test_ms);
  }
  assert(!distance_move.waiting_for_stop && distance_move.status == MOTOR_DISTANCE_RUNNING);
}

static void set_straight_encoder_progress(float forward_mm)
{
  const float mm_per_count = 3.14159265359f * APP_WHEEL_DIAMETER_MM /
                             APP_ENCODER_COUNTS_PER_WHEEL_REV;
  const int64_t counts = (int64_t)llroundf(forward_mm * 0.86602540378f / mm_per_count);
  test_encoder[0].position = distance_move.start_m1_count + counts * APP_OMNI_M1_ENCODER_SIGN;
  test_encoder[1].position = distance_move.start_m2_count - counts * APP_OMNI_M2_ENCODER_SIGN;
}

static void test_debug_straight_uses_shared_heading_only(void)
{
  for (int direction = -1; direction <= 1; direction += 2) {
    reset();
    start_debug_straight((int16_t)(direction * 1000), 500U);
    for (int i = 0; i < 100; ++i) {
      test_pose.x_mm = direction * 5000;
      test_pose.y_mm = (i & 1) ? 2000 : -2000;
      test_pose.heading_mdeg = (i % 4) * 90000;
      test_encoder[2].position = i * 1000;
      test_encoder[2].delta_10ms = (i & 1) ? 100 : -100;
      Debug_Process(test_ms);
      update();
      assert(Debug_GetStatus().state == DEBUG_MOTION_RUNNING);
      assert(Debug_GetStatus().progress == 0 && Debug_GetStatus().remaining == 1000U);
      assert(Motor_GetStatus(1U).target_speed_mm_s == direction * 433);
      assert(Motor_GetStatus(2U).target_speed_mm_s == -direction * 433);
      assert(Motor_GetStatus(3U).target == 0 && Motor_GetStatus(3U).command == 0);
      assert_drive_cap();
    }
    test_imu.yaw_mdeg = 10000LL;
    update();
    Debug_Process(test_ms);
    assert(Motor_GetStatus(3U).target > 0);
    test_imu.yaw_mdeg = -10000LL;
    update();
    Debug_Process(test_ms);
    assert(Motor_GetStatus(3U).target < 0);
    assert_drive_cap();
  }
}

static void test_debug_straight_signed_distance_and_completion(void)
{
  for (int direction = -1; direction <= 1; direction += 2) {
    reset();
    test_encoder[0].position = 10000000000LL;
    test_encoder[1].position = -10000000000LL;
    start_debug_straight((int16_t)(direction * 1000), 500U);
    set_straight_encoder_progress(direction * 50.0f);
    test_pose.x_mm = 9999;
    test_pose.heading_mdeg = 180000;
    test_encoder[2].position = 3000000LL;
    update();
    Debug_Process(test_ms);
    assert(abs(Debug_GetStatus().progress - direction * 50) <= 1);
    assert(abs((int)Debug_GetStatus().remaining - 950) <= 1);
    set_straight_encoder_progress(-direction * 50.0f);
    update();
    Debug_Process(test_ms);
    assert(Debug_GetStatus().progress == 0 && Debug_GetStatus().remaining == 1000U);
    set_straight_encoder_progress(direction * 998.0f);
    update();
    Debug_Process(test_ms);
    assert(Debug_GetStatus().state == DEBUG_MOTION_DONE);
    assert(Debug_GetStatus().progress == direction * 1000);
    assert(Debug_GetStatus().remaining == 0U);
    for (uint8_t id = 1U; id <= 3U; ++id) {
      assert(Motor_GetStatus(id).target == 0);
    }
  }
}

static void test_debug_blocked_front_without_yaw_keeps_rear_zero(void)
{
  reset();
  start_debug_straight(1000, 500U);
  for (int i = 0; i < 100; ++i) {
    test_encoder[0].delta_10ms = 0;
    test_encoder[1].delta_10ms = 13;
    test_encoder[1].position += 13;
    test_pose.y_mm = i * 5;
    test_pose.heading_mdeg = 180000;
    Debug_Process(test_ms);
    update();
    assert(Debug_GetStatus().state == DEBUG_MOTION_RUNNING);
    assert(Motor_GetStatus(1U).target > 0 && Motor_GetStatus(2U).target < 0);
    assert(Motor_GetStatus(3U).target == 0 && Motor_GetStatus(3U).command == 0);
    assert_drive_cap();
  }
}

static void test_debug_straight_default_and_explicit_speed(void)
{
  reset();
  start_debug_straight(1000, 0U);
  assert(active_move_speed_mm_s == 500.0f);
  assert(Motor_GetStatus(1U).target_speed_mm_s == 433);
  reset();
  start_debug_straight(1000, 300U);
  assert(active_move_speed_mm_s == 300.0f);
  assert(Motor_GetStatus(1U).target_speed_mm_s == 260);
}

static void test_debug_turn_keeps_imu_completion(void)
{
  reset();
  Debug_Init(test_ms);
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 1U,
    .motion_opcode = VISION_MOTION_CMD_TURN_REL,
    .motion_arg1 = 900, .motion_speed = 900U
  };
  Debug_Process(test_ms);
  update();
  assert(!active_move_body_straight);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target > 0);
  }
  test_imu.yaw_mdeg = -90000LL;
  update();
  Debug_Process(test_ms);
  assert(Debug_GetStatus().state == DEBUG_MOTION_DONE);
  assert(Debug_GetStatus().progress == 900);
}

static void test_debug_straight_timeout_and_stop(void)
{
  reset();
  start_debug_straight(1000, 500U);
  test_ms = active_start_ms + active_timeout_ms;
  Debug_Process(test_ms);
  assert(Debug_GetStatus().state == DEBUG_MOTION_FAULT);
  assert(Debug_GetStatus().fault == DEBUG_MOTION_FAULT_TIMEOUT);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target == 0);
  }
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 2U,
    .motion_opcode = VISION_MOTION_CMD_STOP
  };
  Debug_Process(test_ms);
  assert(Debug_GetStatus().state == DEBUG_MOTION_STOPPED);
  assert(!active_move_body_straight);
}

static void test_debug_turn_both_feedback_signs_and_no_restart(void)
{
  for (int direction = -1; direction <= 1; direction += 2) {
    for (int feedback_sign = -1; feedback_sign <= 1; feedback_sign += 2) {
      reset();
      Debug_Init(test_ms);
      test_vision = (VisionData){
        .motion_valid = true, .motion_sequence = 1U,
        .motion_opcode = VISION_MOTION_CMD_TURN_REL,
        .motion_arg1 = (int16_t)(direction * 900), .motion_speed = 900U
      };
      Debug_Process(test_ms);
      update();
      test_imu.yaw_mdeg = feedback_sign * 60000LL;
      update();
      assert(angle_turn.status == MOTOR_TURN_RUNNING && angle_turn.slowing);
      assert(abs(Motor_GetStatus(1U).target_speed_mm_s) == 120);
      test_imu.yaw_mdeg = feedback_sign * 90000LL;
      update();
      assert(angle_turn.status == MOTOR_TURN_DONE);
      for (uint8_t id = 1U; id <= 3U; ++id) {
        assert(Motor_GetStatus(id).target == 0);
      }
      Debug_Process(test_ms);
      assert(Debug_GetStatus().state == DEBUG_MOTION_DONE);
      assert(Debug_GetStatus().progress == direction * 900);
      test_imu.yaw_mdeg = feedback_sign * 180000LL;
      for (int i = 0; i < 10; ++i) {
        update();
        Debug_Process(test_ms);
      }
      assert(Debug_GetStatus().state == DEBUG_MOTION_DONE);
      assert(angle_turn.status == MOTOR_TURN_IDLE);
      assert_drive_cap();
      test_vision.motion_sequence = 2U;
      test_vision.motion_arg1 = 450;
      Debug_Process(test_ms);
      assert(angle_turn.status == MOTOR_TURN_RUNNING);
      assert(angle_turn.start_yaw_mdeg == test_imu.yaw_mdeg);
      assert(Motor_GetTurnTravelledDeg() == 0.0f);
    }
  }
}

static void test_turn_timer_timeout_without_debug_polling(void)
{
  reset();
  Debug_Init(test_ms);
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 1U,
    .motion_opcode = VISION_MOTION_CMD_TURN_REL,
    .motion_arg1 = 900, .motion_speed = 900U
  };
  Debug_Process(test_ms);
  for (uint32_t i = 0; i <= APP_MOTOR_TURN_TIMEOUT_MS / APP_MOTOR_CONTROL_PERIOD_MS; ++i) {
    update();
  }
  assert(angle_turn.status == MOTOR_TURN_FAULT);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target == 0);
  }
  Debug_Process(test_ms);
  assert(Debug_GetStatus().state == DEBUG_MOTION_FAULT);
  assert(Debug_GetStatus().fault == DEBUG_MOTION_FAULT_TIMEOUT);
}

static void test_turn_imu_failure_and_stop_cancel(void)
{
  reset();
  Debug_Init(test_ms);
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 1U,
    .motion_opcode = VISION_MOTION_CMD_TURN_REL,
    .motion_arg1 = 900, .motion_speed = 900U
  };
  Debug_Process(test_ms);
  test_imu.ready = false;
  update();
  Debug_Process(test_ms);
  assert(Debug_GetStatus().fault == DEBUG_MOTION_FAULT_IMU);
  test_imu.ready = true;
  test_vision.motion_sequence = 2U;
  Debug_Process(test_ms);
  assert(angle_turn.status == MOTOR_TURN_RUNNING);
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 3U,
    .motion_opcode = VISION_MOTION_CMD_STOP
  };
  Debug_Process(test_ms);
  for (int i = 0; i < 10; ++i) { update(); Debug_Process(test_ms); }
  assert(Debug_GetStatus().state == DEBUG_MOTION_STOPPED);
  assert(angle_turn.status == MOTOR_TURN_IDLE);
  assert_drive_cap();
}

static void test_slow_turn_does_not_accelerate_at_end(void)
{
  reset();
  assert(Motor_TurnAngleSpeed(90.0f, 50.0f) == MOTOR_TURN_RUNNING);
  test_imu.yaw_mdeg = 60000LL;
  update();
  assert(angle_turn.slowing);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target_speed_mm_s == 50);
  }
}

static void test_debug_and_normal_distance_heading_match(void)
{
  reset();
  assert(Motor_MoveDistance(1.0f, 300.0f) == MOTOR_DISTANCE_RUNNING);
  for (int i = 0; i < 20; ++i) { update(); }
  test_imu.yaw_mdeg = 10000LL;
  update();
  assert(Motor_GetStatus(3U).target != 0);
  float normal_targets[3];
  memcpy(normal_targets, target_counts_per_update, sizeof(normal_targets));
  reset();
  start_debug_straight(1000, 300U);
  test_imu.yaw_mdeg = 10000LL;
  update();
  assert(Motor_GetStatus(3U).target != 0);
  for (uint32_t i = 0U; i < 3U; ++i) {
    assert(fabsf(target_counts_per_update[i] - normal_targets[i]) < 0.001f);
  }
}

static void test_debug_straight_stop_wait_is_bounded(void)
{
  reset();
  Debug_Init(test_ms);
  test_encoder[0].delta_10ms = 100;
  test_vision = (VisionData){
    .motion_valid = true, .motion_sequence = 1U,
    .motion_opcode = VISION_MOTION_CMD_MOVE_BODY,
    .motion_arg1 = 1000, .motion_speed = 500U
  };
  Debug_Process(test_ms);
  for (uint32_t i = 0; i <= APP_OMNI_ZERO_TIMEOUT_CYCLES; ++i) { update(); }
  assert(distance_move.status == MOTOR_DISTANCE_FAULT);
  Debug_Process(test_ms);
  assert(Debug_GetStatus().fault == DEBUG_MOTION_FAULT_MOTOR);
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target == 0);
  }
}

#if defined(MOTOR_TRACE_CHECK_ONLY)
static void test_trace_read_only_and_signed_pi(void)
{
  reset(); Motor_SetSpeed(136.0f, 1U);
  test_encoder[0].delta_10ms = -8;
  update();
  const MotorDebugTrace positive = Motor_GetDebugTrace();
  assert(positive.raw_count[0] == -8 && positive.feedback_count[0] == 8);
  assert(positive.error_c100[0] < 0 && positive.p_pwm100[0] < 0);
  assert(positive.correction_pwm100[0] < 0 && positive.output[0] < positive.feedforward[0]);
  const float integral = speed_pids[0].integral;
  const MotorDebugTrace again = Motor_GetDebugTrace();
  assert(speed_pids[0].integral == integral && again.output[0] == positive.output[0]);
  Motor_SetSpeed(-136.0f, 1U);
  test_encoder[0].delta_10ms = 8;
  update();
  const MotorDebugTrace negative = Motor_GetDebugTrace();
  assert(negative.feedback_count[0] == -8 && negative.error_c100[0] > 0);
  assert(negative.correction_pwm100[0] > 0 && negative.output[0] > negative.feedforward[0]);
}

static void test_trace_keeps_post_stop_yaw(void)
{
  reset();
  assert(Motor_TurnAngleSpeed(60.0f, 136.0f) == MOTOR_TURN_RUNNING);
  test_imu.yaw_mdeg = 45000;
  update();
  MotorDebugTrace trace = Motor_GetDebugTrace();
  assert(trace.turn_target_mdeg == 60000 && trace.turn_delta_mdeg == 45000 && trace.slowing);
  test_imu.yaw_mdeg = 60000;
  test_imu.gyro_mdps[2] = 80000;
  update();
  assert(angle_turn.status == MOTOR_TURN_RUNNING);
  test_imu.gyro_mdps[2] = 0;
  for (uint32_t elapsed = 0U; elapsed < APP_DEBUG_TURN_SETTLE_MS;
       elapsed += APP_MOTOR_CONTROL_PERIOD_MS) {
    update();
  }
  trace = Motor_GetDebugTrace();
  assert(trace.turn_state == MOTOR_TURN_DONE && trace.pi_active_mask == 0U);
  assert(trace.output[0] == 0 && trace.output[1] == 0 && trace.output[2] == 0);
  Motor_Stop();
  test_imu.yaw_mdeg = 64000;
  trace = Motor_GetDebugTrace();
  assert(trace.turn_state == MOTOR_TURN_IDLE && trace.turn_delta_mdeg == 64000);
}
#endif

#if defined(MOTOR_TURN_PROFILE_CHECK_ONLY)
static void profile_ticks(uint32_t count)
{
  for (uint32_t i = 0U; i < count; ++i) { update(); }
}

static void profile_stop_ticks(void)
{
  test_imu.gyro_mdps[2] = 0;
  memset(test_encoder, 0, sizeof(test_encoder));
  profile_ticks(APP_DEBUG_TURN_SETTLE_MS / APP_MOTOR_CONTROL_PERIOD_MS);
}

static void assert_targets_zero(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    assert(Motor_GetStatus(id).target == 0);
  }
}

static void test_profile_signed_unwrapped_targets(void)
{
  static const int32_t degrees[] = {60, 180, 360};
  for (int sign = -1; sign <= 1; sign += 2) {
    for (uint32_t i = 0U; i < sizeof(degrees) / sizeof(degrees[0]); ++i) {
      reset();
      const int64_t origin = sign * 10000000359000LL;
      test_imu.yaw_mdeg = origin;
      assert(Motor_TurnAngleSpeed(sign * degrees[i], 136.0f) == MOTOR_TURN_RUNNING);
      update();
      assert(angle_turn.reference_rate_dps * sign > 0.0f);
      test_imu.yaw_mdeg = origin + sign * (degrees[i] * 1000LL - 2000LL);
      update();
      assert(angle_turn.status == MOTOR_TURN_RUNNING && angle_turn.slowing);
      assert(abs(angle_turn.error_mdeg) == 2000);
      test_imu.yaw_mdeg = origin + sign * (degrees[i] * 1000LL + 400LL);
      test_imu.gyro_mdps[2] = sign * 80000;
      update();
      assert(angle_turn.status == MOTOR_TURN_RUNNING);
      assert(angle_turn.phase == MOTOR_TURN_PHASE_SETTLE);
      assert_targets_zero();
      profile_stop_ticks();
      assert(angle_turn.status == MOTOR_TURN_DONE);
      assert(fabsf(Motor_GetTurnTravelledDeg() - (degrees[i] + 0.4f)) < 0.002f);
      assert(angle_turn.start_yaw_mdeg == origin);
    }
  }
}

static void test_profile_stopped_fresh_samples_required(void)
{
  reset();
  assert(Motor_TurnAngleSpeed(60.0f, 136.0f) == MOTOR_TURN_RUNNING);
  test_imu.yaw_mdeg = 60000;
  test_imu.gyro_mdps[2] = 80000;
  profile_ticks(20U);
  assert(angle_turn.status == MOTOR_TURN_RUNNING && angle_turn.settle_ms == 0U);
  test_imu.gyro_mdps[2] = 0;
  test_encoder[0].delta_10ms = -1;
  profile_ticks(20U);
  assert(angle_turn.status == MOTOR_TURN_RUNNING && angle_turn.settle_ms == 0U);
  test_encoder[0].delta_10ms = 0;
  profile_ticks(5U);
  assert(angle_turn.settle_ms == 5U * APP_MOTOR_CONTROL_PERIOD_MS);
  test_imu.gyro_mdps[2] = APP_DEBUG_TURN_STOP_RATE_MDPS + 1;
  update();
  assert(angle_turn.settle_ms == 0U && angle_turn.status == MOTOR_TURN_RUNNING);
  test_imu.gyro_mdps[2] = 0;
  const uint32_t needed = APP_DEBUG_TURN_SETTLE_MS / APP_MOTOR_CONTROL_PERIOD_MS;
  profile_ticks(needed - 1U);
  assert(angle_turn.status == MOTOR_TURN_RUNNING);
  const uint32_t settled = angle_turn.settle_ms;
  test_ms += APP_MOTOR_CONTROL_PERIOD_MS;
  Motor_Update();
  assert(angle_turn.settle_ms == settled && angle_turn.status == MOTOR_TURN_RUNNING);
  update();
  assert(angle_turn.status == MOTOR_TURN_DONE);
}

static void test_profile_reverse_requires_stop_and_is_bounded(void)
{
  reset();
  assert(Motor_TurnAngleSpeed(60.0f, 136.0f) == MOTOR_TURN_RUNNING);
  profile_ticks(25U);
  test_imu.yaw_mdeg = 64000;
  test_imu.gyro_mdps[2] = 80000;
  profile_ticks(20U);
  assert(angle_turn.status == MOTOR_TURN_RUNNING && angle_turn.corrections == 0U);
  assert(angle_turn.phase == MOTOR_TURN_PHASE_BRAKE);
  assert_targets_zero();
  profile_stop_ticks();
  assert(angle_turn.corrections == 1U && angle_turn.last_direction == -1);
  assert(angle_turn.command_rate_dps < 0.0f);
  assert(fabsf(angle_turn.command_rate_dps) <= APP_DEBUG_TURN_TRIM_RATE_DPS);
  test_imu.yaw_mdeg = 57000;
  test_imu.gyro_mdps[2] = -10000;
  update();
  assert(angle_turn.corrections == 1U);
  assert_targets_zero();
  profile_stop_ticks();
  assert(angle_turn.corrections == 2U && angle_turn.last_direction == 1);
  assert(angle_turn.command_rate_dps > 0.0f);
  test_imu.yaw_mdeg = 63000;
  test_imu.gyro_mdps[2] = 10000;
  update();
  profile_stop_ticks();
  assert(angle_turn.status == MOTOR_TURN_FAULT);
  assert(angle_turn.failure == MOTOR_TURN_FAIL_TRIM_LIMIT);
  assert(angle_turn.corrections == APP_DEBUG_TURN_MAX_CORRECTIONS);
  assert_targets_zero();
}

static void test_profile_stale_timeout_and_direction_faults(void)
{
  reset();
  assert(Motor_TurnAngleSpeed(360.0f, 136.0f) == MOTOR_TURN_RUNNING);
  update();
  for (uint32_t elapsed = APP_MOTOR_CONTROL_PERIOD_MS;
       elapsed < APP_DEBUG_TURN_IMU_STALE_MS; elapsed += APP_MOTOR_CONTROL_PERIOD_MS) {
    test_ms += APP_MOTOR_CONTROL_PERIOD_MS;
    Motor_Update();
    assert(angle_turn.status == MOTOR_TURN_RUNNING);
  }
  test_ms += APP_MOTOR_CONTROL_PERIOD_MS;
  Motor_Update();
  assert(angle_turn.failure == MOTOR_TURN_FAIL_IMU_STALE);
  assert_targets_zero();
  reset();
  assert(Motor_TurnAngleSpeed(360.0f, 136.0f) == MOTOR_TURN_RUNNING);
  profile_ticks(APP_MOTOR_TURN_TIMEOUT_MS / APP_MOTOR_CONTROL_PERIOD_MS - 1U);
  assert(angle_turn.status == MOTOR_TURN_RUNNING);
  update();
  assert(angle_turn.status == MOTOR_TURN_FAULT);
  assert(angle_turn.failure == MOTOR_TURN_FAIL_TIMEOUT);
  assert_targets_zero();
  reset();
  assert(Motor_TurnAngleSpeed(60.0f, 136.0f) == MOTOR_TURN_RUNNING);
  test_imu.yaw_mdeg = -APP_DEBUG_TURN_WRONG_WAY_MDEG - 1LL;
  update();
  assert(angle_turn.failure == MOTOR_TURN_FAIL_WRONG_WAY);
  reset();
  assert(Motor_TurnAngleSpeed(60.0f, 136.0f) == MOTOR_TURN_RUNNING);
  test_imu.yaw_mdeg = 60000LL + APP_DEBUG_TURN_OVERRUN_MDEG + 1LL;
  update();
  assert(angle_turn.failure == MOTOR_TURN_FAIL_OVERRUN);
}

static void test_profile_pause_resume_and_stop(void)
{
  reset();
  test_imu.yaw_mdeg = 350000LL;
  assert(Motor_TurnAngleSpeed(60.0f, 136.0f) == MOTOR_TURN_RUNNING);
  profile_ticks(50U);
  assert(angle_turn.reference_rate_dps > 30.0f);
  const uint32_t started = angle_turn.start_ms;
  const uint32_t paused = test_ms;
  Motor_PauseFiniteMotion();
  test_imu.yaw_mdeg = 405000LL;
  profile_ticks(200U);
  assert(angle_turn.status == MOTOR_TURN_RUNNING && finite_motion_paused);
  assert_targets_zero();
  Motor_ResumeFiniteMotion();
  assert(!finite_motion_paused && angle_turn.start_yaw_mdeg == 350000LL);
  assert(angle_turn.start_ms == started + test_ms - paused);
  assert(angle_turn.travelled_mdeg == 55000LL);
  assert(fabsf(angle_turn.reference_rate_dps) <=
         APP_DEBUG_TURN_ACCEL_DPS2 * APP_MOTOR_CONTROL_PERIOD_MS * 0.001f + 0.001f);
  assert(abs(Motor_GetStatus(1U).target_speed_mm_s) < 20);
  update();
  assert(angle_turn.status == MOTOR_TURN_RUNNING);
  Motor_Stop();
  profile_ticks(20U);
  assert(angle_turn.status == MOTOR_TURN_IDLE);
  assert_targets_zero();
}

static void test_profile_debug_reports_actual_done_without_restart(void)
{
  for (int sign = -1; sign <= 1; sign += 2) {
    reset();
    Debug_Init(test_ms);
    test_vision = (VisionData){
      .motion_valid = true, .motion_sequence = 1U,
      .motion_opcode = VISION_MOTION_CMD_TURN_REL,
      .motion_arg1 = (int16_t)(sign * 600), .motion_speed = 600U
    };
    Debug_Process(test_ms);
    update();
    test_imu.yaw_mdeg = sign * 60400LL;
    profile_stop_ticks();
    assert(angle_turn.status == MOTOR_TURN_DONE);
    Debug_Process(test_ms);
    assert(Debug_GetStatus().state == DEBUG_MOTION_DONE);
    assert(abs(Debug_GetStatus().progress - sign * 604) <= 1);
    assert(Debug_GetStatus().progress != sign * 600);
    assert(Debug_GetStatus().remaining == 0U);
    test_imu.yaw_mdeg = sign * 63000LL;
    for (uint32_t i = 0U; i < 20U; ++i) {
      update();
      Debug_Process(test_ms);
    }
    assert(Debug_GetStatus().state == DEBUG_MOTION_DONE);
    assert(angle_turn.status == MOTOR_TURN_IDLE);
    assert_targets_zero();
  }
}

static void test_profile_feedforward_scope_and_original_gains(void)
{
  reset();
  assert(APP_DEBUG_TURN_STATIC_PWM == 70.0f && APP_MOTOR_BASE_PWM == 100);
  assert(APP_MOTOR_PWM_MAX_PERMILLE == 600);
  assert(speed_pids[0].kp == 5.0f && speed_pids[0].ki == 0.12f && speed_pids[0].kd == 0.0f);
  assert(Motor_TurnAngleSpeed(180.0f, 136.0f) == MOTOR_TURN_RUNNING);
  profile_ticks(25U);
  const float target = target_counts_per_update[0];
  const int16_t turn_ff = Motor_GetDebugTrace().feedforward[0];
  assert(turn_ff > 70 && turn_ff < 150);
  assert_drive_cap();
  Motor_Stop();
  profile_ticks(APP_MOTOR_BRAKE_CYCLES);
  Motor_Move(motor_counts_to_mm_s(target) / 0.8660254f, 0.0f, 0.0f);
  update();
  const int16_t move_ff = Motor_GetDebugTrace().feedforward[0];
  assert(abs((abs(move_ff) - abs(turn_ff)) - 30) <= 1);
  assert(speed_pids[0].kp == 5.0f && speed_pids[0].ki == 0.12f);
  assert_drive_cap();
}
#endif

int main(void)
{
#if defined(MOTOR_TURN_PROFILE_CHECK_ONLY)
  test_profile_signed_unwrapped_targets();
  test_profile_stopped_fresh_samples_required();
  test_profile_reverse_requires_stop_and_is_bounded();
  test_profile_stale_timeout_and_direction_faults();
  test_profile_pause_resume_and_stop();
  test_profile_debug_reports_actual_done_without_restart();
  test_profile_feedforward_scope_and_original_gains();
  puts("MotionDebug turn profile: 7 focused boundary groups PASS");
#elif defined(MOTOR_TRACE_CHECK_ONLY)
  test_trace_read_only_and_signed_pi();
  test_trace_keeps_post_stop_yaw();
  puts("Motion diagnostic trace: signed PI, read-only access and post-stop yaw PASS");
#else
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
  test_debug_field_noise_and_real_correction();
  test_debug_straight_uses_shared_heading_only();
  test_debug_straight_signed_distance_and_completion();
  test_debug_blocked_front_without_yaw_keeps_rear_zero();
  test_debug_straight_default_and_explicit_speed();
  test_debug_turn_keeps_imu_completion();
  test_debug_straight_timeout_and_stop();
  test_debug_turn_both_feedback_signs_and_no_restart();
  test_turn_timer_timeout_without_debug_polling();
  test_turn_imu_failure_and_stop_cancel();
  test_slow_turn_does_not_accelerate_at_end();
  test_debug_and_normal_distance_heading_match();
  test_debug_straight_stop_wait_is_bounded();
  puts("MG310 motor: 24 scenarios PASS");
#endif
  return 0;
}
