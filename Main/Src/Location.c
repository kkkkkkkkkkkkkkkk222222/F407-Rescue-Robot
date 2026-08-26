#include "Location.h"

#include <math.h>

#include "app_config.h"
#include "encoder.h"
#include "imu.h"
#include "main.h"

#define LOCATION_PI       3.14159265358979323846f
#define LOCATION_SQRT3    1.73205080756887729353f
#define LOCATION_DEG_RAD  (LOCATION_PI / 180.0f)

typedef struct {
  float x_mm;
  float y_mm;
  float path_mm;
  int64_t heading_unwrapped_mdeg;
  int64_t previous_imu_yaw_mdeg;
  uint8_t start_zone;
  bool imu_sample_valid;
  bool valid;
} LocationState;

static volatile LocationState location;

static uint32_t location_enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void location_leave_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static int32_t location_round(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t location_wrap_heading(int64_t heading_mdeg)
{
  heading_mdeg %= 360000LL;
  if (heading_mdeg < 0LL) {
    heading_mdeg += 360000LL;
  }
  return (int32_t)heading_mdeg;
}

static float location_counts_to_mm(int32_t counts)
{
  const float circumference_mm = LOCATION_PI * (float)APP_WHEEL_DIAMETER_MM;
  return (float)counts * circumference_mm /
         (float)APP_ENCODER_COUNTS_PER_WHEEL_REV;
}

static void location_start_pose(LocationStart start, float *x_mm, float *y_mm,
                                int64_t *heading_mdeg)
{
  const bool right = (start == LOCATION_START_2) ||
                     (start == LOCATION_START_4);
  const bool top = (start == LOCATION_START_1) ||
                   (start == LOCATION_START_2);

  *x_mm = right ? APP_LOCATION_START_CENTER_MM : -APP_LOCATION_START_CENTER_MM;
  *y_mm = top ? APP_LOCATION_START_CENTER_MM : -APP_LOCATION_START_CENTER_MM;
  /* The drawing shows the speed bumps on the field-facing side: left-side
   * starts leave toward +X and right-side starts leave toward -X. */
  *heading_mdeg = right ? 180000LL : 0LL;
}

void Location_Reset(LocationStart start)
{
  if ((start < LOCATION_START_1) || (start > LOCATION_START_4)) {
    start = LOCATION_START_4;
  }

  const IMUData imu = IMU_GetData();
  float x_mm;
  float y_mm;
  int64_t heading_mdeg;
  location_start_pose(start, &x_mm, &y_mm, &heading_mdeg);

  const uint32_t primask = location_enter_critical();
  location.x_mm = x_mm;
  location.y_mm = y_mm;
  location.path_mm = 0.0f;
  location.heading_unwrapped_mdeg = heading_mdeg;
  location.previous_imu_yaw_mdeg = imu.yaw_mdeg;
  location.start_zone = (uint8_t)start;
  location.imu_sample_valid = imu.ready;
  location.valid = imu.ready;
  location_leave_critical(primask);
}

void Location_Init(LocationStart start)
{
  Location_Reset(start);
}

void Location_Update10ms(void)
{
  EncoderStatus encoder[3];
  const IMUData imu = IMU_GetData();
  Encoder_GetAll(encoder);

  if (!imu.ready) {
    location.valid = false;
    location.imu_sample_valid = false;
    return;
  }

  if (!location.imu_sample_valid) {
    location.previous_imu_yaw_mdeg = imu.yaw_mdeg;
    location.imu_sample_valid = true;
    location.valid = true;
    return;
  }

  int64_t yaw_step_mdeg = imu.yaw_mdeg - location.previous_imu_yaw_mdeg;
  location.previous_imu_yaw_mdeg = imu.yaw_mdeg;
  if ((yaw_step_mdeg > APP_LOCATION_MAX_YAW_STEP_MDEG) ||
      (yaw_step_mdeg < -APP_LOCATION_MAX_YAW_STEP_MDEG)) {
    /* Reject an explicit IMU_ZeroYaw() discontinuity or a corrupt sample. */
    yaw_step_mdeg = 0LL;
  }
  yaw_step_mdeg = (int64_t)((float)yaw_step_mdeg *
                            APP_LOCATION_IMU_YAW_SIGN);

  const int64_t previous_heading_mdeg = location.heading_unwrapped_mdeg;
  location.heading_unwrapped_mdeg += yaw_step_mdeg;

  const int32_t m1_counts = encoder[0].delta_10ms *
                            APP_OMNI_M1_ENCODER_SIGN;
  const int32_t m2_counts = encoder[1].delta_10ms *
                            APP_OMNI_M2_ENCODER_SIGN;
  const int32_t m3_counts = encoder[2].delta_10ms *
                            APP_OMNI_M3_ENCODER_SIGN;
  const float m1_mm = location_counts_to_mm(m1_counts);
  const float m2_mm = location_counts_to_mm(m2_counts);
  const float m3_mm = location_counts_to_mm(m3_counts);

  /* Forward kinematics matched to motor.c: M1=v2, M2=v1, M3=v3.
   * The second equation is physical left, hence the sign is opposite to the
   * mathematical Vy used by Motor_Move(). Common wheel rotation cancels. */
  const float forward_mm = (m3_mm - m1_mm) / LOCATION_SQRT3;
  const float left_mm = (m1_mm + m3_mm - 2.0f * m2_mm) / 3.0f;
  const float middle_heading_rad =
      ((float)previous_heading_mdeg + 0.5f * (float)yaw_step_mdeg) *
      0.001f * LOCATION_DEG_RAD;
  const float cosine = cosf(middle_heading_rad);
  const float sine = sinf(middle_heading_rad);

  location.x_mm += forward_mm * cosine - left_mm * sine;
  location.y_mm += forward_mm * sine + left_mm * cosine;
  location.path_mm += sqrtf(forward_mm * forward_mm + left_mm * left_mm);
  location.valid = true;
}

LocationPose Location_GetPose(void)
{
  LocationPose pose;
  const uint32_t primask = location_enter_critical();
  pose.x_mm = location_round(location.x_mm);
  pose.y_mm = location_round(location.y_mm);
  pose.heading_mdeg = location_wrap_heading(location.heading_unwrapped_mdeg);
  pose.path_mm = (location.path_mm >= 4294967295.0f) ? UINT32_MAX :
                 (uint32_t)(location.path_mm + 0.5f);
  pose.start_zone = location.start_zone;
  pose.valid = location.valid;
  pose.inside_field = (location.x_mm >= -APP_LOCATION_FIELD_HALF_MM) &&
                      (location.x_mm <= APP_LOCATION_FIELD_HALF_MM) &&
                      (location.y_mm >= -APP_LOCATION_FIELD_HALF_MM) &&
                      (location.y_mm <= APP_LOCATION_FIELD_HALF_MM);
  location_leave_critical(primask);
  return pose;
}
