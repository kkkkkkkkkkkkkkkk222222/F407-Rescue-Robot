#include "imu.h"

#include <string.h>

#include "app_config.h"
#if !APP_IMU_USE_UART
#include "encoder.h"
#include "main.h"
#include "motor.h"

#define IMU_SPI_MAX_DATA            12U
#define IMU_SPI_TIMEOUT_MS          2U
#define IMU_BOOT_TIME_MS            20U
#define IMU_RESET_TIMEOUT_MS       100U
#define IMU_SETTLING_TIME_MS        250U
#define IMU_CALIBRATION_DISCARD     32U
#define IMU_CALIBRATION_SAMPLES     2048U
#define IMU_CALIBRATION_TIMEOUT_MS  8000U
#define IMU_RUNTIME_TIMEOUT_MS      250U
#define IMU_IDENTITY_CHECK_MS       250U
#define IMU_CAL_MAX_STDDEV_RAW      25LL
#define IMU_CAL_MAX_MEAN_MDPS       5000LL
#define IMU_STATIONARY_RATE_MDPS    250
#define IMU_STATIONARY_DWELL_US     3000000U
#define IMU_BIAS_TIME_CONSTANT_US   30000000.0f
#define IMU_SAMPLE_GAP_WARN_US      5000U
#define IMU_FIFO_MAX_WORDS          128U
#define IMU_FIFO_GYRO_TAG             1U
#define IMU_FIFO_TIMESTAMP_TAG        4U
#define IMU_FIFO_STREAM_TIMESTAMP 0x46U

#define LSM6DSV16X_WHO_AM_I         0x0FU
#define LSM6DSV16X_IF_CFG           0x03U
#define LSM6DSV16X_CTRL1            0x10U
#define LSM6DSV16X_CTRL2            0x11U
#define LSM6DSV16X_CTRL3            0x12U
#define LSM6DSV16X_CTRL6            0x15U
#define LSM6DSV16X_CTRL7            0x16U
#define LSM6DSV16X_CTRL8            0x17U
#define LSM6DSV16X_STATUS_REG       0x1EU
#define LSM6DSV16X_OUTX_L_G         0x22U
#define LSM6DSV16X_HAODR_CFG        0x62U
#define LSM6DSV16X_TIMESTAMP0       0x40U
#define LSM6DSV16X_INTERNAL_FREQ    0x4FU
#define LSM6DSV16X_FUNCTIONS_ENABLE 0x50U
#define LSM6DSV16X_FIFO_CTRL3        0x09U
#define LSM6DSV16X_FIFO_CTRL4        0x0AU
#define LSM6DSV16X_FIFO_STATUS1      0x1BU
#define LSM6DSV16X_FIFO_DATA_OUT_TAG 0x78U
#define LSM6DSV16X_OUT_TEMP_L       0x20U

#define LSM6DSV16X_ID               0x70U
#define LSM6DSV16X_I2C_I3C_DISABLE  0x01U
#define LSM6DSV16X_SW_RESET         0x01U
#define LSM6DSV16X_BDU_IF_INC       0x44U
#define LSM6DSV16X_HIGH_ACCURACY    0x10U
#define LSM6DSV16X_ODR_1000_HZ      0x09U
#define LSM6DSV16X_HAODR_1000_HZ    0x01U
#define LSM6DSV16X_GY_500_DPS       0x02U
#define LSM6DSV16X_GY_LPF1_ENABLE   0x01U
#define LSM6DSV16X_XL_4_G           0x01U
#define LSM6DSV16X_DATA_READY       0x03U

#define IMU_GYRO_SCALE_NUMERATOR    175LL
#define IMU_GYRO_SCALE_DENOMINATOR  10LL
#define IMU_ACCEL_SCALE_NUMERATOR   122LL
#define IMU_ACCEL_SCALE_DENOMINATOR 1000LL

static IMUData imu_data;
static int64_t gyro_bias_sum[3];
static int64_t yaw_udeg;
static int64_t integration_remainder;
static int64_t previous_corrected_z;
static uint32_t last_sensor_tick;
static int64_t sensor_clock_denominator;
static uint32_t stationary_time_us;
static float bias_adjustment_remainder[3];
static bool sample_time_valid;
static bool runtime_poll_started;
static bool identity_check_started;
static uint32_t last_successful_sample_ms;
static uint32_t last_identity_check_ms;
static volatile bool fifo_restart_pending;
static int16_t fifo_gyro[3];
static uint32_t fifo_tick;
static bool fifo_gyro_valid, fifo_tick_valid;
static uint32_t last_poll_ms;
static bool poll_valid;
static uint32_t recovery_last_attempt_ms;
static uint16_t recovery_samples;

extern SPI_HandleTypeDef hspi3;

static uint32_t imu_enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void imu_leave_critical(uint32_t primask)
{
  if (primask == 0U) {
    __enable_irq();
  }
}

static int16_t read_i16(const uint8_t *bytes)
{
  return (int16_t)(((uint16_t)bytes[1] << 8U) | bytes[0]);
}

static int32_t divide_round(int64_t value, int64_t divisor)
{
  if (value >= 0) {
    return (int32_t)((value + (divisor / 2LL)) / divisor);
  }
  return (int32_t)((value - (divisor / 2LL)) / divisor);
}

static int64_t divide_round64(int64_t value, int64_t divisor)
{
  if (value >= 0) {
    return (value + (divisor / 2LL)) / divisor;
  }
  return (value - (divisor / 2LL)) / divisor;
}

static bool spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
  if ((tx == NULL) || (rx == NULL) || (length == 0U) ||
      (length > (IMU_SPI_MAX_DATA + 1U))) {
    return false;
  }

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)tx, rx, length,
                              IMU_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return status == HAL_OK;
}

static bool spi_write(uint8_t reg, uint8_t value)
{
  const uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};
  uint8_t rx[2] = {0U, 0U};
  if (!spi_transfer(tx, rx, sizeof(tx))) {
    ++imu_data.error_count;
    return false;
  }
  return true;
}

static bool spi_read(uint8_t reg, uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) || (length > IMU_SPI_MAX_DATA)) {
    ++imu_data.error_count;
    return false;
  }

  uint8_t tx[IMU_SPI_MAX_DATA + 1U] = {0U};
  uint8_t rx[IMU_SPI_MAX_DATA + 1U] = {0U};
  tx[0] = (uint8_t)(reg | 0x80U);
  if (!spi_transfer(tx, rx, (uint16_t)(length + 1U))) {
    ++imu_data.error_count;
    return false;
  }
  memcpy(data, &rx[1], length);
  return true;
}

static bool spi_write_checked(uint8_t reg, uint8_t value, uint8_t mask)
{
  for (uint8_t attempt = 0; attempt < 3U; ++attempt) {
    uint8_t readback = 0U;
    if (spi_write(reg, value) && spi_read(reg, &readback, 1U) &&
        (readback & mask) == (value & mask)) { return true; }
    if (attempt < 2U) { HAL_Delay(1U); }
  }
  ++imu_data.error_count;
  return false;
}

static bool read_raw(int16_t gyro[3], int16_t accel[3])
{
  uint8_t raw[12];
  if (!spi_read(LSM6DSV16X_OUTX_L_G, raw, sizeof(raw))) {
    return false;
  }

  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    gyro[axis] = read_i16(&raw[axis * 2U]);
    accel[axis] = read_i16(&raw[6U + (axis * 2U)]);
  }
  return true;
}

static bool wait_for_reset(void)
{
  const uint32_t deadline = HAL_GetTick() + IMU_RESET_TIMEOUT_MS;
  uint8_t ctrl3 = LSM6DSV16X_SW_RESET;
  do {
    if (!spi_read(LSM6DSV16X_CTRL3, &ctrl3, 1U)) {
      return false;
    }
    if ((ctrl3 & LSM6DSV16X_SW_RESET) == 0U) {
      return true;
    }
    HAL_Delay(1U);
  } while ((int32_t)(HAL_GetTick() - deadline) < 0);
  ++imu_data.error_count;
  return false;
}

static bool read_calibration_sample(int16_t gyro[3], int16_t accel[3],
                                    uint32_t deadline)
{
  while ((int32_t)(HAL_GetTick() - deadline) < 0) {
    uint8_t status = 0U;
    if (spi_read(LSM6DSV16X_STATUS_REG, &status, 1U) &&
        (status & LSM6DSV16X_DATA_READY) == LSM6DSV16X_DATA_READY &&
        read_raw(gyro, accel)) {
      return true;
    }
    HAL_Delay(1U);
  }
  return false;
}

static bool calibrate_gyro(void)
{
  const uint32_t deadline = HAL_GetTick() + IMU_CALIBRATION_TIMEOUT_MS;

  /* Discard the first samples after turn-on so filter transients are excluded. */
  for (uint16_t sample = 0U; sample < IMU_CALIBRATION_DISCARD; ++sample) {
    int16_t gyro[3];
    int16_t accel[3];
    if (!read_calibration_sample(gyro, accel, deadline)) {
      ++imu_data.error_count;
      return false;
    }
  }

  while ((int32_t)(HAL_GetTick() - deadline) < 0) {
    int64_t sum[3] = {0LL, 0LL, 0LL};
    int64_t sum_squares[3] = {0LL, 0LL, 0LL};
    uint16_t gravity_good = 0U;
    int64_t gravity_norm_sum = 0LL;

    for (uint16_t sample = 0U; sample < IMU_CALIBRATION_SAMPLES; ++sample) {
      int16_t gyro[3];
      int16_t accel[3];
      if (!read_calibration_sample(gyro, accel, deadline)) {
        ++imu_data.error_count;
        return false;
      }
      for (uint8_t axis = 0U; axis < 3U; ++axis) {
        const int64_t value = gyro[axis];
        sum[axis] += value;
        sum_squares[axis] += value * value;
      }
      int64_t norm_sq = 0LL;
      for (uint8_t axis = 0U; axis < 3U; ++axis) {
        const int64_t mg = divide_round((int64_t)accel[axis] *
            IMU_ACCEL_SCALE_NUMERATOR, IMU_ACCEL_SCALE_DENOMINATOR);
        norm_sq += mg * mg;
      }
      gravity_norm_sum += norm_sq;
      if (norm_sq >= 900LL * 900LL && norm_sq <= 1100LL * 1100LL) { ++gravity_good; }
    }

    /* A single acceleration spike must not discard an otherwise stationary
     * two-second window. Gyro variance still rejects actual rotation/shaking. */
    const int64_t mean_norm = gravity_norm_sum / IMU_CALIBRATION_SAMPLES;
    bool stationary = (uint32_t)gravity_good * 100U >= IMU_CALIBRATION_SAMPLES * 95U &&
                      mean_norm >= 900LL * 900LL && mean_norm <= 1100LL * 1100LL;
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
      const int64_t variance_numerator =
          sum_squares[axis] * IMU_CALIBRATION_SAMPLES -
          sum[axis] * sum[axis];
      const int64_t variance_limit =
          IMU_CAL_MAX_STDDEV_RAW * IMU_CAL_MAX_STDDEV_RAW *
          IMU_CALIBRATION_SAMPLES * IMU_CALIBRATION_SAMPLES;
      const int64_t mean_mdps = divide_round64(
          sum[axis] * IMU_GYRO_SCALE_NUMERATOR,
          IMU_GYRO_SCALE_DENOMINATOR * IMU_CALIBRATION_SAMPLES);
      if ((variance_numerator > variance_limit) ||
          (mean_mdps > IMU_CAL_MAX_MEAN_MDPS) ||
          (mean_mdps < -IMU_CAL_MAX_MEAN_MDPS)) {
        stationary = false;
        break;
      }
    }
    if (stationary) {
      for (uint8_t axis = 0U; axis < 3U; ++axis) {
        gyro_bias_sum[axis] = sum[axis];
      }
      return true;
    }
  }

  ++imu_data.error_count;
  return false;
}

static bool imu_init_once(void)
{
  memset(&imu_data, 0, sizeof(imu_data));
  imu_data.init_result = IMU_INIT_ID_ERROR;
  memset(gyro_bias_sum, 0, sizeof(gyro_bias_sum));
  yaw_udeg = 0LL;
  integration_remainder = 0LL;
  previous_corrected_z = 0LL;
  last_sensor_tick = 0U;
  sensor_clock_denominator = 46080LL * 10000LL;
  stationary_time_us = 0U;
  memset(bias_adjustment_remainder, 0, sizeof(bias_adjustment_remainder));
  sample_time_valid = false;
  runtime_poll_started = false;
  identity_check_started = false;
  last_successful_sample_ms = 0U;
  last_identity_check_ms = 0U;
  fifo_restart_pending = true;
  fifo_gyro_valid = fifo_tick_valid = false;
  poll_valid = false;
  recovery_last_attempt_ms = 0U;
  recovery_samples = 0U;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(IMU_BOOT_TIME_MS);

  for (uint8_t attempt = 0U; attempt < 5U; ++attempt) {
    if (spi_read(LSM6DSV16X_WHO_AM_I, &imu_data.device_id, 1U) &&
        (imu_data.device_id == LSM6DSV16X_ID)) {
      break;
    }
    HAL_Delay(2U);
  }
  if (imu_data.device_id != LSM6DSV16X_ID) {
    ++imu_data.error_count;
    return false;
  }

  imu_data.init_result = IMU_INIT_RESET_ERROR;
  if (!spi_write(LSM6DSV16X_CTRL3, LSM6DSV16X_SW_RESET) ||
      !wait_for_reset()) {
    return false;
  }

  imu_data.init_result = IMU_INIT_CONFIG_ERROR;
  if (!spi_write_checked(LSM6DSV16X_IF_CFG,
                         LSM6DSV16X_I2C_I3C_DISABLE, 0x01U) ||
      !spi_write_checked(LSM6DSV16X_CTRL3,
                         LSM6DSV16X_BDU_IF_INC, 0x44U) ||
      !spi_write_checked(LSM6DSV16X_CTRL1, 0x00U, 0x7FU) ||
      !spi_write_checked(LSM6DSV16X_CTRL2, 0x00U, 0x7FU) ||
      !spi_write_checked(LSM6DSV16X_HAODR_CFG,
                         LSM6DSV16X_HAODR_1000_HZ, 0x03U) ||
      !spi_write_checked(LSM6DSV16X_CTRL6,
                         LSM6DSV16X_GY_500_DPS, 0x0FU) ||
      !spi_write_checked(LSM6DSV16X_CTRL7,
                         LSM6DSV16X_GY_LPF1_ENABLE, 0x01U) ||
      !spi_write_checked(LSM6DSV16X_CTRL8,
                         LSM6DSV16X_XL_4_G, 0x03U) ||
      !spi_write_checked(LSM6DSV16X_CTRL1,
                         LSM6DSV16X_HIGH_ACCURACY |
                         LSM6DSV16X_ODR_1000_HZ, 0x7FU) ||
      !spi_write_checked(LSM6DSV16X_CTRL2,
                         LSM6DSV16X_HIGH_ACCURACY |
                         LSM6DSV16X_ODR_1000_HZ, 0x7FU)) {
    return false;
  }

  HAL_Delay(IMU_SETTLING_TIME_MS);
  imu_data.init_result = IMU_INIT_CALIBRATION_ERROR;
  if (!calibrate_gyro()) {
    return false;
  }

  uint8_t fine = 0U;
  uint8_t functions_enable = 0U;
  if (!spi_read(LSM6DSV16X_INTERNAL_FREQ, &fine, 1U) ||
      !spi_read(LSM6DSV16X_FUNCTIONS_ENABLE, &functions_enable, 1U) ||
      !spi_write_checked(LSM6DSV16X_FUNCTIONS_ENABLE,
                         (uint8_t)(functions_enable | 0x40U), 0x40U) ||
      !spi_write_checked(LSM6DSV16X_FIFO_CTRL4, 0U, 0xFFU) ||
      !spi_write_checked(LSM6DSV16X_FIFO_CTRL3, 0x90U, 0xFFU)) {
    imu_data.init_result = IMU_INIT_CONFIG_ERROR;
    return false;
  }
  imu_data.clock_fine = (int8_t)fine;
  /* AN5763: timestamp seconds/tick = 10000 / (46080*(10000+13*FREQ_FINE)). */
  sensor_clock_denominator = 46080LL * (10000LL + 13LL * imu_data.clock_fine);

  imu_data.ready = true;
  imu_data.init_result = IMU_INIT_OK;
  return true;
}

bool IMU_Init(void)
{
  for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
    if (imu_init_once()) { return true; }
    /* A moving calibration is not a bus glitch; do not accept a false zero. */
    if (imu_data.init_result == IMU_INIT_CALIBRATION_ERROR) { return false; }
    HAL_Delay(20U);
  }
  return false;
}

static void imu_latch_runtime_fault(void)
{
  const uint32_t primask = imu_enter_critical();
  if (imu_data.ready) {
    imu_data.ready = false;
    imu_data.stationary = false;
    ++imu_data.error_count;
  }
  sample_time_valid = false;
  imu_data.recovering = true;
  imu_data.heading_uncertain = true;
  recovery_samples = 0U;
  fifo_restart_pending = true;
  imu_leave_critical(primask);
}

static void imu_check_runtime_timeout(uint32_t now_ms)
{
  if (!runtime_poll_started) {
    runtime_poll_started = true;
    last_successful_sample_ms = now_ms;
    return;
  }
  if ((uint32_t)(now_ms - last_successful_sample_ms) >=
      IMU_RUNTIME_TIMEOUT_MS) {
    imu_latch_runtime_fault();
  }
}

static bool imu_stationary_gate(const int32_t gyro_mdps[3], const int32_t accel_mg[3])
{
  int64_t acceleration_sq = 0LL;
  EncoderStatus encoder[3];
  Encoder_GetAll(encoder);
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    const MotorStatus motor = Motor_GetStatus((uint8_t)(axis + 1U));
    if ((motor.command != 0) || (motor.target_speed_mm_s != 0) ||
        (encoder[axis].delta_10ms != 0) ||
        (gyro_mdps[axis] > IMU_STATIONARY_RATE_MDPS) ||
        (gyro_mdps[axis] < -IMU_STATIONARY_RATE_MDPS)) {
      return false;
    }
    acceleration_sq += (int64_t)accel_mg[axis] * accel_mg[axis];
  }
  return (acceleration_sq >= 950LL * 950LL) &&
         (acceleration_sq <= 1050LL * 1050LL);
}

static void imu_update_bias(const int64_t corrected[3], uint32_t elapsed_us)
{
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    bias_adjustment_remainder[axis] += (float)corrected[axis] *
        ((float)elapsed_us / IMU_BIAS_TIME_CONSTANT_US);
    const int32_t adjustment = (int32_t)bias_adjustment_remainder[axis];
    gyro_bias_sum[axis] += adjustment;
    bias_adjustment_remainder[axis] -= (float)adjustment;
  }
}

static bool imu_integrate_sample(const int16_t gyro[3], const int16_t accel[3],
                                 uint32_t sensor_tick)
{
  /* A task may zero yaw between FIFO reads; commit one sample atomically. */
  const uint32_t primask = imu_enter_critical();
  if (fifo_restart_pending) { imu_leave_critical(primask); return true; }
  int64_t corrected_gyro[3];
  int32_t gyro_mdps[3];
  int32_t accel_mg[3];
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    corrected_gyro[axis] =
        ((int64_t)gyro[axis] * IMU_CALIBRATION_SAMPLES) - gyro_bias_sum[axis];
    if (axis == 2U) {
      corrected_gyro[axis] = divide_round64(
          corrected_gyro[axis] * APP_IMU_GYRO_Z_SCALE_PPM, 1000000LL);
    }
    gyro_mdps[axis] = divide_round(
        corrected_gyro[axis] * IMU_GYRO_SCALE_NUMERATOR,
        IMU_GYRO_SCALE_DENOMINATOR * IMU_CALIBRATION_SAMPLES);
    accel_mg[axis] = divide_round(
        (int64_t)accel[axis] * IMU_ACCEL_SCALE_NUMERATOR,
        IMU_ACCEL_SCALE_DENOMINATOR);
  }

  uint32_t elapsed_us = 0U;
  uint32_t elapsed_ns = 0U;
  bool integration_valid = false;
  if (sample_time_valid) {
    const uint32_t tick_delta = sensor_tick - last_sensor_tick;
    /* Bound before multiplication: a reset/long outage must not overflow. */
    if ((tick_delta > 0U) && (tick_delta <= 10000U)) {
      const int64_t dt_ns = divide_round64(
          (int64_t)tick_delta * 10000000000000LL, sensor_clock_denominator);
      integration_valid = (dt_ns > 0LL) &&
          (dt_ns >= 500000LL) && (dt_ns <= 1500000LL);
      if (integration_valid) {
        elapsed_ns = (uint32_t)dt_ns;
        elapsed_us = (elapsed_ns + 500U) / 1000U;
      }
    }
  }
  if (sample_time_valid && !integration_valid) {
    ++imu_data.integration_gap_count;
    imu_leave_critical(primask);
    return false;
  }
  const bool stationary = integration_valid &&
      imu_stationary_gate(gyro_mdps, accel_mg);

  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    imu_data.gyro_mdps[axis] = gyro_mdps[axis];
    imu_data.accel_mg[axis] = accel_mg[axis];
    imu_data.gyro_bias_mdps[axis] = divide_round(
        gyro_bias_sum[axis] * IMU_GYRO_SCALE_NUMERATOR,
        IMU_GYRO_SCALE_DENOMINATOR * IMU_CALIBRATION_SAMPLES);
  }
  if (sample_time_valid) {
    imu_data.sample_interval_us = elapsed_us;
    if (elapsed_us > imu_data.max_sample_interval_us) {
      imu_data.max_sample_interval_us = elapsed_us;
    }
    if (!integration_valid || (elapsed_us > IMU_SAMPLE_GAP_WARN_US)) {
      ++imu_data.integration_gap_count;
    }
    if (integration_valid) {
      /* Carry fractional microdegrees rather than growing a nanosecond-scale
       * numerator that would overflow after only a few full rotations. */
      const int64_t denominator = 2LL * IMU_GYRO_SCALE_DENOMINATOR *
          IMU_CALIBRATION_SAMPLES * 1000000LL;
      const int64_t numerator = integration_remainder +
          (previous_corrected_z + corrected_gyro[2]) *
          IMU_GYRO_SCALE_NUMERATOR * elapsed_ns;
      yaw_udeg += numerator / denominator;
      integration_remainder = numerator % denominator;
      imu_data.yaw_mdeg = divide_round64(yaw_udeg, 1000LL);
    }
  } else {
    sample_time_valid = true;
  }
  if (stationary) {
    if (stationary_time_us < IMU_STATIONARY_DWELL_US) {
      stationary_time_us += elapsed_us;
    } else {
      /* Use unscaled raw residual for offset estimation, not a yaw gain. */
      int64_t residual[3];
      for (uint8_t axis = 0U; axis < 3U; ++axis) {
        residual[axis] = (int64_t)gyro[axis] * IMU_CALIBRATION_SAMPLES -
                         gyro_bias_sum[axis];
      }
      imu_update_bias(residual, elapsed_us);
      ++imu_data.bias_update_count;
    }
  } else {
    stationary_time_us = 0U;
    memset(bias_adjustment_remainder, 0, sizeof(bias_adjustment_remainder));
  }
  imu_data.stationary = stationary && (stationary_time_us >= IMU_STATIONARY_DWELL_US);
  last_sensor_tick = sensor_tick;
  previous_corrected_z = corrected_gyro[2];
  ++imu_data.sample_count;
  imu_leave_critical(primask);
  return true;
}

static void imu_fifo_fault(void)
{
  ++imu_data.fifo_fault_count;
  imu_latch_runtime_fault();
}

void IMU_Update(uint32_t now_ms)
{
  if (!imu_data.ready && !imu_data.recovering) { return; }
  if (imu_data.recovering && fifo_restart_pending) {
    if (now_ms - recovery_last_attempt_ms < 100U) { return; }
    recovery_last_attempt_ms = now_ms;
    uint8_t id = 0U;
    if (!spi_read(LSM6DSV16X_WHO_AM_I, &id, 1U) || id != LSM6DSV16X_ID) { return; }
    /* No sensor reset or moving zero calibration. Restore the stream only. */
  }
  if (poll_valid && now_ms - last_poll_ms > imu_data.max_poll_interval_ms) {
    imu_data.max_poll_interval_ms = now_ms - last_poll_ms;
  }
  last_poll_ms = now_ms;
  poll_valid = true;

  /* Start only after boot/UI initialization. ZeroYaw also discards pre-zero data.
   * SPI stays in this foreground owner, never in the task calling ZeroYaw. */
  if (fifo_restart_pending) {
    const uint32_t mask = imu_enter_critical();
    fifo_restart_pending = false;
    sample_time_valid = false;
    fifo_gyro_valid = fifo_tick_valid = false;
    stationary_time_us = 0U;
    imu_leave_critical(mask);
    if (!spi_write_checked(LSM6DSV16X_FIFO_CTRL4, 0U, 0xFFU) ||
        !spi_write_checked(LSM6DSV16X_FIFO_CTRL4,
                           IMU_FIFO_STREAM_TIMESTAMP, 0xFFU)) {
      imu_fifo_fault();
    }
    last_successful_sample_ms = now_ms;
    runtime_poll_started = true;
    return;
  }

  if (!identity_check_started || now_ms - last_identity_check_ms >= IMU_IDENTITY_CHECK_MS) {
    uint8_t id = 0U, temp[2];
    identity_check_started = true;
    last_identity_check_ms = now_ms;
    if (!spi_read(LSM6DSV16X_WHO_AM_I, &id, 1U) || id != LSM6DSV16X_ID) {
      /* A failed auxiliary identity read does not consume FIFO data. */
      ++imu_data.error_count;
    }
    if (spi_read(LSM6DSV16X_OUT_TEMP_L, temp, 2U)) {
      imu_data.temperature_mdeg_c = 25000 + divide_round((int64_t)read_i16(temp) * 1000, 256);
    }
  }

  uint8_t status[2], accel_bytes[6];
  if (!spi_read(LSM6DSV16X_FIFO_STATUS1, status, 2U)) {
    imu_check_runtime_timeout(now_ms);
    return;
  }
  const uint16_t words = status[0] | ((uint16_t)(status[1] & 1U) << 8U);
  if (words > imu_data.fifo_peak_words) { imu_data.fifo_peak_words = words; }
  if ((status[1] & 0x48U) != 0U) {
    ++imu_data.fifo_overrun_count;
    imu_fifo_fault();
    return;
  }
  /* Bound catch-up latency; never claim a stale angle is a live control input. */
  if (words > IMU_FIFO_MAX_WORDS) { imu_fifo_fault(); return; }
  if (words == 0U) { imu_check_runtime_timeout(now_ms); return; }
  if (!spi_read(0x28U, accel_bytes, 6U)) { imu_check_runtime_timeout(now_ms); return; }
  int16_t accel[3];
  for (uint8_t i = 0U; i < 3U; ++i) { accel[i] = read_i16(&accel_bytes[i * 2U]); }

  /* Only gyro + timestamp are batched, both at BDR_MAX, uncompressed.
   * Pair one of each in either order and retain an incomplete pair across polls. */
  for (uint16_t i = 0U; i < words; ++i) {
    uint8_t entry[7];
    if (fifo_restart_pending) { return; }
    if (!spi_read(LSM6DSV16X_FIFO_DATA_OUT_TAG, entry, 7U)) {
      imu_fifo_fault(); return;
    }
    const uint8_t tag = entry[0] >> 3U;
    if (tag == IMU_FIFO_GYRO_TAG && !fifo_gyro_valid) {
      for (uint8_t axis = 0U; axis < 3U; ++axis) {
        fifo_gyro[axis] = read_i16(&entry[1U + axis * 2U]);
      }
      fifo_gyro_valid = true;
    } else if (tag == IMU_FIFO_TIMESTAMP_TAG && !fifo_tick_valid) {
      fifo_tick = (uint32_t)entry[1] | ((uint32_t)entry[2] << 8U) |
          ((uint32_t)entry[3] << 16U) | ((uint32_t)entry[4] << 24U);
      fifo_tick_valid = true;
    } else {
      imu_fifo_fault(); return;
    }
    if (fifo_gyro_valid && fifo_tick_valid) {
      if (fifo_restart_pending) { return; }
      if (!imu_integrate_sample(fifo_gyro, accel, fifo_tick)) {
        imu_fifo_fault(); return;
      }
      fifo_gyro_valid = fifo_tick_valid = false;
      if (imu_data.recovering && ++recovery_samples >= 50U) {
        imu_data.recovering = false;
        imu_data.ready = true;
        ++imu_data.recovery_count;
        /* The lost yaw is NOT reconstructed by a healthy FIFO. */
      }
      last_successful_sample_ms = now_ms;
      runtime_poll_started = true;
    }
  }
  imu_check_runtime_timeout(now_ms);
}

void IMU_ConfirmHeadingReference(void)
{
  const uint32_t mask = imu_enter_critical();
  if (imu_data.ready && !imu_data.recovering) { imu_data.heading_uncertain = false; }
  imu_leave_critical(mask);
}

void IMU_ZeroYaw(void)
{
  const uint32_t primask = imu_enter_critical();
  yaw_udeg = 0LL;
  integration_remainder = 0LL;
  previous_corrected_z = 0LL;
  imu_data.yaw_mdeg = 0;
  sample_time_valid = false;
  fifo_restart_pending = true;
  imu_leave_critical(primask);
}

IMUData IMU_GetData(void)
{
  const uint32_t primask = imu_enter_critical();
  const IMUData snapshot = imu_data;
  imu_leave_critical(primask);
  return snapshot;
}
#endif /* !APP_IMU_USE_UART */
