#include "imu.h"

#include <string.h>

#include "main.h"

#define IMU_SPI_MAX_DATA            12U
#define IMU_SPI_DELAY_CYCLES        20U
#define IMU_BOOT_TIME_MS            10U
#define IMU_RESET_TIMEOUT_MS        10U
#define IMU_CALIBRATION_SAMPLES     128U
#define IMU_CALIBRATION_TIMEOUT_MS  2000U
#define IMU_MAX_INTEGRATION_GAP_MS  100U
#define IMU_RUNTIME_TIMEOUT_MS      250U
#define IMU_IDENTITY_CHECK_MS       250U
#define IMU_CAL_MAX_ABS_MEAN_RAW    300LL
#define IMU_CAL_MAX_RANGE_RAW       200

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

#define LSM6DSV16X_ID               0x70U
#define LSM6DSV16X_I2C_I3C_DISABLE  0x01U
#define LSM6DSV16X_SW_RESET         0x01U
#define LSM6DSV16X_BDU_IF_INC       0x44U
#define LSM6DSV16X_ODR_120_HZ       0x06U
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
static int64_t yaw_numerator;
static uint32_t last_sample_ms;
static bool sample_time_valid;
static bool runtime_poll_started;
static bool identity_check_started;
static uint32_t last_successful_sample_ms;
static uint32_t last_identity_check_ms;

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

static void spi_delay(void)
{
  for (volatile uint8_t cycle = 0U; cycle < IMU_SPI_DELAY_CYCLES; ++cycle) {
    __NOP();
  }
}

static uint8_t spi_transfer(uint8_t output)
{
  uint8_t input = 0U;
  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
    HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin,
                      (output & mask) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    spi_delay();
    HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_SET);
    input <<= 1U;
    if (HAL_GPIO_ReadPin(IMU_MISO_GPIO_Port, IMU_MISO_Pin) == GPIO_PIN_SET) {
      input |= 1U;
    }
    spi_delay();
  }
  return input;
}

static void spi_write(uint8_t reg, uint8_t value)
{
  HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  (void)spi_transfer((uint8_t)(reg & 0x7FU));
  (void)spi_transfer(value);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static bool spi_read(uint8_t reg, uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) || (length > IMU_SPI_MAX_DATA)) {
    ++imu_data.error_count;
    return false;
  }

  HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  (void)spi_transfer((uint8_t)(reg | 0x80U));
  for (uint16_t index = 0U; index < length; ++index) {
    data[index] = spi_transfer(0xFFU);
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return true;
}

static bool spi_write_checked(uint8_t reg, uint8_t value, uint8_t mask)
{
  uint8_t readback = 0U;
  spi_write(reg, value);
  if (!spi_read(reg, &readback, 1U) ||
      ((readback & mask) != (value & mask))) {
    ++imu_data.error_count;
    return false;
  }
  return true;
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

static bool calibrate_gyro(void)
{
  int64_t sum[3] = {0LL, 0LL, 0LL};
  int16_t minimum[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
  int16_t maximum[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
  uint16_t collected = 0U;
  const uint32_t deadline = HAL_GetTick() + IMU_CALIBRATION_TIMEOUT_MS;

  while ((collected < IMU_CALIBRATION_SAMPLES) &&
         ((int32_t)(HAL_GetTick() - deadline) < 0)) {
    uint8_t status = 0U;
    if (!spi_read(LSM6DSV16X_STATUS_REG, &status, 1U)) {
      return false;
    }
    if ((status & LSM6DSV16X_DATA_READY) != LSM6DSV16X_DATA_READY) {
      HAL_Delay(1U);
      continue;
    }

    int16_t gyro[3];
    int16_t accel[3];
    if (!read_raw(gyro, accel)) {
      return false;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
      sum[axis] += gyro[axis];
      if (gyro[axis] < minimum[axis]) {
        minimum[axis] = gyro[axis];
      }
      if (gyro[axis] > maximum[axis]) {
        maximum[axis] = gyro[axis];
      }
    }
    ++collected;
  }

  if (collected != IMU_CALIBRATION_SAMPLES) {
    ++imu_data.error_count;
    return false;
  }
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    const int64_t absolute_sum = (sum[axis] < 0LL) ? -sum[axis] : sum[axis];
    if ((absolute_sum >
         IMU_CAL_MAX_ABS_MEAN_RAW * IMU_CALIBRATION_SAMPLES) ||
        (((int32_t)maximum[axis] - minimum[axis]) >
         IMU_CAL_MAX_RANGE_RAW)) {
      ++imu_data.error_count;
      return false;
    }
    gyro_bias_sum[axis] = sum[axis];
  }
  return true;
}

bool IMU_Init(void)
{
  memset(&imu_data, 0, sizeof(imu_data));
  memset(gyro_bias_sum, 0, sizeof(gyro_bias_sum));
  yaw_numerator = 0LL;
  last_sample_ms = 0U;
  sample_time_valid = false;
  runtime_poll_started = false;
  identity_check_started = false;
  last_successful_sample_ms = 0U;
  last_identity_check_ms = 0U;

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

  spi_write(LSM6DSV16X_CTRL3, LSM6DSV16X_SW_RESET);
  if (!wait_for_reset() ||
      !spi_write_checked(LSM6DSV16X_IF_CFG,
                         LSM6DSV16X_I2C_I3C_DISABLE, 0x01U) ||
      !spi_write_checked(LSM6DSV16X_CTRL3,
                         LSM6DSV16X_BDU_IF_INC, 0x44U) ||
      !spi_write_checked(LSM6DSV16X_CTRL6,
                         LSM6DSV16X_GY_500_DPS, 0x0FU) ||
      !spi_write_checked(LSM6DSV16X_CTRL7,
                         LSM6DSV16X_GY_LPF1_ENABLE, 0x01U) ||
      !spi_write_checked(LSM6DSV16X_CTRL8,
                         LSM6DSV16X_XL_4_G, 0x03U) ||
      !spi_write_checked(LSM6DSV16X_CTRL1,
                         LSM6DSV16X_ODR_120_HZ, 0x0FU) ||
      !spi_write_checked(LSM6DSV16X_CTRL2,
                         LSM6DSV16X_ODR_120_HZ, 0x0FU)) {
    return false;
  }

  HAL_Delay(20U);
  if (!calibrate_gyro()) {
    return false;
  }

  imu_data.ready = true;
  return true;
}

static void imu_latch_runtime_fault(void)
{
  const uint32_t primask = imu_enter_critical();
  if (imu_data.ready) {
    imu_data.ready = false;
    ++imu_data.error_count;
  }
  sample_time_valid = false;
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

void IMU_Update(uint32_t now_ms)
{
  if (!imu_data.ready) {
    return;
  }

  if (!identity_check_started ||
      ((uint32_t)(now_ms - last_identity_check_ms) >=
       IMU_IDENTITY_CHECK_MS)) {
    uint8_t device_id = 0U;
    identity_check_started = true;
    last_identity_check_ms = now_ms;
    if (!spi_read(LSM6DSV16X_WHO_AM_I, &device_id, 1U) ||
        (device_id != LSM6DSV16X_ID)) {
      const uint32_t primask = imu_enter_critical();
      imu_data.device_id = device_id;
      imu_leave_critical(primask);
      imu_latch_runtime_fault();
      return;
    }
  }

  uint8_t status = 0U;
  if (!spi_read(LSM6DSV16X_STATUS_REG, &status, 1U) ||
      ((status & LSM6DSV16X_DATA_READY) != LSM6DSV16X_DATA_READY)) {
    imu_check_runtime_timeout(now_ms);
    return;
  }

  int16_t gyro[3];
  int16_t accel[3];
  if (!read_raw(gyro, accel)) {
    imu_check_runtime_timeout(now_ms);
    return;
  }

  int64_t corrected_gyro[3];
  int32_t gyro_mdps[3];
  int32_t accel_mg[3];
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    corrected_gyro[axis] =
        ((int64_t)gyro[axis] * IMU_CALIBRATION_SAMPLES) - gyro_bias_sum[axis];
    gyro_mdps[axis] = divide_round(
        corrected_gyro[axis] * IMU_GYRO_SCALE_NUMERATOR,
        IMU_GYRO_SCALE_DENOMINATOR * IMU_CALIBRATION_SAMPLES);
    accel_mg[axis] = divide_round(
        (int64_t)accel[axis] * IMU_ACCEL_SCALE_NUMERATOR,
        IMU_ACCEL_SCALE_DENOMINATOR);
  }

  const uint32_t primask = imu_enter_critical();
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    imu_data.gyro_mdps[axis] = gyro_mdps[axis];
    imu_data.accel_mg[axis] = accel_mg[axis];
  }
  if (sample_time_valid) {
    const uint32_t elapsed_ms = now_ms - last_sample_ms;
    if (elapsed_ms <= IMU_MAX_INTEGRATION_GAP_MS) {
      yaw_numerator += corrected_gyro[2] * IMU_GYRO_SCALE_NUMERATOR * elapsed_ms;
      imu_data.yaw_mdeg = divide_round(
          yaw_numerator,
          1000LL * IMU_GYRO_SCALE_DENOMINATOR * IMU_CALIBRATION_SAMPLES);
    }
  } else {
    sample_time_valid = true;
  }
  last_sample_ms = now_ms;
  ++imu_data.sample_count;
  imu_leave_critical(primask);
  last_successful_sample_ms = now_ms;
  runtime_poll_started = true;
}

void IMU_ZeroYaw(void)
{
  const uint32_t primask = imu_enter_critical();
  yaw_numerator = 0LL;
  imu_data.yaw_mdeg = 0;
  sample_time_valid = false;
  imu_leave_critical(primask);
}

IMUData IMU_GetData(void)
{
  const uint32_t primask = imu_enter_critical();
  const IMUData snapshot = imu_data;
  imu_leave_critical(primask);
  return snapshot;
}
