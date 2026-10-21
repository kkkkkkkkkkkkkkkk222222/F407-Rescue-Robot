#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define APP_IMU_USE_UART 0
#include "../Main/Src/imu.c"

SPI_HandleTypeDef hspi3;
static uint8_t entries[256][7], regs[128];
static unsigned head, tail;
static uint32_t ms;
static bool overrun, spi_failure;
static bool calibration_mode, continuous_vibration;
static unsigned calibration_reads;
uint32_t HAL_GetTick(void) { return ms; }
void HAL_Delay(uint32_t delay) { ms += delay; }
void Encoder_GetAll(EncoderStatus out[3]) { memset(out, 0, 3 * sizeof(*out)); }
MotorStatus Motor_GetStatus(uint8_t id) { (void)id; return (MotorStatus){0}; }

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout)
{
  (void)spi; (void)timeout;
  memset(rx, 0, size);
  if (spi_failure) return 1;
  uint8_t reg = tx[0] & 0x7f;
  if (!(tx[0] & 0x80)) {
    regs[reg] = tx[1];
    if (reg == LSM6DSV16X_FIFO_CTRL4 && tx[1] == 0) head = tail = 0;
  } else if (calibration_mode && reg == LSM6DSV16X_STATUS_REG) {
    rx[1] = 3;
  } else if (calibration_mode && reg == LSM6DSV16X_OUTX_L_G) {
    ++ms;
    ++calibration_reads;
    if (!continuous_vibration && calibration_reads != 100U) {
      rx[11] = 0x05; rx[12] = 0x20;
    }
  } else if (reg == LSM6DSV16X_FIFO_STATUS1) {
    unsigned count = tail - head;
    rx[1] = count & 255;
    rx[2] = (count >> 8) | (overrun ? 0x48 : 0);
  } else if (reg == LSM6DSV16X_FIFO_DATA_OUT_TAG) {
    assert(head < tail);
    memcpy(rx + 1, entries[head++ % 256], 7);
  } else if (reg == 0x28) {
    rx[5] = 0x05; rx[6] = 0x20;
  } else if (reg == LSM6DSV16X_WHO_AM_I) {
    rx[1] = LSM6DSV16X_ID;
  } else {
    for (unsigned i = 1; i < size; ++i) rx[i] = regs[reg + i - 1];
  }
  return HAL_OK;
}

static void reset_test(void)
{
  imu_data = (IMUData){.ready=true};
  memset(gyro_bias_sum, 0, sizeof(gyro_bias_sum));
  memset(regs, 0, sizeof(regs));
  yaw_udeg = integration_remainder = previous_corrected_z = 0;
  sensor_clock_denominator = 46080LL * 10000LL;
  sample_time_valid = fifo_gyro_valid = fifo_tick_valid = false;
  identity_check_started = runtime_poll_started = poll_valid = false;
  overrun = spi_failure = false;
  calibration_mode = continuous_vibration = false;
  calibration_reads = 0;
  fifo_restart_pending = true;
  head = tail = ms = stationary_time_us = 0;
  IMU_Update(ms);
  assert(regs[LSM6DSV16X_FIFO_CTRL4] == 0x46);
}

static void push_gyro(int16_t z)
{
  uint8_t *v = entries[tail++ % 256];
  memset(v, 0, 7); v[0] = 1 << 3;
  v[5] = (uint16_t)z & 255; v[6] = (uint16_t)z >> 8;
}
static void push_time(uint32_t t)
{
  uint8_t *v = entries[tail++ % 256];
  memset(v, 0, 7); v[0] = 4 << 3;
  for (unsigned i = 0; i < 4; ++i) v[i+1] = (uint8_t)(t >> (8*i));
}
static int64_t run_turn(bool batch, bool timestamp_first, int sign, uint32_t origin)
{
  reset_test();
  for (unsigned i = 0; i <= 36000; ++i) {
    uint32_t tick = origin + (uint32_t)((uint64_t)i * 46080 / 1000);
    if (timestamp_first) push_time(tick);
    push_gyro((int16_t)(3430 * sign));
    if (!timestamp_first) push_time(tick);
    if (!batch || i % 16 == 15 || i == 36000) {
      ms = i + 1; IMU_Update(ms); assert(imu_data.ready);
    }
  }
  assert(imu_data.sample_count == 36001);
  assert(llabs(imu_data.yaw_mdeg - 2160900LL * sign) <= 1);
  return imu_data.yaw_mdeg;
}
int main(void)
{
  assert(run_turn(false, false, 1, 0) == run_turn(true, false, 1, 0));
  assert(run_turn(false, true, -1, UINT32_MAX-1000) ==
         run_turn(true, true, -1, UINT32_MAX-1000));
  reset_test(); push_gyro(1000); IMU_Update(++ms);
  assert(imu_data.sample_count == 0 && imu_data.ready);
  push_time(46); IMU_Update(++ms); assert(imu_data.sample_count == 1);
  push_gyro(1000); push_time(4600); IMU_Update(++ms);
  assert(!imu_data.ready && imu_data.fifo_fault_count == 1);
  reset_test(); overrun=true; IMU_Update(++ms);
  assert(!imu_data.ready && imu_data.fifo_overrun_count == 1);
  reset_test(); for (unsigned i=0;i<65;++i) { push_gyro(0); push_time(i*46); }
  IMU_Update(++ms); assert(!imu_data.ready);
  reset_test(); push_gyro(1); push_gyro(2); IMU_Update(++ms);
  assert(!imu_data.ready);
  reset_test(); spi_failure=true; IMU_Update(++ms); assert(imu_data.ready);
  IMU_Update(251); assert(!imu_data.ready && imu_data.recovering);
  spi_failure=false; IMU_Update(351);
  for (unsigned i=0;i<60;++i) {
    push_gyro(0); push_time(i*46); IMU_Update(352+i);
  }
  assert(imu_data.ready && !imu_data.recovering && imu_data.heading_uncertain);
  IMU_ConfirmHeadingReference(); assert(!imu_data.heading_uncertain);
  reset_test(); push_gyro(1000); push_time(46); IMU_ZeroYaw(); IMU_Update(++ms);
  assert(head == tail && imu_data.yaw_mdeg == 0 && !sample_time_valid);
  reset_test(); IMU_Update(251); assert(!imu_data.ready);
  reset_test(); calibration_mode=true;
  assert(calibrate_gyro());
  reset_test(); calibration_mode=continuous_vibration=true;
  assert(!calibrate_gyro());
  puts("PASS: batch equivalence, signs, timestamp wrap, partial pair, gap, overrun, backlog, duplicate, SPI fault, zero, timeout");
  return 0;
}
