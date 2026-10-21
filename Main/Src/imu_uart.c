#include "app_config.h"

#if APP_IMU_USE_UART
#include "imu.h"
#include "main.h"
#include <string.h>

/* Vendor STM32F103 Gyro.c protocol: 5A AA/BB low high sum8, 115200 8N1. */
static UART_HandleTypeDef gyro_uart;
static volatile IMUData data;
static uint8_t packet[5], packet_length;
static bool angle_seen, rate_seen, ever_ready, uart_started;
static int16_t previous_angle_raw;
static int64_t angle_counts, origin_counts;
static uint32_t angle_ms, rate_ms, last_poll_ms, good_samples;
static uint32_t period_sum, period_samples;
static uint32_t config_due_ms;

enum {
  CONFIG_OBSERVE, CONFIG_UNLOCK_WAIT, CONFIG_RATE_WAIT,
  CONFIG_SAVE_WAIT, CONFIG_VERIFY, CONFIG_DONE, CONFIG_FAILED
};

static uint32_t lock(void)
{
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  return mask;
}

static void unlock(uint32_t mask)
{
  __set_PRIMASK(mask);
}

static void lose_stream(void)
{
  if (!data.recovering) {
    ++data.integration_gap_count;
  }
  data.ready = false;
  data.recovering = true;
  /* An outage can hide complete revolutions. A healthy link cannot prove yaw. */
  if (ever_ready && HAL_GetTick() - angle_ms >= APP_IMU_UART_UNWRAP_MAX_GAP_MS) {
    data.heading_uncertain = true;
  }
  good_samples = 0U;
}

static int64_t counts_to_mdeg(int64_t counts)
{
  /* Scale the accumulated count, not rounded individual deltas. */
  return counts * 180000LL * APP_IMU_UART_YAW_SIGN / 32768LL;
}

static void receive_frame(uint8_t type, int16_t raw, uint32_t now)
{
  if (type == 0xAAU) {
    data.gyro_mdps[2] = (int32_t)((int64_t)raw * 2000000LL *
                                          APP_IMU_UART_YAW_SIGN / 32768LL);
    rate_ms = now;
    rate_seen = true;
    ++data.uart_rate_frames;
    return;
  }
  if (type != 0xBBU) { return; }

  if (angle_seen) {
    const uint32_t dt = now - angle_ms;
    data.sample_interval_us = dt * 1000U;
    if (data.sample_interval_us > data.max_sample_interval_us) {
      data.max_sample_interval_us = data.sample_interval_us;
    }
    if (dt >= APP_IMU_UART_STALE_MS) {
      lose_stream();
      period_sum = period_samples = 0U;
    }
    int32_t delta = (int32_t)raw - previous_angle_raw;
    if (delta > 32767) { delta -= 65536; }
    if (delta < -32768) { delta += 65536; }
    angle_counts += delta;
    period_sum += dt;
    if (++period_samples >= 8U) {
      data.uart_output_period_ms = (period_sum + 4U) / 8U;
      period_sum = period_samples = 0U;
    }
  } else {
    /* The first absolute module angle becomes our local zero reference. */
    angle_counts = origin_counts = raw;
    angle_seen = true;
  }
  previous_angle_raw = raw;
  angle_ms = now;
  data.yaw_mdeg = counts_to_mdeg(angle_counts - origin_counts);
  ++data.sample_count;

  if (!rate_seen || now - rate_ms >= APP_IMU_UART_STALE_MS) {
    lose_stream();
    return;
  }
  if (good_samples < APP_IMU_UART_RECOVERY_SAMPLES) { ++good_samples; }
  if (good_samples >= APP_IMU_UART_RECOVERY_SAMPLES) {
    if (data.recovering && ever_ready) { ++data.recovery_count; }
    data.ready = true;
    data.recovering = false;
    data.init_result = IMU_INIT_OK;
    ever_ready = true;
  }
}

static void receive_byte(uint8_t value, uint32_t now)
{
  ++data.uart_rx_bytes;
  packet[packet_length++] = value;
  /* Sliding synchronization also retains a header inside a damaged frame. */
  while (packet_length > 0U) {
    if (packet[0] == 0x5AU &&
        (packet_length < 2U || packet[1] == 0xAAU ||
         packet[1] == 0xBBU || packet[1] == 0xCCU)) {
      if (packet_length < sizeof(packet)) { return; }
      const uint8_t sum = (uint8_t)(packet[0] + packet[1] + packet[2] + packet[3]);
      if (sum == packet[4]) {
        receive_frame(packet[1], (int16_t)((uint16_t)packet[2] |
                                           ((uint16_t)packet[3] << 8U)), now);
        packet_length = 0U;
        return;
      }
      ++data.error_count;
      ++data.uart_checksum_errors;
    }
    --packet_length;
    memmove(packet, packet + 1U, packet_length);
  }
}

void IMU_UART_IRQHandler(void)
{
  /* SR then DR clears ORE/FE/NE/PE. RX stays enabled: no HAL one-shot rearm. */
  const uint32_t status = APP_IMU_UART_INSTANCE->SR;
  if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE |
                 USART_SR_NE | USART_SR_PE)) == 0U) { return; }
  const uint8_t value = (uint8_t)APP_IMU_UART_INSTANCE->DR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    ++data.error_count;
    ++data.uart_rx_errors;
    packet_length = 0U;
    lose_stream();
    return;
  }
  receive_byte(value, HAL_GetTick());
}

static bool send_command(uint8_t reg, uint8_t low, uint8_t high)
{
  uint8_t command[] = {0x55U, 0xAAU, reg, low, high};
  if (HAL_UART_Transmit(&gyro_uart, command, sizeof(command), 2U) == HAL_OK) {
    return true;
  }
  const uint32_t mask = lock();
  ++data.uart_config_errors;
  data.uart_config_state = CONFIG_FAILED;
  unlock(mask);
  return false;
}

static void configure_output_rate(uint32_t now)
{
#if APP_IMU_UART_AUTO_RATE
  const IMUData snapshot = IMU_GetData();
  switch (snapshot.uart_config_state) {
    case CONFIG_OBSERVE:
      if (!snapshot.ready || snapshot.uart_output_period_ms == 0U) { return; }
      if (snapshot.uart_output_period_ms <= 7U) {
        data.uart_config_state = CONFIG_DONE;
      } else if (send_command(0x13U, 0x8EU, 0x5FU)) {
        data.uart_config_state = CONFIG_UNLOCK_WAIT;
        config_due_ms = now + 100U;
      }
      break;
    case CONFIG_UNLOCK_WAIT:
      if ((int32_t)(now - config_due_ms) < 0) { return; }
      if (send_command(0x02U, 0x0BU, 0x00U)) { /* RRATE = 200 Hz */
        data.uart_config_state = CONFIG_RATE_WAIT;
        config_due_ms = now + 100U;
      }
      break;
    case CONFIG_RATE_WAIT:
      if ((int32_t)(now - config_due_ms) < 0) { return; }
      if (send_command(0x00U, 0x00U, 0x00U)) {
        data.uart_config_state = CONFIG_SAVE_WAIT;
        config_due_ms = now + 100U;
      }
      break;
    case CONFIG_SAVE_WAIT: {
      if ((int32_t)(now - config_due_ms) < 0) { return; }
      const uint32_t mask = lock();
      period_sum = period_samples = 0U;
      data.uart_output_period_ms = 0U;
      data.uart_config_state = CONFIG_VERIFY;
      unlock(mask);
      config_due_ms = now + 1000U;
      break;
    }
    case CONFIG_VERIFY:
      if (snapshot.ready && snapshot.uart_output_period_ms > 0U &&
          snapshot.uart_output_period_ms <= 7U) {
        data.uart_config_state = CONFIG_DONE;
      } else if ((int32_t)(now - config_due_ms) >= 0) {
        const uint32_t mask = lock();
        ++data.uart_config_errors;
        data.uart_config_state = CONFIG_FAILED;
        unlock(mask);
      }
      break;
    default: break;
  }
#else
  (void)now;
#endif
}

bool IMU_Init(void)
{
  HAL_NVIC_DisableIRQ(APP_IMU_UART_IRQn);
  memset((void *)&data, 0, sizeof(data));
  memset(&gyro_uart, 0, sizeof(gyro_uart));
  packet_length = 0U;
  angle_seen = rate_seen = ever_ready = uart_started = false;
  angle_counts = origin_counts = 0LL;
  angle_ms = rate_ms = good_samples = period_sum = period_samples = 0U;
  last_poll_ms = HAL_GetTick();
  data.init_result = IMU_INIT_RX_TIMEOUT;
  data.recovering = true;
  gyro_uart.Instance = APP_IMU_UART_INSTANCE;
  gyro_uart.Init.BaudRate = APP_IMU_UART_BAUD;
  gyro_uart.Init.WordLength = UART_WORDLENGTH_8B;
  gyro_uart.Init.StopBits = UART_STOPBITS_1;
  gyro_uart.Init.Parity = UART_PARITY_NONE;
  gyro_uart.Init.Mode = UART_MODE_TX_RX;
  gyro_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  gyro_uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&gyro_uart) != HAL_OK) {
    data.init_result = IMU_INIT_CONFIG_ERROR;
    return false;
  }
  __HAL_UART_CLEAR_OREFLAG(&gyro_uart);
  /* Above TIM6, with bounded parsing only; never transmit or delay in IRQ. */
  HAL_NVIC_SetPriority(APP_IMU_UART_IRQn, 3U, 0U);
  HAL_NVIC_ClearPendingIRQ(APP_IMU_UART_IRQn);
  uart_started = true;
  __HAL_UART_ENABLE_IT(&gyro_uart, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&gyro_uart, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(&gyro_uart, UART_IT_PE);
  HAL_NVIC_EnableIRQ(APP_IMU_UART_IRQn);
  const uint32_t start = HAL_GetTick();
  while (HAL_GetTick() - start < APP_IMU_UART_START_TIMEOUT_MS) {
    IMU_Update(0U);
    const IMUData snapshot = IMU_GetData();
    if (snapshot.ready && (snapshot.uart_config_state == CONFIG_DONE ||
                          snapshot.uart_config_state == CONFIG_FAILED ||
                          !APP_IMU_UART_AUTO_RATE)) { return true; }
    HAL_Delay(1U);
  }
  /* Keep listening after boot timeout: a late powered module can recover. */
  return IMU_GetData().ready;
}

void IMU_Update(uint32_t now_ms)
{
  (void)now_ms;
  if (!uart_started) { return; }
  /* Startup HAL time and runtime Robot time have different zero points. */
  const uint32_t now = HAL_GetTick();
  const uint32_t mask = lock();
  if (now - last_poll_ms > data.max_poll_interval_ms) {
    data.max_poll_interval_ms = now - last_poll_ms;
  }
  last_poll_ms = now;
  if (data.ready && (!angle_seen || !rate_seen ||
      now - angle_ms >= APP_IMU_UART_STALE_MS ||
      now - rate_ms >= APP_IMU_UART_STALE_MS)) { lose_stream(); }
  unlock(mask);
  configure_output_rate(now);
}

void IMU_ZeroYaw(void)
{
  const uint32_t mask = lock();
  origin_counts = angle_counts;
  data.yaw_mdeg = 0LL;
  unlock(mask);
}

void IMU_ConfirmHeadingReference(void)
{
  const uint32_t mask = lock();
  if (data.ready && !data.recovering) { data.heading_uncertain = false; }
  unlock(mask);
}

IMUData IMU_GetData(void)
{
  const uint32_t mask = lock();
  IMUData snapshot = data;
  snapshot.uart_angle_age_ms = angle_seen ? HAL_GetTick() - angle_ms : UINT32_MAX;
  snapshot.uart_rate_age_ms = rate_seen ? HAL_GetTick() - rate_ms : UINT32_MAX;
  unlock(mask);
  return snapshot;
}
#endif /* APP_IMU_USE_UART */
