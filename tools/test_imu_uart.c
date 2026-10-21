#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "../Main/Src/imu_uart.c"

USART_TypeDef test_usart2;
static uint32_t ms, stream_start, stream_period;
static bool simulate_stream, tx_failure;
static uint8_t commands[16][5];
static unsigned command_count;

uint32_t HAL_GetTick(void) { return ms; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart)
{
  assert(uart->Instance == USART2 && uart->Init.BaudRate == 115200U);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes,
                                  uint16_t size, uint32_t timeout)
{
  (void)uart;
  assert(size == 5U && timeout <= 2U && command_count < 16U);
  assert(bytes[0] == 0x55U && bytes[1] == 0xAAU);
  if (tx_failure) { return 1; }
  memcpy(commands[command_count++], bytes, size);
  if (bytes[2] == 0x02U) {
    assert(bytes[3] == 0x0BU && bytes[4] == 0U);
    stream_period = 5U;
  }
  return HAL_OK;
}

static void byte(uint8_t value)
{
  test_usart2.SR = USART_SR_RXNE;
  test_usart2.DR = value;
  IMU_UART_IRQHandler();
}

static void frame(uint8_t type, int16_t raw)
{
  uint8_t bytes[] = {0x5AU, type, (uint8_t)raw, (uint8_t)((uint16_t)raw >> 8), 0U};
  bytes[4] = (uint8_t)(bytes[0] + bytes[1] + bytes[2] + bytes[3]);
  for (unsigned i = 0; i < 5; ++i) { byte(bytes[i]); }
}

static void sample(int16_t raw, uint32_t dt)
{
  ms += dt;
  frame(0xAAU, 0);
  frame(0xBBU, raw);
}

void HAL_Delay(uint32_t delay)
{
  while (delay-- > 0U) {
    ++ms;
    if (simulate_stream && ms >= stream_start && ms % stream_period == 0U) {
      sample(0, 0U);
    }
  }
}

static void boot(uint32_t period)
{
  ms = 0U;
  simulate_stream = true;
  stream_start = 500U;
  stream_period = period;
  tx_failure = false;
  command_count = 0U;
  assert(IMU_Init());
  simulate_stream = false;
  assert(data.ready && data.uart_config_state == CONFIG_DONE);
}

int main(void)
{
  boot(50U);
  assert(command_count == 3U && commands[0][2] == 0x13U && commands[2][2] == 0U);
  assert(data.uart_output_period_ms == 5U);
  boot(5U);
  assert(command_count == 0U); /* No flash save on a configured module. */

  for (int direction = -1; direction <= 1; direction += 2) {
    boot(5U);
    for (int i = 1; i <= 768; ++i) { sample((int16_t)(i * 256 * direction), 5U); }
    assert(data.yaw_mdeg == 1080000LL * direction * APP_IMU_UART_YAW_SIGN);
    IMU_ZeroYaw();
    sample((int16_t)(769 * 256 * direction), 5U);
    assert(llabs(data.yaw_mdeg - 1406LL * direction * APP_IMU_UART_YAW_SIGN) <= 1);
  }
  boot(5U);
  frame(0xAAU, 16384);
  assert(data.gyro_mdps[2] == 1000000L * APP_IMU_UART_YAW_SIGN);
  frame(0xAAU, -16384);
  assert(data.gyro_mdps[2] == -1000000L * APP_IMU_UART_YAW_SIGN);
  const uint32_t count = data.sample_count;
  frame(0xAAU, 0); frame(0xCCU, 0);
  assert(data.sample_count == count);
  /* Noise, truncated packet, bad sum, and an embedded new frame header. */
  byte(0x99U); byte(0x5AU); byte(0xAAU); byte(0x5AU); byte(0xBBU);
  byte(0U); byte(0U); byte(0x15U);
  assert(data.sample_count == count + 1U && data.uart_checksum_errors > 0U);

  boot(5U);
  sample(100, 200U); /* Short interruption: absolute angle preserves heading. */
  assert(!data.ready && !data.heading_uncertain);
  sample(101, 5U); sample(102, 5U);
  assert(data.ready && !data.heading_uncertain && data.recovery_count == 1U);
  sample(103, 500U);
  sample(104, 5U); sample(105, 5U);
  assert(data.ready && data.heading_uncertain);
  IMU_ConfirmHeadingReference();
  assert(!data.heading_uncertain);

  boot(5U);
  test_usart2.SR = USART_SR_RXNE | USART_SR_ORE;
  IMU_UART_IRQHandler();
  assert(data.uart_rx_errors == 1U && !data.ready && !data.heading_uncertain);
  sample(1, 5U); sample(2, 5U); sample(3, 5U);
  assert(data.ready);
  ms += 151U;
  frame(0xAAU, 0);
  IMU_Update(1U); /* Health uses HAL time, not the application clock argument. */
  assert(!data.ready);

  boot(5U);
  for (unsigned i = 0; i < 40; ++i) { ms += 5U; frame(0xBBU, 0); }
  assert(!data.ready); /* Angle-only traffic cannot hide a stale rate. */
  boot(5U);
  ms = UINT32_MAX - 10U;
  sample(0, 0); sample(0, 5); sample(0, 5);
  IMU_ConfirmHeadingReference();
  sample(256, 5U);
  assert(data.ready && llabs(data.yaw_mdeg - 1406LL * APP_IMU_UART_YAW_SIGN) <= 1);

  simulate_stream = false;
  ms = 0U;
  assert(!IMU_Init());
  for (unsigned i = 0; i < 10; ++i) { sample(0, 5U); }
  IMU_Update(0U);
  assert(data.ready && !data.heading_uncertain); /* Late supply recovers. */
  boot(5U);
  tx_failure = true;
  data.uart_config_state = CONFIG_OBSERVE;
  data.uart_output_period_ms = 50U;
  IMU_Update(0U);
  assert(data.uart_config_state == CONFIG_FAILED && data.ready);
  puts("PASS UART: protocol/resync, both signs, three turns, zero, AA freshness, "
       "errors, short/long outage recovery, tick wrap, boot recovery, rate config");
  return 0;
}
