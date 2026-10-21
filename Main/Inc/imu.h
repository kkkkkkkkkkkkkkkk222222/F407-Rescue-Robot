#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  IMU_INIT_ID_ERROR = 0,
  IMU_INIT_RESET_ERROR,
  IMU_INIT_CONFIG_ERROR,
  IMU_INIT_CALIBRATION_ERROR,
  IMU_INIT_OK,
  IMU_INIT_RX_TIMEOUT
} IMUInitResult;

typedef struct {
  int32_t accel_mg[3];
  int32_t gyro_mdps[3];
  int64_t yaw_mdeg;
  uint32_t sample_count;
  uint32_t error_count;
  uint8_t device_id;
  IMUInitResult init_result;
  bool ready;
  int32_t gyro_bias_mdps[3];
  uint32_t sample_interval_us;
  uint32_t max_sample_interval_us;
  uint32_t integration_gap_count;
  uint32_t bias_update_count;
  int8_t clock_fine;
  bool stationary;
  uint16_t fifo_peak_words;
  uint32_t fifo_overrun_count;
  uint32_t fifo_fault_count;
  uint32_t max_poll_interval_ms;
  int32_t temperature_mdeg_c;
  bool recovering;
  bool heading_uncertain;
  uint32_t recovery_count;
  /* UART backend only; legacy SPI diagnostics above stay source compatible. */
  uint32_t uart_rx_bytes;
  uint32_t uart_checksum_errors;
  uint32_t uart_rx_errors;
  uint32_t uart_rate_frames;
  uint32_t uart_angle_age_ms;
  uint32_t uart_rate_age_ms;
  uint32_t uart_config_errors;
  uint32_t uart_output_period_ms;
  uint8_t uart_config_state;
} IMUData;

bool IMU_Init(void);
void IMU_Update(uint32_t now_ms);
void IMU_ZeroYaw(void);
/* Only after an independent stationary heading reference has been accepted. */
void IMU_ConfirmHeadingReference(void);
IMUData IMU_GetData(void);
/* Dedicated USART2 RXNE/error handler; does not share console HAL callbacks. */
void IMU_UART_IRQHandler(void);

#endif
