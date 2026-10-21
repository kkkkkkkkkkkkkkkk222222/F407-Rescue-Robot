#include "main.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "Task.h"
#include "Robot.h"
#include "DebugConsole.h"
#include <stdio.h>
#include <string.h>

static void send(const char *line)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)strlen(line), 30U);
}

void BoardF405_ReportVersion(void)
{
  send("NORMALDEBUG BOARD=F405_ROBO MCU=STM32F405RG MODE=5 LCD=0\r\n"
       "RDK=UART4 PC10/PC11 GYRO=UART5 PC12/PD2 CONSOLE=USART1 PA9/PB7 115200 8N1\r\n"
       "SEND HELP; AUTOMATIC TASK REQUIRES UPPER CONFIG\r\n");
}

void BoardF405_Report(void)
{
  const IMUData imu = IMU_GetData();
  const TaskStatus task = Task_GetStatus();
  EncoderStatus enc[3];
  Encoder_GetAll(enc);
  char text[256];
  (void)snprintf(text, sizeof(text),
      "NORMALDEBUG MS=%lu TASK=%u FAULT=%u PAUSED=%u CAMERA=%u AUDIT=%u/%u\r\n",
      (unsigned long)Robot_GetMilliseconds(), task.state, task.fault,
      DebugConsole_IsActive(), task.camera_angle, task.audit_ready, task.audit_valid);
  send(text);
  (void)snprintf(text, sizeof(text),
      "IMU UART=5 READY=%u SAMPLE=%lu YAW_MOD_MDEG=%ld Z_MDPS=%ld "
      "ANGLE_AGE_MS=%lu RATE_AGE_MS=%lu PERIOD_MS=%lu CFG=%u "
      "CRC_ERRORS=%lu RX_ERRORS=%lu UNCERTAIN=%u\r\n",
      imu.ready, (unsigned long)imu.sample_count, (long)(imu.yaw_mdeg % 360000LL),
      (long)imu.gyro_mdps[2], (unsigned long)imu.uart_angle_age_ms,
      (unsigned long)imu.uart_rate_age_ms, (unsigned long)imu.uart_output_period_ms,
      imu.uart_config_state, (unsigned long)imu.uart_checksum_errors,
      (unsigned long)imu.uart_rx_errors, imu.heading_uncertain);
  send(text);
  (void)snprintf(text, sizeof(text),
      "ENC RAW32=%ld,%ld,%ld DELTA10=%ld,%ld,%ld M3_INVALID=%lu\r\n",
      (long)(int32_t)enc[0].position, (long)(int32_t)enc[1].position,
      (long)(int32_t)enc[2].position, (long)enc[0].delta_10ms,
      (long)enc[1].delta_10ms, (long)enc[2].delta_10ms,
      (unsigned long)BoardF405_EncoderInvalidTransitions());
  send(text);
  for (uint8_t id = 1; id <= 3; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    (void)snprintf(text, sizeof(text), "MOTOR %u TARGET_MM_S=%d ACTUAL_MM_S=%d PWM=%d\r\n",
                   id, motor.target_speed_mm_s, motor.measured_speed_mm_s, motor.command);
    send(text);
  }
}
