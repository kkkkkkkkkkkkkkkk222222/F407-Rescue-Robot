#ifndef LCD_H
#define LCD_H

#include <stdbool.h>
#include <stdint.h>

#define LCD_BLACK   0x0000U
#define LCD_BLUE    0x001FU
#define LCD_RED     0xF800U
#define LCD_WHITE   0xFFFFU
#define LCD_GREEN   0x07E0U
#define LCD_YELLOW  0xFFE0U
#define LCD_CYAN    0x07FFU

typedef struct {
  uint32_t now_ms;
  uint32_t uart_last_rx_ms;
  uint8_t uart_last_byte;
  bool uart_active;
  bool uart_received;
  bool motor_test_running;
  bool imu_ready;
  int32_t imu_yaw_mdeg;
  int32_t imu_gyro_z_mdps;
} LCDDashboard;

bool LCD_Init(void);
void LCD_FillScreen(uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void LCD_DrawText(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t background);
void LCD_DrawDashboard(const LCDDashboard *dashboard);

#endif
