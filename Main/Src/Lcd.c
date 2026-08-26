#include "Lcd.h"

#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "encoder.h"
#include "main.h"
#include "motor.h"
#include "Task.h"
#include "vision.h"

extern SPI_HandleTypeDef hspi2;

#define ST7735_SWRESET  0x01U
#define ST7735_SLPOUT   0x11U
#define ST7735_NORON    0x13U
#define ST7735_INVOFF   0x20U
#define ST7735_DISPON   0x29U
#define ST7735_CASET    0x2AU
#define ST7735_RASET    0x2BU
#define ST7735_RAMWR    0x2CU
#define ST7735_MADCTL   0x36U
#define ST7735_COLMOD   0x3AU
#define ST7735_FRMCTR1  0xB1U
#define ST7735_FRMCTR2  0xB2U
#define ST7735_FRMCTR3  0xB3U
#define ST7735_INVCTR   0xB4U
#define ST7735_PWCTR1   0xC0U
#define ST7735_PWCTR2   0xC1U
#define ST7735_PWCTR3   0xC2U
#define ST7735_PWCTR4   0xC3U
#define ST7735_PWCTR5   0xC4U
#define ST7735_VMCTR1   0xC5U
#define ST7735_GMCTRP1  0xE0U
#define ST7735_GMCTRN1  0xE1U

static const uint8_t font_5x7[64][5] = {
  ['-' - ' '] = {0x08, 0x08, 0x08, 0x08, 0x08},
  ['/' - ' '] = {0x20, 0x10, 0x08, 0x04, 0x02},
  [':' - ' '] = {0x00, 0x36, 0x36, 0x00, 0x00},
  ['0' - ' '] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
  ['1' - ' '] = {0x00, 0x42, 0x7F, 0x40, 0x00},
  ['2' - ' '] = {0x42, 0x61, 0x51, 0x49, 0x46},
  ['3' - ' '] = {0x21, 0x41, 0x45, 0x4B, 0x31},
  ['4' - ' '] = {0x18, 0x14, 0x12, 0x7F, 0x10},
  ['5' - ' '] = {0x27, 0x45, 0x45, 0x45, 0x39},
  ['6' - ' '] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
  ['7' - ' '] = {0x01, 0x71, 0x09, 0x05, 0x03},
  ['8' - ' '] = {0x36, 0x49, 0x49, 0x49, 0x36},
  ['9' - ' '] = {0x06, 0x49, 0x49, 0x29, 0x1E},
  ['A' - ' '] = {0x7E, 0x11, 0x11, 0x11, 0x7E},
  ['B' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x36},
  ['C' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x22},
  ['D' - ' '] = {0x7F, 0x41, 0x41, 0x22, 0x1C},
  ['E' - ' '] = {0x7F, 0x49, 0x49, 0x49, 0x41},
  ['F' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x01},
  ['G' - ' '] = {0x3E, 0x41, 0x49, 0x49, 0x7A},
  ['H' - ' '] = {0x7F, 0x08, 0x08, 0x08, 0x7F},
  ['I' - ' '] = {0x00, 0x41, 0x7F, 0x41, 0x00},
  ['J' - ' '] = {0x20, 0x40, 0x41, 0x3F, 0x01},
  ['K' - ' '] = {0x7F, 0x08, 0x14, 0x22, 0x41},
  ['L' - ' '] = {0x7F, 0x40, 0x40, 0x40, 0x40},
  ['M' - ' '] = {0x7F, 0x02, 0x0C, 0x02, 0x7F},
  ['N' - ' '] = {0x7F, 0x04, 0x08, 0x10, 0x7F},
  ['O' - ' '] = {0x3E, 0x41, 0x41, 0x41, 0x3E},
  ['P' - ' '] = {0x7F, 0x09, 0x09, 0x09, 0x06},
  ['Q' - ' '] = {0x3E, 0x41, 0x51, 0x21, 0x5E},
  ['R' - ' '] = {0x7F, 0x09, 0x19, 0x29, 0x46},
  ['S' - ' '] = {0x46, 0x49, 0x49, 0x49, 0x31},
  ['T' - ' '] = {0x01, 0x01, 0x7F, 0x01, 0x01},
  ['U' - ' '] = {0x3F, 0x40, 0x40, 0x40, 0x3F},
  ['V' - ' '] = {0x1F, 0x20, 0x40, 0x20, 0x1F},
  ['W' - ' '] = {0x3F, 0x40, 0x38, 0x40, 0x3F},
  ['X' - ' '] = {0x63, 0x14, 0x08, 0x14, 0x63},
  ['Y' - ' '] = {0x07, 0x08, 0x70, 0x08, 0x07},
  ['Z' - ' '] = {0x61, 0x51, 0x49, 0x45, 0x43},
  ['_' - ' '] = {0x40, 0x40, 0x40, 0x40, 0x40},
};

#if APP_ENABLE_TASK
static uint8_t task_layout = 0xFFU;
#endif

static void select_lcd(void)
{
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
}

static void deselect_lcd(void)
{
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
}

static bool transmit(const uint8_t *data, uint16_t size)
{
  return HAL_SPI_Transmit(&hspi2, (uint8_t *)data, size, 100U) == HAL_OK;
}

static bool write_command(uint8_t command, const uint8_t *data, uint8_t size)
{
  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
  select_lcd();
  bool ok = transmit(&command, 1U);
  if ((size > 0U) && (data != NULL)) {
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    ok = transmit(data, size) && ok;
  }
  deselect_lcd();
  return ok;
}

static void set_address_window(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  const uint32_t x0 = (uint32_t)x + APP_LCD_X_OFFSET;
  const uint32_t y0 = (uint32_t)y + APP_LCD_Y_OFFSET;
  const uint32_t x1 = x0 + width - 1U;
  const uint32_t y1 = y0 + height - 1U;
  const uint8_t columns[4] = {(uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1};
  const uint8_t rows[4] = {(uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1};
  (void)write_command(ST7735_CASET, columns, sizeof(columns));
  (void)write_command(ST7735_RASET, rows, sizeof(rows));
  (void)write_command(ST7735_RAMWR, NULL, 0U);
}

static void set_backlight(bool enabled)
{
  HAL_GPIO_WritePin(TFT_BLK_GPIO_Port, TFT_BLK_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool LCD_Init(void)
{
  static const uint8_t frame_rate[] = {0x01, 0x2C, 0x2D};
  static const uint8_t frame_rate3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
  static const uint8_t gamma_pos[] = {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                                      0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
  static const uint8_t gamma_neg[] = {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                      0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
  bool ok = true;

  set_backlight(false);
  deselect_lcd();
  HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(20U);
  HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(120U);

  ok = write_command(ST7735_SWRESET, NULL, 0U) && ok;
  HAL_Delay(150U);
  ok = write_command(ST7735_SLPOUT, NULL, 0U) && ok;
  HAL_Delay(120U);
  ok = write_command(ST7735_FRMCTR1, frame_rate, sizeof(frame_rate)) && ok;
  ok = write_command(ST7735_FRMCTR2, frame_rate, sizeof(frame_rate)) && ok;
  ok = write_command(ST7735_FRMCTR3, frame_rate3, sizeof(frame_rate3)) && ok;
  const uint8_t inversion = 0x07U;
  ok = write_command(ST7735_INVCTR, &inversion, 1U) && ok;
  const uint8_t power1[] = {0xA2, 0x02, 0x84};
  const uint8_t power2 = 0xC5U;
  const uint8_t power3[] = {0x0A, 0x00};
  const uint8_t power4[] = {0x8A, 0x2A};
  const uint8_t power5[] = {0x8A, 0xEE};
  const uint8_t vcom = 0x0EU;
  ok = write_command(ST7735_PWCTR1, power1, sizeof(power1)) && ok;
  ok = write_command(ST7735_PWCTR2, &power2, 1U) && ok;
  ok = write_command(ST7735_PWCTR3, power3, sizeof(power3)) && ok;
  ok = write_command(ST7735_PWCTR4, power4, sizeof(power4)) && ok;
  ok = write_command(ST7735_PWCTR5, power5, sizeof(power5)) && ok;
  ok = write_command(ST7735_VMCTR1, &vcom, 1U) && ok;
  ok = write_command(ST7735_INVOFF, NULL, 0U) && ok;
  const uint8_t memory_access = 0xC8U;
  const uint8_t color_mode = 0x05U;
  ok = write_command(ST7735_MADCTL, &memory_access, 1U) && ok;
  ok = write_command(ST7735_COLMOD, &color_mode, 1U) && ok;
  ok = write_command(ST7735_GMCTRP1, gamma_pos, sizeof(gamma_pos)) && ok;
  ok = write_command(ST7735_GMCTRN1, gamma_neg, sizeof(gamma_neg)) && ok;
  ok = write_command(ST7735_NORON, NULL, 0U) && ok;
  HAL_Delay(10U);
  ok = write_command(ST7735_DISPON, NULL, 0U) && ok;
  HAL_Delay(100U);
  LCD_FillScreen(LCD_BLACK);
  set_backlight(true);
  return ok;
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  if ((x >= APP_LCD_WIDTH) || (y >= APP_LCD_HEIGHT) || (width == 0U) || (height == 0U)) {
    return;
  }
  if (((uint32_t)x + width) > APP_LCD_WIDTH) {
    width = APP_LCD_WIDTH - x;
  }
  if (((uint32_t)y + height) > APP_LCD_HEIGHT) {
    height = APP_LCD_HEIGHT - y;
  }

  uint8_t pixels[128];
  for (uint32_t i = 0; i < sizeof(pixels); i += 2U) {
    pixels[i] = (uint8_t)(color >> 8);
    pixels[i + 1U] = (uint8_t)color;
  }
  uint32_t remaining = (uint32_t)width * height;
  set_address_window(x, y, width, height);
  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
  select_lcd();
  while (remaining > 0U) {
    const uint16_t count = (remaining > 64U) ? 64U : (uint16_t)remaining;
    if (!transmit(pixels, (uint16_t)(count * 2U))) {
      break;
    }
    remaining -= count;
  }
  deselect_lcd();
}

void LCD_FillScreen(uint16_t color)
{
  LCD_FillRect(0U, 0U, APP_LCD_WIDTH, APP_LCD_HEIGHT, color);
}

static void draw_character(uint16_t x, uint16_t y, char character, uint16_t color, uint16_t background)
{
  if ((character >= 'a') && (character <= 'z')) {
    character = (char)(character - ('a' - 'A'));
  }
  if ((character < ' ') || (character > '_')) {
    character = ' ';
  }
  const uint8_t *glyph = font_5x7[(uint8_t)character - (uint8_t)' '];
  uint8_t pixels[6U * 8U * 2U];
  uint32_t index = 0U;
  for (uint16_t row = 0; row < 8U; ++row) {
    for (uint16_t column = 0; column < 6U; ++column) {
      const uint8_t bits = (column < 5U) ? glyph[column] : 0U;
      const uint16_t pixel = ((bits & (1U << row)) != 0U) ? color : background;
      pixels[index++] = (uint8_t)(pixel >> 8);
      pixels[index++] = (uint8_t)pixel;
    }
  }
  set_address_window(x, y, 6U, 8U);
  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
  select_lcd();
  (void)transmit(pixels, sizeof(pixels));
  deselect_lcd();
}

void LCD_DrawText(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t background)
{
  if ((text == NULL) || (((uint32_t)y + 8U) > APP_LCD_HEIGHT)) {
    return;
  }
  while ((*text != '\0') && (((uint32_t)x + 6U) <= APP_LCD_WIDTH)) {
    draw_character(x, y, *text, color, background);
    x += 6U;
    ++text;
  }
}

static void dashboard_write(uint16_t x, uint16_t y, uint16_t width,
                            const char *text)
{
  LCD_FillRect(x, y, width, 8U, LCD_BLACK);
  LCD_DrawText(x, y, text, LCD_WHITE, LCD_BLACK);
}

#if APP_ENABLE_TASK || (!APP_ENABLE_MOTION_TEST && !APP_ENABLE_LOCATION_DEMO)
static bool dashboard_motor_fault(void)
{
  for (uint8_t id = 1U; id <= 3U; ++id) {
    const MotorStatus motor = Motor_GetStatus(id);
    if (motor.direction_fault || motor.stall_fault) {
      return true;
    }
  }
  return false;
}

static const char *dashboard_uart_text(const LCDDashboard *dashboard)
{
  if (!dashboard->uart_active) {
    return "DMA ERR";
  }
  if (!dashboard->uart_received) {
    return "WAIT";
  }
  if ((uint32_t)(dashboard->now_ms - dashboard->uart_last_rx_ms) <=
      APP_VISION_TIMEOUT_MS) {
    return "RX OK";
  }
  return "TIMEOUT";
}
#endif

#if APP_ENABLE_LOCATION_DEMO
static void dashboard_write(uint16_t x, uint16_t y, uint16_t width,
                            const char *text);

#define LOCATION_MAP_X      8
#define LOCATION_MAP_Y      40
#define LOCATION_MAP_SIZE   112
#define LOCATION_TRAIL_SIZE 48U

static int16_t location_last_x;
static int16_t location_last_y;
static bool location_marker_drawn;
static int16_t location_trail_x[LOCATION_TRAIL_SIZE];
static int16_t location_trail_y[LOCATION_TRAIL_SIZE];
static uint8_t location_trail_count;
static uint8_t location_trail_next;

static void location_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                               uint16_t color)
{
  int16_t dx = (int16_t)abs(x1 - x0);
  const int16_t sx = (x0 < x1) ? 1 : -1;
  const int16_t dy = (int16_t)-abs(y1 - y0);
  const int16_t sy = (y0 < y1) ? 1 : -1;
  int16_t error = (int16_t)(dx + dy);

  for (;;) {
    if ((x0 >= 0) && (y0 >= 0)) {
      LCD_FillRect((uint16_t)x0, (uint16_t)y0, 1U, 1U, color);
    }
    if ((x0 == x1) && (y0 == y1)) {
      break;
    }
    const int16_t twice_error = (int16_t)(2 * error);
    if (twice_error >= dy) {
      error = (int16_t)(error + dy);
      x0 = (int16_t)(x0 + sx);
    }
    if (twice_error <= dx) {
      error = (int16_t)(error + dx);
      y0 = (int16_t)(y0 + sy);
    }
  }
}

static int16_t location_map_x(int32_t x_mm)
{
  if (x_mm < -1500) {
    x_mm = -1500;
  } else if (x_mm > 1500) {
    x_mm = 1500;
  }
  return (int16_t)(LOCATION_MAP_X +
      ((x_mm + 1500) * (LOCATION_MAP_SIZE - 1) + 1500) / 3000);
}

static int16_t location_map_y(int32_t y_mm)
{
  if (y_mm < -1500) {
    y_mm = -1500;
  } else if (y_mm > 1500) {
    y_mm = 1500;
  }
  return (int16_t)(LOCATION_MAP_Y +
      ((1500 - y_mm) * (LOCATION_MAP_SIZE - 1) + 1500) / 3000);
}

static void location_draw_vertical_bumps(uint16_t x, uint16_t y)
{
  for (uint16_t i = 0U; i < 3U; ++i) {
    LCD_FillRect((uint16_t)(x + i * 3U), y, 1U, 9U, LCD_GRAY);
  }
}

static void location_draw_horizontal_bumps(uint16_t x, uint16_t y)
{
  for (uint16_t i = 0U; i < 3U; ++i) {
    LCD_FillRect(x, (uint16_t)(y + i * 3U), 9U, 1U, LCD_GRAY);
  }
}

static void location_draw_static_map(void)
{
  const uint16_t right = LOCATION_MAP_X + LOCATION_MAP_SIZE - 1U;
  const uint16_t bottom = LOCATION_MAP_Y + LOCATION_MAP_SIZE - 1U;

  LCD_FillRect(LOCATION_MAP_X, LOCATION_MAP_Y, LOCATION_MAP_SIZE, 1U, LCD_WHITE);
  LCD_FillRect(LOCATION_MAP_X, bottom, LOCATION_MAP_SIZE, 1U, LCD_WHITE);
  LCD_FillRect(LOCATION_MAP_X, LOCATION_MAP_Y, 1U, LOCATION_MAP_SIZE, LCD_WHITE);
  LCD_FillRect(right, LOCATION_MAP_Y, 1U, LOCATION_MAP_SIZE, LCD_WHITE);
  LCD_FillRect(LOCATION_MAP_X + 55U, LOCATION_MAP_Y + 1U,
               1U, LOCATION_MAP_SIZE - 2U, LCD_GRAY);
  LCD_FillRect(LOCATION_MAP_X + 1U, LOCATION_MAP_Y + 55U,
               LOCATION_MAP_SIZE - 2U, 1U, LCD_GRAY);

  /* Four 300 mm start zones. */
  LCD_FillRect(LOCATION_MAP_X + 1U, LOCATION_MAP_Y + 1U, 11U, 11U, LCD_MAGENTA);
  LCD_FillRect(right - 11U, LOCATION_MAP_Y + 1U, 11U, 11U, LCD_MAGENTA);
  LCD_FillRect(LOCATION_MAP_X + 1U, bottom - 11U, 11U, 11U, LCD_MAGENTA);
  LCD_FillRect(right - 11U, bottom - 11U, 11U, 11U, LCD_MAGENTA);

  /* 660 x 360 mm outer safe-zone frame and 600 x 300 mm usable area. */
  LCD_FillRect(LOCATION_MAP_X + 44U, LOCATION_MAP_Y + 1U,
               24U, 13U, LCD_MAGENTA);
  LCD_FillRect(LOCATION_MAP_X + 45U, LOCATION_MAP_Y + 2U,
               22U, 11U, LCD_RED);
  LCD_FillRect(LOCATION_MAP_X + 55U, LOCATION_MAP_Y + 2U,
               1U, 11U, LCD_BLACK);
  LCD_FillRect(LOCATION_MAP_X + 44U, bottom - 13U,
               24U, 13U, LCD_MAGENTA);
  LCD_FillRect(LOCATION_MAP_X + 45U, bottom - 12U,
               22U, 11U, LCD_CYAN);
  LCD_FillRect(LOCATION_MAP_X + 55U, bottom - 12U,
               1U, 11U, LCD_BLACK);

  /* The drawing has three bumps on both field-facing sides of every start. */
  location_draw_vertical_bumps(LOCATION_MAP_X + 14U, LOCATION_MAP_Y + 1U);
  location_draw_horizontal_bumps(LOCATION_MAP_X + 1U, LOCATION_MAP_Y + 14U);
  location_draw_vertical_bumps(right - 20U, LOCATION_MAP_Y + 1U);
  location_draw_horizontal_bumps(right - 9U, LOCATION_MAP_Y + 14U);
  location_draw_vertical_bumps(LOCATION_MAP_X + 14U, bottom - 9U);
  location_draw_horizontal_bumps(LOCATION_MAP_X + 1U, bottom - 20U);
  location_draw_vertical_bumps(right - 20U, bottom - 9U);
  location_draw_horizontal_bumps(right - 9U, bottom - 20U);
}

static void location_add_trail_point(int16_t x, int16_t y)
{
  if ((location_trail_count != 0U) &&
      (location_last_x == x) && (location_last_y == y)) {
    return;
  }
  location_trail_x[location_trail_next] = x;
  location_trail_y[location_trail_next] = y;
  location_trail_next = (uint8_t)((location_trail_next + 1U) %
                                  LOCATION_TRAIL_SIZE);
  if (location_trail_count < LOCATION_TRAIL_SIZE) {
    ++location_trail_count;
  }
}

static void location_draw_trail(void)
{
  const uint8_t first = (location_trail_count < LOCATION_TRAIL_SIZE) ? 0U :
                        location_trail_next;
  for (uint8_t i = 0U; i < location_trail_count; ++i) {
    const uint8_t index = (uint8_t)((first + i) % LOCATION_TRAIL_SIZE);
    LCD_FillRect((uint16_t)location_trail_x[index],
                 (uint16_t)location_trail_y[index], 1U, 1U, LCD_GREEN);
  }
}

static int16_t location_round_pixel(float value)
{
  return (int16_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t location_triangle_edge(int16_t ax, int16_t ay,
                                      int16_t bx, int16_t by,
                                      int16_t px, int16_t py)
{
  return (int32_t)(px - ax) * (int32_t)(by - ay) -
         (int32_t)(py - ay) * (int32_t)(bx - ax);
}

static void location_fill_triangle(int16_t x0, int16_t y0,
                                   int16_t x1, int16_t y1,
                                   int16_t x2, int16_t y2,
                                   uint16_t color)
{
  int16_t minimum_x = x0;
  int16_t maximum_x = x0;
  int16_t minimum_y = y0;
  int16_t maximum_y = y0;
  const int16_t xs[2] = {x1, x2};
  const int16_t ys[2] = {y1, y2};
  for (uint8_t i = 0U; i < 2U; ++i) {
    if (xs[i] < minimum_x) minimum_x = xs[i];
    if (xs[i] > maximum_x) maximum_x = xs[i];
    if (ys[i] < minimum_y) minimum_y = ys[i];
    if (ys[i] > maximum_y) maximum_y = ys[i];
  }

  for (int16_t y = minimum_y; y <= maximum_y; ++y) {
    int16_t span_start = -1;
    int16_t span_end = -1;
    for (int16_t x = minimum_x; x <= maximum_x; ++x) {
      const int32_t edge0 = location_triangle_edge(x0, y0, x1, y1, x, y);
      const int32_t edge1 = location_triangle_edge(x1, y1, x2, y2, x, y);
      const int32_t edge2 = location_triangle_edge(x2, y2, x0, y0, x, y);
      const bool has_negative = (edge0 < 0) || (edge1 < 0) || (edge2 < 0);
      const bool has_positive = (edge0 > 0) || (edge1 > 0) || (edge2 > 0);
      if (!(has_negative && has_positive)) {
        if (span_start < 0) {
          span_start = x;
        }
        span_end = x;
      }
    }
    if ((span_start >= 0) && (y >= 0)) {
      LCD_FillRect((uint16_t)span_start, (uint16_t)y,
                   (uint16_t)(span_end - span_start + 1), 1U, color);
    }
  }
}

static void location_draw_robot_triangle(int16_t center_x, int16_t center_y,
                                         int32_t heading_mdeg,
                                         uint16_t color)
{
  const float heading_rad = (float)heading_mdeg * 0.001f * 0.01745329252f;
  const float forward_x = cosf(heading_rad);
  const float forward_y = -sinf(heading_rad);
  const float left_x = -sinf(heading_rad);
  const float left_y = -cosf(heading_rad);

  /* Isosceles marker: the two equal short sides meet at the front point,
   * while the longer base represents the rear of the car. */
  const int16_t nose_x = (int16_t)(center_x +
      location_round_pixel(4.0f * forward_x));
  const int16_t nose_y = (int16_t)(center_y +
      location_round_pixel(4.0f * forward_y));
  const int16_t rear_left_x = (int16_t)(center_x +
      location_round_pixel(-3.0f * forward_x + 6.0f * left_x));
  const int16_t rear_left_y = (int16_t)(center_y +
      location_round_pixel(-3.0f * forward_y + 6.0f * left_y));
  const int16_t rear_right_x = (int16_t)(center_x +
      location_round_pixel(-3.0f * forward_x - 6.0f * left_x));
  const int16_t rear_right_y = (int16_t)(center_y +
      location_round_pixel(-3.0f * forward_y - 6.0f * left_y));

  location_fill_triangle(nose_x, nose_y, rear_left_x, rear_left_y,
                         rear_right_x, rear_right_y, color);
  location_draw_line(nose_x, nose_y, rear_left_x, rear_left_y, LCD_WHITE);
  location_draw_line(rear_left_x, rear_left_y,
                     rear_right_x, rear_right_y, LCD_WHITE);
  location_draw_line(rear_right_x, rear_right_y, nose_x, nose_y, LCD_WHITE);
  LCD_FillRect((uint16_t)nose_x, (uint16_t)nose_y, 2U, 2U, LCD_RED);
}

static void dashboard_draw_location(const LCDDashboard *dashboard)
{
  static bool layout_drawn;
  char text[24];
  const LocationPose *pose = &dashboard->location;

  if (!layout_drawn) {
    LCD_FillScreen(LCD_BLACK);
    layout_drawn = true;
  }
  if (location_marker_drawn) {
    const int16_t erase_x = (location_last_x > 8) ? location_last_x - 8 : 0;
    const int16_t erase_y = (location_last_y > 8) ? location_last_y - 8 : 0;
    LCD_FillRect((uint16_t)erase_x, (uint16_t)erase_y, 17U, 17U, LCD_BLACK);
  }
  location_draw_static_map();

  (void)snprintf(text, sizeof(text), "X:%ld Y:%ld",
                 (long)pose->x_mm, (long)pose->y_mm);
  dashboard_write(0U, 4U, 128U, text);
  const int32_t heading = pose->heading_mdeg;
  const char *state = !pose->valid ? "IMUERR" :
                      (!pose->inside_field ? "OUT" :
                      (dashboard->location_demo_running ? "EXIT" : "STOP"));
  (void)snprintf(text, sizeof(text), "H:%ld.%01ld Z%u %s",
                 (long)(heading / 1000L),
                 (long)((heading % 1000L) / 100L),
                 pose->start_zone, state);
  dashboard_write(0U, 20U, 128U, text);

  const int16_t current_x = location_map_x(pose->x_mm);
  const int16_t current_y = location_map_y(pose->y_mm);
  location_add_trail_point(current_x, current_y);
  location_draw_trail();
  location_last_x = current_x;
  location_last_y = current_y;
  location_draw_robot_triangle(location_last_x, location_last_y, heading,
                               pose->valid ? LCD_YELLOW : LCD_RED);
  location_marker_drawn = true;
}
#endif

#if APP_ENABLE_TASK
static const char *const task_state_names[] = {
  "WAIT_CONFIG", "START",       "FIND_OBJECT", "CRAB_OBJECT",
  "RETURN_SAFE", "DROP_OBJECT", "STOPPED"
};

static const char *const task_labels[][5] = {
  {"COLOR:", "ZONE:",  "CFG:",   "UART:",  "MOTOR:"},
  {"TIME:",  "DIST:",  "SPEED:", "ZONE:",  "MOTOR:"},
  {"TIME:",  "FOUND:", "DIST:",  "TYPE:",  "MOTOR:"},
  {"TIME:",  "GRAB:",  "COUNT:", "LOAD:",  "MOTOR:"},
  {"TIME:",  "CARGO:", "DEST:",  "NEAR:",  "NAV:"},
  {"TIME:",  "PHASE:", "CHECK:", "CARGO:", "MOTOR:"},
  {"TIME:",  "FOUND:", "GRAB:",  "COUNT:", "MOTOR:"}
};

static void dashboard_draw_task_layout(TaskState state)
{
  const uint8_t state_index = (state <= TASK_STOPPED) ? (uint8_t)state :
                                                        (uint8_t)TASK_STOPPED;
  LCD_FillScreen(LCD_BLACK);
  LCD_DrawText(15U, 4U, "RESCUE TASK", LCD_YELLOW, LCD_BLACK);
  LCD_DrawText(0U, 20U, "STATE:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 34U, "RX0:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 48U, "RX1:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 62U, "RX2:", LCD_CYAN, LCD_BLACK);
  LCD_DrawText(0U, 76U, "RX3:", LCD_CYAN, LCD_BLACK);
  for (uint8_t line = 0U; line < 5U; ++line) {
    LCD_DrawText(0U, (uint16_t)(94U + line * 14U),
                 task_labels[state_index][line], LCD_GREEN, LCD_BLACK);
  }
  task_layout = state_index;
}

static void dashboard_write_value(uint8_t line, const char *text)
{
  dashboard_write(42U, (uint16_t)(94U + line * 14U), 86U, text);
}

static void dashboard_write_rx(uint8_t line, const char *text)
{
  dashboard_write(30U, (uint16_t)(34U + line * 14U), 98U, text);
}

static void dashboard_format_counts(char *text, size_t size, uint8_t counts)
{
  (void)snprintf(text, size, "N%u C%u H%u D%u",
                 VISION_COUNT_NORMAL(counts), VISION_COUNT_CORE(counts),
                 VISION_COUNT_CASUALTY(counts), VISION_COUNT_DANGER(counts));
}

static const char *dashboard_nav_text(uint8_t direction)
{
  switch (direction) {
    case VISION_NAV_FORWARD:
      return "FORWARD";
    case VISION_NAV_TURN_LEFT:
      return "LEFT";
    case VISION_NAV_TURN_RIGHT:
      return "RIGHT";
    case VISION_NAV_BACKWARD:
      return "BACK";
    default:
      return "HOLD";
  }
}

static const char *dashboard_drop_text(TaskDropPhase phase)
{
  static const char *const names[] = {
    "ENTER", "RELEASE", "CAMERA", "VERIFY", "LEAVE", "RETRY"
  };
  return (phase <= TASK_DROP_RETRY_BACK) ? names[phase] : "ERROR";
}

static void dashboard_draw_frame(const VisionData *vision)
{
  char text[18];
  if ((vision->last_frame[0] != VISION_FRAME_HEAD_1) ||
      (vision->last_frame[1] != VISION_FRAME_HEAD_2) ||
      (vision->last_frame[VISION_FRAME_SIZE - 1U] != VISION_FRAME_TAIL)) {
    dashboard_write_rx(0U, "-- -- -- --");
    dashboard_write_rx(1U, "-- -- -- --");
    dashboard_write_rx(2U, "-- -- -- --");
    dashboard_write_rx(3U, "-- -- --");
    return;
  }

  for (uint8_t line = 0U; line < 3U; ++line) {
    const uint8_t offset = (uint8_t)(line * 4U);
    (void)snprintf(text, sizeof(text), "%02X %02X %02X %02X",
                   vision->last_frame[offset], vision->last_frame[offset + 1U],
                   vision->last_frame[offset + 2U], vision->last_frame[offset + 3U]);
    dashboard_write_rx(line, text);
  }
  (void)snprintf(text, sizeof(text), "%02X %02X %02X",
                 vision->last_frame[12], vision->last_frame[13],
                 vision->last_frame[14]);
  dashboard_write_rx(3U, text);
}

static void dashboard_draw_task(const LCDDashboard *dashboard)
{
  char text[24];
  const TaskStatus task = Task_GetStatus();
  const VisionData vision = Vision_GetSnapshot();
  const uint8_t state = (task.state <= TASK_STOPPED) ? (uint8_t)task.state :
                                                       (uint8_t)TASK_STOPPED;
  const bool motor_fault = dashboard_motor_fault();

  if (task_layout != state) {
    dashboard_draw_task_layout((TaskState)state);
  }
  dashboard_write(42U, 20U, 86U, task_state_names[state]);
  dashboard_draw_frame(&vision);

  switch ((TaskState)state) {
    case TASK_WAIT_CONFIG:
      dashboard_write_value(0U, task.color == VISION_COLOR_RED ? "RED" :
                                (task.color == VISION_COLOR_BLUE ? "BLUE" : "--"));
      if (task.start_zone == 0U) {
        dashboard_write_value(1U, "--");
      } else {
        (void)snprintf(text, sizeof(text), "%u", task.start_zone);
        dashboard_write_value(1U, text);
      }
      dashboard_write_value(2U, vision.config_ready ? "OK" : "WAIT 3");
      dashboard_write_value(3U, dashboard_uart_text(dashboard));
      dashboard_write_value(4U, motor_fault ? "FAULT" : "OK");
      break;

    case TASK_START:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      dashboard_write_value(0U, text);
      (void)snprintf(text, sizeof(text), "%lumm", (unsigned long)task.distance_mm);
      dashboard_write_value(1U, text);
      (void)snprintf(text, sizeof(text), "%ldMM/S",
                     (long)APP_GO_DISTANCE_SPEED_MM_S);
      dashboard_write_value(2U, text);
      (void)snprintf(text, sizeof(text), "%u", task.start_zone);
      dashboard_write_value(3U, text);
      dashboard_write_value(4U, motor_fault ? "FAULT" : "OK");
      break;

    case TASK_FIND_OBJECT:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      dashboard_write_value(0U, text);
      dashboard_write_value(1U, task.found ? "YES" : "NO");
      (void)snprintf(text, sizeof(text), "%lumm", (unsigned long)task.distance_mm);
      dashboard_write_value(2U, text);
      dashboard_format_counts(text, sizeof(text), task.cargo_counts);
      dashboard_write_value(3U, text);
      dashboard_write_value(4U, motor_fault ? "FAULT" : "OK");
      break;

    case TASK_CRAB_OBJECT:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      dashboard_write_value(0U, text);
      dashboard_write_value(1U, task.grabbed ? "YES" : "NO");
      (void)snprintf(text, sizeof(text), "%u", task.object_count);
      dashboard_write_value(2U, text);
      dashboard_write_value(3U, vision.unknown ? "UNKNOWN" :
                                (VISION_COUNT_DANGER(task.cargo_counts) != 0U ? "DANGER" :
                                (task.cargo_valid ? "VALID" :
                                (task.grabbed ? "VERIFY" : "APPROACH"))));
      dashboard_write_value(4U, motor_fault ? "FAULT" : "OK");
      break;

    case TASK_RETURN_SAFE:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      dashboard_write_value(0U, text);
      dashboard_format_counts(text, sizeof(text), task.cargo_counts);
      dashboard_write_value(1U, text);
      dashboard_write_value(2U, task.destination == TASK_DEST_MATERIAL ? "MATERIAL" :
                                (task.destination == TASK_DEST_CASUALTY ? "CASUALTY" : "--"));
      dashboard_write_value(3U, task.near_safe ? "YES" : "NO");
      dashboard_write_value(4U, task.nav_fresh ? dashboard_nav_text(task.nav_direction) :
                                                "TIMEOUT");
      break;

    case TASK_DROP_OBJECT:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      dashboard_write_value(0U, text);
      dashboard_write_value(1U, dashboard_drop_text(task.drop_phase));
      dashboard_write_value(2U, task.drop_phase < TASK_DROP_VERIFY ? "WAIT" :
                                (task.claw_empty ? "EMPTY" :
                                (task.drop_phase == TASK_DROP_RETRY_BACK ? "LOADED" : "VISION")));
      dashboard_format_counts(text, sizeof(text), task.cargo_counts);
      dashboard_write_value(3U, text);
      dashboard_write_value(4U, motor_fault ? "FAULT" : "OK");
      break;

    default:
      (void)snprintf(text, sizeof(text), "%us", task.remaining_s);
      dashboard_write_value(0U, text);
      dashboard_write_value(1U, task.found ? "YES" : "NO");
      dashboard_write_value(2U, task.grabbed ? "YES" : "NO");
      (void)snprintf(text, sizeof(text), "%u", task.object_count);
      dashboard_write_value(3U, text);
      dashboard_write_value(4U, motor_fault ? "FAULT" : "OK");
      break;
  }
}
#elif !APP_ENABLE_LOCATION_DEMO
static void dashboard_draw_test(const LCDDashboard *dashboard)
{
  static bool layout_drawn;
  char text[24];
#if !APP_ENABLE_MOTION_TEST
  EncoderStatus encoders[3];
#endif

  if (!layout_drawn) {
    LCD_FillScreen(LCD_BLACK);
#if APP_ENABLE_MOTION_TEST
    LCD_DrawText(30U, 4U, "IMU ANGLE", LCD_YELLOW, LCD_BLACK);
    LCD_DrawText(0U, 24U, "STATE:", LCD_CYAN, LCD_BLACK);
    LCD_DrawText(0U, 56U, "TOTAL:", LCD_GREEN, LCD_BLACK);
    LCD_DrawText(0U, 92U, "CURRENT:", LCD_GREEN, LCD_BLACK);
    LCD_DrawText(6U, 140U, "TURN CAR BY HAND", LCD_YELLOW, LCD_BLACK);
#else
    LCD_DrawText(6U, 4U, "LCD WHEEL TEST", LCD_YELLOW, LCD_BLACK);
    LCD_DrawText(0U, 24U, "M1:", LCD_CYAN, LCD_BLACK);
    LCD_DrawText(0U, 38U, "M2:", LCD_CYAN, LCD_BLACK);
    LCD_DrawText(0U, 52U, "M3:", LCD_CYAN, LCD_BLACK);
    LCD_DrawText(0U, 86U, "UART3:", LCD_GREEN, LCD_BLACK);
    LCD_DrawText(0U, 104U, "SERVO:", LCD_GREEN, LCD_BLACK);
    LCD_DrawText(0U, 122U, "WHEEL:", LCD_GREEN, LCD_BLACK);
#endif
    layout_drawn = true;
  }

#if APP_ENABLE_MOTION_TEST
  const char *state = dashboard->imu_ready ? "READY" : "IMU ERR";
  dashboard_write(42U, 24U, 86U, state);
  const int64_t yaw = dashboard->imu_yaw_mdeg;
  const int64_t yaw_abs = (yaw < 0LL) ? -yaw : yaw;
  (void)snprintf(text, sizeof(text), "%c%ld.%01ld DEG",
                 (yaw < 0LL) ? '-' : '+',
                 (long)(yaw_abs / 1000LL),
                 (long)((yaw_abs % 1000LL) / 100LL));
  dashboard_write(42U, 56U, 86U, text);

  int64_t current_yaw = yaw % 360000LL;
  if (current_yaw < 0LL) {
    current_yaw += 360000LL;
  }
  (void)snprintf(text, sizeof(text), "%ld.%01ld DEG",
                 (long)(current_yaw / 1000LL),
                 (long)((current_yaw % 1000LL) / 100LL));
  dashboard_write(48U, 92U, 80U, text);
#else
  Encoder_GetAll(encoders);
  for (uint8_t id = 0U; id < 3U; ++id) {
    (void)snprintf(text, sizeof(text), "%lld", (long long)encoders[id].position);
    dashboard_write(24U, (uint16_t)(24U + id * 14U), 104U, text);
  }
  if (dashboard->uart_received &&
      ((uint32_t)(dashboard->now_ms - dashboard->uart_last_rx_ms) <=
       APP_VISION_TIMEOUT_MS)) {
    (void)snprintf(text, sizeof(text), "RX %02X", dashboard->uart_last_byte);
  } else {
    (void)strcpy(text, dashboard_uart_text(dashboard));
  }
  dashboard_write(42U, 86U, 86U, text);
  dashboard_write(42U, 104U, 86U, "READY");
#if APP_ENABLE_AUTOMATIC_MOTOR_TEST
  if (dashboard_motor_fault()) {
    (void)strcpy(text, "FAULT RESET");
  } else if (dashboard->motor_test_running) {
    (void)snprintf(text, sizeof(text), "PID %ldMM",
                   (long)APP_MOTOR_SPEED_TEST_TARGET_MM_S);
  } else {
    (void)strcpy(text, "KEY START");
  }
#else
  (void)strcpy(text, "STOP");
#endif
  dashboard_write(42U, 122U, 86U, text);
#endif
}
#endif

void LCD_DrawDashboard(const LCDDashboard *dashboard)
{
  if (dashboard == NULL) {
    return;
  }
#if APP_ENABLE_TASK
  dashboard_draw_task(dashboard);
#elif APP_ENABLE_LOCATION_DEMO
  dashboard_draw_location(dashboard);
#else
  dashboard_draw_test(dashboard);
#endif
}
