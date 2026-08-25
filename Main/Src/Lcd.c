#include "Lcd.h"

#include <stddef.h>
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

#if APP_ENABLE_TASK || !APP_ENABLE_MOTION_TEST
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
#else
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
    LCD_DrawText(0U, 28U, "STATE:", LCD_CYAN, LCD_BLACK);
    LCD_DrawText(0U, 64U, "YAW:", LCD_GREEN, LCD_BLACK);
    LCD_DrawText(0U, 100U, "UNIT:", LCD_CYAN, LCD_BLACK);
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
  dashboard_write(42U, 28U, 86U, state);
  const int64_t yaw = dashboard->imu_yaw_mdeg;
  const int64_t yaw_abs = (yaw < 0LL) ? -yaw : yaw;
  (void)snprintf(text, sizeof(text), "%c%ld.%01ld DEG",
                 (yaw < 0LL) ? '-' : '+',
                 (long)(yaw_abs / 1000LL),
                 (long)((yaw_abs % 1000LL) / 100LL));
  dashboard_write(30U, 64U, 98U, text);
  dashboard_write(42U, 100U, 86U, "DEGREE");
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
#else
  dashboard_draw_test(dashboard);
#endif
}
