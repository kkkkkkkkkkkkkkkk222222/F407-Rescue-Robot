#include "Lcd.h"

/* NormalDebug deliberately has no external LCD or blocking display transfers. */
bool LCD_Init(void) { return false; }
void LCD_FillScreen(uint16_t color) { (void)color; }
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  (void)x; (void)y; (void)w; (void)h; (void)color;
}
void LCD_DrawText(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg)
{
  (void)x; (void)y; (void)text; (void)color; (void)bg;
}
void LCD_DrawDashboard(const LCDDashboard *dashboard) { (void)dashboard; }
