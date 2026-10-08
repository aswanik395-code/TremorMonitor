#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"   /* for I2C_HandleTypeDef via main.h -> stm32f1xx_hal.h */

/* Your board prints "ADDRESS_SELECT 0x78 or 0x7A" - those are 8-bit write
   addresses. 0x78 -> 7-bit 0x3C (far more common default).
   0x7A -> 7-bit 0x3D. Check the solder jumper on the board if unsure. */
#define SSD1306_I2C_ADDR   (0x3C << 1)

#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64

/* Most 1.3" I2C OLEDs use the SH1106 chip, which has 132 columns of RAM
   internally with the visible 128 columns offset by 2. Leave this at 2
   for a 1.3" module. If you have a 0.96" SSD1306 instead, set it to 0. */
#define SSD1306_COL_OFFSET 2

void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
/* x: 0-127, page: 0-7 (each page = 8 pixel rows). Text is drawn 6px wide per char. */
void SSD1306_DrawString(uint8_t x, uint8_t page, const char *str);

#endif
