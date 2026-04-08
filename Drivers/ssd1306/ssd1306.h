/*
 * ssd1306.h
 *
 *  Created on: Oct 24, 2025
 *      Author: marrob
 */

#ifndef _SSD1306_H
#define _SSD1306_H

#include "main.h"     // stm32fxxx_hal.h
#include <stdlib.h>
#include <string.h>

#define SSD1306_WHITE           0
#define SSD1306_BLACK           1

#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          32
#define SSD1306_BPP             1   //Bits per pixel color

// --- SSD1306 0.91" I2C display ---
//
//    0,0---------------127,0
//     |                  |
//     |                  |
//    0,31--------------127,31
//

#define SSD1306_OK              0
#define SSD1306_ERROR           1

#define SSD1306_I2C_DEV_ADDRESS 0x78

uint8_t SSD1306_Init(I2C_HandleTypeDef *i2c, uint8_t address);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void SSD1306_Off(void);
void SSD1306_Update(void);
void SSD1306_Clear(void);


//--- ReDesign 2025 --- gfx ------


// Used for easy define big Bitmap as 0bXXXXXXXXX image
#define _BMP8(d)                                                        ((d)&0xFF)
#define _BMP16(d)                                      (((d)>>8)&0xFF), ((d)&0xFF)
#define _BMP24(d)                    (((d)>>16)&0xFF), (((d)>>8)&0xFF), ((d)&0xFF)
#define _BMP32(d)  (((d)>>24)&0xFF), (((d)>>16)&0xFF), (((d)>>8)&0xFF), ((d)&0xFF)

typedef struct _FontTypeDef
{
  uint8_t StartChar;      //A space karakter előtt lévő karakterek száma...
  uint8_t Width;          //Egy karaker szélessége px-ben
  uint8_t Height;         //Egy karakter magassága px-ben
  uint8_t RowSapce;       //Két sor közötti szünet px-ben
  int (*getFirstByteIndexOfChar)(char ch);
  uint8_t *FontBitmap;
}FontTypeDef;

extern FontTypeDef FontType5x7;
extern FontTypeDef FontType7x11;
extern FontTypeDef FontType10x14;

void SSD1306_SetCursor(int16_t x, int16_t y);
void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void SSD1306_DrawCircle(int16_t centerX, int16_t centerY, int16_t radius, uint16_t color);
void SSD1306_DrawChar(int16_t x, int16_t y, const char ch, const FontTypeDef *font, uint16_t color);
void SSD1306_DrawString(const char *string, const FontTypeDef *font, uint16_t color);
void SSD1306_FilledRectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void SSD1306_SetCursor(int16_t x, int16_t y);



#endif /* _SSD1306_H */

/* *****************************************************************************
 End of File
 */
