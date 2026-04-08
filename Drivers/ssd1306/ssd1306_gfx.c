/*
 * gfx.c
 *
 *  Created on: Nov 27, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "ssd1306.h"
//#include <stdio.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t _cursor_x = 0;
int16_t _cursor_y = 0;

uint16_t forecolor = SSD1306_WHITE;
uint16_t backcolor = SSD1306_BLACK;

/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/*
 * x: 0, XXX_WIDTH - 1
 * y: 0, XXX_HEIGHT - 1
 */
void SSD1306_SetCursor(int16_t x, int16_t y)
{
  _cursor_x = x;
  _cursor_y = y;
}

void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  // Bresenham's Line Drawing Algorithm
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steps, k;

  float xIncrement, yIncrement, x = x0, y = y0;

  if (abs(dx) > abs(dy)) {
     steps = abs(dx);
  } else {
     steps = abs(dy);
  }

  xIncrement = (float)dx / (float)steps;
  yIncrement = (float)dy / (float)steps;

  SSD1306_DrawPixel((int)x, (int)y, color);

  for (k = 0; k < steps; k++) {
     x += xIncrement;
     y += yIncrement;
     SSD1306_DrawPixel((int)x, (int)y, color);
  }
}

void SSD1306_DrawCircle(int16_t centerX, int16_t centerY, int16_t radius, uint16_t color)
{
  // Midpoint Circle Drawing Algorithm
  int x = radius;
  int y = 0;
  int radiusError = 1 - x;

  while (x >= y)
  {
    SSD1306_DrawPixel(centerX + x, centerY - y, color);
    SSD1306_DrawPixel(centerX - x, centerY - y, color);
    SSD1306_DrawPixel(centerX + x, centerY + y, color);
    SSD1306_DrawPixel(centerX - x, centerY + y, color);
    SSD1306_DrawPixel(centerX + y, centerY - x, color);
    SSD1306_DrawPixel(centerX - y, centerY - x, color);
    SSD1306_DrawPixel(centerX + y, centerY + x, color);
    SSD1306_DrawPixel(centerX - y, centerY + x, color);

    y++;

    if (radiusError < 0)
    {
       radiusError += 2 * y + 1;
    } else
    {
       x--;
       radiusError += 2 * (y - x + 1);
    }
  }
}

//--- ReDesign 2025.10.24. ---
void SSD1306_DrawChar(int16_t x, int16_t y, const char ch, const FontTypeDef *font, uint16_t color)
{
  int byte_index = 0;
  if( 0 <= ch  && ch <= font->StartChar)
  { //Ez szimbolum lesz, ami karakterek előtt van, nem szabványos
    byte_index = font->getFirstByteIndexOfChar(ch);
  }
  else
  {
    // 0x20-tól kezdődnek a karakterek az ASCII táblában
    // StartChar az első karakter a táblában
    byte_index = font->getFirstByteIndexOfChar(ch - 0x20 + font->StartChar);
  }

  //uint8_t row = 0;
  uint8_t bits = 0;
  for(int column_index = 0; column_index < font->Height; column_index ++) //y irány, tobább mehet mint egy bájt!
  {
    //soronként haladok először végig, így van tárolva is a karakter
    for(int row_index = 0; row_index < font->Width; row_index++ ) // x irány, tovább mehet mint egy bájt!
    {
      if ((row_index % 8) == 0) // A "((row_index & 7) == 0)" és "((row_index % 8) == 0)" ugyan az
      {// minden új bájtnál indexnél beolvas egy bájtöt és növeli az indexet (row index 0..7 ig megy most)
        bits = font->FontBitmap[byte_index];
        //printf("%d.  0x%02X\r\n", row++, bits); //debug only
        byte_index++;
      }
       if(0x80 & bits)
         SSD1306_DrawPixel(x + row_index , y + column_index, forecolor);
       else
         SSD1306_DrawPixel(x + row_index , y + column_index, backcolor);
       bits <<= 1;
    }
  }
}

void SSD1306_DrawString(const char *string, const FontTypeDef *font, uint16_t color)
{
  uint16_t saved_start_x = _cursor_x;

  while(*string != 0)
  {
    if((*string)=='\n')
    {
      // Uj sor eseteén hozzá adja az Y-hoz a karakter magasságot.
      // Az X-et vissza állitja a kezdo pozicioba
      _cursor_y += font->Height + font->RowSapce;
      _cursor_x = saved_start_x;
    }
    else
    {
      SSD1306_DrawChar(_cursor_x, _cursor_y, (*string), font, color);
      _cursor_x += font->Width + 1;
    }
    string++;
  }
}

void SSD1306_FilledRectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  for(int16_t x = x0; x < x1; x++ )
    for(int16_t y =y0; y < y1; y++ )
      SSD1306_DrawPixel(x, y, color);
}




/* *****************************************************************************
 End of File
 */
