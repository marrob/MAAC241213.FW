/******************** (C) COPYRIGHT 2011 marrob Design *************************
* File Name          : MCP3421.h
* Author             : Margit Robert
* Date First Issued  : 26/10/2013
* Description        : MCP3421 
********************************************************************************/
#ifndef _MCP3421__H_
#define _MCP3421__H_  1

#include "main.h"

/* Public define -------------------------------------------------------------*/
#define MCP3421_OK      0
#define MCP3421_FAIL    1

#define MCP3421_DRY           0x80

#define MCP3421_MODE_CONT     0x10 //Continuous (Default)
#define MCP3421_MODE_ONESHOT  0x80 //One-Shot módban, 1-et kell irni, ami az új mérés inditását jelzi.
                                   //Continous módban nincs értelme irni.
                                   //Olvasáskor ha nulla akkor frissült mérés

#define MCP3421_RES_12        0x00 //240    SPS (Default)
#define MCP3421_RES_14        0x04 //60     SPS
#define MCP3421_RES_16        0x08 //15     SPS
#define MCP3421_RES_18        0x0C //3.75   SPS
#define MCP3421_RES_MASK      0x0C

#define MCP3421_PGA_1x        0x00 //1x (Default)
#define MCP3421_PGA_2x        0x01 //2x
#define MCP3421_PGA_4x        0x02 //4x
#define MCP3421_PGA_8x        0x03 //8x
#define MCP3421_PGA_MASK      0x03

/* Public typedef ------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
uint8_t MCP3421_Init(I2C_HandleTypeDef *i2c, uint8_t address);
int32_t MCP3421_Blocking_GetValue(uint8_t config);

void MCP3421_NonBlocking_Start(uint8_t config);
int32_t MCP3421_NonBlocking_GetVale(void);

#endif //_MCP3421__H_
/******************* (C) COPYRIGHT 2011 marrob Design *****END OF FILE******/

