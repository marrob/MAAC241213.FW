/*
 * tmp100.c
 *
 *  Created on: Apr 29, 2025
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tmp100.h"


/* Private define ------------------------------------------------------------*/
#define TMP100_IO_TIMEOUT 100
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/


uint8_t TMP100_Read(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t regAddress, uint16_t *dataOut)
{
  uint8_t buffer[2];

  if(HAL_I2C_Master_Transmit(i2c, devAddress, (uint8_t[]){regAddress}, 1, TMP100_IO_TIMEOUT ) != HAL_OK)
    return TMP100_FAIL;

  if(HAL_I2C_Master_Receive(i2c, devAddress, buffer, sizeof(buffer), TMP100_IO_TIMEOUT)!= HAL_OK)
    return TMP100_FAIL;

  *dataOut = (buffer[0] << 8) | buffer[1];

  return TMP100_OK;
}

/*
 *  12bit, 1LSB: 0.0625C
 *  1600LSB = 100C
 *  16LSB = 1C -> 16LSB/C
 *
 *  pl:
 *  1.) 25C -> 25C x 16LSB/C = 400LSB (0x190)
 *  2.) 400LSB -> 400LSB x 0.0625 = 25C
 */
float TMP100_ConvertToCelsius(uint16_t data)
{
  data >>=4;
  return  data * 0.0625F;
}
/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
