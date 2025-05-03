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


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
