/*
 * ina226.c
 *
 *  Created on: Apr 28, 2025
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ina226.h"


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define INA226_IO_TIMEOUT 100


/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/


uint8_t INA226_Read(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t regAddress, uint16_t *dataOut)
{
  uint8_t buffer[2];

  if(HAL_I2C_Master_Transmit(i2c, devAddress, (uint8_t[]){regAddress}, 1, INA226_IO_TIMEOUT ) != HAL_OK)
    return INA226_FAIL;

  if(HAL_I2C_Master_Receive(i2c, devAddress, buffer, sizeof(buffer), INA226_IO_TIMEOUT)!= HAL_OK)
    return INA226_FAIL;

  *dataOut = (buffer[0] << 8) | buffer[1];

  return INA226_OK;
}


uint8_t INA226_Write(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t regAddress, const uint16_t *dataIn)
{



  return INA226_OK;
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
