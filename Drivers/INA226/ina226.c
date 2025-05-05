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
#define INA226_IO_TIMEOUT 50


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


/*
float INA226_ConvertToCurrent(float shunt, uint16_t data)
{
  float result = 0;
  if(data & 0x8000)
  {
    //negative number
    data = ~data;
    result = -1 * (data + 1) * 0.0000025F;
    result = result / shunt;
  }
  else
  {
    result = data * 0.0000025F;
    result = result / shunt;
  }

  return result;
}
*/

// Bits: 15bit + 1sign
// LSB:2.5uV
// 0x8300 -> -80mV
// INA226_ConvertToCurrentV2(1, 0x8300);
//ez az elegáns megoldás...
float INA226_ConvertToCurrent(float shunt, uint16_t reg_value)
{
  float result = 0;
  int16_t signed_reg_value = (int16_t)reg_value;
  result = signed_reg_value * 0.0000025F;
  result = result / shunt;
  return result;
}



//Bits:15bit
//LSB:1.25mV
//FS: 40.96(0x7FFF)
float INA226_ConvertToVoltage(uint16_t reg_value)
{
  return reg_value * 0.00125;
}





/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
