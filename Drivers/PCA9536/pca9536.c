/*
 * pc9536.c
 *
 *  Created on: Apr 29, 2025
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pca9536.h"


/* Private define ------------------------------------------------------------*/
#define PCA9536_IO_TIMEOUT 50
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/*
/// Todo
uint8_t PCA9536_Read(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t regAddress, uint16_t *dataOut)
{
  uint8_t buffer[2];

  if(HAL_I2C_Master_Transmit(i2c, devAddress, (uint8_t[]){regAddress}, 1, PCA9536_IO_TIMEOUT ) != HAL_OK)
    return PCA9536_FAIL;

  if(HAL_I2C_Master_Receive(i2c, devAddress, buffer, sizeof(buffer), PCA9536_IO_TIMEOUT)!= HAL_OK)
    return PCA9536_FAIL;

  *dataOut = (buffer[0] << 8) | buffer[1];

  return PCA9536_OK;
}
*/

uint8_t PCA9536_Write(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t command, uint8_t dataOut)
{
  if(HAL_I2C_Master_Transmit(i2c, devAddress, (uint8_t[]){command, dataOut }, 2, PCA9536_IO_TIMEOUT ) != HAL_OK)
    return PCA9536_FAIL;

  return PCA9536_OK;
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
