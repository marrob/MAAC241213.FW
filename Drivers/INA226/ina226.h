/*
 * ina226.h
 *
 *  Created on: Apr 28, 2025
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INA226_H_
#define INA226_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#define INA226_OK      0
#define INA226_FAIL    1


#define INA226_REG_SHUNT_VOLTAGE  0x01
#define INA226_REG_BUS_VOLTAGE    0x02
#define INA226_REG_DIEID          0xFF


uint8_t INA226_Read(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t regAddress, uint16_t *read_word);
float INA226_ConvertToCurrent(float shunt, uint16_t reg_value);
float INA226_ConvertToVoltage(uint16_t reg_value);

#endif /* INA226_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
