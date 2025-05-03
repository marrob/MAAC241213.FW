/*
 * tmp100.h
 *
 *  Created on: Apr 29, 2025
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TMP100_H_
#define TMP100_H_

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define TMP100_OK      0
#define TMP100_FAIL    1


#define TMP100_REG_TEMPERATURE  0x00
#define TMP100_REG_CONFIG       0x01
#define TMP100_REG_TLOW         0x02
#define TMP100_REG_THIGH        0x03

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


uint8_t TMP100_Read(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t regAddress, uint16_t *read_word);
float TMP100_ConvertToCelsius(uint16_t data);

#endif /* TMP100_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
