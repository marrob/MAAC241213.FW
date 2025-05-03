/*
 * pca9536.h
 *
 *  Created on: Apr 29, 2025
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PCA9536_H_
#define PCA9536_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
#define PCA9536_OK          0
#define PCA9536_FAIL        1

#define PCA9536_CMD_READ    0x00
#define PCA9536_CMD_WRITE   0x01
#define PCA9536_CMD_PLI     0x02
#define PCA9536_CMD_CONFIG  0x03

#define PCA9536_O0          0x01
#define PCA9536_O1          0x02
#define PCA9536_O2          0x04
#define PCA9536_O3          0x08

#define PCA9536_I0          0x01
#define PCA9536_I1          0x02
#define PCA9536_I2          0x04
#define PCA9536_I3          0x08


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t PCA9536_Write(I2C_HandleTypeDef *i2c, uint8_t devAddress, uint8_t command, uint8_t dataOut);

#endif /* PCA9536_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
