/*
 * stm32f1_date_time.h
 *
 *  Created on: Dec 31, 2023
 *      Author: marrob
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DATETIME_STM32F1_DATE_TIME_H_
#define DATETIME_STM32F1_DATE_TIME_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <time.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DateTime_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef DateTime_Set(struct tm *tm_info);
struct tm DateTime_Get(void);


#endif /* DATETIME_STM32F1_DATE_TIME_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
