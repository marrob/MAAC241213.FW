/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct _Devic_t
{
  uint8_t DO;
  uint8_t DI;

  struct _Diag
  {
    uint32_t LcdTimeout;
    uint32_t UartUnknwonCnt;
    uint32_t UartErrorCnt;
    uint32_t UpTimeSec;
    uint32_t TransactionCnt;
  }Diag;

}Device_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEVICE_OK   0
#define DEVICE_FAIL 1

#define DEVICE_NAME             "MPRT220416"
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               "220903_1752"
#define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
#define DEVICE_PCB              "V00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "github.com/marrob"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)

/*--- UART COM ---*/
#define UART_BUFFER_SIZE    64
#define UART_CMD_LENGTH     35
#define UART_ARG_LENGTH     35
#define UART_TERIMINATION_CHAR  '\r' //0x0D


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/*--- UART COM ---*/
void UartCom_Init(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma);
void UartCom_Task(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIVE_LED_Pin GPIO_PIN_3
#define LIVE_LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
