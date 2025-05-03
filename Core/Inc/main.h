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
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum _PwrSeqStatus_e
{
  PWR_UNKNOWN = 0x0000,
  PWR_CLK_RUN = 0x8001,
  PWR_ETH_RUN = 0x8002,
  PWR_P20_RUN = 0x8004,
  PWR_P24_RUN = 0x8008,
  PWR_NVME_RUN = 0x8010,
}PowerStatus_t;


typedef enum _TriClkStates_e
{
  STRI_START,                   //0
  STRI_WAIT,                    //1
  STRI_IDLE,                    //2
  STRI_OCXO1_WARM,              //3
  STRI_OCXO1_WARM_CPLT,         //4
  STRI_OCXO2_WARM,              //5
  STRI_OCXO2_WARM_CPLT,         //6
  STRI_OCXO3_WARM,              //7
  STRI_OCXO3_WARM_CPLT,         //8
  STRI_WARM_CPLT,                //9

}TriClkStates_t;


typedef struct OCXO_st
{
  uint16_t Voltage;
  uint16_t Current;
  uint16_t INA226_DIE_ID;
  uint16_t Temperature;
  bool     IsLocked;
  uint32_t WarmUpMs;
}OCXO_t;

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


  struct _Triclock
  {
    uint32_t MCP3421_Value;
    OCXO_t OCXO1, OCXO2, OCXO3;
    struct
    {
      TriClkStates_t Next;
      TriClkStates_t Curr;
      TriClkStates_t Pre;
    }State;
  }TriClock;

  PowerStatus_t PowerStatus;

  struct _PC_st
  {

    bool DisplayIsOn;

    struct
    {
      bool Pre;
      bool Curr;
    }State;

  }PC;


}Device_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEVICE_OK   0
#define DEVICE_FAIL 1

#define DEVICE_NAME             "MAAC241213.FW"
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               "250422_1204"
#define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
#define DEVICE_PCB              "V00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "github.com/marrob"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)

//--- UART COM ---
#define UART_BUFFER_SIZE    64
#define UART_CMD_LENGTH     35
#define UART_ARG_LENGTH     35
#define UART_TERIMINATION_CHAR  '\r' //0x0D


//--- MCP3421 ---
//MCP3421A0T-E/CH:0xD0
//MCP3421A1T-E/CH:0xD2
#define MCP3421_I2C_DEVICE_ADDRESS  0xD0  //U40
#define TRICLOCK_PCA9536_ADDRESS    0x82  //U120

#define OCXO3_INA226_ADDRESS        0x88  //U117
#define OCXO3_TMP100_ADDRESS        0x92  //U122

#define OCXO2_INA226_ADDRESS        0x8A  //U369
#define OCXO2_TMP100_ADDRESS        0x94  //U375

#define OCXO1_INA226_ADDRESS        0x8C  //U370
#define OCXO1_TMP100_ADDRESS        0x98  //U376



/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

//--- UART COM ---
void UartCom_Init(UART_HandleTypeDef *uart, DMA_HandleTypeDef *dma);
void UartCom_Task(void);


//--- PowerSeq ---
void PwrSeq_Init(void);
void PwrSeq_Task(void);


//--- TriClock ---
void TriClock_Init(I2C_HandleTypeDef *i2ch);
void TriClock_Task(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CLK_EN_Pin GPIO_PIN_13
#define CLK_EN_GPIO_Port GPIOC
#define ETH_EN_Pin GPIO_PIN_14
#define ETH_EN_GPIO_Port GPIOC
#define P20_EN_Pin GPIO_PIN_15
#define P20_EN_GPIO_Port GPIOC
#define P24_EN_Pin GPIO_PIN_0
#define P24_EN_GPIO_Port GPIOC
#define NVME_EN_Pin GPIO_PIN_1
#define NVME_EN_GPIO_Port GPIOC
#define DISP_PWM_Pin GPIO_PIN_1
#define DISP_PWM_GPIO_Port GPIOA
#define DISP_EN_Pin GPIO_PIN_2
#define DISP_EN_GPIO_Port GPIOA
#define LIVE_LED_Pin GPIO_PIN_3
#define LIVE_LED_GPIO_Port GPIOA
#define DC_ON_N_Pin GPIO_PIN_4
#define DC_ON_N_GPIO_Port GPIOA
#define RST_H_N_Pin GPIO_PIN_5
#define RST_H_N_GPIO_Port GPIOA
#define OCXO2_LOCK_N_Pin GPIO_PIN_7
#define OCXO2_LOCK_N_GPIO_Port GPIOA
#define OCXO1_LOCK_N_Pin GPIO_PIN_4
#define OCXO1_LOCK_N_GPIO_Port GPIOC
#define OCXO3_LOCK_N_Pin GPIO_PIN_5
#define OCXO3_LOCK_N_GPIO_Port GPIOC
#define LOCK_EXT_N_Pin GPIO_PIN_0
#define LOCK_EXT_N_GPIO_Port GPIOB
#define ADC_SYNC_Pin GPIO_PIN_8
#define ADC_SYNC_GPIO_Port GPIOC
#define PC_INTERLOCK_Pin GPIO_PIN_9
#define PC_INTERLOCK_GPIO_Port GPIOC
#define STAT_LED_Pin GPIO_PIN_8
#define STAT_LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
