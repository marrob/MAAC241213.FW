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

#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct REFOXCO_st
{
  float Voltage; //csak az I2C-s tudja
  float Current; //csak az I2C-s tudja
  uint16_t LSB_Voltage;
  uint16_t LSB_Current;
  uint16_t INA226_DIE_ID;  //diag only 0x2260
  uint16_t LSB_Temperature;
  float Temperature;          //I2C-s   hőmérő
  float LegacyTemperature;    //Analog -hőmérő
  bool ExtRef;
}REFOCXO_t;

typedef struct OCXO_st
{
  float Voltage;
  float Current;
  float Temperature;
  uint16_t LSB_Voltage;
  uint16_t LSB_Current;
  uint16_t INA226_DIE_ID;  //diag only 0x2260
  uint16_t LSB_Temperature;
  bool     IsLocked;
}OCXO_t;

typedef struct _Devic_t
{
  struct _Diag
  {
    uint32_t LcdTimeout;
    uint32_t UartUnknwonCnt;
    uint32_t UartErrorCnt;
    uint32_t UartDmaOverrunCnt;
    uint32_t UartDmaErrorCnt;
    uint32_t UartDmaStartErrorCnt;
    uint32_t UpTimeSec;
    uint32_t TransactionCnt;
    uint32_t BacklightChangedCnt;
    uint32_t PcPsuOnCnt;
    uint32_t PcPsuOffCnt;
    uint32_t ForceBacklightOnCnt;
  }Diag;

  struct _Triclock
  {
    OCXO_t OCXO1, OCXO2, OCXO3;
    REFOCXO_t REFOCXO;
  }TriClock;


  struct _PC_st
  {
    bool BacklightIsOn;
    bool BacklightIsOnPre;
    uint8_t BacklightIntensity;
    uint8_t BacklightIntensityPre;
    bool PsuStatePre;
    bool PsuState;
    bool BacklightOnProtectionCompleted;

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
#define DEVICE_FW               "250717_2028"
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

#define REFOCXO_INA226_ADDRESS      0x80  //U109
#define REFOCXO_TMP100_ADDRESS      0x90  //U114

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


//--- Backlight ---
void Backlight_On(void);
void Backlight_Off(void);
void Backlight_Init(TIM_HandleTypeDef *htim, uint8_t percent);
void Backlight_SetDuty(uint8_t percent);
uint8_t Backlight_GetDuty(void);

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
#define BLIGHT_PWM_Pin GPIO_PIN_1
#define BLIGHT_PWM_GPIO_Port GPIOA
#define BLIGHT_EN_Pin GPIO_PIN_2
#define BLIGHT_EN_GPIO_Port GPIOA
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
#define REF_EXT_N_Pin GPIO_PIN_0
#define REF_EXT_N_GPIO_Port GPIOB
#define ADC_SYNC_Pin GPIO_PIN_8
#define ADC_SYNC_GPIO_Port GPIOC
#define BLIGHT_RLY_Pin GPIO_PIN_9
#define BLIGHT_RLY_GPIO_Port GPIOC
#define STAT_LED_Pin GPIO_PIN_8
#define STAT_LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
