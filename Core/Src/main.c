/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  *
  *- Az orak tapjai az eszkoz aram ala helyezese utan fixen allandoan mennek.
  *- A PC be kikapcsolast a PCPSU vezerli atx szabvany szerint. Ezzel a FW-nek nem kell foglalkoznia.
  *
  * OCXO3 -> 25MHz
  * OCXO2 -> 20MHz
  * OCXO1 -> 24MHz
  *
  *                 |  Legacy Triclock   |          I2C Triclock          |
  * REFOCXO Temp    |nincs(152.918C ±1%) |van (MAAC kártya méri)          |
  * REFOCXO ExtRef  |van                 |van                             |
  * REFOCXO Current |nincs (0A)          |van (0..1A)                     |
  * REFOCXO Voltage |nincs (0V)          |van (0..12V)                    |
  *
  * OCXO1 Temp      |nincs (0C)          |van (0..70)                     |
  * OCXO1 Voltage   |nincs (0V)          |van (0..1A)                     |
  * OCXO1 Current   |nincs (0A)          |van (0..12V)                    |
  * OCXO1 IsLocked  |van (Legacy Locks)  |van                             |
  *
  *
  *
  *- Bekapcsoáls után a Baklight relé nincs meghzuva
  *- Bekpcsolás után a PWM vonalon 3.3V DC kell hogy legyen (ez a legsötétebb háttér világitas)
  *
  *- Elsődleges cél: A felhasználó ne lássa a boot képernyőt, viszont ha probléma van, legyen lehetőség debuggolásra.
  *- Bekapcsolási folyamat:
  *   - Ha a felhasználó megnyomja a POWER BUTTON-t, a PC tápja (PC PSU) a DC_ON vonalon jelez magas szinttel a kontrollernek.
  *   - A kontroller ekkor egy időzítést indít.
  *   - Ha egy meghatározott időn belül a PC a soros porton nem jelzi, hogy minden rendben, a kontroller bekapcsolja a kijelzőt a debughoz.
  *-
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "LiveLed.h"
#include "vt100.h"
#include "display.h"
#include "mcp3421.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
LiveLED_HandleTypeDef hLiveLed;
Device_t Device;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//--- Live LED ---
void LiveLedOn(void);
void LiveLedOff(void);

//--- Tools ---
void UpTimeTask(void);

void DrawDisplay(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

  //--- Backlight Init ---
  Backlight_Init(&htim2, 0);

  //--- Display ---
  DisplayInit(&hi2c2, SSD1306_I2C_DEV_ADDRESS);
  DisplayClear();
  DisplayUpdate();
  DisplayDrawString("   CORE AUDIO   ", &GfxFont7x8, SSD1306_WHITE );
  DisplayUpdate();

  HAL_Delay(100);

  //--- LiveLed ---
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  /*
  //--- EEPROM ---
  Eeprom_Init(&hi2c2, EEPROM_DEVICE_ADDRESS);

  uint32_t startSign;
  Eeprom_ReadU32(EEPROM_ADDR_FIRST_START, &startSign);
  if(startSign != 0x55AA)
  {
    //--- FIRST START ---
    Eeprom_WriteU32(EEPROM_ADDR_FIRST_START, 0x55AA);
    Eeprom_WriteU32(EEPROM_ADDR_BOOTUP_CNT, 0);
    Eeprom_WriteU32(EEPROM_ADDR_BKLIGHT_TIMEOUT_SEC, 60);
  }

  //--- Backlight Timeout ---
  Eeprom_ReadU32(EEPROM_ADDR_BKLIGHT_TIMEOUT_SEC, &Device.PC.BacklightTimeoutSec);

  //--- BOOTUP COUNTER ---
  Eeprom_ReadU32(EEPROM_ADDR_BOOTUP_CNT, &Device.Diag.BootupCnt);
  Device.Diag.BootupCnt++;
  Eeprom_WriteU32(EEPROM_ADDR_BOOTUP_CNT, Device.Diag.BootupCnt);

*/
  //--- Communication ---
  UartCom_Init(&huart1, &hdma_usart1_rx);

  //-- Power Sequence ---
  PwrSeq_Init();

  //--- TriClock ---
  TriClock_Init(&hi2c2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static int32_t timestamp;
    if(HAL_GetTick() - timestamp > 100)
    {
      timestamp = HAL_GetTick();
      DrawDisplay();
    }

    LiveLedTask(&hLiveLed);
    UpTimeTask();
    UartCom_Task();
    TriClock_Task();
    PwrSeq_Task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}




/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 480;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CLK_EN_Pin|ETH_EN_Pin|P20_EN_Pin|P24_EN_Pin
                          |NVME_EN_Pin|ADC_SYNC_Pin|BLIGHT_RLY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BLIGHT_EN_Pin|LIVE_LED_Pin|STAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLK_EN_Pin ETH_EN_Pin P20_EN_Pin P24_EN_Pin
                           NVME_EN_Pin ADC_SYNC_Pin BLIGHT_RLY_Pin */
  GPIO_InitStruct.Pin = CLK_EN_Pin|ETH_EN_Pin|P20_EN_Pin|P24_EN_Pin
                          |NVME_EN_Pin|ADC_SYNC_Pin|BLIGHT_RLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BLIGHT_EN_Pin LIVE_LED_Pin STAT_LED_Pin */
  GPIO_InitStruct.Pin = BLIGHT_EN_Pin|LIVE_LED_Pin|STAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_ON_N_Pin RST_H_N_Pin OCXO2_LOCK_N_Pin */
  GPIO_InitStruct.Pin = DC_ON_N_Pin|RST_H_N_Pin|OCXO2_LOCK_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OCXO1_LOCK_N_Pin OCXO3_LOCK_N_Pin */
  GPIO_InitStruct.Pin = OCXO1_LOCK_N_Pin|OCXO3_LOCK_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : REF_EXT_N_Pin */
  GPIO_InitStruct.Pin = REF_EXT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(REF_EXT_N_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.Diag.UpTimeSec++;
  }
}
/* LEDs ----------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}
/* printf --------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  //HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

enum Disp_e
{
  DISP_NONE,
  DISP_VERSION,
  DISP_OCXO1,
  DISP_OCXO2,
  DISP_OCXO3,
  DISP_REF,
  DISP_PC,
  DISP_LIVE,
};

#define DISP_INTER_DELAY_MS 4000

void DrawDisplay(void)
{
  static int32_t timestamp;
  static enum Disp_e dispPre, dispCurr, dispNext;
  static char string[120];
  DisplayClear();
  switch(dispCurr)
  {
    case DISP_NONE:{
      dispNext = DISP_VERSION;
      break;
    }
    case DISP_VERSION:{
      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      /*0123456789012345*/
      DisplaySetCursor(0, 0);
      DisplayDrawString("   CORE AUDIO   ", &GfxFont7x8, SSD1306_WHITE );
      DisplaySetCursor(0, 8);
      DisplayDrawString("      AAC       ", &GfxFont7x8, SSD1306_WHITE );
      DisplaySetCursor(0, 16);
      DisplayDrawString(DEVICE_FW, &GfxFont7x8, SSD1306_WHITE );
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_OCXO1;
      break;
    }
    case DISP_OCXO1:{

      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      DisplaySetCursor(0, 0);
      sprintf(string,"OCXO1\nU:%05.2fV\nI:%.3fmA\nt:%05.02fC-%s",
          Device.TriClock.OCXO1.Voltage,
          Device.TriClock.OCXO1.Current,
          Device.TriClock.OCXO1.Temperature,
          Device.TriClock.OCXO1.IsLocked?"LOCK":"UNLOCK");
      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE );
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_OCXO2;
      break;
    }

    case DISP_OCXO2:{
      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      DisplaySetCursor(0, 0);
      sprintf(string,"OCXO2\nU:%05.2fV\nI:%.3fmA\nt:%05.02fC-%s",
          Device.TriClock.OCXO2.Voltage,
          Device.TriClock.OCXO2.Current,
          Device.TriClock.OCXO2.Temperature,
          Device.TriClock.OCXO2.IsLocked?"LOCK":"UNLOCK");
      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE );
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_OCXO3;
      break;
    }

    case DISP_OCXO3:
    {
      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      DisplaySetCursor(0, 0);
      sprintf(string,"OCXO3\nU:%05.2fV\nI:%.3fmA\nt:%05.02fC-%s",
          Device.TriClock.OCXO3.Voltage,
          Device.TriClock.OCXO3.Current,
          Device.TriClock.OCXO3.Temperature,
          Device.TriClock.OCXO3.IsLocked?"LOCK":"UNLOCK");
      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE);
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_REF;
      break;
    }

    case DISP_REF:
    {
      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      DisplaySetCursor(0, 0);
      sprintf(string,"REF\nU:%05.2fV\nI:%.3fmA\nt:%05.02fC-%s",
          Device.TriClock.REFOCXO.Voltage,
          Device.TriClock.REFOCXO.Current,
          Device.TriClock.REFOCXO.Temperature,
          Device.TriClock.REFOCXO.ExtRef?"EXT":"INT");
      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE);
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_PC;
      break;
    }

    case DISP_PC:
    {
      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      DisplaySetCursor(0, 0);
      sprintf(string,"PC PSU:%s\nBK:%s INTE:%02d\nON CNT:%ld",
          Device.PC.PsuState?"ON":"OFF",
          Device.PC.BacklightIsOn?"ON":"OFF",
          Device.PC.BacklightIntensity,
          Device.Diag.PcPsuOnCnt);
      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE);
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_LIVE;
      break;
    }

    case DISP_LIVE:{
      if(dispPre != dispCurr)
        timestamp = HAL_GetTick();
      DisplaySetCursor(0, 0);
                       /*0123456789012345*/
      sprintf(string,"BootCnt:%ld\nUpTime:%ld", Device.Diag.BootupCnt, Device.Diag.UpTimeSec);
      DisplayDrawString(string, &GfxFont7x8, SSD1306_WHITE);
      if(HAL_GetTick() - timestamp > DISP_INTER_DELAY_MS)
        dispNext = DISP_VERSION;
      break;
    }
  }

  dispPre = dispCurr;
  dispCurr = dispNext;
  DisplayUpdate();
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
