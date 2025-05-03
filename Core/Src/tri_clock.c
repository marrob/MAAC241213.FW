/*
 * tri_clock.c
 *
 *  Created on: Apr 22, 2025
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include "mcp3421.h"
#include "ina226.h"
#include "tmp100.h"
#include "pca9536.h"


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef *_i2ch;
extern Device_t Device;


#define PCA_OCXO3_ON  PCA9536_O0
#define PCA_OCXO2_ON  PCA9536_O1
#define PCA_OCXO1_ON  PCA9536_O2
#define PCA_LED_ON    PCA9536_O3


#define INTER_STATE_DEALY_MS 3000


/*
 * OCXO3 -> 25MHz
 * OCXO2 -> 20MHz
 * OCXO1 -> 24MHz
 *
 *
 *
 */

/* Private function prototypes -----------------------------------------------*/
inline static bool Get_OCXO1_Lock(void);
inline static bool Get_OCXO2_Lock(void);
inline static bool Get_OCXO3_Lock(void);
inline static bool Get_Lock_External(void);
void TriClock_PowerTask(void);
/* Private user code ---------------------------------------------------------*/


void TriClock_Init(I2C_HandleTypeDef *i2ch)
{
  _i2ch = i2ch;

  //--- ADC ---
  MCP3421_Init(i2ch, MCP3421_I2C_DEVICE_ADDRESS);
  MCP3421_NonBlocking_Start(MCP3421_PGA_1x | MCP3421_RES_18);
  HAL_Delay(5);

//  while(true)
  {
    //--- Port Expander ---

    PCA9536_Write(i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_CONFIG, 0x00); //every ports are output
    //HAL_Delay(5);

    PCA9536_Write(i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, 0x00); //every ports are off
    //HAL_Delay(5);

    PCA9536_Write(i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, PCA_LED_ON ); //every port are output
   //HAL_Delay(5);
  }

  Device.TriClock.OCXO3.WarmUpMs = 0;
  Device.TriClock.OCXO2.WarmUpMs = 0;
  Device.TriClock.OCXO1.WarmUpMs = 0;
}


void TriClock_Task(void)
{

  static uint32_t timestamp;

  if(HAL_GetTick() -  timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.TriClock.MCP3421_Value = MCP3421_NonBlocking_GetVale();
    HAL_Delay(5);

    //--- OCXO3 ---
    INA226_Read(_i2ch, OCXO3_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.OCXO3.INA226_DIE_ID);
    INA226_Read(_i2ch, OCXO3_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.OCXO3.Voltage);
    INA226_Read(_i2ch, OCXO3_INA226_ADDRESS, INA226_REG_SHUNT_VOLTAGE, &Device.TriClock.OCXO3.Current);
    TMP100_Read(_i2ch, OCXO3_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.OCXO3.Temperature);
    Device.TriClock.OCXO3.IsLocked = Get_OCXO3_Lock();


    //--- OCXO2 ---
    INA226_Read(_i2ch, OCXO2_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.OCXO2.INA226_DIE_ID);
    INA226_Read(_i2ch, OCXO2_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.OCXO2.Voltage);
    INA226_Read(_i2ch, OCXO2_INA226_ADDRESS, INA226_REG_SHUNT_VOLTAGE, &Device.TriClock.OCXO2.Current);
    TMP100_Read(_i2ch, OCXO2_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.OCXO2.Temperature);
    Device.TriClock.OCXO2.IsLocked = Get_OCXO2_Lock();

    //--- OCXO1 ---
    INA226_Read(_i2ch, OCXO1_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.OCXO1.INA226_DIE_ID);
    INA226_Read(_i2ch, OCXO1_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.OCXO1.Voltage);
    INA226_Read(_i2ch, OCXO1_INA226_ADDRESS, INA226_REG_SHUNT_VOLTAGE, &Device.TriClock.OCXO1.Current);
    TMP100_Read(_i2ch, OCXO1_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.OCXO1.Temperature);
    Device.TriClock.OCXO1.IsLocked = Get_OCXO1_Lock();

  }

  TriClock_PowerTask();
}


void TriClock_PowerTask(void)
{

  static uint32_t timestamp;

  switch(Device.TriClock.State.Curr)
  {

    case STRI_START:
    {
      Device.TriClock.State.Next = STRI_WAIT;
      timestamp = HAL_GetTick();
      break;
    }

    case STRI_WAIT:
    {
      if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS )
      {
        Device.TriClock.State.Next = STRI_IDLE;
        timestamp = HAL_GetTick();
      }
      break;
    }

    case STRI_IDLE:
    {
      Device.TriClock.State.Next = STRI_OCXO1_WARM;
    }

    //--- OCXO1 ---
    case STRI_OCXO1_WARM:
    {
      if(Device.TriClock.State.Pre != Device.TriClock.State.Curr)
      {
        Device.TriClock.OCXO1.WarmUpMs = 0;
        timestamp = HAL_GetTick();
        PCA9536_Write(_i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, PCA_LED_ON | PCA_OCXO1_ON);
      }

      if(Device.TriClock.OCXO1.IsLocked)
      {
        Device.TriClock.State.Next = STRI_OCXO1_WARM_CPLT;
        Device.TriClock.OCXO1.WarmUpMs = HAL_GetTick() - timestamp;
      }
      break;
    }

    case STRI_OCXO1_WARM_CPLT:
    {
      if(Device.TriClock.State.Pre != Device.TriClock.State.Curr)
      {
        timestamp = HAL_GetTick();
      }

      if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS )
      {
        Device.TriClock.State.Next = STRI_OCXO3_WARM;
      }
      break;
    }
    // --- OCXO2 ---
    case STRI_OCXO2_WARM:
    {
      if(Device.TriClock.State.Pre != Device.TriClock.State.Curr)
      {
        timestamp = HAL_GetTick();
        Device.TriClock.OCXO2.WarmUpMs = 0;
        PCA9536_Write(_i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, PCA_LED_ON | PCA_OCXO1_ON | PCA_OCXO2_ON);
      }

      if(Device.TriClock.OCXO2.IsLocked)
      {
        Device.TriClock.State.Next = STRI_OCXO2_WARM_CPLT;
        Device.TriClock.OCXO2.WarmUpMs = HAL_GetTick() - timestamp;
      }
      break;
    }

    case STRI_OCXO2_WARM_CPLT:
    {
      if(Device.TriClock.State.Pre != Device.TriClock.State.Curr)
      {
        timestamp = HAL_GetTick();
      }

      if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS )
      {
        Device.TriClock.State.Next = STRI_OCXO3_WARM;
      }
      break;
    }

    // --- OCXO3 ---
    case STRI_OCXO3_WARM:
    {
      if(Device.TriClock.State.Pre != Device.TriClock.State.Curr)
      {
        timestamp = HAL_GetTick();
        Device.TriClock.OCXO3.WarmUpMs = 0;
        PCA9536_Write(_i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, PCA_LED_ON | PCA_OCXO1_ON | PCA_OCXO3_ON);
      }

      if(Device.TriClock.OCXO3.IsLocked)
      {
        Device.TriClock.State.Next = STRI_OCXO3_WARM_CPLT;
        Device.TriClock.OCXO3.WarmUpMs = HAL_GetTick() - timestamp;
      }
      break;
    }

    case STRI_OCXO3_WARM_CPLT:
    {
      if(Device.TriClock.State.Pre != Device.TriClock.State.Curr)
      {
        timestamp = HAL_GetTick();
      }

      if((HAL_GetTick() - timestamp) > INTER_STATE_DEALY_MS )
      {
        Device.TriClock.State.Next =  STRI_WARM_CPLT;
      }
      break;
    }

    case STRI_WARM_CPLT:
    {
      break;
    }
  }

  Device.TriClock.State.Pre = Device.TriClock.State.Curr;
  Device.TriClock.State.Curr = Device.TriClock.State.Next;

}




inline static bool Get_OCXO1_Lock(void){
  return HAL_GPIO_ReadPin(OCXO1_LOCK_N_GPIO_Port, OCXO1_LOCK_N_Pin ) == GPIO_PIN_RESET;
}


inline static bool Get_OCXO2_Lock(void){
  return HAL_GPIO_ReadPin(OCXO2_LOCK_N_GPIO_Port, OCXO2_LOCK_N_Pin ) == GPIO_PIN_RESET;
}

inline static bool Get_OCXO3_Lock(void){
  return HAL_GPIO_ReadPin(OCXO3_LOCK_N_GPIO_Port, OCXO3_LOCK_N_Pin ) == GPIO_PIN_RESET;
}

inline static bool Get_Lock_External(void){
  return HAL_GPIO_ReadPin(LOCK_EXT_N_GPIO_Port, LOCK_EXT_N_Pin ) == GPIO_PIN_RESET;
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
