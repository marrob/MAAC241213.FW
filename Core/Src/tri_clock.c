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

/* Private function prototypes -----------------------------------------------*/
inline static bool Get_OCXO1_Lock(void);
inline static bool Get_OCXO2_Lock(void);
inline static bool Get_OCXO3_Lock(void);
inline static bool Get_External_Reference(void);

/* Private user code ---------------------------------------------------------*/

void TriClock_Init(I2C_HandleTypeDef *i2ch)
{
  _i2ch = i2ch;

  //--- ADC ---
  MCP3421_Init(i2ch, MCP3421_I2C_DEVICE_ADDRESS);
  MCP3421_NonBlocking_Start(MCP3421_PGA_1x | MCP3421_RES_18);
  HAL_Delay(5);

  //--- Port Expander ---
  PCA9536_Write(i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_CONFIG, 0x00); //every ports are output
  PCA9536_Write(i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, 0x00); //every ports are off
  PCA9536_Write(i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, PCA_LED_ON ); //LED...
  //--- Minden OCXO bekapcsol ---
  PCA9536_Write(_i2ch, TRICLOCK_PCA9536_ADDRESS, PCA9536_CMD_WRITE, PCA_LED_ON | PCA_OCXO1_ON | PCA_OCXO2_ON | PCA_OCXO3_ON);

}

void TriClock_Task(void)
{
  static uint32_t timestamp;

  if(HAL_GetTick() -  timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    HAL_Delay(5);

    //--- OCXO3 ---
    INA226_Read(_i2ch, OCXO3_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.OCXO3.INA226_DIE_ID);

    INA226_Read(_i2ch, OCXO3_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.OCXO3.LSB_Voltage);
    Device.TriClock.OCXO1.Voltage = INA226_ConvertToVoltage(Device.TriClock.OCXO1.LSB_Voltage);

    INA226_Read(_i2ch, OCXO3_INA226_ADDRESS, INA226_REG_SHUNT_VOLTAGE, &Device.TriClock.OCXO3.LSB_Current);
    Device.TriClock.OCXO3.Current = INA226_ConvertToCurrent(0.04F, Device.TriClock.OCXO3.LSB_Current);

    TMP100_Read(_i2ch, OCXO3_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.OCXO3.LSB_Temperature);
    Device.TriClock.OCXO3.Temperature = TMP100_ConvertToCelsius(Device.TriClock.OCXO3.LSB_Temperature);

    Device.TriClock.OCXO3.IsLocked = Get_OCXO3_Lock();


    //--- OCXO2 ---
    INA226_Read(_i2ch, OCXO2_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.OCXO2.INA226_DIE_ID);

    INA226_Read(_i2ch, OCXO2_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.OCXO2.LSB_Voltage);
    Device.TriClock.OCXO2.Voltage = INA226_ConvertToVoltage(Device.TriClock.OCXO2.LSB_Voltage);

    INA226_Read(_i2ch, OCXO2_INA226_ADDRESS, INA226_REG_SHUNT_VOLTAGE, &Device.TriClock.OCXO2.LSB_Current);
    Device.TriClock.OCXO2.Current = INA226_ConvertToCurrent(0.04F, Device.TriClock.OCXO2.LSB_Current);

    TMP100_Read(_i2ch, OCXO2_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.OCXO2.LSB_Temperature);
    Device.TriClock.OCXO2.Temperature = TMP100_ConvertToCelsius(Device.TriClock.OCXO2.LSB_Temperature);

    Device.TriClock.OCXO2.IsLocked = Get_OCXO2_Lock();

    //--- OCXO1 ---
    INA226_Read(_i2ch, OCXO1_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.OCXO1.INA226_DIE_ID);

    INA226_Read(_i2ch, OCXO1_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.OCXO1.LSB_Voltage);
    Device.TriClock.OCXO3.Voltage = INA226_ConvertToVoltage(Device.TriClock.OCXO1.LSB_Voltage);

    INA226_Read(_i2ch, OCXO1_INA226_ADDRESS, INA226_REG_SHUNT_VOLTAGE, &Device.TriClock.OCXO1.LSB_Current);
    Device.TriClock.OCXO1.Current = INA226_ConvertToCurrent(0.04F, Device.TriClock.OCXO1.LSB_Current);

    TMP100_Read(_i2ch, OCXO1_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.OCXO1.LSB_Temperature);
    Device.TriClock.OCXO1.Temperature = TMP100_ConvertToCelsius(Device.TriClock.OCXO1.LSB_Temperature);

    Device.TriClock.OCXO1.IsLocked = Get_OCXO1_Lock();


    //--- REF OCXO ---
    INA226_Read(_i2ch, REFOCXO_INA226_ADDRESS, INA226_REG_DIEID, &Device.TriClock.REFOCXO.INA226_DIE_ID);

    INA226_Read(_i2ch, REFOCXO_INA226_ADDRESS, INA226_REG_BUS_VOLTAGE, &Device.TriClock.REFOCXO.LSB_Voltage);
    Device.TriClock.REFOCXO.Voltage = INA226_ConvertToVoltage(Device.TriClock.REFOCXO.LSB_Voltage);

    TMP100_Read(_i2ch, REFOCXO_TMP100_ADDRESS, TMP100_REG_TEMPERATURE, &Device.TriClock.REFOCXO.LSB_Temperature);
    Device.TriClock.REFOCXO.Temperature = TMP100_ConvertToCelsius(Device.TriClock.REFOCXO.LSB_Temperature);

    // Ez az I2C és a Legacy órában is azonos
    Device.TriClock.REFOCXO.ExtRef = Get_External_Reference();


    uint16_t legacy_temp_lsb = MCP3421_NonBlocking_GetVale();
    float lsb = 2 * (2.048 / 16384);
    float volts = legacy_temp_lsb * lsb;
    Device.TriClock.REFOCXO.LegacyTemperature = (-2.3654*volts*volts) + (-78.154*volts) + 153.857;

    //--- Legacy Locks  ---
    Device.TriClock.LegacyIsLocked3 = Get_OCXO3_Lock();
    Device.TriClock.LegacyIsLocked2 = Get_OCXO2_Lock();
    Device.TriClock.LegacyIsLocked1 = Get_OCXO1_Lock();
  }
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

inline static bool Get_External_Reference(void){
  return HAL_GPIO_ReadPin(REF_EXT_N_GPIO_Port, REF_EXT_N_Pin ) == GPIO_PIN_RESET;
}

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
