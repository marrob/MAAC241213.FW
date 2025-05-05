/*
 * power_up_sequece.c
 *
 *  Created on: Apr 22, 2025
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdio.h>


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern Device_t Device;

/* Private function prototypes -----------------------------------------------*/

inline static void PowerOn_CLK(void);
inline static void PowerOn_ETH(void);
inline static void PowerOn_P20(void);
inline static void PowerOn_P24(void);
inline static void PowerOn_NVME(void);

bool PcPsuIsOn(void);

/* Private user code ---------------------------------------------------------*/
void PwrSeq_Init(void)
{

  //--- Minden tápegység megy ---
  HAL_Delay(250);
  PowerOn_ETH();
  HAL_Delay(250);
  PowerOn_P20();
  HAL_Delay(250);
  PowerOn_P24();
  HAL_Delay(250);
  PowerOn_CLK();
  HAL_Delay(250);
  PowerOn_NVME();
  HAL_Delay(250);

  Device.PC.PsuState = false;
  Device.PC.PsuStatePre = false;
  Device.PC.BacklightIsOn = false;
}

void PwrSeq_Task(void)
{
  static uint32_t timestamp;

  Device.PC.PsuState =  PcPsuIsOn();

  if(Device.PC.PsuStatePre != Device.PC.PsuState)
  {
    if(Device.PC.PsuState == true )
    {
      //Elintult a PC és várjuk, hogy a PC UART-on bekapcsolja a kijelzőt
      timestamp = HAL_GetTick();
      Device.Diag.PcPsuOnCnt ++;
    }
    else
    { //Leállt a PC
      timestamp = 0;
      Device.Diag.PcPsuOffCnt ++;
      //kikapcsolja a kijelzőt, talán könnyebb debugolni
      Device.PC.BacklightIsOn = false;
      Device.PC.BacklightIntensity = 0;
    }
    Device.PC.PsuStatePre =  Device.PC.PsuState;
  }

  if(Device.PC.PsuState == true)
  {
    if(HAL_GetTick() - timestamp > 10000)
    {
       if(Device.PC.BacklightIsOn == false)
       {
         //ha: fut a PC és letet az idő és nincs bekacsolva a kijelző -> akkor bekapcsolja a kijelzőt
         Device.PC.BacklightIsOn = true;
         Device.PC.BacklightIntensity = 50;
         Device.Diag.ForceBacklightOnCnt++;
       }
       else
       {//lejárt a timeout, de a PC már bekacsolta a kijelzőt
         //ideális esetben itt ciklik
         timestamp = 0;
       }
    }
  }

  if(Device.PC.BacklightIsOnPre != Device.PC.BacklightIsOn)
  {
    if(Device.PC.BacklightIsOn)
      Backlight_On();
    else
      Backlight_Off();
    Device.PC.BacklightIsOnPre = Device.PC.BacklightIsOn;
  }

  //--- A fényerő modositasanak lehetosege mindig el ---
  if(Device.PC.BacklightIntensityPre != Device.PC.BacklightIntensity)
  {
    Backlight_SetDuty(Device.PC.BacklightIntensity);
    Device.Diag.BacklightChangedCnt++;
    Device.PC.BacklightIntensityPre = Device.PC.BacklightIntensity;
  }
}


inline static void PowerOn_CLK(void){
  HAL_GPIO_WritePin(CLK_EN_GPIO_Port, CLK_EN_Pin, GPIO_PIN_SET);
}
inline static bool DoesRun_CLK(void){
  return HAL_GPIO_ReadPin(CLK_EN_GPIO_Port, CLK_EN_Pin) == GPIO_PIN_SET;
}

inline static void PowerOn_ETH(void){
  HAL_GPIO_WritePin(ETH_EN_GPIO_Port, ETH_EN_Pin, GPIO_PIN_SET);
}
inline static bool DoesRun_ETH(void){
  return HAL_GPIO_ReadPin(ETH_EN_GPIO_Port, ETH_EN_Pin) == GPIO_PIN_SET;
}

inline static void PowerOn_P20(void){
  HAL_GPIO_WritePin(P20_EN_GPIO_Port, P20_EN_Pin, GPIO_PIN_SET);
}

inline static bool DoesRun_P20(void){
  return HAL_GPIO_ReadPin(P20_EN_GPIO_Port, P20_EN_Pin) == GPIO_PIN_SET;
}

inline static void PowerOn_P24(void){
  HAL_GPIO_WritePin(P24_EN_GPIO_Port, P24_EN_Pin, GPIO_PIN_SET);
}

inline static bool DoesRun_P24(void){
  return HAL_GPIO_ReadPin(P24_EN_GPIO_Port, P24_EN_Pin) == GPIO_PIN_SET;
}

inline static void PowerOn_NVME(void){
  HAL_GPIO_WritePin(NVME_EN_GPIO_Port, NVME_EN_Pin, GPIO_PIN_SET);
}

inline static bool DoesRun_NVME(void){
  return HAL_GPIO_ReadPin(NVME_EN_GPIO_Port, NVME_EN_Pin) == GPIO_PIN_SET;
}


//--- Backlight  ---
static TIM_HandleTypeDef *_htim;
void Backlight_On(void)
{
  HAL_GPIO_WritePin(BLIGHT_RLY_GPIO_Port, BLIGHT_RLY_Pin, GPIO_PIN_SET);
}

void Backlight_Off(void)
{
  HAL_GPIO_WritePin(BLIGHT_RLY_GPIO_Port, BLIGHT_RLY_Pin, GPIO_PIN_RESET);
}


/*
 * APB2 f: 72MHz
 * Prescaler: 59
 * ARR: 480
 * fpwm: 2500Hz
 *
 * - a kijelző invertált PWM-et vár, 0% PWM-hez tartozik a maximális fényerő
 * - a CH Polarity High -> Low legyen, igy init után 3.3V-van a PWM-en
 * - és bekapcsolás után azonnal inicilaizálja a kijelzőt mert csak akkor áll be az default értéke a portnak
 */
void Backlight_Init(TIM_HandleTypeDef *htim, uint8_t percent)
{
  _htim = htim;
  HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);
  Backlight_SetDuty(percent);
}

void Backlight_SetDuty(uint8_t percent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(_htim);
  uint32_t ccr = (arr * percent) / 100;
  __HAL_TIM_SET_COMPARE(_htim,TIM_CHANNEL_2, ccr);
}

uint8_t Backlight_GetDuty(void)
{
  return Device.PC.BacklightIntensity;
}

//--- PC ---
bool PcPsuIsOn(void)
{
  bool on = HAL_GPIO_ReadPin(DC_ON_N_GPIO_Port, DC_ON_N_Pin) == GPIO_PIN_RESET;
  return on;
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
