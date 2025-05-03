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
inline static bool DoesRun_CLK(void);

inline static void PowerOn_ETH(void);
inline static bool DoesRun_ETH(void);

inline static void PowerOn_P20(void);
inline static bool DoesRun_P20(void);

inline static void PowerOn_P24(void);
inline static bool DoesRun_P24(void);

inline static void PowerOn_NVME(void);
inline static bool DoesRun_NVME(void);

static uint16_t StatusRead(void);


void DisplayOn(void);
void DisplayOff(void);

/* Private user code ---------------------------------------------------------*/


void PwrSeq_Init(void)
{
  //--- Init ---

  PowerOn_CLK();
  PowerOn_ETH();
  PowerOn_P20();
  PowerOn_P24();
  PowerOn_NVME();

  Device.PC.State.Pre = false;
  Device.PC.State.Curr = false;
  Device.PC.DisplayIsOn = false;

}



void PwrSeq_Task(void)
{
  static uint32_t timestamp;


  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.PowerStatus = StatusRead();
  }



  //--- Ha nincs felfütve akkor, nem indulhat a PC ---
  if(Device.TriClock.State.Curr == STRI_WARM_CPLT)
  {
    HAL_GPIO_WritePin(PC_INTERLOCK_GPIO_Port, PC_INTERLOCK_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(PC_INTERLOCK_GPIO_Port, PC_INTERLOCK_Pin, GPIO_PIN_RESET);
  }



  static uint32_t pcStartTimestamp;
  Device.PC.State.Curr =  HAL_GPIO_ReadPin(DC_ON_N_GPIO_Port, DC_ON_N_Pin) == GPIO_PIN_RESET;

  if(Device.PC.State.Pre != Device.PC.State.Curr)
  {
    if(Device.PC.State.Curr == true )
    {
      //Elintult a PC és várjuk, hogy a PC UART-on bekapcsolja a kijelzőt
      pcStartTimestamp = HAL_GetTick();
    }

    if(Device.PC.State.Curr == false)
    { //Leállt a PC
      pcStartTimestamp = 0;
    }

    Device.PC.State.Pre =  Device.PC.State.Curr;
  }

  if(Device.PC.State.Curr == true)
  {
    if(HAL_GetTick() - pcStartTimestamp > 10000)
    {
      /*
       if(DisplayIsOn == false)
       {
        //ha fut a PC és letet az idő, és nincs bekacsolva a kijelző, akkor bekapcsolja a kijelzőt
         DisplayOn();
       }
       */
    }
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


void DisplayOn(void)
{
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_SET);
}

void DisplayOff(void)
{
  HAL_GPIO_WritePin(DISP_EN_GPIO_Port, DISP_EN_Pin, GPIO_PIN_RESET);
}



static uint16_t StatusRead(void)
{
  uint16_t status = 0;

  if(DoesRun_CLK())
    status |= PWR_CLK_RUN;
  else
    status &= ~(PWR_CLK_RUN & 0x7FFF);

  if(DoesRun_ETH())
    status |= PWR_ETH_RUN;
  else
    status &= ~(PWR_ETH_RUN & 0x7FFF);

  if(DoesRun_P20())
    status |= PWR_P20_RUN;
  else
    status &= ~(PWR_P20_RUN & 0x7FFF);

  if(DoesRun_P24())
    status |= PWR_P24_RUN;
  else
    status &= ~(PWR_P24_RUN & 0x7FFF);

  if(DoesRun_NVME())
    status |= PWR_NVME_RUN;
  else
    status &= ~(PWR_NVME_RUN & 0x7FFF);

  return status;
}



/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
