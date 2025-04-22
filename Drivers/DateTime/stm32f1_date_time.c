/*
 * date_time.c
 *
 *  Created on: Dec 31, 2023
 *      Author: marrob
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1_date_time.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static RTC_HandleTypeDef *_hrtc;
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

void DateTime_Init(RTC_HandleTypeDef *hrtc)
{
  _hrtc = hrtc;
  /*
   * A dátumont az F1-es család nem kezeli.
   * Visszatöltés után már atumatikusan növeli.
   */
  hrtc->DateToUpdate.Year = HAL_RTCEx_BKUPRead(hrtc, RTC_BKP_DR1);
  hrtc->DateToUpdate.Month = HAL_RTCEx_BKUPRead(hrtc, RTC_BKP_DR2);
  hrtc->DateToUpdate.Date = HAL_RTCEx_BKUPRead(hrtc, RTC_BKP_DR3);
}

struct tm DateTime_Get(void)
{
  static struct tm tm_info;
  tm_info.tm_year = 1985;
  tm_info.tm_mon = 1;
  tm_info.tm_mday = 23;
  tm_info.tm_hour = 0;
  tm_info.tm_min = 0;
  tm_info.tm_sec = 0;

  RTC_TimeTypeDef time;
  if(HAL_RTC_GetTime(_hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    return tm_info;

  RTC_DateTypeDef date;
  if(HAL_RTC_GetDate(_hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
    return tm_info;

  tm_info.tm_year = 2000 - 1900 + date.Year ;
  tm_info.tm_mon = date.Month - 1;
  tm_info.tm_mday = date.Date;
  tm_info.tm_hour = time.Hours;
  tm_info.tm_min = time.Minutes;
  tm_info.tm_sec = time.Seconds;

  HAL_RTCEx_BKUPWrite(_hrtc, RTC_BKP_DR1, date.Year);
  HAL_RTCEx_BKUPWrite(_hrtc, RTC_BKP_DR2, date.Month);
  HAL_RTCEx_BKUPWrite(_hrtc, RTC_BKP_DR3, date.Date);

  return tm_info;
}

HAL_StatusTypeDef DateTime_Set(struct tm *tm_info)
{
  RTC_DateTypeDef date;
  date.Year = tm_info->tm_year - 100;
  date.Month = tm_info->tm_mon + 1;
  date.Date = tm_info->tm_mday;
  date.WeekDay = 1;
  if(HAL_RTC_SetDate(_hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
    return HAL_ERROR;

  RTC_TimeTypeDef time;
  time.Hours = tm_info->tm_hour;
  time.Minutes = tm_info->tm_min;
  time.Seconds = tm_info->tm_sec;

  if(HAL_RTC_SetTime(_hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
    return HAL_ERROR;

  /*
   * Az F1 család nem kezeli miden dátum modositáskor
   * elmentem a backupregisterekbe az aktuálsi állapotot
   * induláskor meg, 1x kiolvasom.
   */
  HAL_RTCEx_BKUPWrite(_hrtc, RTC_BKP_DR1, date.Year);
  HAL_RTCEx_BKUPWrite(_hrtc, RTC_BKP_DR2, date.Month);
  HAL_RTCEx_BKUPWrite(_hrtc, RTC_BKP_DR3, date.Date);
  return HAL_OK;
}


/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
