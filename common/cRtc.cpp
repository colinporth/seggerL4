// mRtc.cpp
//{{{  includes
#include "cRtc.h"

#include "stm32l4xx.h"

using namespace std;
//}}}

//{{{
void cRtc::init() {

  // Configue LSE as RTC clock source
  RCC_OscInitTypeDef rccOscInitStruct = {0};
  rccOscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  rccOscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  rccOscInitStruct.LSEState = RCC_LSE_ON;
  rccOscInitStruct.LSIState = RCC_LSI_OFF;
  if (HAL_RCC_OscConfig (&rccOscInitStruct))
    printf ("HAL_RCC_OscConfig failed\n");

  RCC_PeriphCLKInitTypeDef periphClkInitStruct = {0};
  periphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  periphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig (&periphClkInitStruct))
    printf ("HAL_RCCEx_PeriphCLKConfig failed\n");
  __HAL_RCC_RTC_ENABLE();

  // Configure LSE RTC prescaler and RTC data registers
  writeProtectDisable();
  if (enterInitMode()) {
    //{{{  init rtc
    RTC->CR = RTC_HOURFORMAT_24;
    RTC->PRER = (uint32_t)(0x00FF);
    RTC->PRER |= (uint32_t)(0x7F << 16U);

    // Exit Initialization mode
    RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

    // If CR_BYPSHAD bit = 0, wait for synchro else this check is not needed
    if ((RTC->CR & RTC_CR_BYPSHAD) == RESET)
      if (!waitForSynchro())
        printf ("timeout waiting for synchro\n");

    //RTC->TAFCR &= (uint32_t)~RTC_TAFCR_ALARMOUTTYPE;
    //RTC->TAFCR |= (uint32_t)RTC_OUTPUT_TYPE_OPENDRAIN;
    }
    //}}}
  writeProtectEnable();

  loadDateTime();
  uint32_t clockDateTimeValue = mDateTime.getValue();

  cDateTime buildDateTime (mBuildDate, mBuildTime);
  uint32_t buildDateTimeValue = buildDateTime.getValue();
  if (clockDateTimeValue < buildDateTimeValue + kBuildSecs) {
    // set clockDateTime from buildDateTime
    mDateTime.setFromValue (buildDateTimeValue + kBuildSecs);
    printf ("cRtc::init set clock < build %d < %d\n", clockDateTimeValue, buildDateTimeValue);
    saveDateTime();
    mClockSet = true;
    }
  }
//}}}

//{{{
void cRtc::getClockAngles (float& hours, float& minutes, float& seconds, float& subSeconds) {

  loadDateTime();

  hours = (1.f - ((mDateTime.Hours + (mDateTime.Minutes / 60.f)) / 6.f)) * kPi;
  minutes = (1.f - ((mDateTime.Minutes + (mDateTime.Seconds / 60.f))/ 30.f)) * kPi;
  seconds =  (1.f - (mDateTime.Seconds / 30.f)) * kPi;
  subSeconds =  (1.f - ((255 - mDateTime.SubSeconds) / 128.f)) * kPi;
  }
//}}}

//{{{
void cRtc::loadDateTime() {

  mDateTime.SubSeconds = RTC->SSR;
  mDateTime.SecondFraction = RTC->PRER & RTC_PRER_PREDIV_S;

  uint32_t tr = RTC->TR;
  mDateTime.TimeFormat = (tr & RTC_TR_PM) >> 16U;
  mDateTime.Hours = getByteFromBcd ((tr & (RTC_TR_HT | RTC_TR_HU)) >> 16U);
  mDateTime.Minutes = getByteFromBcd ((tr & (RTC_TR_MNT | RTC_TR_MNU)) >> 8U);
  mDateTime.Seconds = getByteFromBcd (tr & (RTC_TR_ST | RTC_TR_SU));

  uint32_t dr = RTC->DR;
  mDateTime.Year = getByteFromBcd ((dr & (RTC_DR_YT | RTC_DR_YU)) >> 16U);
  mDateTime.WeekDay = (dr & RTC_DR_WDU) >> 13U;
  mDateTime.Month = getByteFromBcd ((dr & (RTC_DR_MT | RTC_DR_MU)) >> 8U);
  mDateTime.Date = getByteFromBcd (dr & (RTC_DR_DT | RTC_DR_DU));
  }
//}}}
//{{{
void cRtc::saveDateTime() {

  writeProtectDisable();
  if (enterInitMode()) {
    if ((RTC->CR & RTC_CR_FMT) == (uint32_t)RESET)
      mDateTime.TimeFormat = 0x00U;

    if ((mDateTime.Month & 0x10U) == 0x10U)
      mDateTime.Month = (uint8_t)((mDateTime.Month & (uint8_t)~(0x10U)) + (uint8_t)0x0AU);

    // Set the RTC_DR register
    uint32_t tmp = (getBcdFromByte (mDateTime.Year) << 16) |
                   (getBcdFromByte (mDateTime.Month) << 8) |
                    getBcdFromByte (mDateTime.Date) |
                   (mDateTime.WeekDay << 13);
    RTC->DR = (uint32_t)(tmp & RTC_DR_RESERVED_MASK);

    // Set the RTC_TR register
    tmp = ((getBcdFromByte (mDateTime.Hours) << 16) |
           (getBcdFromByte (mDateTime.Minutes) << 8) |
            getBcdFromByte (mDateTime.Seconds) |
          (mDateTime.TimeFormat) << 16);
    RTC->TR = (uint32_t)(tmp & RTC_TR_RESERVED_MASK);

    // Clear the bits to be configured
    RTC->CR &= (uint32_t)~RTC_CR_BCK;

    // Configure the RTC_CR register
    RTC->CR |= mDateTime.DayLightSaving | mDateTime.StoreOperation;

    // Exit Initialization mode
    RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

    if ((RTC->CR & RTC_CR_BYPSHAD) == RESET)
      if (!waitForSynchro())
        printf ("setDateTime - timeout waiting for synchro\n");
    }

  writeProtectEnable();
  }
//}}}

//{{{
void cRtc::writeProtectDisable() {
  RTC->WPR = 0xCAU;
  RTC->WPR = 0x53U;
  }
//}}}
//{{{
bool cRtc::enterInitMode() {

  // Check if the Initialization mode is set
  if ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET) {
    // Set the Initialization mode
    RTC->ISR = (uint32_t)RTC_INIT_MASK;

    /* Get tick */
    uint32_t tickstart = HAL_GetTick();

    // Wait till RTC is in INIT state and if Time out is reached exit
    while ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
      if ((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
        return false;
    }

  return true;
  }
//}}}
//{{{
bool cRtc::waitForSynchro() {

  // Clear RSF flag
  RTC->ISR &= (uint32_t)RTC_RSF_MASK;

  uint32_t tickstart = HAL_GetTick();

  // Wait the registers to be synchronised
  while ((RTC->ISR & RTC_ISR_RSF) == (uint32_t)RESET)
    if ((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
      return false;

  return true;
  }
//}}}
//{{{
void cRtc::writeProtectEnable() {
  RTC->WPR = 0xFFU;
  }
//}}}
