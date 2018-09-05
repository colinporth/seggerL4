// cRtc.h
#pragma once
#include <stdint.h>
#include <string>
#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "../common/utils.h"

class cRtc {
public:
  void init();

  bool getClockSet() { return mClockSet; }
  void getClockAngles (float& hours, float& minutes, float& seconds, float& subSeconds);
  std::string getClockTimeString() { return mDateTime.getTimeString(); }
  std::string getClockTimeDateString() { return mDateTime.getTimeDateString(); }
  std::string getBuildTimeDateString() { return mBuildTime + " "  + mBuildDate; }

private:
  uint8_t getBcdFromByte (uint8_t byte) { return ((byte / 10) << 4) | (byte % 10); }
  uint8_t getByteFromBcd (uint8_t bcd) { return (((bcd & 0xF0) >> 4) * 10) + (bcd & 0x0F); }

  void loadDateTime();
  void saveDateTime();

  void writeProtectDisable();
  bool enterInitMode();
  bool waitForSynchro();
  void writeProtectEnable();

  const float kPi = 3.1415926f;
  const int kBuildSecs = 11;
  const std::string mBuildTime = __TIME__;
  const std::string mBuildDate = __DATE__;

  //{{{
  class cDateTime {
  public:
    cDateTime() {}
    //{{{
    cDateTime (const std::string& buildDateStr, const std::string& buildTimeStr) {

      // buildDateStr - dd:mmm:yyyy
      Date = ((buildDateStr[4] == ' ') ? 0 : buildDateStr[4] - 0x30) * 10 + (buildDateStr[5] -0x30);
      Year = (buildDateStr[9] - 0x30) * 10 + (buildDateStr[10] -0x30);

      Month = 0;
      for (int i = 0; i < 12; i++)
        if ((buildDateStr[0] == *kMonth[i]) && (buildDateStr[1] == *(kMonth[i]+1)) && (buildDateStr[2] == *(kMonth[i]+2))) {
          Month = i;
          break;
          }

      // buildTimeStr - hh:mm:ss
      Hours = (buildTimeStr[0]  - 0x30) * 10 + (buildTimeStr[1] -0x30);
      Minutes = (buildTimeStr[3] - 0x30) * 10 + (buildTimeStr[4] -0x30);
      Seconds = (buildTimeStr[6] - 0x30) * 10 + (buildTimeStr[7] -0x30);
      }
    //}}}

    //{{{
    uint32_t getValue() {
      return ((((Year*12 + Month)*31 + Date)*24 + Hours)*60 + Minutes)*60 + Seconds;
      }
    //}}}
    //{{{
    std::string getTimeString() {
      return dec(Hours,2) + ":" + dec(Minutes,2) + ":" + dec(Seconds,2);
             //dec(SubSeconds) + " " + dec(SecondFraction);
      }
    //}}}
    //{{{
    std::string getDateString() {
      return std::string(kMonth[Month]) + " " + dec(Date,2) + " " + dec(2000 + Year,4);
      }
    //}}}
    //{{{
    std::string getTimeDateString() {
      return dec(Hours,2) + ":" + dec(Minutes,2) + ":" + dec(Seconds,2) + " " +
             kMonth[Month] + " " + dec(Date,2) + " " + dec(2000 + Year,4);
             //dec(SubSeconds) + " " + dec(SecondFraction);
      }
    //}}}

    //{{{
    void setFromValue (uint32_t value) {
      TimeFormat = RTC_HOURFORMAT12_AM;
      DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      StoreOperation = RTC_STOREOPERATION_RESET;

      Seconds = value % 60;
      value /= 60;
      Minutes = value % 60;
      value /= 60;
      Hours = value % 24;
      value /= 24;
      Date = value % 31;
      value /= 31;
      Month = value % 12;
      value /= 12;
      Year = value;

      WeekDay = RTC_WEEKDAY_FRIDAY;  // wrong
      }
    //}}}

    uint8_t Year;
    uint8_t Month;
    uint8_t WeekDay;
    uint8_t Date;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    uint8_t TimeFormat;
    uint32_t SubSeconds;
    uint32_t SecondFraction;
    uint32_t DayLightSaving;
    uint32_t StoreOperation;

  private:
    const char* kMonth[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    };
  //}}}
  cDateTime mDateTime;

  RTC_HandleTypeDef mRtcHandle;
  bool mClockSet = false;
  };
