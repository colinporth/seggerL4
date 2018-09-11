// main.cpp
//{{{  includes
#include <algorithm>
#include <string>
#include <ctype.h>

#include "cmsis_os.h"

#include "../common/stm32l4xx_nucleo_144.h"
#include "../common/heap.h"
#include "../common/cRtc.h"

#include "cLcd.h"

using namespace std;
//}}}
const string kHello = "smallLcd " + string(__TIME__) + " " + string(__DATE__);
//{{{
class cFilter {
public:
  uint16_t getMedianValue() {
    memcpy (mSorted, mValues, kMaxIndex * 2);
    return quickSelect (0, mMaxIndex-1, mMaxIndex/2);
    }

  uint16_t getAverageMedianValue() {
    memcpy (mSorted, mValues, kMaxIndex * 2);
    uint16_t median =  quickSelect (0, mMaxIndex-1, mMaxIndex/2);
    uint16_t medianBefore = mMaxIndex > 5 ? quickSelect (0, mMaxIndex-1, (mMaxIndex/2) -1) : median;
    uint16_t medianAfter = mMaxIndex > 5 ? quickSelect (0, mMaxIndex-1, (mMaxIndex/2) + 1) : median;

    return (median + medianBefore + medianAfter) / 3;
    }

  uint16_t getAverageValue() {
    uint32_t sum = 0;
    for (int i = 0; i < mMaxIndex; i++)
      sum += mValues[i];
    return sum / mMaxIndex;
    }

  void addValue (uint16_t value) {
    mValues[mCurIndex] = value;
    mCurIndex = (mCurIndex + 1) % kMaxIndex;
    if (mMaxIndex < kMaxIndex)
      mMaxIndex++;
    }

  //{{{
  void clear() {
    mCurIndex = 0;
    mMaxIndex = 0;
    }
  //}}}

private:
  static const int kMaxIndex = 9;

  //{{{
  uint8_t partition (uint8_t p, uint8_t r) {

    uint16_t pivot = mSorted[r];
    while (p < r) {
      while (mSorted[p] < pivot)
        p++;

      while (mSorted[r] > pivot)
        r--;

      if (mSorted[p] == mSorted[r])
        p++;
      else if (p < r) {
        uint16_t swap = mSorted[p];
        mSorted[p] = mSorted[r];
        mSorted[r] = swap;
        }
      }

    return r;
    }
  //}}}
  //{{{
  uint16_t quickSelect (uint8_t p, uint8_t r, uint8_t k) {

    if (p == r)
      return mSorted[p];

    uint8_t j = partition (p, r);
    uint8_t length = j - p + 1;
    if (length == k)
      return mSorted[j];
    else if (k < length)
      return quickSelect (p, j - 1, k);
    else
      return quickSelect (j + 1, r, k - length);
    }
  //}}}

  uint16_t mValues[kMaxIndex];
  uint16_t mSorted[kMaxIndex];
  uint8_t mCurIndex = 0;
  uint32_t mMaxIndex = 0;
  };
//}}}

// vars
cLcd* lcd = nullptr;
cRtc* rtc = nullptr;

uint16_t vRefIntValueCalibrated = 0;

enum eReadState { ePress, eReadX, eReadY };

volatile eReadState mReadState = eReadX;
volatile bool mConverted = false;
volatile uint32_t mConversions = 0;
volatile uint16_t vRefIntValue = 0;
volatile uint16_t vBatValue = 0;
volatile uint16_t v5vValue = 0;
volatile uint16_t xValue = 0;
volatile uint16_t yValue = 0;

volatile bool mPressed = false;
cPointF mTouch;
cFilter xFilter;
cFilter yFilter;

ADC_HandleTypeDef AdcHandle;
extern "C" { void ADC1_IRQHandler() { HAL_ADC_IRQHandler (&AdcHandle); } }

//{{{  rank1 vrefint
//sConfig.Channel = ADC_CHANNEL_VREFINT;
//sConfig.Rank = ADC_REGULAR_RANK_1;
//if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
  //printf ("HAL_ADC_Init failed\n");
//}}}
//{{{  rank2 vbat
//sConfig.Channel = ADC_CHANNEL_VBAT;
//sConfig.Rank = ADC_REGULAR_RANK_2;
//if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
  //printf ("HAL_ADC_Init failed\n");
//}}}
//{{{  rank3 e5v > pa0
//// pa0 - e5v - adc channel 5
//GPIO_InitStruct.Pin = GPIO_PIN_0;
//GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
//HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

//sConfig.Channel = ADC_CHANNEL_5;
//sConfig.Rank = ADC_REGULAR_RANK_2;
//if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
  //printf ("HAL_ADC_Init failed\n");
//}}}

//{{{
void clockConfig() {
// System Clock source = PLL (MSI)
// SYSCLK(Hz)          = 120000000
// HCLK(Hz)            = 120000000
// AHB Prescaler       = 1
// APB1 Prescaler      = 1
// APB2 Prescaler      = 1
// MSI Frequency(Hz)   = 4000000
// PLL_M               = 1
// PLL_N               = 60
// PLL_Q               = 2
// PLL_R               = 2
// PLL_P               = 7
// Flash Latency(WS)   = 5

  // Enable voltage range 1 boost mode for frequency above 80 Mhz
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  __HAL_RCC_PWR_CLK_DISABLE();

  // Enable MSI Oscillator and activate PLL with MSI as source
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    while (1);

  // avoid undershoot due to maximum frequency, select PLL system clock AHB prescaler divider 2 as first step
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    while (1);

  // AHB prescaler divider at 1 as second step
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    while (1);
  }
//}}}
//{{{
void uiThread (void* arg) {

  cPointF centre = cPointF (160.f, 240.f);
  float radius = 20.f;
  const float maxRadius = 160.f;

  lcd->tftInit();
  lcd->display (70);

  while (true) {
    //if (lcd->isChanged() || (lcd->getPresentTime() >= 1000)) {
    if (true) {
      lcd->start();
      lcd->clear (kBlack);
      lcd->setShowInfo (BSP_PB_GetState (BUTTON_KEY) == 0);

      //float vRef = (3.f * vRefIntValueCalibrated) / vRefIntValue;
      //float v5v = v5vValue * ((vRef * (39.f + 27.f) / 39.f) / 4096.f);
      //float vBat = vBatValue * ((vRef * 3.f) / 4096.f);
      //lcd->text (kWhite, 20, "vBat  " + dec (vBatValue) + " " +
      //                                  dec (int (vBat),1,' ') + "." +
      //                                  dec (int (vBat * 100) % 100, 2,'0'), cRect (0, 40, 320, 60));
      //lcd->text (kWhite, 20, "vRef " + dec (vRefIntValue) + " " +
      //                                 dec (int (vRef),1,' ') + "." +
      //                                 dec (int (vRef * 100) % 100, 2,'0') + " " +
      //                                 dec (v5vValue) + " " +
      //                                 dec (int (v5v),1,' ') + "." +
      //                                 dec (int (v5v * 100) % 100, 2,'0'), cRect (0, 20, 320, 40));
      lcd->drawInfo();
      //{{{  get clock
      float hourA;
      float minuteA;
      float secondA;
      float subSecondA;
      rtc->getClockAngles (hourA, minuteA, secondA, subSecondA);
      //}}}
      //{{{  render clock
      int steps = 64;
      float width = 4.f;
      lcd->aEllipse (centre, cPointF(radius-width, radius), steps);
      lcd->aRender (sRgba (128,128,128, 192), false);
      lcd->aEllipseOutline (centre, cPointF(radius, radius), width, steps);
      lcd->aRender (sRgba (180,180,0, 255), false);

      float handWidth = radius > 60.f ? radius / 20.f : 3.f;
      float hourR = radius * 0.75f;
      lcd->aPointedLine (centre, centre + cPointF (hourR * sin (hourA), hourR * cos (hourA)), handWidth);
      float minuteR = radius * 0.9f;
      lcd->aPointedLine (centre, centre + cPointF (minuteR * sin (minuteA), minuteR * cos (minuteA)), handWidth);
      lcd->aRender (kWhite);

      float secondR = radius * 0.95f;
      lcd->aPointedLine (centre, centre + cPointF (secondR * sin (secondA), secondR * cos (secondA)), handWidth);
      lcd->aRender (sRgba (255,0,0, 180));

      float subSecondR = radius * 0.95f;
      lcd->aPointedLine (centre, centre + cPointF (minuteR * sin (subSecondA), minuteR * cos (subSecondA)), 3.f);
      lcd->aRender (sRgba (255,255,0, 128));
      //}}}

      if (mPressed) {
        lcd->text (kWhite, 20,
                   dec (xValue,4,' ') + ":" + dec (yValue, 4, ' ') + " " + dec(mConversions),
                   cRect (0, 40, 320, 60));
        int steps = 32;
        lcd->aEllipse (mTouch, cPointF (16.f, 16.f), steps);
        lcd->aRender (kGreen, false);
        }

      lcd->text (kWhite, 30, rtc->getClockTimeDateString(), cRect (0, 426, 320, 480));
      lcd->present();

      if (radius < maxRadius) {
        radius *= 1.1f;
        if (radius > maxRadius)
          radius = maxRadius;
        lcd->change();
        }
      }
    vTaskDelay (10);
    }
  }
//}}}
//{{{
void appThread (void* arg) {
// pa0:e5v  pa1:y-  pa2:y+  pa3:x+  pa4:x-

  vRefIntValueCalibrated = *((uint16_t*)VREFINT_CAL_ADDR);

  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC_CONFIG (RCC_ADCCLKSOURCE_SYSCLK);
  //{{{  adc config
  AdcHandle.Instance = ADC1;
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ENABLE;
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV; // ADC_EOC_SEQ_CONV;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;
  AdcHandle.Init.NbrOfDiscConversion   = 1;
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  AdcHandle.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init (&AdcHandle) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  HAL_NVIC_SetPriority (ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC1_IRQn);
  //}}}

  mReadState = ePress;

  // look for press
  // pa2:y+ hiZ
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  //  pa3:x+ > 0v, pa4:x- > 0v
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

  // pa1:y- - adcChannel6 rank1
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  if (HAL_ADCEx_Calibration_Start (&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
    printf ("HAL_ADCEx_Calibration_Start failed\n");

  while (true) {
    mConverted = false;
    HAL_ADC_Start_IT (&AdcHandle);
    while (!mConverted)
      vTaskDelay (1);
    }
  }
//}}}
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* adcHandle) {

  uint16_t value = HAL_ADC_GetValue (adcHandle);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  switch (mReadState) {
    case ePress:
      printf ("ePress %d\n", value);
      if (value < 2500) {
        //{{{  select readX
        // pa2:y+ > adcChannel7 - select xValue
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
        sConfig.Channel = ADC_CHANNEL_7;

        // pa1:y- > hiZ
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

        // pa4:x- > 0v, pa3:x+ > 3v
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
        HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

        if (HAL_ADC_ConfigChannel (adcHandle, &sConfig) != HAL_OK)
          printf ("HAL_ADC_Init failed\n");

        mReadState = eReadX;
        }
        //}}}
      else {
        mPressed = false;
        xFilter.clear();
        yFilter.clear();
        }
      break;

    case eReadX :
      //{{{  read x, select readY
      xFilter.addValue (value);
      xValue = xFilter.getAverageMedianValue();
      mTouch.x = ((xValue - 300) * 320) / float(3700 - 300);

      // pa4:x+ > adcChannel9 rank1 - select yValue
      GPIO_InitStruct.Pin = GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
      sConfig.Channel = ADC_CHANNEL_9;

      // pa3:x > hiZ
      GPIO_InitStruct.Pin = GPIO_PIN_3;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

      // pa1:y- > 0v, pa2:y+ > 3v
      GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

      if (HAL_ADC_ConfigChannel (adcHandle, &sConfig) != HAL_OK)
        printf ("HAL_ADC_Init failed\n");

      mReadState = eReadY;
      //}}}
      break;

    case eReadY:
      //{{{  read Y, select press
      yFilter.addValue (value);
      yValue = yFilter.getAverageMedianValue();
      mTouch.y = ((yValue - 350) * 480) / float(3800 - 350);
      mPressed = true;

      // pa1:y- - adcChannel6 rank1
      GPIO_InitStruct.Pin = GPIO_PIN_1;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

      // pa2:y+ hiZ
      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

      //  pa3:x+ > 0v, pa4:x- > 0v
      GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

      sConfig.Channel = ADC_CHANNEL_6;
      if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
        printf ("HAL_ADC_Init failed\n");

      mReadState = ePress;
      //}}}
      break;
      }

  mConversions++;
  mConverted = true;
  }
//}}}

int main() {
  HAL_Init();
  clockConfig();

  printf ("%s\n", kHello.c_str());

  BSP_LED_Init (LED_RED);
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);

  rtc = new cRtc();
  rtc->init();

  lcd = new cLcd();
  lcd->init (kHello);

  TaskHandle_t uiHandle;
  xTaskCreate ((TaskFunction_t)uiThread, "ui", 4096, 0, 4, &uiHandle);
  TaskHandle_t appHandle;
  xTaskCreate ((TaskFunction_t)appThread, "app", 1024, 0, 4, &appHandle);
  vTaskStartScheduler();

  return 0;
  }
