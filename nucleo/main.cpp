// main.cpp
//{{{  includes
#include <algorithm>
#include <string>
#include <ctype.h>

#include "cmsis_os.h"

#include "../common/stm32l4xx_nucleo_144.h"
#include "../common/heap.h"
#include "../common/cRtc.h"
#include "../common/cFilter.h"

#include "cLcd.h"

using namespace std;
//}}}
const string kHello = "smallLcd " + string(__TIME__) + " " + string(__DATE__);
//{{{  vars
cLcd* lcd = nullptr;
cRtc* rtc = nullptr;

uint16_t vRefIntValueCalibrated = 0;

enum eTouchState { eTouchPress, eTouchReadX, eTouchReadY };
volatile eTouchState mTouchState = eTouchReadX;

volatile uint16_t vRefIntValue = 0;
volatile uint16_t vBatValue = 0;
volatile uint16_t v5vValue = 0;

SemaphoreHandle_t mConvertedSem;
volatile bool mConverted = false;
volatile uint16_t xValue = 0;
volatile uint16_t yValue = 0;
volatile bool mPressed = false;
cPointF mTouch;
cFilter xFilter (9);
cFilter yFilter (9);

ADC_HandleTypeDef gAdcHandle;
//}}}
extern "C" { void ADC1_IRQHandler() { HAL_ADC_IRQHandler (&gAdcHandle); } }

//{{{  rank1 vrefint
//sConfig.Channel = ADC_CHANNEL_VREFINT;
//sConfig.Rank = ADC_REGULAR_RANK_1;
//if (HAL_ADC_ConfigChannel (&gAdcHandle, &sConfig) != HAL_OK)
  //printf ("HAL_ADC_Init failed\n");
//}}}
//{{{  rank2 vbat
//sConfig.Channel = ADC_CHANNEL_VBAT;
//sConfig.Rank = ADC_REGULAR_RANK_2;
//if (HAL_ADC_ConfigChannel (&gAdcHandle, &sConfig) != HAL_OK)
  //printf ("HAL_ADC_Init failed\n");
//}}}
//{{{  rank3 e5v > pa0
//// pa0 - e5v - adc channel 5
//GPIO_InitStruct.Pin = GPIO_PIN_0;
//GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
//HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

//sConfig.Channel = ADC_CHANNEL_5;
//sConfig.Rank = ADC_REGULAR_RANK_2;
//if (HAL_ADC_ConfigChannel (&gAdcHandle, &sConfig) != HAL_OK)
  //printf ("HAL_ADC_Init failed\n");
//}}}
//{{{
void selectTouch (eTouchState touchState) {

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_2;

  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  switch (touchState) {
    case eTouchPress:
      //{{{  pa3:x+ > 0v, pa4:x- > 0v  pa2:y+ > hiZ  pa1:y- - adcChannel6 rank1
      // pa2:y+ hiZ
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

      sConfig.Channel = ADC_CHANNEL_6;
      break;
      //}}}
    case eTouchReadX:
      //{{{  pa4:x- > 0v, pa3:x+ > 3v  pa1:y- > hiZ  pa2:y+ > adcChannel7 - select xValue
      // pa2:y+ > adcChannel7 - select xValue
      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

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

      sConfig.Channel = ADC_CHANNEL_7;

      break;
      //}}}
    case eTouchReadY:
      //{{{  pa1:y- > 0v, pa2:y+ > 3v  pa3:x+ > hiZ  pa4:x+ > adcChannel9 rank1 - select yValue
      // pa4:x+ > adcChannel9 rank1 - select yValue
      GPIO_InitStruct.Pin = GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
      HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

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

      sConfig.Channel = ADC_CHANNEL_9;

      break;
      //}}}
    }

  if (HAL_ADC_ConfigChannel (&gAdcHandle, &sConfig) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  mTouchState = touchState;
  }
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

  float radius = 20.f;
  const float maxRadius = 160.f;
  cPointF centre = cPointF (160.f, 240.f);

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

      if (mPressed) {
        //{{{  touch radius and string
        radius = (centre - mTouch).magnitude();
        lcd->aEllipse (mTouch, cPointF (16.f,16.f), 32);
        lcd->aRender (kYellow, false);

        lcd->text (kWhite, 22,
                   dec (xValue,4,' ') + "," + dec (yValue, 4, ' ') + " " +
                   dec (int(mTouch.x)) + "." + dec (int(mTouch.x * 10) % 10, 1,'0') + "," +
                   dec (int(mTouch.y)) + "." + dec (int(mTouch.y * 10) % 10, 1,'0'),
                   cRect (0, 20, 320, 42));
        }
        //}}}
      else {
        //{{{  anim radius
        radius *= 1.1f;
        if (radius > maxRadius)
          radius = maxRadius;
        }
        //}}}

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
      lcd->text (kWhite, 30, rtc->getClockTimeDateString(), cRect (0, 426, 320, 480));
      lcd->present();
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
  gAdcHandle.Instance = ADC1;
  gAdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  gAdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  gAdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  gAdcHandle.Init.ScanConvMode          = ENABLE;
  gAdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV; // ADC_EOC_SEQ_CONV;
  gAdcHandle.Init.LowPowerAutoWait      = DISABLE;
  gAdcHandle.Init.ContinuousConvMode    = DISABLE;
  gAdcHandle.Init.NbrOfConversion       = 1;
  gAdcHandle.Init.DiscontinuousConvMode = ENABLE;
  gAdcHandle.Init.NbrOfDiscConversion   = 1;
  gAdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  gAdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  gAdcHandle.Init.DMAContinuousRequests = DISABLE;
  gAdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  gAdcHandle.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init (&gAdcHandle) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  HAL_NVIC_SetPriority (ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (ADC1_IRQn);
  //}}}

  selectTouch (eTouchPress);
  if (HAL_ADCEx_Calibration_Start (&gAdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
    printf ("HAL_ADCEx_Calibration_Start failed\n");

  while (true) {
    mConverted = false;
    HAL_ADC_Start_IT (&gAdcHandle);
    //if (!xSemaphoreTake (mConvertedSem, 5000))
    //  printf ("appThread mConvertedSem take fail\n");
    while (!mConverted) {}

    if (mTouchState == eTouchPress)
      vTaskDelay (mPressed ? 1 : 50);
    }
  }
//}}}
//{{{
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* adcHandle) {

  uint16_t value = HAL_ADC_GetValue (adcHandle);

  switch (mTouchState) {
    case eTouchPress:
      printf ("ePress %d\n", value);
      if (value < 3000)
        selectTouch (eTouchReadX);
      else {
        mPressed = false;
        xFilter.clear();
        yFilter.clear();
        }
      break;

    case eTouchReadX :
      xValue = xFilter.getAverageMedianValue (value);
      selectTouch (eTouchReadY);
      break;

    case eTouchReadY:
      //  read Y, select press
      yValue = yFilter.getAverageMedianValue (value);
      mTouch.y = ((yValue - 450) * 480) / float(3800 - 450);
      mTouch.x = ((xValue - 450) * 320) / float(3700 - 450);
      mPressed = true;

      selectTouch (eTouchPress);
      break;
      }

  //portBASE_TYPE taskWoken = pdFALSE;
  //if (xSemaphoreGiveFromISR (mConvertedSem, &taskWoken) == pdTRUE)
  //  portEND_SWITCHING_ISR (taskWoken);

  mConverted = true;
  }
//}}}

int main() {
  HAL_Init();
  clockConfig();

  printf ("%s\n", kHello.c_str());

  BSP_LED_Init (LED_RED);
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);

  vSemaphoreCreateBinary (mConvertedSem);

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
