// main.cpp
//{{{  includes
#include <algorithm>
#include <string>
#include <ctype.h>

#include "cmsis_os.h"

#include "../common/stm32l4xx_nucleo_144.h"
#include "../common/heap.h"
#include "../common/cRtc.h"
#include "../common/cTraceVec.h"

#include "cLcd.h"

using namespace std;
//}}}
const string kHello = "smallLcd " + string(__TIME__) + " " + string(__DATE__);

// vars
cLcd* lcd = nullptr;
cRtc* rtc = nullptr;
cTraceVec mTraceVec;

ADC_HandleTypeDef AdcHandle;
extern "C" { void ADC1_IRQHandler() { HAL_ADC_IRQHandler (&AdcHandle); } }

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* AdcHandle) {
  auto value = HAL_ADC_GetValue (AdcHandle);
  mTraceVec.addSample (0, value);
  }

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
      mTraceVec.draw (lcd, 40, 400);

      lcd->cLcd::text (kWhite, 30, rtc->getClockTimeDateString(), cRect (0, 426, 320, 480));
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
void adcThread (void* arg) {
// pa0  5v
// pa1  yDown
// pa2  yUp
// pa3  xRight
// pa4  xLeft

  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC_CONFIG (RCC_ADCCLKSOURCE_SYSCLK);

  ADC_ChannelConfTypeDef sConfig;
  //{{{  touch
  // yDown pullup adc input
  //sConfig.Channel = ADC_CHANNEL_6;  //  PA1 x
  //GPIO_InitTypeDef GPIO_InitStruct;
  //GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  //GPIO_InitStruct.Pull = GPIO_PULLUP;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // xLeft, yUp hiZ
  //GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // xleft gnd
  //GPIO_InitStruct.Pin = GPIO_PIN_4;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  //HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // resetLo
  //}}}
  //{{{  x
  // pa2 yUp - adc
  sConfig.Channel = ADC_CHANNEL_7;
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // pa1 yDown hiZ
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // pa4 xLeft 0v
  // pa3 xRight 3v
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // resetLo
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // resetHi
  //}}}
  //{{{  y
  //// xRight - adc
  //sConfig.Channel = ADC_CHANNEL_9;  //  PA3 y
  //GPIO_InitTypeDef GPIO_InitStruct;
  //GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4;
  //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  //// xLeft hiZ
  //GPIO_InitStruct.Pin = GPIO_PIN_3;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  //// yDown 0v, yUp 3v
  //GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  //HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // resetLo
  //HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // resetHi
  //}}}

  //{{{  adc config
  AdcHandle.Instance = ADC1;
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ENABLE;
  AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
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

//  channel config
  //sConfig.Channel = ADC_CHANNEL_VBAT;
  //sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  sConfig.Rank = 1;
  sConfig.Channel = ADC_CHANNEL_5;  //  PA0 5v
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  //if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
  //  printf ("HAL_ADC_Init failed\n");

  if (HAL_ADCEx_Calibration_Start (&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
    printf ("HAL_ADCEx_Calibration_Start failed\n");

  while (true) {
    HAL_ADC_Start_IT (&AdcHandle);
    vTaskDelay (5);
    }
  //float kScale = ((3.3f * (39.f + 27.f) / 39.f) / 4096.f) * 1000;
  //float kScale = (3.3f / 4096.f) * 1000;
  //float kScale = 3.f * (3.3f / 4096.f) * 1000;
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

  mTraceVec.addTrace (320, 1, 1);
  mTraceVec.addTrace (320, 1, 1);

  TaskHandle_t uiHandle;
  xTaskCreate ((TaskFunction_t)uiThread, "ui", 4096, 0, 4, &uiHandle);
  TaskHandle_t adcHandle;
  xTaskCreate ((TaskFunction_t)adcThread, "adc", 1024, 0, 3, &adcHandle);
  vTaskStartScheduler();

  return 0;
  }
