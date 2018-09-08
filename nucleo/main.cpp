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

// vars
cLcd* lcd = nullptr;
cRtc* rtc = nullptr;

ADC_HandleTypeDef AdcHandle;
//void DMA2_Stream0_IRQHandler() { HAL_DMA_IRQHandler (AdcHandle.DMA_Handle); }

//{{{
void adcInit() {
// pa0  5v
// pa1  y-
// pa2  x+
// pa3  y+
// pa4  x-

  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC_CONFIG (RCC_ADCCLKSOURCE_SYSCLK);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // resetLo
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // resetHi

  AdcHandle.Instance = ADC1;
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;      // Asynchronous clock mode, input ADC clock not divided
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;        // 12-bit resolution for converted data
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;       // Right-alignment for converted data
  AdcHandle.Init.ScanConvMode          = DISABLE;                   // Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1)
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;       // EOC flag picked-up to indicate conversion end
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   // Auto-delayed conversion feature disabled
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                    // Continuous mode disabled to have only 1 conversion at each conversion trig
  AdcHandle.Init.NbrOfConversion       = 1;                         // Parameter discarded because sequencer is disabled
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   // Parameter discarded because sequencer is disabled
  AdcHandle.Init.NbrOfDiscConversion   = 1;                         // Parameter discarded because sequencer is disabled
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        // Software start to trig the 1st conversion manually, without external event
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; // Parameter discarded because software trigger chosen
  AdcHandle.Init.DMAContinuousRequests = DISABLE;                   // DMA one-shot mode selected (not applied to this example)
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;  // DR register is overwritten with the last conversion result in case of overrun
  AdcHandle.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init (&AdcHandle) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  ADC_ChannelConfTypeDef sConfig;
  //sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Channel = ADC_CHANNEL_7;
  //sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  //sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;       // Rank of sampled channel number ADCx_CHANNEL
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5; // Sampling time (number of clock cycles unit)
  sConfig.SingleDiff = ADC_SINGLE_ENDED;  // Single-ended input channel
  sConfig.OffsetNumber = ADC_OFFSET_NONE; // No offset subtraction
  sConfig.Offset = 0; // Parameter discarded because offset correction is disabled
  if (HAL_ADC_ConfigChannel (&AdcHandle, &sConfig) != HAL_OK)
    printf ("HAL_ADC_Init failed\n");

  if (HAL_ADCEx_Calibration_Start (&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
    printf ("HAL_ADCEx_Calibration_Start failed\n");

  if (HAL_ADC_Start (&AdcHandle) != HAL_OK)
    printf ("HAL_ADC_Start failed\n");
  }
//}}}

//{{{
void uiThread (void* arg) {

  int avVal = 0;
  int convertedValue = 0;

  cPointF centre = cPointF (160.f, 240.f);
  float radius = 20.f;
  const float maxRadius = 160.f;

  lcd->tftInit();
  lcd->display (70);

  while (true) {
    if (HAL_ADC_PollForConversion (&AdcHandle, 10) != HAL_OK)
      printf ("HAL_ADC_PollForConversion failed\n");
    else {
      convertedValue = HAL_ADC_GetValue (&AdcHandle);

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
      lcd->cLcd::text (kWhite, 30, rtc->getClockTimeDateString(), cRect (0, 426, 320, 480));
      lcd->cLcd::text (kWhite, 26, dec(convertedValue), cRect (0, 20, 320, 46));
      lcd->present();

      if (radius < maxRadius) {
        radius *= 1.1f;
        if (radius > maxRadius)
          radius = maxRadius;
        lcd->change();
        }
      }
    else
      vTaskDelay (1);

      //if (avVal == 0)
      //  avVal = ConvertedValue;
      //lse
      //  avVal = (avVal*50 + ConvertedValue) / 51;
      //float kScale = ((3.3f * (39.f + 27.f) / 39.f) / 4096.f) * 1000;
      //float kScale = (3.3f / 4096.f) * 1000;
      //float kScale = 3.f * (3.3f / 4096.f) * 1000;
      }
    }
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

  adcInit();

  TaskHandle_t uiHandle;
  xTaskCreate ((TaskFunction_t)uiThread, "ui", 4096, 0, 4, &uiHandle);
  vTaskStartScheduler();

  return 0;
  }
