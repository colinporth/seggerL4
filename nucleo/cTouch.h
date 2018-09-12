// cTouch.h
// pa0:e5v  pa1:y-  pa2:y+  pa3:x+  pa4:x-
#pragma once
//{{{  includes
#include "cmsis_os.h"
#include "../common/utils.h"
#include "../common/cPointRect.h"
#include "../common/stm32l4xx_nucleo_144.h"
#include "../common/cFilter.h"
//}}}

class cTouch {
public:
  enum eState { ePress, eReadX, eReadY };

  //{{{
  void init() {

    //vSemaphoreCreateBinary (mConvertedSem);
    vRefIntValueCalibrated = *((uint16_t*)VREFINT_CAL_ADDR);

    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC_CONFIG (RCC_ADCCLKSOURCE_SYSCLK);
    //{{{  adc config
    mAdcHandle.Instance = ADC1;
    mAdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    mAdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    mAdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    mAdcHandle.Init.ScanConvMode          = ENABLE;
    mAdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV; // ADC_EOC_SEQ_CONV;
    mAdcHandle.Init.LowPowerAutoWait      = DISABLE;
    mAdcHandle.Init.ContinuousConvMode    = DISABLE;
    mAdcHandle.Init.NbrOfConversion       = 1;
    mAdcHandle.Init.DiscontinuousConvMode = ENABLE;
    mAdcHandle.Init.NbrOfDiscConversion   = 1;
    mAdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    mAdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    mAdcHandle.Init.DMAContinuousRequests = DISABLE;
    mAdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    mAdcHandle.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init (&mAdcHandle) != HAL_OK)
      printf ("HAL_ADC_Init failed\n");

    HAL_NVIC_SetPriority (ADC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (ADC1_IRQn);
    //}}}

    selectConversion (ePress);
    if (HAL_ADCEx_Calibration_Start (&mAdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
      printf ("HAL_ADCEx_Calibration_Start failed\n");
    }
  //}}}

  eState getState() { return mState; }
  bool getPressed() { return mPressed; }
  cPointF getTouch() { return mTouch; }

  uint16_t getValueX() { return xValue; }
  uint16_t getValueY() { return yValue; }

  //{{{
  void start() {
    mConverted = false;
    HAL_ADC_Start_IT (&mAdcHandle);
    }
  //}}}
  //{{{
  void wait() {
    //if (!xSemaphoreTake (mConvertedSem, 5000))
    //  printf ("appThread mConvertedSem take fail\n");
    while (!mConverted) {}
    }
  //}}}

  //{{{
  void irqHandler() {
    HAL_ADC_IRQHandler (&mAdcHandle);
    }
  //}}}
  //{{{
  void converted() {

    uint16_t value = HAL_ADC_GetValue (&mAdcHandle);

    switch (mState) {
      case ePress:
        if (value < 3000)
          selectConversion (eReadX);
        else {
          mPressed = false;
          xFilter.clear();
          yFilter.clear();
          }
        break;

      case eReadX :
        xValue = xFilter.getAverageMedianValue (value);
        selectConversion (eReadY);
        break;

      case eReadY:
        //  read Y, select press
        yValue = yFilter.getAverageMedianValue (value);
        mTouch.y = ((yValue - 450) * 480) / float(3800 - 450);
        mTouch.x = ((xValue - 450) * 320) / float(3700 - 450);
        mPressed = true;

        selectConversion (ePress);
        break;
        }

    //portBASE_TYPE taskWoken = pdFALSE;
    //if (xSemaphoreGiveFromISR (mConvertedSem, &taskWoken) == pdTRUE)
    //  portEND_SWITCHING_ISR (taskWoken);
    mConverted = true;
    }
  //}}}

  //{{{  rank1 vrefint
  //sConfig.Channel = ADC_CHANNEL_VREFINT;
  //sConfig.Rank = ADC_REGULAR_RANK_1;
  //if (HAL_ADC_ConfigChannel (&mAdcHandle, &sConfig) != HAL_OK)
    //printf ("HAL_ADC_Init failed\n");
  //}}}
  //{{{  rank2 vbat
  //sConfig.Channel = ADC_CHANNEL_VBAT;
  //sConfig.Rank = ADC_REGULAR_RANK_2;
  //if (HAL_ADC_ConfigChannel (&mAdcHandle, &sConfig) != HAL_OK)
    //printf ("HAL_ADC_Init failed\n");
  //}}}
  //{{{  rank3 e5v > pa0
  //// pa0 - e5v - adc channel 5
  //GPIO_InitStruct.Pin = GPIO_PIN_0;
  //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  //sConfig.Channel = ADC_CHANNEL_5;
  //sConfig.Rank = ADC_REGULAR_RANK_2;
  //if (HAL_ADC_ConfigChannel (&mAdcHandle, &sConfig) != HAL_OK)
    //printf ("HAL_ADC_Init failed\n");
  //}}}

private:
  //{{{
  void selectConversion (eState state) {

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_2;

    ADC_ChannelConfTypeDef sConfig;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    switch (state) {
      case ePress:
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
      case eReadX:
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
      case eReadY:
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

    if (HAL_ADC_ConfigChannel (&mAdcHandle, &sConfig) != HAL_OK)
      printf ("HAL_ADC_Init failed\n");

    mState = state;
    }
  //}}}

  ADC_HandleTypeDef mAdcHandle;

  volatile eState mState = eReadX;
  volatile bool mConverted = false;
  volatile uint16_t xValue = 0;
  volatile uint16_t yValue = 0;

  cFilter xFilter;
  cFilter yFilter;
  cPointF mTouch;
  volatile bool mPressed = false;

  uint16_t vRefIntValueCalibrated = 0;
  //volatile uint16_t vRefIntValue = 0;
  //volatile uint16_t vBatValue = 0;
  //volatile uint16_t v5vValue = 0;
  //SemaphoreHandle_t mConvertedSem;
  };
