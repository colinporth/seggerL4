// main.cpp
//{{{  includes
#include <algorithm>
#include <string>
#include <ctype.h>

#include "cmsis_os.h"

#include "../common/stm32l4xx_nucleo_144.h"
#include "../common/heap.h"
#include "../common/cRtc.h"
#include "cTouch.h"

#include "cLcd.h"

using namespace std;
//}}}
const string kHello = "smallLcd " + string(__TIME__) + " " + string(__DATE__);

//{{{  vars
cLcd* gLcd = nullptr;
cRtc* gRtc = nullptr;
cTouch* gTouch = nullptr;

float mRadius = 160.f;
cPointF mCentre = cPointF (160.f, 240.f);

enum eMove { eNotPressed, ePressed, eMoveCentre, eMoveRadius};
eMove mMove = eNotPressed;
//}}}
extern "C" { void ADC1_IRQHandler() { gTouch->irqHandler(); } }
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* adcHandle) { gTouch->converted(); }

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

  gLcd->tftInit();
  gLcd->display (70);

  while (true) {
    if (gLcd->isChanged() || (gLcd->getPresentTime() >= 1000)) {
      gLcd->start();
      gLcd->clear (kBlack);
      gLcd->setShowInfo (BSP_PB_GetState (BUTTON_KEY) == 0);

      //float vRef = (3.f * vRefIntValueCalibrated) / vRefIntValue;
      //float v5v = v5vValue * ((vRef * (39.f + 27.f) / 39.f) / 4096.f);
      //float vBat = vBatValue * ((vRef * 3.f) / 4096.f);
      //gLcd->text (kWhite, 20, "vBat  " + dec (vBatValue) + " " +
      //                                  dec (int (vBat),1,' ') + "." +
      //                                  dec (int (vBat * 100) % 100, 2,'0'), cRect (0, 40, 320, 60));
      //gLcd->text (kWhite, 20, "vRef " + dec (vRefIntValue) + " " +
      //                                 dec (int (vRef),1,' ') + "." +
      //                                 dec (int (vRef * 100) % 100, 2,'0') + " " +
      //                                 dec (v5vValue) + " " +
      //                                 dec (int (v5v),1,' ') + "." +
      //                                 dec (int (v5v * 100) % 100, 2,'0'), cRect (0, 20, 320, 40));
      gLcd->drawInfo();

      if (gTouch->getPressed()) {
        //{{{  touch radius and string
        gLcd->aEllipse (gTouch->getTouch(), cPointF (16.f,16.f), 32);
        gLcd->aRender (kYellow, false);

        gLcd->text (kWhite, 22,
                   dec (gTouch->getValueX(),4,' ') + "," + dec (gTouch->getValueY(), 4, ' ') + " " +
                   dec (int(gTouch->getTouch().x)) + "." + dec (int(gTouch->getTouch().x * 10) % 10, 1,'0') + "," +
                   dec (int(gTouch->getTouch().y)) + "." + dec (int(gTouch->getTouch().y * 10) % 10, 1,'0'),
                   cRect (0, 20, 320, 42));
        }
        //}}}

      //{{{  get clock
      float hourA;
      float minuteA;
      float secondA;
      float subSecondA;
      gRtc->getClockAngles (hourA, minuteA, secondA, subSecondA);
      //}}}
      //{{{  render clock
      auto r = mRadius;
      auto c = mCentre;
      gLcd->aEllipse (c, cPointF(r-4.f, r), 64);
      gLcd->aRender (mMove == eMoveCentre ? kGrey : sRgba (128,128,128, 192), false);

      gLcd->aEllipseOutline (c, cPointF(r, r), 4.f, 64);
      gLcd->aRender (mMove == eMoveRadius ? kWhite : sRgba (180,180,0, 255), false);

      float handWidth = r > 60.f ? r / 20.f : 3.f;
      float hourR = r * 0.75f;
      gLcd->aPointedLine (c, c + cPointF (hourR * sin (hourA), hourR * cos (hourA)), handWidth);
      float minuteR = r * 0.9f;
      gLcd->aPointedLine (c, c + cPointF (minuteR * sin (minuteA), minuteR * cos (minuteA)), handWidth);
      gLcd->aRender (kWhite);

      float secondR = r * 0.95f;
      gLcd->aPointedLine (c, c + cPointF (secondR * sin (secondA), secondR * cos (secondA)), handWidth);
      gLcd->aRender (sRgba (255,0,0, 180));

      //float subSecondR = r * 0.95f;
      //gLcd->aPointedLine (c, c + cPointF (minuteR * sin (subSecondA), minuteR * cos (subSecondA)), 3.f);
      //gLcd->aRender (sRgba (255,255,0, 128));
      //}}}
      gLcd->text (kWhite, 30, gRtc->getClockTimeDateString(), cRect (0, 426, 320, 480));
      gLcd->present();
      }

    vTaskDelay (10);
    }

  }
//}}}
//{{{
void appThread (void* arg) {

  gTouch->init();
  cPointF offset;

  while (true) {
    gTouch->start();
    gTouch->wait();

    if (gTouch->getState() == cTouch::ePress) {
      if (gTouch->getPressed()) {
        if (mMove == eNotPressed) {
          if ((mCentre - gTouch->getTouch()).magnitude() < mRadius-4.f) {
            mMove = eMoveCentre;
            offset = gTouch->getTouch() - mCentre;
            }
          else if ((mCentre - gTouch->getTouch()).magnitude() < mRadius+24.f)
            mMove = eMoveRadius;
          else
            mMove = ePressed;
          }

        if (mMove == eMoveCentre)
          mCentre = gTouch->getTouch() - offset;
        else if (mMove == eMoveRadius)
          mRadius = (mCentre - gTouch->getTouch()).magnitude();

        gLcd->change();
        vTaskDelay (1);
        }
      else {
        mMove = eNotPressed;
        vTaskDelay (50);
        }
      }
    }
  }
//}}}

int main() {
  HAL_Init();
  clockConfig();
  printf ("%s\n", kHello.c_str());

  BSP_LED_Init (LED_RED);
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);

  gRtc = new cRtc();
  gRtc->init();

  gLcd = new cLcd();
  gLcd->init (kHello);

  gTouch = new cTouch (gLcd->getSize());

  TaskHandle_t uiHandle;
  xTaskCreate ((TaskFunction_t)uiThread, "ui", 4096, 0, 4, &uiHandle);
  TaskHandle_t appHandle;
  xTaskCreate ((TaskFunction_t)appThread, "app", 1024, 0, 4, &appHandle);
  vTaskStartScheduler();

  return 0;
  }
