//// main.cpp
//{{{  includes
#include <algorithm>
#include <string>
#include <ctype.h>

#include "cmsis_os.h"
#include "stm32l4xx.h"

#include "../common/heap.h"
#include "../common/cRtc.h"

#include "cLcd.h"

using namespace std;
//}}}
const string kHello = "smallLcd " + string(__TIME__) + " " + string(__DATE__);

// vars
cLcd* lcd = nullptr;
cRtc* rtc = nullptr;

//{{{
void uiThread (void* arg) {

  cPointF centre = cPointF (160.f, 240.f);
  float radius = 20.f;
  const float maxRadius = 160.f;

  lcd->tftInit();
  lcd->display (70);

  int count = 0;
  while (true) {
    if (lcd->isChanged() || (count == 1000)) {
      count = 0;
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
      lcd->aRender (sRgba565 (128,128,128, 192), false);
      lcd->aEllipseOutline (centre, cPointF(radius, radius), width, steps);
      lcd->aRender (sRgba565 (180,180,0, 255), false);

      float handWidth = radius > 60.f ? radius / 20.f : 3.f;
      float hourR = radius * 0.75f;
      lcd->aPointedLine (centre, centre + cPointF (hourR * sin (hourA), hourR * cos (hourA)), handWidth);
      float minuteR = radius * 0.9f;
      lcd->aPointedLine (centre, centre + cPointF (minuteR * sin (minuteA), minuteR * cos (minuteA)), handWidth);
      lcd->aRender (kWhite);

      float secondR = radius * 0.95f;
      lcd->aPointedLine (centre, centre + cPointF (secondR * sin (secondA), secondR * cos (secondA)), handWidth);
      lcd->aRender (sRgba565 (255,0,0, 180));
      //}}}
      lcd->cLcd::text (kWhite, 30, rtc->getClockTimeDateString(), cRect (0, 426, 320, 480));
      lcd->present();

      if (radius < maxRadius) {
        radius *= 1.04f;
        if (radius > maxRadius)
          radius = maxRadius;
        lcd->change();
        }
      }
    else {
      count++;
      vTaskDelay (1);
      }
    }
  }
//}}}
//{{{
void appThread (void* arg) {

  //uint32_t offset = 0;
  //while (true)
  //  for (int j = 4; j <= 0x3F; j++) {
  //    offset += HAL_GetTick();
  //    sdRamTest (uint16_t(offset++), (uint16_t*)(SDRAM_DEVICE_ADDR + (j * 0x00200000)), 0x00200000);
  //    vTaskDelay (200);
  //    }

  //  accel
  //lsm303c_init();
  while (true) {
   // while (lsm303c_la_ready()) {
   //   lsm303c_la (la);
   //   mTraceVec.addSample (0, la[0]);
    //  mTraceVec.addSample (1, la[1]);
    //  mTraceVec.addSample (2, la[2]);
    //  lcd->change();
    //  }

    //lcd->info (COL_YELLOW, "MF x:" + dec(mf[0]) + " y:" + dec(mf[1]) + " z:" + dec(mf[2]));
    //while (lsm303c_mf_ready())
    //  lsm303c_mf (mf);
    vTaskDelay (2);
    }
  }
//}}}

//{{{
//void clockConfig() {
////   System Clock       = PLL (HSE BYPASS)
////   SYSCLK(Hz)         = 400000000 (CPU Clock)
////   HCLK(Hz)           = 200000000 (AXI and AHBs Clock)
////   AHB Prescaler      = 2
////   D1 APB3 Prescaler  = 2 (APB3 Clock  100MHz)
////   D2 APB1 Prescaler  = 2 (APB1 Clock  100MHz)
////   D2 APB2 Prescaler  = 2 (APB2 Clock  100MHz)
////   D3 APB4 Prescaler  = 2 (APB4 Clock  100MHz)
////   HSE Frequency(Hz)  = 8000000
////   PLL_M              = 4
////   PLL_N              = 400
////   PLL_P              = 2
////   PLL_Q              = 4
////   PLL_R              = 2
////   VDD(V)             = 3.3
////   Flash Latency(WS)  = 4

  //MODIFY_REG (PWR->CR3, PWR_CR3_SCUEN, 0);

  //// Voltage scaling optimises power consumption when clocked below maximum system frequency
  //__HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);
  //while (!__HAL_PWR_GET_FLAG (PWR_FLAG_VOSRDY)) {}

  //// Enable D2 domain SRAM Clocks
  //__HAL_RCC_D2SRAM1_CLK_ENABLE();
  //__HAL_RCC_D2SRAM2_CLK_ENABLE();
  //__HAL_RCC_D2SRAM3_CLK_ENABLE();

  //// enable HSE Oscillator, activate PLL HSE source
  //RCC_OscInitTypeDef rccOscInit;
  //rccOscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  //rccOscInit.HSEState = RCC_HSE_BYPASS;
  //rccOscInit.HSIState = RCC_HSI_OFF;
  //rccOscInit.CSIState = RCC_CSI_OFF;
  //rccOscInit.PLL.PLLState = RCC_PLL_ON;
  //rccOscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  //rccOscInit.PLL.PLLM = 4;
  //rccOscInit.PLL.PLLN = 400;
  //rccOscInit.PLL.PLLP = 2;
  //rccOscInit.PLL.PLLQ = 4;
  //rccOscInit.PLL.PLLR = 2;
  //rccOscInit.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  //rccOscInit.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  //HAL_RCC_OscConfig (&rccOscInit);

  //// select PLL system clock source. config bus clocks dividers
  //RCC_ClkInitTypeDef rccClkInit;
  //rccClkInit.ClockType = (RCC_CLOCKTYPE_SYSCLK  | RCC_CLOCKTYPE_HCLK |
                          //RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2 |
                          //RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1);
  //rccClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  //rccClkInit.SYSCLKDivider = RCC_SYSCLK_DIV1;
  //rccClkInit.AHBCLKDivider = RCC_HCLK_DIV2;
  //rccClkInit.APB1CLKDivider = RCC_APB1_DIV2;
  //rccClkInit.APB2CLKDivider = RCC_APB2_DIV2;
  //rccClkInit.APB3CLKDivider = RCC_APB3_DIV2;
  //rccClkInit.APB4CLKDivider = RCC_APB4_DIV2;
  //HAL_RCC_ClockConfig (&rccClkInit, FLASH_LATENCY_2);
  ////HAL_RCC_ClockConfig (&rccClkInit, FLASH_LATENCY_4);

  //// PLL3_VCO In  = HSE_VALUE / PLL3M = 1 Mhz
  //// PLL3_VCO Out = PLL3_VCO In * PLL3N = 100 Mhz
  //// PLLLCDCLK    = PLL3_VCO Out / PLL3R = 100/4 = 25Mhz
  //// LTDC clock   = PLLLCDCLK = 25Mhz
  //RCC_PeriphCLKInitTypeDef rccPeriphClkInit;
  //rccPeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  //rccPeriphClkInit.PLL3.PLL3M = 8;
  //rccPeriphClkInit.PLL3.PLL3N = 100;
  //rccPeriphClkInit.PLL3.PLL3R = 4;
  //rccPeriphClkInit.PLL3.PLL3P = 2;
  //rccPeriphClkInit.PLL3.PLL3Q = 7;
  //HAL_RCCEx_PeriphCLKConfig (&rccPeriphClkInit);
  //}
//}}}

int main() {
  HAL_Init();
  //clockConfig();

  printf ("%s\n", kHello.c_str());

  BSP_LED_Init (LED_RED);
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);
  lcd = new cLcd();
  lcd->init (kHello);
  rtc = new cRtc();
  rtc->init();

  TaskHandle_t uiHandle;
  xTaskCreate ((TaskFunction_t)uiThread, "ui", 1024, 0, 4, &uiHandle);
  TaskHandle_t appHandle;
  xTaskCreate ((TaskFunction_t)appThread, "app", 4096, 0, 4, &appHandle);
  vTaskStartScheduler();

  return 0;
  }
