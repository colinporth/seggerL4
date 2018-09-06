#pragma once
//{{{
#ifdef __cplusplus
  extern "C" {
#endif
//}}}
#include "stm32l4xx_hal.h"

typedef enum { LED1 = 0, LED_GREEN = LED1, LED2 = 1, LED_BLUE = LED2, LED3 = 2, LED_RED = LED3 } Led_TypeDef;
typedef enum { BUTTON_USER = 0, /* Alias */ BUTTON_KEY = BUTTON_USER } Button_TypeDef;
typedef enum { BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1 } ButtonMode_TypeDef;
typedef enum { JOY_NONE = 0, JOY_SEL = 1, JOY_DOWN = 2, JOY_LEFT = 3, JOY_RIGHT = 4, JOY_UP = 5 } JOYState_TypeDef;

#define USE_STM32L4XX_NUCLEO_144

void BSP_LED_Init (Led_TypeDef Led);
void BSP_LED_On (Led_TypeDef Led);
void BSP_LED_Off (Led_TypeDef Led);
void BSP_LED_Toggle (Led_TypeDef Led);

void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
uint32_t BSP_PB_GetState (Button_TypeDef Button);

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
  uint32_t BSP_SMPS_Init (uint32_t VoltageRange);
  uint32_t BSP_SMPS_DeInit();
  uint32_t BSP_SMPS_Enable (uint32_t Delay, uint32_t Power_Good_Check);
  uint32_t BSP_SMPS_Disable();
  uint32_t BSP_SMPS_Supply_Enable (uint32_t Delay, uint32_t Power_Good_Check);
  uint32_t BSP_SMPS_Supply_Disable();
#endif

//{{{
#ifdef __cplusplus
  }
#endif
//}}}
