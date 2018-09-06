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

//{{{  leds
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_7
#define LED1_GPIO_PORT                          GPIOC
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOC_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_7
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_14
#define LED3_GPIO_PORT                          GPIOB
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) {__HAL_RCC_GPIOC_CLK_ENABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_ENABLE();   }} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  do { if((__INDEX__) == 0) {__HAL_RCC_GPIOC_CLK_DISABLE();} else\
                                                                    {__HAL_RCC_GPIOB_CLK_DISABLE();   }} while(0)
//}}}
//{{{  buttons
#define BUTTONn                                 1

#define USER_BUTTON_PIN                       GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                 GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_LINE                 GPIO_PIN_13
#define USER_BUTTON_EXTI_IRQn                 EXTI15_10_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    USER_BUTTON_GPIO_CLK_ENABLE()
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   USER_BUTTON_GPIO_CLK_DISABLE()

/* Aliases */
#define KEY_BUTTON_PIN                       USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                 USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()         USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()        USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                 USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                 USER_BUTTON_EXTI_IRQn
//}}}
//{{{  otg
#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_5
#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
//}}}

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
  #define SMPS_OK  0
  #define SMPS_KO  1
#endif

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
  #ifdef USE_ADP5301ACBZ
    //{{{  adp
    #define PORT_SMPS               GPIOG
    #define PIN_SMPS_ENABLE         GPIO_PIN_11
    #define PIN_SMPS_POWERGOOD      GPIO_PIN_12
    #define PIN_SMPS_SWITCH_ENABLE  GPIO_PIN_13

    #define PWR_GPIO_SMPS           PWR_GPIO_G
    #define PWR_GPIO_ENABLE         PWR_GPIO_BIT_11
    #define PWR_GPIO_SWITCH_ENABLE  PWR_GPIO_BIT_13

    #define PWR_AND_CLK_SMPS()   do { __HAL_RCC_PWR_CLK_ENABLE(); \
                                      HAL_PWREx_EnableVddIO2(); \
                                      __HAL_RCC_GPIOG_CLK_ENABLE(); } while(0)
    //}}}
  #endif
#endif

void BSP_LED_Init (Led_TypeDef Led);
void BSP_LED_DeInit (Led_TypeDef Led);
void BSP_LED_On (Led_TypeDef Led);
void BSP_LED_Off (Led_TypeDef Led);
void BSP_LED_Toggle (Led_TypeDef Led);

void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void BSP_PB_DeInit (Button_TypeDef Button);
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
