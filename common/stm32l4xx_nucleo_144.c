// stm32l4xx_nucleo_144.c
#include "stm32l4xx_nucleo_144.h"
//{{{  leds defines
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
//{{{  buttons defines
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
//{{{  otg defines
#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_5
#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()

#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()    __HAL_RCC_GPIOG_CLK_ENABLE()
//}}}
//{{{  smps defines
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

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
  #ifdef USE_ADP5301ACBZ
    /* ######################################################################## */
    /* #define PORT_SMPS               GPIOG              */
    /* #define PIN_SMPS_ENABLE         GPIO_PIN_11        */
    /* #define PIN_SMPS_POWERGOOD      GPIO_PIN_12        */
    /* #define PIN_SMPS_SWITCH_ENABLE  GPIO_PIN_13        */

    /* IN CASE OF SMPS VOLTAGE RANGE SELECTION            */
    /* #define PIN_SMPS_V1             GPIO_PIN_10        */
    /* ######################################################################## */
  #endif
#endif

//}}}

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN};
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};

//{{{
/**
  * @brief  Configure LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_Init (Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}
//}}}
//{{{
/**
  * @brief  Turn selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_On (Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}
//}}}
//{{{
/**
  * @brief  Turn selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_Off (Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}
//}}}
//{{{
/**
  * @brief  Toggle the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg  LED1
  *     @arg  LED2
  *     @arg  LED3
  * @retval None
  */
void BSP_LED_Toggle (Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}
//}}}

//{{{
/**
  * @brief  Configure Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  else if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}
//}}}
//{{{
/**
  * @brief  Return the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval The Button GPIO pin value.
  */
uint32_t BSP_PB_GetState (Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}
//}}}

#ifdef USE_STM32L4XX_NUCLEO_144_SMPS
  #ifdef USE_ADP5301ACBZ
    //{{{
    /**
      * @brief  DeInitialize the external SMPS component
      * @note   Low power consumption GPIO settings
      * @retval SMPS status
      */
    uint32_t BSP_SMPS_DeInit()
    {
      GPIO_InitTypeDef GPIO_InitStruct;

      PWR_AND_CLK_SMPS();

      /* --------------------------------------------------------------------------------------  */
      /* Added for Deinit if No PIN_SMPS_ENABLE & PIN_SMPS_SWITCH_ENABLE are not disabled before */

      /* Disable SMPS SWITCH */
      HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_RESET);

      HAL_Delay(1);

      /* Disable SMPS */
      HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_RESET);

      /* --------------------------------------------------------------------------------------  */

      /* To be updated */
      /* Set all GPIO in analog state to reduce power consumption, */
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Pin = PIN_SMPS_SWITCH_ENABLE;

      /* SWITCH */
      HAL_GPIO_Init (GPIOG, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE;
      /* --------- SMPS VOLTAGE RANGE SELECTION ----------------------------------*/
      /* ######################################################################## */
      /* GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE | PIN_SMPS_V1; */

      /* ENABLE = OFF */
      HAL_GPIO_Init (GPIOG, &GPIO_InitStruct);

      return SMPS_OK;
    }
    //}}}
    //{{{
    /**
      * @brief  Initialize the external SMPS component
      * @param  VoltageRange: Select operating SMPS supply
      *           @arg DCDC_AND_BOARD_DEPENDENT
      * @note   VoltageRange is not used with all boards
      *           VoltageRange is not used with MB11312A/S
      *           i.e. SMPS only PWR_REGULATOR_VOLTAGE_SCALE2.
      * @retval SMPS status
      */
    uint32_t BSP_SMPS_Init (uint32_t VoltageRange)
    {
      PWR_AND_CLK_SMPS();

      GPIO_InitTypeDef GPIO_InitStruct;

      /* Upon wake UP (standby case)                               */
      /* IF PIN_SMPS_ENABLE was pulled up                          */
      /* Then maintain PIN_SMPS_ENABLE = high                      */
      /* Needed to keep ENABLE HIGH                                */
      if (READ_BIT(PWR->PUCRG, PWR_GPIO_ENABLE))
      {
        HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_SET);
      }
      else
      {
        HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_SMPS, PWR_GPIO_SWITCH_ENABLE);
        HAL_PWREx_EnablePullUpPullDownConfig();
        /* Level shifter consumes because of missing pull up/down, so pull it up (only one autorized PA13) */
        HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_13); /* SWD/TMS */
        HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, GPIO_PIN_3); /* SWO */
      }
      /* ------------------------------------------------------------------------ */
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Pull = GPIO_PULLUP;

      GPIO_InitStruct.Pin = PIN_SMPS_POWERGOOD;

      HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

      /* ------------------------------------------------------------------------ */

      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Pull = GPIO_NOPULL;

      GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE;
      /* --------- ADD SMPS VOLTAGE RANGE SELECTION -----------------------------*/
      /* - > Applicable to ST1PS02D1QTR on MB1312A/S */
      /* GPIO_InitStruct.Pin = PIN_SMPS_ENABLE | PIN_SMPS_SWITCH_ENABLE | PIN_SMPS_V1; */

      HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

      /* --------- SMPS VOLTAGE RANGE SELECTION ----------------------------------*/
      /* ######################################################################## */
      /* - > Not applicable to ADP5301A on MB1312A/S */
      /* ######################################################################## */
      /* - > Applicable to ST1PS02D1QTR on MB1312A/S */
      /* Control to be added */

      /* ST1PS02D1QTR on MB1312 */
      /* if (VoltageRange == ST1PS02D1QTR_VOUT_1_25) */
      /* HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_V1, GPIO_PIN_SET); */
      /* 1.25V                  */
      /* D0/D1/D2 = H/L/L       */
      /* else */

      /* */
      /* ST1PS02D1QTR on MB1312 */
      /* ST1PS02D1QTR_VOUT_1_05 */
      /* 1.05V                  */
      /* D0/D1/D2 = L/L/L       */
      /* HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_V1, GPIO_PIN_RESET); */
      /* ######################################################################## */
      return SMPS_OK;
    }
    //}}}
    //{{{
    /**
      * @brief  Enable the external SMPS component
      * @param  Delay: delay in ms after enable
      * @param  Power_Good_Check: Enable Power good check
      * @note   Power_Good_Check
      *  is not used with all boards
      *           VoltageRange is not used with MB11312A/S
      *           i.e. SMPS only PWR_REGULATOR_VOLTAGE_SCALE2 by board.
      * @retval SMPS status
      *           @arg SMPS_OK: SMPS ENABLE OK
      *           @arg SMPS_KO: POWER GOOD CHECK FAILS
      *
      */
    uint32_t BSP_SMPS_Enable (uint32_t Delay, uint32_t Power_Good_Check)
    {
      PWR_AND_CLK_SMPS();

      HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_SET);

      HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_SMPS, PWR_GPIO_BIT_11);
      HAL_PWREx_EnablePullUpPullDownConfig();

      /* Delay upon request */
      if (Delay != 0)
      {
        HAL_Delay(Delay);
      }

      /* CHECK POWER GOOD or NOT */
      if (Power_Good_Check != 0)
      {
        if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_POWERGOOD)))
        {
          /* POWER GOOD KO */
          return SMPS_KO;
        }
      }

      /* SMPS ENABLED */
      return SMPS_OK;
    }
    //}}}
    //{{{
    /**
      * @brief  Disable the external SMPS component
      * @note   SMPS SWITCH should be disabled first !
      * @retval SMPS status
      *           @arg SMPS_OK: SMPS DISABLE OK - DONE
      *           @arg SMPS_KO: POWER GOOD CHECK FAILS
      *
      */
    uint32_t BSP_SMPS_Disable()
    {

      PWR_AND_CLK_SMPS();

      /* Check if SWITCH is DISABLE */
      if (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE) != GPIO_PIN_RESET)
      {
        /* ERROR AS SWITCH SHOULD BE DISABLED */
        return SMPS_KO;
      }

      /* Disable SMPS */
      HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_ENABLE, GPIO_PIN_RESET);

      HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_SMPS, PWR_GPIO_ENABLE);
      HAL_PWREx_EnablePullUpPullDownConfig();

      /* SMPS DISABLED */
      return SMPS_OK;
    }
    //}}}
    //{{{
    /**
      * @brief  Enable the external SMPS SWITCH component
      * @param  Delay: delay in ms before SMPS SWITCH ENABLE
      * @param  Power_Good_Check: Enable Power good check
      * @note   Power_Good_Check
      *  is not used with all boards
      *           VoltageRange is not used with MB11312A/S
      *           i.e. SMPS only PWR_REGULATOR_VOLTAGE_SCALE2 by board.
      * @retval SMPS status
      *           @arg SMPS_OK: SMPS ENABLE OK
      *           @arg SMPS_KO: POWER GOOD CHECK FAILS
      *
      */
    uint32_t BSP_SMPS_Supply_Enable (uint32_t Delay, uint32_t Power_Good_Check)
    {
      PWR_AND_CLK_SMPS();

      if (Delay != 0)
      {
        HAL_Delay(Delay);
      }
      /* CHECK POWER GOOD or NOT */
      if (Power_Good_Check != 0)
      {
        if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PORT_SMPS, PIN_SMPS_POWERGOOD)))
        {
          /* POWER GOOD KO */
          return SMPS_KO;
        }
      }

      /* SMPS SWITCH ENABLE */
      HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_SET);

      return SMPS_OK;
    }
    //}}}
    //{{{
    /**
      * @brief  Disable the external SMPS component
      * @retval SMPS status
      *           @arg SMPS_OK: SMPS SWITCH DISABLE OK
      *
      */
    uint32_t BSP_SMPS_Supply_Disable()
    {
      PWR_AND_CLK_SMPS();

      /* SMPS SWITCH DISABLED */
      HAL_GPIO_WritePin(PORT_SMPS, PIN_SMPS_SWITCH_ENABLE, GPIO_PIN_RESET);

      return SMPS_OK;
    }
    //}}}
  #endif
#endif
