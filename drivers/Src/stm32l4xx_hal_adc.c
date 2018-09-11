//{{{
/*  (+) 12-bit, 10-bit, 8-bit or 6-bit configurable resolution.
  (+) Interrupt generation at the end of regular conversion and in case of
      analog watchdog or overrun events.
  (+) Single and continuous conversion modes.
  (+) Scan mode for conversion of several channels sequentially.
  (+) Data alignment with in-built data coherency.
  (+) Programmable sampling time (channel wise)
  (+) External trigger (timer or EXTI) with configurable polarity
  (+) DMA request generation for transfer of conversions data of regular group.
  (+) Configurable delay between conversions in Dual interleaved mode.
  (+) ADC channels selectable single/differential input.
  (+) ADC offset shared on 4 offset instances.
  (+) ADC calibration
  (+) ADC conversion of regular group.
  (+) ADC supply requirements: 1.62 V to 3.6 V.
  (+) ADC input range: from Vref- (connected to Vssa) to Vref+ (connected to
      Vdda or to an external voltage reference).

    (#) Enable the ADC interface
        (++) As prerequisite, ADC clock must be configured at RCC top level.

        (++) Two clock settings are mandatory:
             (+++) ADC clock (core clock, also possibly conversion clock).

             (+++) ADC clock (conversions clock).
                   Two possible clock sources: synchronous clock derived from APB clock
                   or asynchronous clock derived from system clock, PLLSAI1 or the PLLSAI2
                   running up to 80MHz.

             (+++) Example:
                   Into HAL_ADC_MspInit() (recommended code location) or with
                   other device clock parameters configuration:
               (+++) __HAL_RCC_ADC_CLK_ENABLE();                  (mandatory)

               RCC_ADCCLKSOURCE_PLL enable:                       (optional: if asynchronous clock selected)
               (+++) RCC_PeriphClkInitTypeDef   RCC_PeriphClkInit;
               (+++) PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
               (+++) PeriphClkInit.AdcClockSelection    = RCC_ADCCLKSOURCE_PLL;
               (+++) HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

        (++) ADC clock source and clock prescaler are configured at ADC level with
             parameter "ClockPrescaler" using function HAL_ADC_Init().

    (#) ADC pins configuration
         (++) Enable the clock for the ADC GPIOs
              using macro __HAL_RCC_GPIOx_CLK_ENABLE()
         (++) Configure these ADC pins in analog mode
              using function HAL_GPIO_Init()

    (#) Optionally, in case of usage of ADC with interruptions:
         (++) Configure the NVIC for ADC
              using function HAL_NVIC_EnableIRQ(ADCx_IRQn)
         (++) Insert the ADC interruption handler function HAL_ADC_IRQHandler()
              into the function of corresponding ADC interruption vector
              ADCx_IRQHandler().

    (#) Optionally, in case of usage of DMA:
         (++) Configure the DMA (DMA channel, mode normal or circular, ...)
              using function HAL_DMA_Init().
         (++) Configure the NVIC for DMA
              using function HAL_NVIC_EnableIRQ(DMAx_Channelx_IRQn)
         (++) Insert the ADC interruption handler function HAL_ADC_IRQHandler()
              into the function of corresponding DMA interruption vector
              DMAx_Channelx_IRQHandler().

    (#) Configure the ADC parameters (resolution, data alignment, ...)
        and regular group parameters (conversion trigger, sequencer, ...)
        using function HAL_ADC_Init().

    (#) Configure the channels for regular group parameters (channel number,
        channel rank into sequencer, ..., into regular group)
        using function HAL_ADC_ConfigChannel().

    (#) Optionally, configure the analog watchdog parameters (channels
        monitored, thresholds, ...)
        using function HAL_ADC_AnalogWDGConfig().

     *** Execution of ADC conversions ***
     ====================================
     [..]

    (#) Optionally, perform an automatic ADC calibration to improve the
        conversion accuracy
        using function HAL_ADCEx_Calibration_Start().

    (#) ADC driver can be used among three modes: polling, interruption,
        transfer by DMA.

        (++) ADC conversion by polling:
          (+++) Activate the ADC peripheral and start conversions
                using function HAL_ADC_Start()
          (+++) Wait for ADC conversion completion
                using function HAL_ADC_PollForConversion()
          (+++) Retrieve conversion results
                using function HAL_ADC_GetValue()
          (+++) Stop conversion and disable the ADC peripheral
                using function HAL_ADC_Stop()

        (++) ADC conversion by interruption:
          (+++) Activate the ADC peripheral and start conversions
                using function HAL_ADC_Start_IT()
          (+++) Wait for ADC conversion completion by call of function
                HAL_ADC_ConvCpltCallback()
                (this function must be implemented in user program)
          (+++) Retrieve conversion results
                using function HAL_ADC_GetValue()
          (+++) Stop conversion and disable the ADC peripheral
                using function HAL_ADC_Stop_IT()

        (++) ADC conversion with transfer by DMA:
          (+++) Activate the ADC peripheral and start conversions
                using function HAL_ADC_Start_DMA()
          (+++) Wait for ADC conversion completion by call of function
                HAL_ADC_ConvCpltCallback() or HAL_ADC_ConvHalfCpltCallback()
                (these functions must be implemented in user program)
          (+++) Conversion results are automatically transferred by DMA into
                destination variable address.
          (+++) Stop conversion and disable the ADC peripheral
                using function HAL_ADC_Stop_DMA()

     [..]

    (@) Callback functions must be implemented in user program:
      (+@) HAL_ADC_ErrorCallback()
      (+@) HAL_ADC_LevelOutOfWindowCallback() (callback of analog watchdog)
      (+@) HAL_ADC_ConvCpltCallback()
      (+@) HAL_ADC_ConvHalfCpltCallback

     *** Deinitialization of ADC ***
     ============================================================
     [..]

    (#) Disable the ADC interface
      (++) ADC clock can be hard reset and disabled at RCC top level.
        (++) Hard reset of ADC peripherals
             using macro __ADCx_FORCE_RESET(), __ADCx_RELEASE_RESET().
        (++) ADC clock disable
             using the equivalent macro/functions as configuration step.
             (+++) Example:
                   Into HAL_ADC_MspDeInit() (recommended code location) or with
                   other device clock parameters configuration:
               (+++) RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI14;
               (+++) RCC_OscInitStructure.HSI14State = RCC_HSI14_OFF; (if not used for system clock)
               (+++) HAL_RCC_OscConfig(&RCC_OscInitStructure);

    (#) ADC pins configuration
         (++) Disable the clock for the ADC GPIOs
              using macro __HAL_RCC_GPIOx_CLK_DISABLE()

    (#) Optionally, in case of usage of ADC with interruptions:
         (++) Disable the NVIC for ADC
              using function HAL_NVIC_EnableIRQ(ADCx_IRQn)

    (#) Optionally, in case of usage of DMA:
         (++) Deinitialize the DMA
              using function HAL_DMA_Init().
         (++) Disable the NVIC for DMA
              using function HAL_NVIC_EnableIRQ(DMAx_Channelx_IRQn)
  */
//}}}
#include "stm32l4xx_hal.h"
//{{{  defines
/*!< ADC_CFGR fields of parameters that can be updated when no regular conversion is on-going */

#define ADC_CFGR_FIELDS_1  ((uint32_t)(ADC_CFGR_RES    | ADC_CFGR_ALIGN   |\
                                       ADC_CFGR_CONT   | ADC_CFGR_OVRMOD  |\
                                       ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM |\
                                       ADC_CFGR_EXTEN  | ADC_CFGR_EXTSEL))

/* Delay to wait before setting ADEN once ADCAL has been reset must be at least 4 ADC clock cycles.
   Assuming lowest ADC clock (140 KHz according to DS), this 4 ADC clock cycles duration is equal to
   4 / 140,000 = 0.028 ms.
   ADC_ENABLE_TIMEOUT set to 2 is a margin large enough to ensure the 4 ADC clock cycles have elapsed
   while waiting for ADRDY to become 1 */
#define ADC_ENABLE_TIMEOUT              ((uint32_t)  2)    /*!< ADC enable time-out value  */
#define ADC_DISABLE_TIMEOUT             ((uint32_t)  2)    /*!< ADC disable time-out value */


/* Timeout to wait for current conversion on going to be completed.           */
/* Timeout fixed to longest ADC conversion possible, for 1 channel:           */
/*   - maximum sampling time (640.5 adc_clk)                                  */
/*   - ADC resolution (Tsar 12 bits= 12.5 adc_clk)                            */
/*   - ADC clock with prescaler 256                                           */
/*     (from asynchronous clock, assuming clock frequency same as CPU for     */
/*      this calculation)                                                     */
/*   - ADC oversampling ratio 256                                             */
/*   Calculation: 653 * 256 * 256 = 42795008 CPU clock cycles max             */
/* Unit: cycles of CPU clock.                                                 */
#define ADC_CONVERSION_TIME_MAX_CPU_CYCLES ((uint32_t) 42795008)  /*!< ADC conversion completion time-out value */

// ex defines
#define ADC_JSQR_FIELDS  ((uint32_t)(ADC_JSQR_JL | ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN |\
                                     ADC_JSQR_JSQ1  | ADC_JSQR_JSQ2 |\
                                      ADC_JSQR_JSQ3 | ADC_JSQR_JSQ4 ))  /*!< ADC_JSQR fields of parameters that can be updated anytime
                                                                             once the ADC is enabled */

/* Fixed timeout value for ADC calibration.                                   */
/* Values defined to be higher than worst cases: low clock frequency,         */
/* maximum prescalers.                                                        */
/* Ex of profile low frequency : f_ADC at 0.14 MHz (minimum value             */
/* according to Data sheet), calibration_time MAX = 112 / f_ADC               */
/*           112 / 140,000 = 0.8 ms                                           */
/* At maximum CPU speed (80 MHz), this means                                  */
/*    0.8 ms * 80 MHz = 64000 CPU cycles                                      */
#define ADC_CALIBRATION_TIMEOUT         (64000U)    /*!< ADC calibration time-out value */
//}}}

//{{{  weak callbacks
__weak void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_MspDeInit (ADC_HandleTypeDef* hadc) { UNUSED(hadc);}
__weak void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_LevelOutOfWindowCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_ErrorCallback (ADC_HandleTypeDef *hadc) { UNUSED(hadc); }
__weak void HAL_ADCEx_InjectedConvCpltCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADCEx_InjectedQueueOverflowCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADCEx_LevelOutOfWindow2Callback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADCEx_LevelOutOfWindow3Callback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADCEx_EndOfSamplingCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
//}}}

//{{{
/**
  * @brief  Initialize the ADC peripheral and regular group according to
  *         parameters specified in structure "ADC_InitTypeDef".
  * @note   As prerequisite, ADC clock must be configured at RCC top level
  *         (refer to description of RCC configuration for ADC
  *         in header of this file).
  * @note   Possibility to update parameters on the fly:
  *         This function initializes the ADC MSP (HAL_ADC_MspInit()) only when
  *         coming from ADC state reset. Following calls to this function can
  *         be used to reconfigure some parameters of ADC_InitTypeDef
  *         structure on the fly, without modifying MSP configuration. If ADC
  *         MSP has to be modified again, HAL_ADC_DeInit() must be called
  *         before HAL_ADC_Init().
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_InitTypeDef".
  * @note   This function configures the ADC within 2 scopes: scope of entire
  *         ADC and scope of regular group. For parameters details, see comments
  *         of structure "ADC_InitTypeDef".
  * @note   Parameters related to common ADC registers (ADC clock mode) are set
  *         only if all ADCs are disabled.
  *         If this is not the case, these common parameters setting are
  *         bypassed without error reporting: it can be the intended behaviour in
  *         case of update of a parameter of ADC_InitTypeDef on the fly,
  *         without  disabling the other ADCs.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Init (ADC_HandleTypeDef* hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpCFGR = 0U;
  __IO uint32_t wait_loop_index = 0;

  /* Check ADC handle */
  if(hadc == NULL)
    return HAL_ERROR;


  /* Actions performed only if ADC is coming from state reset:                */
  /* - Initialization of ADC MSP                                              */
  if (hadc->State == HAL_ADC_STATE_RESET)
  {
    /* Init the low level hardware */
    HAL_ADC_MspInit(hadc);

    /* Set ADC error code to none */
    ADC_CLEAR_ERRORCODE(hadc);

    /* Initialize Lock */
    hadc->Lock = HAL_UNLOCKED;
  }

  /* - Exit from deep-power-down mode and ADC voltage regulator enable        */
  if (LL_ADC_IsDeepPowerDownEnabled(hadc->Instance) != 0U)
  {
    /* Disable ADC deep power down mode */
    LL_ADC_DisableDeepPowerDown(hadc->Instance);

    /* System was in deep power down mode, calibration must
     be relaunched or a previously saved calibration factor
     re-applied once the ADC voltage regulator is enabled */
  }

  if(LL_ADC_IsInternalRegulatorEnabled(hadc->Instance) == 0U)
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(hadc->Instance);

    /* Delay for ADC stabilization time */
    /* Wait loop initialization and execution */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles.                                           */
    wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (1000000 * 2)));
    while(wait_loop_index != 0)
    {
      wait_loop_index--;
    }
  }

  /* Verification that ADC voltage regulator is correctly enabled, whether    */
  /* or not ADC is coming from state reset (if any potential problem of       */
  /* clocking, voltage regulator would not be enabled).                       */
  if(LL_ADC_IsInternalRegulatorEnabled(hadc->Instance) == 0U)
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

    /* Set ADC error code to ADC IP internal error */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

    tmp_hal_status = HAL_ERROR;
  }

  /* Configuration of ADC parameters if previous preliminary actions are      */
  /* correctly completed and if there is no conversion on going on regular    */
  /* group (ADC may already be enabled at this point if HAL_ADC_Init() is     */
  /* called to update a parameter on the fly).                                */
  if(   (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
     && (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
    )
  {
    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY,
                      HAL_ADC_STATE_BUSY_INTERNAL);

    /* Configuration of common ADC parameters                                 */

    /* Parameters update conditioned to ADC state:                            */
    /* Parameters that can be updated only when ADC is disabled:              */
    /*  - clock configuration                                                 */
    if ((ADC_IS_ENABLE(hadc) == RESET)   &&
         (ADC_ANY_OTHER_ENABLED(hadc) == RESET) )
    {
      /* Reset configuration of ADC common register CCR:                      */
      /*                                                                      */
      /*   - ADC clock mode and ACC prescaler (CKMODE and PRESC bits)are set  */
      /*     according to adc->Init.ClockPrescaler. It selects the clock      */
      /*    source and sets the clock division factor.                        */
      /*                                                                      */
      /* Some parameters of this register are not reset, since they are set   */
      /* by other functions and must be kept in case of usage of this         */
      /* function on the fly (update of a parameter of ADC_InitTypeDef        */
      /* without needing to reconfigure all other ADC groups/channels         */
      /* parameters):                                                         */
      /*   - when multimode feature is available, multimode-related           */
      /*     parameters: MDMA, DMACFG, DELAY, DUAL (set by API                */
      /*     HAL_ADCEx_MultiModeConfigChannel() )                             */
      /*   - internal measurement paths: Vbat, temperature sensor, Vref       */
      /*     (set into HAL_ADC_ConfigChannel() or                             */
      /*     HAL_ADCEx_InjectedConfigChannel() )                              */
      LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc->Instance), hadc->Init.ClockPrescaler);
    }

    /* Configuration of ADC:                                                  */
    /*  - resolution                               Init.Resolution            */
    /*  - data alignment                           Init.DataAlign             */
    /*  - external trigger to start conversion     Init.ExternalTrigConv      */
    /*  - external trigger polarity                Init.ExternalTrigConvEdge  */
    /*  - continuous conversion mode               Init.ContinuousConvMode    */
    /*  - overrun                                  Init.Overrun               */
    /*  - discontinuous mode                       Init.DiscontinuousConvMode */
    /*  - discontinuous mode channel count         Init.NbrOfDiscConversion   */
    tmpCFGR  = (ADC_CFGR_CONTINUOUS(hadc->Init.ContinuousConvMode)           |
                hadc->Init.Overrun                                           |
                hadc->Init.DataAlign                                         |
                hadc->Init.Resolution                                        |
                ADC_CFGR_REG_DISCONTINUOUS(hadc->Init.DiscontinuousConvMode)  );

    if (hadc->Init.DiscontinuousConvMode == ENABLE)
    {
      tmpCFGR |= ADC_CFGR_DISCONTINUOUS_NUM(hadc->Init.NbrOfDiscConversion);
    }

    /* Enable external trigger if trigger selection is different of software  */
    /* start.                                                                 */
    /* Note: This configuration keeps the hardware feature of parameter       */
    /*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
    /*       software start.                                                  */
    if (hadc->Init.ExternalTrigConv != ADC_SOFTWARE_START)
    {
      tmpCFGR |= (  (hadc->Init.ExternalTrigConv & ADC_CFGR_EXTSEL)
                  | hadc->Init.ExternalTrigConvEdge
                 );
    }

    /* Update Configuration Register CFGR */
    MODIFY_REG(hadc->Instance->CFGR, ADC_CFGR_FIELDS_1, tmpCFGR);

    /* Parameters update conditioned to ADC state:                            */
    /* Parameters that can be updated when ADC is disabled or enabled without */
    /* conversion on going on regular and injected groups:                    */
    /*  - DMA continuous request          Init.DMAContinuousRequests          */
    /*  - LowPowerAutoWait feature        Init.LowPowerAutoWait               */
    /*  - Oversampling parameters         Init.Oversampling                   */
    if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET)
    {
      tmpCFGR = ( ADC_CFGR_DFSDM(hadc)                                 |
                  ADC_CFGR_AUTOWAIT(hadc->Init.LowPowerAutoWait)       |
                  ADC_CFGR_DMACONTREQ(hadc->Init.DMAContinuousRequests) );

      MODIFY_REG(hadc->Instance->CFGR, ADC_CFGR_FIELDS_2, tmpCFGR);

      if (hadc->Init.OversamplingMode == ENABLE)
      {
        /* Configuration of Oversampler:                                      */
        /*  - Oversampling Ratio                                              */
        /*  - Right bit shift                                                 */
        /*  - Triggered mode                                                  */
        /*  - Oversampling mode (continued/resumed)                           */
        MODIFY_REG(hadc->Instance->CFGR2,
                   ADC_CFGR2_OVSR  |
                   ADC_CFGR2_OVSS  |
                   ADC_CFGR2_TROVS |
                   ADC_CFGR2_ROVSM,
                   ADC_CFGR2_ROVSE                       |
                   hadc->Init.Oversampling.Ratio         |
                   hadc->Init.Oversampling.RightBitShift |
                   hadc->Init.Oversampling.TriggeredMode |
                   hadc->Init.Oversampling.OversamplingStopReset
                  );
      }
      else
      {
        /* Disable ADC oversampling scope on ADC group regular */
        CLEAR_BIT(hadc->Instance->CFGR2, ADC_CFGR2_ROVSE);
      }

    }   /*  if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET) */

    /* Configuration of regular group sequencer:                              */
    /* - if scan mode is disabled, regular channels sequence length is set to */
    /*   0x00: 1 channel converted (channel on regular rank 1)                */
    /*   Parameter "NbrOfConversion" is discarded.                            */
    /*   Note: Scan mode is not present by hardware on this device, but       */
    /*   emulated by software for alignment over all STM32 devices.           */
    /* - if scan mode is enabled, regular channels sequence length is set to  */
    /*   parameter "NbrOfConversion".                                         */

    if (hadc->Init.ScanConvMode == ADC_SCAN_ENABLE)
    {
      /* Set number of ranks in regular group sequencer */
      MODIFY_REG(hadc->Instance->SQR1, ADC_SQR1_L, (hadc->Init.NbrOfConversion - (uint8_t)1));
    }
    else
    {
      CLEAR_BIT(hadc->Instance->SQR1, ADC_SQR1_L);
    }

    /* Initialize the ADC state */
    /* Clear HAL_ADC_STATE_BUSY_INTERNAL bit, set HAL_ADC_STATE_READY bit */
    ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_READY);
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

    tmp_hal_status = HAL_ERROR;
  }

  /* Return function status */
  return tmp_hal_status;
}
//}}}
//{{{
/**
  * @brief  Deinitialize the ADC peripheral registers to their default reset
  *         values, with deinitialization of the ADC MSP.
  * @note   For devices with several ADCs: reset of ADC common registers is done
  *         only if all ADCs sharing the same common group are disabled.
  *         (function "HAL_ADC_MspDeInit()" is also called under the same conditions:
  *         all ADC instances use the same core clock at RCC level, disabling
  *         the core clock reset all ADC instances).
  *         If this is not the case, reset of these common parameters reset is
  *         bypassed without error reporting: it can be the intended behavior in
  *         case of reset of a single ADC while the other ADCs sharing the same
  *         common group is still running.
  * @note   By default, HAL_ADC_DeInit() set ADC in mode deep power-down:
  *         this saves more power by reducing leakage currents
  *         and is particularly interesting before entering MCU low-power modes.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_DeInit (ADC_HandleTypeDef* hadc)
{
  /* Check ADC handle */
  if (hadc == NULL)
    return HAL_ERROR;

  /* Set ADC state */
  SET_BIT(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL);

  /* Stop potential conversion on going, on regular and injected groups */
  /* Note: No check on ADC_ConversionStop() return status,              */
  /*       if the conversion stop failed, it is up to                   */
  /*       HAL_ADC_MspDeInit() to reset the ADC IP.                     */
  ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped            */
  /* Flush register JSQR: reset the queue sequencer when injected             */
  /* queue sequencer is enabled and ADC disabled.                             */
  /* The software and hardware triggers of the injected sequence are both     */
  /* internally disabled just after the completion of the last valid          */
  /* injected sequence.                                                       */
  SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQM);

  /* Disable the ADC peripheral */
  /* No check on ADC_Disable() return status, if the ADC disabling process
    failed, it is up to HAL_ADC_MspDeInit() to reset the ADC IP */
  ADC_Disable(hadc);

  /* ========== Reset ADC registers ========== */
  /* Reset register IER */
  __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_AWD3  | ADC_IT_AWD2 | ADC_IT_AWD1 |
                              ADC_IT_JQOVF | ADC_IT_OVR  | ADC_IT_JEOS  | ADC_IT_JEOC |
                              ADC_IT_EOS   | ADC_IT_EOC  | ADC_IT_EOSMP | ADC_IT_RDY));

  /* Reset register ISR */
  __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_AWD3  | ADC_FLAG_AWD2 | ADC_FLAG_AWD1 |
                              ADC_FLAG_JQOVF | ADC_FLAG_OVR  | ADC_FLAG_JEOS  | ADC_FLAG_JEOC |
                              ADC_FLAG_EOS   | ADC_FLAG_EOC  | ADC_FLAG_EOSMP | ADC_FLAG_RDY));

  /* Reset register CR */
 /* Bits ADC_CR_JADSTP, ADC_CR_ADSTP, ADC_CR_JADSTART, ADC_CR_ADSTART,
    ADC_CR_ADCAL, ADC_CR_ADDIS and ADC_CR_ADEN are in access mode "read-set":
    no direct reset applicable.
    Update CR register to reset value where doable by software */
  CLEAR_BIT(hadc->Instance->CR, ADC_CR_ADVREGEN | ADC_CR_ADCALDIF);
  SET_BIT(hadc->Instance->CR, ADC_CR_DEEPPWD);

  /* Reset register CFGR */
  CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_FIELDS);
  SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);

  /* Reset register CFGR2 */
  CLEAR_BIT(hadc->Instance->CFGR2, ADC_CFGR2_ROVSM  | ADC_CFGR2_TROVS   | ADC_CFGR2_OVSS |
                                  ADC_CFGR2_OVSR  | ADC_CFGR2_JOVSE | ADC_CFGR2_ROVSE    );

  /* Reset register SMPR1 */
  CLEAR_BIT(hadc->Instance->SMPR1, ADC_SMPR1_FIELDS);

  /* Reset register SMPR2 */
  CLEAR_BIT(hadc->Instance->SMPR2, ADC_SMPR2_SMP18 | ADC_SMPR2_SMP17 | ADC_SMPR2_SMP16 |
                             ADC_SMPR2_SMP15 | ADC_SMPR2_SMP14 | ADC_SMPR2_SMP13 |
                             ADC_SMPR2_SMP12 | ADC_SMPR2_SMP11 | ADC_SMPR2_SMP10);

  /* Reset register TR1 */
  CLEAR_BIT(hadc->Instance->TR1, ADC_TR1_HT1 | ADC_TR1_LT1);

  /* Reset register TR2 */
  CLEAR_BIT(hadc->Instance->TR2, ADC_TR2_HT2 | ADC_TR2_LT2);

  /* Reset register TR3 */
  CLEAR_BIT(hadc->Instance->TR3, ADC_TR3_HT3 | ADC_TR3_LT3);

  /* Reset register SQR1 */
  CLEAR_BIT(hadc->Instance->SQR1, ADC_SQR1_SQ4 | ADC_SQR1_SQ3 | ADC_SQR1_SQ2 | ADC_SQR1_SQ1 | ADC_SQR1_L);

  /* Reset register SQR2 */
  CLEAR_BIT(hadc->Instance->SQR2, ADC_SQR2_SQ9 | ADC_SQR2_SQ8 | ADC_SQR2_SQ7 | ADC_SQR2_SQ6 | ADC_SQR2_SQ5);

  /* Reset register SQR3 */
  CLEAR_BIT(hadc->Instance->SQR3, ADC_SQR3_SQ14 | ADC_SQR3_SQ13 | ADC_SQR3_SQ12 | ADC_SQR3_SQ11 | ADC_SQR3_SQ10);

  /* Reset register SQR4 */
  CLEAR_BIT(hadc->Instance->SQR4, ADC_SQR4_SQ16 | ADC_SQR4_SQ15);

  /* Register JSQR was reset when the ADC was disabled */
  /* Reset register DR */
  /* bits in access mode read only, no direct reset applicable*/

  /* Reset register OFR1 */
  CLEAR_BIT(hadc->Instance->OFR1, ADC_OFR1_OFFSET1_EN | ADC_OFR1_OFFSET1_CH | ADC_OFR1_OFFSET1);
  /* Reset register OFR2 */
  CLEAR_BIT(hadc->Instance->OFR2, ADC_OFR2_OFFSET2_EN | ADC_OFR2_OFFSET2_CH | ADC_OFR2_OFFSET2);
  /* Reset register OFR3 */
  CLEAR_BIT(hadc->Instance->OFR3, ADC_OFR3_OFFSET3_EN | ADC_OFR3_OFFSET3_CH | ADC_OFR3_OFFSET3);
  /* Reset register OFR4 */
  CLEAR_BIT(hadc->Instance->OFR4, ADC_OFR4_OFFSET4_EN | ADC_OFR4_OFFSET4_CH | ADC_OFR4_OFFSET4);

  /* Reset registers JDR1, JDR2, JDR3, JDR4 */
  /* bits in access mode read only, no direct reset applicable*/

  /* Reset register AWD2CR */
  CLEAR_BIT(hadc->Instance->AWD2CR, ADC_AWD2CR_AWD2CH);

  /* Reset register AWD3CR */
  CLEAR_BIT(hadc->Instance->AWD3CR, ADC_AWD3CR_AWD3CH);

  /* Reset register DIFSEL */
  CLEAR_BIT(hadc->Instance->DIFSEL, ADC_DIFSEL_DIFSEL);

  /* Reset register CALFACT */
  CLEAR_BIT(hadc->Instance->CALFACT, ADC_CALFACT_CALFACT_D | ADC_CALFACT_CALFACT_S);


  /* ========== Reset common ADC registers ========== */
  /* Software is allowed to change common parameters only when all the other
     ADCs are disabled.   */
  if ((ADC_IS_ENABLE(hadc) == RESET) && (ADC_ANY_OTHER_ENABLED(hadc) == RESET))
    /* Reset configuration of ADC common register CCR:
      - clock mode: CKMODE, PRESCEN
      - multimode related parameters (when this feature is available): MDMA,
        DMACFG, DELAY, DUAL (set by HAL_ADCEx_MultiModeConfigChannel() API)
      - internal measurement paths: Vbat, temperature sensor, Vref (set into
        HAL_ADC_ConfigChannel() or HAL_ADCEx_InjectedConfigChannel() )
    */
    ADC_CLEAR_COMMON_CONTROL_REGISTER(hadc);

  /* DeInit the low level hardware.
     For example:
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();
    __HAL_RCC_ADC_CLK_DISABLE();
    Keep in mind that all ADCs use the same clock: disabling
    the clock will reset all ADCs.
  */
  HAL_ADC_MspDeInit(hadc);

  /* Set ADC error code to none */
  ADC_CLEAR_ERRORCODE(hadc);

  /* Reset injected channel configuration parameters */
  hadc->InjectionConfig.ContextQueue = 0;
  hadc->InjectionConfig.ChannelCount = 0;

  /* Set ADC state */
  hadc->State = HAL_ADC_STATE_RESET;

  __HAL_UNLOCK(hadc);
  return HAL_OK;
  }
//}}}

//{{{
/**
  * @brief  Return the ADC handle state.
  * @note   ADC state machine is managed by bitfields, ADC status must be
  *         compared with states bits.
  *         For example:
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_REG_BUSY)) "
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_AWD1)    ) "
  * @param hadc ADC handle
  * @retval ADC handle state (bitfield on 32 bits)
  */
uint32_t HAL_ADC_GetState (ADC_HandleTypeDef* hadc) {
  return hadc->State;
  }
//}}}
//{{{
uint32_t HAL_ADC_GetError (ADC_HandleTypeDef* hadc) {
  return hadc->ErrorCode;
  }
//}}}
//{{{
/**
  * @brief  Get ADC regular group conversion result.
  * @note   Reading register DR automatically clears ADC flag EOC
  *         (ADC group regular end of unitary conversion).
  * @note   This function does not clear ADC flag EOS
  *         (ADC group regular end of sequence conversion).
  *         Occurrence of flag EOS rising:
  *          - If sequencer is composed of 1 rank, flag EOS is equivalent
  *            to flag EOC.
  *          - If sequencer is composed of several ranks, during the scan
  *            sequence flag EOC only is raised, at the end of the scan sequence
  *            both flags EOC and EOS are raised.
  *         To clear this flag, either use function:
  *         in programming model IT: @ref HAL_ADC_IRQHandler(), in programming
  *         model polling: @ref HAL_ADC_PollForConversion()
  *         or @ref __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOS).
  * @param hadc ADC handle
  * @retval ADC group regular conversion data
  */
uint32_t HAL_ADC_GetValue (ADC_HandleTypeDef* hadc) {

  /* Note: EOC flag is not cleared here by software because automatically     */
  /*       cleared by hardware when reading register DR.                      */
  /* Return ADC converted value */
  return hadc->Instance->DR;
  }
//}}}
//{{{
/**
  * @brief  Get ADC injected group conversion result.
  * @note   Reading register JDRx automatically clears ADC flag JEOC
  *         (ADC group injected end of unitary conversion).
  * @note   This function does not clear ADC flag JEOS
  *         (ADC group injected end of sequence conversion)
  *         Occurrence of flag JEOS rising:
  *          - If sequencer is composed of 1 rank, flag JEOS is equivalent
  *            to flag JEOC.
  *          - If sequencer is composed of several ranks, during the scan
  *            sequence flag JEOC only is raised, at the end of the scan sequence
  *            both flags JEOC and EOS are raised.
  *         Flag JEOS must not be cleared by this function because
  *         it would not be compliant with low power features
  *         (feature low power auto-wait, not available on all STM32 families).
  *         To clear this flag, either use function:
  *         in programming model IT: @ref HAL_ADC_IRQHandler(), in programming
  *         model polling: @ref HAL_ADCEx_InjectedPollForConversion()
  *         or @ref __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_JEOS).
  * @param hadc ADC handle
  * @param InjectedRank the converted ADC injected rank.
  *          This parameter can be one of the following values:
  *            @arg @ref ADC_INJECTED_RANK_1 ADC group injected rank 1
  *            @arg @ref ADC_INJECTED_RANK_2 ADC group injected rank 2
  *            @arg @ref ADC_INJECTED_RANK_3 ADC group injected rank 3
  *            @arg @ref ADC_INJECTED_RANK_4 ADC group injected rank 4
  * @retval ADC group injected conversion data
  */
uint32_t HAL_ADCEx_InjectedGetValue (ADC_HandleTypeDef* hadc, uint32_t InjectedRank)
{
  uint32_t tmp_jdr = 0;

  /* Get ADC converted value */
  switch(InjectedRank) {
    case ADC_INJECTED_RANK_4:
      tmp_jdr = hadc->Instance->JDR4;
      break;
    case ADC_INJECTED_RANK_3:
      tmp_jdr = hadc->Instance->JDR3;
      break;
    case ADC_INJECTED_RANK_2:
      tmp_jdr = hadc->Instance->JDR2;
      break;
    case ADC_INJECTED_RANK_1:
    default:
      tmp_jdr = hadc->Instance->JDR1;
      break;
    }

  /* Return ADC converted value */
  return tmp_jdr;
  }
//}}}

//{{{
/**
  * @brief  Enable ADC, start conversion of regular group with interruption.
  * @note   Interruptions enabled in this function according to initialization
  *         setting : EOC (end of conversion), EOS (end of sequence),
  *         OVR overrun.
  *         Each of these interruptions has its dedicated callback function.
  * @note   Case of multimode enabled (when multimode feature is available):
  *         HAL_ADC_Start_IT() must be called for ADC Slave first, then for
  *         ADC Master.
  *         For ADC Slave, ADC is enabled only (conversion is not started).
  *         For ADC Master, ADC is enabled and multimode conversion is started.
  * @note   To guarantee a proper reset of all interruptions once all the needed
  *         conversions are obtained, HAL_ADC_Stop_IT() must be called to ensure
  *         a correct stop of the IT-based conversions.
  * @note   By default, HAL_ADC_Start_IT() does not enable the End Of Sampling
  *         interruption. If required (e.g. in case of oversampling with trigger
  *         mode), the user must:
  *          1. first clear the EOSMP flag if set with macro __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOSMP)
  *          2. then enable the EOSMP interrupt with macro __HAL_ADC_ENABLE_IT(hadc, ADC_IT_EOSMP)
  *          before calling HAL_ADC_Start_IT().
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_Start_IT (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  ADC_TypeDef* tmpADC_Master;

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
    __HAL_LOCK(hadc);

    /* Enable the ADC peripheral */
    tmp_hal_status = ADC_Enable(hadc);

    /* Start conversion if ADC is effectively enabled */
    if (tmp_hal_status == HAL_OK) {
      /* Set ADC state                                                        */
      /* - Clear state bitfield related to regular group conversion results   */
      /* - Set state bitfield related to regular operation                    */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                        HAL_ADC_STATE_REG_BUSY);

      /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
        - by default if ADC is Master or Independent or if multimode feature is not available
        - if multimode setting is set to independent mode (no dual regular or injected conversions are configured) */
      if (ADC_NONMULTIMODE_OR_MULTIMODEMASTER(hadc))
        CLEAR_BIT (hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);

      /* Set ADC error code */
      /* Check if a conversion is on going on ADC group injected */
      if (HAL_IS_BIT_SET (hadc->State, HAL_ADC_STATE_INJ_BUSY))
        /* Reset ADC error code fields related to regular conversions only */
        CLEAR_BIT (hadc->ErrorCode, (HAL_ADC_ERROR_OVR|HAL_ADC_ERROR_DMA));
      else
        /* Reset all ADC error code fields */
        ADC_CLEAR_ERRORCODE (hadc);

      /* Clear ADC group regular conversion flag and overrun flag               */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG (hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK (hadc);

      /* Disable all interruptions before enabling the desired ones */
      __HAL_ADC_DISABLE_IT (hadc, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));

      /* Enable ADC end of conversion interrupt */
      switch (hadc->Init.EOCSelection) {
        case ADC_EOC_SEQ_CONV:
          __HAL_ADC_ENABLE_IT (hadc, ADC_IT_EOS);
          break;
        /* case ADC_EOC_SINGLE_CONV */
        default:
          __HAL_ADC_ENABLE_IT (hadc, ADC_IT_EOC);
          break;
        }

      /* Enable ADC overrun interrupt */
      /* If hadc->Init.Overrun is set to ADC_OVR_DATA_PRESERVED, only then is
         ADC_IT_OVR enabled; otherwise data overwrite is considered as normal
         behavior and no CPU time is lost for a non-processed interruption */
      if (hadc->Init.Overrun == ADC_OVR_DATA_PRESERVED)
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

      /* Enable conversion of regular group.                                  */
      /* If software start has been selected, conversion starts immediately.  */
      /* If external trigger has been selected, conversion will start at next */
      /* trigger event.                                                       */
      /* Case of multimode enabled (when multimode feature is available):     */
      /*  - if ADC is slave and dual regular conversions are enabled, ADC is  */
      /*    enabled only (conversion is not started),                         */
      /*  - if ADC is master, ADC is enabled and conversion is started.       */
      if(ADC_INDEPENDENT_OR_NONMULTIMODEREGULAR_SLAVE(hadc)) {
        /* Multimode feature is not available or ADC Instance is Independent or Master,
           or is not Slave ADC with dual regular conversions enabled.
           Then set HAL_ADC_STATE_INJ_BUSY and reset HAL_ADC_STATE_INJ_EOC if JAUTO is set. */
        if (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO) != RESET) {
          ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

          /* Enable as well injected interruptions in case
           HAL_ADCEx_InjectedStart_IT() has not been called beforehand. This
           allows to start regular and injected conversions when JAUTO is
           set with a single call to HAL_ADC_Start_IT() */
          switch(hadc->Init.EOCSelection) {
            case ADC_EOC_SEQ_CONV:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
            break;
            /* case ADC_EOC_SINGLE_CONV */
            default:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
            break;
            }
          }

        /* Start ADC group regular conversion */
        LL_ADC_REG_StartConversion(hadc->Instance);
        }

      else {
        /* hadc is the handle of a Slave ADC with dual regular conversions
           enabled. Therefore, ADC_CR_ADSTART is NOT set */
        SET_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
        /* if Master ADC JAUTO bit is set, Slave injected interruptions
           are enabled nevertheless (for same reason as above) */
        tmpADC_Master = ADC_MASTER_REGISTER(hadc);
        if (READ_BIT(tmpADC_Master->CFGR, ADC_CFGR_JAUTO) != RESET) {
          /* First, update Slave State in setting HAL_ADC_STATE_INJ_BUSY bit
             and in resetting HAL_ADC_STATE_INJ_EOC bit */
          ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
          /* Next, set Slave injected interruptions */
          switch(hadc->Init.EOCSelection) {
            case ADC_EOC_SEQ_CONV:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
            break;
            /* case ADC_EOC_SINGLE_CONV */
            default:
              __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
              __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
            break;
            }
          }
        }
      }
    else
      __HAL_UNLOCK(hadc);
    }
  else
    tmp_hal_status = HAL_BUSY;

  return tmp_hal_status;
  }
//}}}
//{{{
/**
  * @brief  Stop ADC conversion of regular group (and injected group in
  *         case of auto_injection mode), disable interrution of
  *         end-of-conversion, disable ADC peripheral.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop_IT (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  __HAL_LOCK(hadc);

  /* 1. Stop potential conversion on going, on ADC groups regular and injected */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK) {
    /* Disable ADC end of conversion interrupt for regular group */
    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));

    /* 2. Disable the ADC peripheral */
    tmp_hal_status = ADC_Disable(hadc);

    /* Check if ADC is effectively disabled */
    if (tmp_hal_status == HAL_OK)
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                        HAL_ADC_STATE_READY);
    }

  __HAL_UNLOCK(hadc);
  return tmp_hal_status;
  }
//}}}
//{{{
/**
  * @brief  Enable ADC, start conversion of injected group with interruption.
  * @note   Interruptions enabled in this function according to initialization
  *         setting : JEOC (end of conversion) or JEOS (end of sequence)
  * @note   Case of multimode enabled (when multimode feature is enabled):
  *         HAL_ADCEx_InjectedStart_IT() API must be called for ADC slave first,
  *         then for ADC master.
  *         For ADC slave, ADC is enabled only (conversion is not started).
  *         For ADC master, ADC is enabled and multimode conversion is started.
  * @param hadc ADC handle.
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  if (ADC_IS_CONVERSION_ONGOING_INJECTED(hadc))
    return HAL_BUSY;
  else {
    /* In case of software trigger detection enabled, JQDIS must be set
      (which can be done only if ADSTART and JADSTART are both cleared).
       If JQDIS is not set at that point, returns an error
       - since software trigger detection is disabled. User needs to
       resort to HAL_ADCEx_DisableInjectedQueue() API to set JQDIS.
       - or (if JQDIS is intentionally reset) since JEXTEN = 0 which means
         the queue is empty */
    if ((READ_BIT(hadc->Instance->JSQR, ADC_JSQR_JEXTEN) == RESET)
    && (READ_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS) == RESET)) {
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
      return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hadc);

    /* Enable the ADC peripheral */
    tmp_hal_status = ADC_Enable(hadc);

    /* Start conversion if ADC is effectively enabled */
    if (tmp_hal_status == HAL_OK) {
      /* Check if a regular conversion is ongoing */
      if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_REG_BUSY))
        /* Reset ADC error code field related to injected conversions only */
        CLEAR_BIT(hadc->ErrorCode, HAL_ADC_ERROR_JQOVF);
      else
        /* Set ADC error code to none */
        ADC_CLEAR_ERRORCODE(hadc);

      /* Set ADC state                                                        */
      /* - Clear state bitfield related to injected group conversion results  */
      /* - Set state bitfield related to injected operation                   */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC,
                        HAL_ADC_STATE_INJ_BUSY);

      /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
        - by default if ADC is Master or Independent or if multimode feature is not available
        - if multimode setting is set to independent mode (no dual regular or injected conversions are configured) */
      if (ADC_NONMULTIMODE_OR_MULTIMODEMASTER(hadc))
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);

      /* Clear ADC group injected group conversion flag */
      /* (To ensure of no unknown state from potential previous ADC operations) */
      __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JEOC | ADC_FLAG_JEOS));

      /* Process unlocked */
      /* Unlock before starting ADC conversions: in case of potential         */
      /* interruption, to let the process to ADC IRQ Handler.                 */
      __HAL_UNLOCK(hadc);

      /* Enable ADC Injected context queue overflow interrupt if this feature   */
      /* is enabled.                                                            */
      if ((hadc->Instance->CFGR & ADC_CFGR_JQM) != RESET)
        __HAL_ADC_ENABLE_IT(hadc, ADC_FLAG_JQOVF);

      /* Enable ADC end of conversion interrupt */
      switch(hadc->Init.EOCSelection) {
        case ADC_EOC_SEQ_CONV:
          __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);
          __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOS);
          break;
        /* case ADC_EOC_SINGLE_CONV */
        default:
          __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOS);
          __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);
          break;
        }

      /* Enable conversion of injected group, if automatic injected conversion  */
      /* is disabled.                                                           */
      /* If software start has been selected, conversion starts immediately.    */
      /* If external trigger has been selected, conversion will start at next   */
      /* trigger event.                                                         */
      /* Case of multimode enabled (when multimode feature is available):       */
      /* if ADC is slave,                                                       */
      /*    - ADC is enabled only (conversion is not started),                  */
      /*    - if multimode only concerns regular conversion, ADC is enabled     */
      /*     and conversion is started.                                         */
      /* If ADC is master or independent,                                       */
      /*    - ADC is enabled and conversion is started.                         */

      /* Are injected conversions that of a dual Slave ? */
      if (ADC_INDEPENDENT_OR_NONMULTIMODEINJECTED_SLAVE(hadc)) {
        /* hadc is not the handle of a Slave ADC with dual injected conversions enabled:
           set ADSTART only if JAUTO is cleared */
        if (HAL_IS_BIT_CLR(hadc->Instance->CFGR, ADC_CFGR_JAUTO))
          SET_BIT(hadc->Instance->CR, ADC_CR_JADSTART) ;
        }
      else
        /* hadc is the handle of a Slave ADC with dual injected conversions enabled: ADSTART is not set */
          SET_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
      }
    else
      __HAL_UNLOCK(hadc);

    return tmp_hal_status;
    }
  }
//}}}
//{{{
/**
  * @brief  Stop conversion of injected channels, disable interruption of
  *         end-of-conversion. Disable ADC peripheral if no regular conversion
  *         is on going.
  * @note   If ADC must be disabled and if conversion is on going on
  *         regular group, function HAL_ADC_Stop must be used to stop both
  *         injected and regular groups, and disable the ADC.
  * @note   If injected group mode auto-injection is enabled,
  *         function HAL_ADC_Stop must be used.
  * @note   Case of multimode enabled (when multimode feature is available):
  *         HAL_ADCEx_InjectedStop_IT() API must be called for ADC master first,
  *         then for ADC slave.
  *         For ADC master, conversion is stopped and ADC is disabled.
  *         For ADC slave, ADC is disabled only (conversion stop of ADC master
  *         has already stopped conversion of ADC slave).
  * @note   In case of auto-injection mode, HAL_ADC_Stop() must be used.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential conversion on going on injected group only. */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_INJECTED_GROUP);

  /* Disable ADC peripheral if injected conversions are effectively stopped   */
  /* and if no conversion on the other group (regular group) is intended to   */
  /* continue.                                                                */
  if (tmp_hal_status == HAL_OK) {
    /* Disable ADC end of conversion interrupt for injected channels */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_JEOC | ADC_IT_JEOS | ADC_FLAG_JQOVF));

    if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
      /* 2. Disable the ADC peripheral */
      tmp_hal_status = ADC_Disable(hadc);

      /* Check if ADC is effectively disabled */
      if (tmp_hal_status == HAL_OK) {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
        }
      }
    /* Conversion on injected group is stopped, but ADC not disabled since    */
    /* conversion on regular group is still running.                          */
    else
      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
  }
//}}}
//{{{
/**
  * @brief  Stop ADC conversion of ADC groups regular and injected,
  *         disable interrution of end-of-conversion,
  *         disable ADC peripheral if no conversion is on going
  *         on injected group.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_RegularStop_IT (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  __HAL_LOCK(hadc);

  /* 1. Stop potential regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped
    and if no injected conversion is on-going */
  if (tmp_hal_status == HAL_OK) {
    /* Clear HAL_ADC_STATE_REG_BUSY bit */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

    /* Disable all regular-related interrupts */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_EOC | ADC_IT_EOS | ADC_IT_OVR));

    /* 2. Disable ADC peripheral if no injected conversions are on-going */
    if (ADC_IS_CONVERSION_ONGOING_INJECTED(hadc) == RESET) {
      tmp_hal_status = ADC_Disable(hadc);
      /* if no issue reported */
      if (tmp_hal_status == HAL_OK)
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_READY);
      }
    else
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }

  __HAL_UNLOCK(hadc);
  return tmp_hal_status;
  }
//}}}

//{{{
/**
  * @brief  Enable ADC, start conversion of regular group and transfer result through DMA.
  * @note   Interruptions enabled in this function:
  *         overrun (if applicable), DMA half transfer, DMA transfer complete.
  *         Each of these interruptions has its dedicated callback function.
  * @note   Case of multimode enabled (when multimode feature is available): HAL_ADC_Start_DMA()
  *         is designed for single-ADC mode only. For multimode, the dedicated
  *         HAL_ADCEx_MultiModeStart_DMA() function must be used.
  * @param hadc ADC handle
  * @param pData Destination Buffer address.
  * @param Length Number of data to be transferred from ADC peripheral to memory
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Start_DMA (ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
    /* Process locked */
    __HAL_LOCK(hadc);

    /* Ensure that dual regular conversions are not enabled or unavailable.   */
    /* Otherwise, dedicated API HAL_ADCEx_MultiModeStart_DMA() must be used.  */
    if (ADC_IS_DUAL_REGULAR_CONVERSION_ENABLE(hadc) == RESET) {
      /* Enable the ADC peripheral */
      tmp_hal_status = ADC_Enable(hadc);

      /* Start conversion if ADC is effectively enabled */
      if (tmp_hal_status == HAL_OK) {
        /* Set ADC state                                                        */
        /* - Clear state bitfield related to regular group conversion results   */
        /* - Set state bitfield related to regular operation                    */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
                          HAL_ADC_STATE_REG_BUSY);

        /* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
          - by default if ADC is Master or Independent or if multimode feature is not available
          - if multimode setting is set to independent mode (no dual regular or injected conversions are configured) */
        if (ADC_NONMULTIMODE_OR_MULTIMODEMASTER(hadc))
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);

        /* Check if a conversion is on going on ADC group injected */
        if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
          /* Reset ADC error code fields related to regular conversions only */
          CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
        else
          /* Reset all ADC error code fields */
          ADC_CLEAR_ERRORCODE(hadc);

        /* Set the DMA transfer complete callback */
        hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

        /* Set the DMA half transfer complete callback */
        hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

        /* Set the DMA error callback */
        hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;

        /* Manage ADC and DMA start: ADC overrun interruption, DMA start,     */
        /* ADC start (in case of SW start):                                   */
        /* Clear regular group conversion flag and overrun flag               */
        /* (To ensure of no unknown state from potential previous ADC         */
        /* operations)                                                        */
        __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

        /* Process unlocked */
        /* Unlock before starting ADC conversions: in case of potential         */
        /* interruption, to let the process to ADC IRQ Handler.                 */
        __HAL_UNLOCK(hadc);

        /* With DMA, overrun event is always considered as an error even if
           hadc->Init.Overrun is set to ADC_OVR_DATA_OVERWRITTEN. Therefore,
           ADC_IT_OVR is enabled. */
        __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

        /* Enable ADC DMA mode */
        SET_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

        /* Start the DMA channel */
        HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

        /* Enable conversion of regular group.                                  */
        /* If software start has been selected, conversion starts immediately.  */
        /* If external trigger has been selected, conversion will start at next */
        /* trigger event.                                                       */
        /* Start ADC group regular conversion */
        LL_ADC_REG_StartConversion(hadc->Instance);
        }
      else
        __HAL_UNLOCK(hadc);
      }
    else {
      tmp_hal_status = HAL_ERROR;
      __HAL_UNLOCK(hadc);
      }
    }
  else
    tmp_hal_status = HAL_BUSY;

  return tmp_hal_status;
  }
//}}}
//{{{
/**
  * @brief  Stop ADC conversion of regular group (and injected group in
  *         case of auto_injection mode), disable ADC DMA transfer, disable
  *         ADC peripheral.
  * @note:  ADC peripheral disable is forcing stop of potential
  *         conversion on ADC group injected. If ADC group injected is under use, it
  *         should be preliminarily stopped using HAL_ADCEx_InjectedStop function.
  * @note   Case of multimode enabled (when multimode feature is available):
  *         HAL_ADC_Stop_DMA() function is dedicated to single-ADC mode only.
  *         For multimode, the dedicated HAL_ADCEx_MultiModeStop_DMA() API must be used.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADC_Stop_DMA (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  __HAL_LOCK(hadc);

  /* 1. Stop potential ADC group regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped */
  if (tmp_hal_status == HAL_OK) {
    /* Disable ADC DMA (ADC DMA configuration of continous requests is kept) */
    CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

    /* Disable the DMA channel (in case of DMA in circular mode or stop       */
    /* while DMA transfer is on going)                                        */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Check if DMA channel effectively disabled */
    if (tmp_hal_status != HAL_OK)
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* 2. Disable the ADC peripheral */
    /* Update "tmp_hal_status" only if DMA channel disabling passed, to keep  */
    /* in memory a potential failing status.                                  */
    if (tmp_hal_status == HAL_OK)
      tmp_hal_status = ADC_Disable(hadc);
    else
      ADC_Disable(hadc);

    /* Check if ADC is effectively disabled */
    if (tmp_hal_status == HAL_OK)
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_READY);
    }

  __HAL_UNLOCK(hadc);
  return tmp_hal_status;
  }
//}}}
//{{{
/**
  * @brief  Stop ADC conversion of regular group (and injected group in
  *         case of auto_injection mode), disable ADC DMA transfer, disable
  *         ADC peripheral if no conversion is on going
  *         on injected group.
  * @note   HAL_ADCEx_RegularStop_DMA() function is dedicated to single-ADC mode only.
  *         For multimode (when multimode feature is available),
  *         HAL_ADCEx_RegularMultiModeStop_DMA() API must be used.
  * @param hadc ADC handle
  * @retval HAL status.
  */
HAL_StatusTypeDef HAL_ADCEx_RegularStop_DMA (ADC_HandleTypeDef* hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hadc);

  /* 1. Stop potential regular conversion on going */
  tmp_hal_status = ADC_ConversionStop(hadc, ADC_REGULAR_GROUP);

  /* Disable ADC peripheral if conversions are effectively stopped
     and if no injected conversion is on-going */
  if (tmp_hal_status == HAL_OK)
  {
    /* Clear HAL_ADC_STATE_REG_BUSY bit */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

    /* Disable ADC DMA (ADC DMA configuration ADC_CFGR_DMACFG is kept) */
    CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

    /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* while DMA transfer is on going)                                        */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Check if DMA channel effectively disabled */
    if (tmp_hal_status != HAL_OK)
    {
      /* Update ADC state machine to error */
      SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);
    }

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* 2. Disable the ADC peripheral */
    /* Update "tmp_hal_status" only if DMA channel disabling passed, to keep in */
    /* memory a potential failing status.                                     */
    if (ADC_IS_CONVERSION_ONGOING_INJECTED(hadc) == RESET)
    {
      if (tmp_hal_status == HAL_OK)
      {
        tmp_hal_status = ADC_Disable(hadc);
      }
      else
      {
        ADC_Disable(hadc);
      }

      /* Check if ADC is effectively disabled */
      if (tmp_hal_status == HAL_OK)
      {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_INJ_BUSY,
                          HAL_ADC_STATE_READY);
      }
    }
    else
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
    }
  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}
//}}}

//{{{
void HAL_ADC_IRQHandler (ADC_HandleTypeDef* hadc) {

  uint32_t overrun_error = 0; /* flag set if overrun occurrence has to be considered as an error */
  uint32_t tmp_isr = hadc->Instance->ISR;
  uint32_t tmp_ier = hadc->Instance->IER;
  uint32_t tmp_cfgr = 0x0;
  ADC_TypeDef *tmpADC_Master;

  /* ========== Check End of Sampling flag for ADC group regular ========== */
  if (((tmp_isr & ADC_FLAG_EOSMP) == ADC_FLAG_EOSMP) && ((tmp_ier & ADC_IT_EOSMP) == ADC_IT_EOSMP)) {
    /* Update state machine on end of sampling status if not in error state */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOSMP);
      /* End Of Sampling callback */
      HAL_ADCEx_EndOfSamplingCallback(hadc);
      /* Clear regular group conversion flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOSMP );
     }

  /* ====== Check ADC group regular end of unitary conversion sequence conversions ===== */
  if ((((tmp_isr & ADC_FLAG_EOC) == ADC_FLAG_EOC) && ((tmp_ier & ADC_IT_EOC) == ADC_IT_EOC)) ||
      (((tmp_isr & ADC_FLAG_EOS) == ADC_FLAG_EOS) && ((tmp_ier & ADC_IT_EOS) == ADC_IT_EOS))  ) {
    /* Update state machine on conversion status if not in error state */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular     */
    /* by external trigger, continuous mode or scan sequence on going         */
    /* to disable interruption.                                               */
    if(ADC_IS_SOFTWARE_START_REGULAR(hadc)) {
      /* Get relevant register CFGR in ADC instance of ADC master or slave    */
      /* in function of multimode state (for devices with multimode           */
      /* available).                                                          */
      if (ADC_INDEPENDENT_OR_NONMULTIMODEREGULAR_SLAVE(hadc))
        /* check CONT bit directly in handle ADC CFGR register */
        tmp_cfgr = READ_REG(hadc->Instance->CFGR);
      else {
        /* else need to check Master ADC CONT bit */
        tmpADC_Master = ADC_MASTER_REGISTER(hadc);
        tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
        }

      /* Carry on if continuous mode is disabled */
      if (READ_BIT (tmp_cfgr, ADC_CFGR_CONT) != ADC_CFGR_CONT) {
        /* If End of Sequence is reached, disable interrupts */
        if( __HAL_ADC_GET_FLAG (hadc, ADC_FLAG_EOS) ) {
          /* Allowed to modify bits ADC_IT_EOC/ADC_IT_EOS only if bit         */
          /* ADSTART==0 (no conversion on going)                              */
          if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
            /* Disable ADC end of sequence conversion interrupt */
            /* Note: Overrun interrupt was enabled with EOC interrupt in      */
            /* HAL_Start_IT(), but is not disabled here because can be used   */
            /* by overrun IRQ process below.                                  */
            __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC | ADC_IT_EOS);

            /* Set ADC state */
            CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
            if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
              SET_BIT(hadc->State, HAL_ADC_STATE_READY);
            }
          else {
            /* Change ADC state to error state */
            SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

            /* Set ADC error code to ADC IP internal error */
            SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
            }
          }
        }
      }

    /* Conversion complete callback */
    /* Note: Into callback function "HAL_ADC_ConvCpltCallback()",             */
    /*       to determine if conversion has been triggered from EOC or EOS,   */
    /*       possibility to use:                                              */
    /*        " if( __HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOS)) "                */
    HAL_ADC_ConvCpltCallback (hadc);

    /* Clear regular group conversion flag */
    /* Note: in case of overrun set to ADC_OVR_DATA_PRESERVED, end of         */
    /*       conversion flags clear induces the release of the preserved data.*/
    /*       Therefore, if the preserved data value is needed, it must be     */
    /*       read preliminarily into HAL_ADC_ConvCpltCallback().              */
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS) );
    }

  /* ====== Check ADC group injected end of unitary conversion sequence conversions ===== */
  if( (((tmp_isr & ADC_FLAG_JEOC) == ADC_FLAG_JEOC) && ((tmp_ier & ADC_IT_JEOC) == ADC_IT_JEOC)) ||
      (((tmp_isr & ADC_FLAG_JEOS) == ADC_FLAG_JEOS) && ((tmp_ier & ADC_IT_JEOS) == ADC_IT_JEOS))  ) {
    /* Update state machine on conversion status if not in error state */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
      /* Set ADC state */
      SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);

    /* Get relevant register CFGR in ADC instance of ADC master or slave  */
    /* in function of multimode state (for devices with multimode         */
    /* available).                                                        */
    if (ADC_INDEPENDENT_OR_NONMULTIMODEINJECTED_SLAVE(hadc))
      tmp_cfgr = READ_REG(hadc->Instance->CFGR);
    else {
      tmpADC_Master = ADC_MASTER_REGISTER(hadc);
      tmp_cfgr = READ_REG(tmpADC_Master->CFGR);
      }

    /* Disable interruption if no further conversion upcoming by injected     */
    /* external trigger or by automatic injected conversion with regular      */
    /* group having no further conversion upcoming (same conditions as        */
    /* regular group interruption disabling above),                           */
    /* and if injected scan sequence is completed.                            */
    if (ADC_IS_SOFTWARE_START_INJECTED(hadc)                   ||
       ((READ_BIT (tmp_cfgr, ADC_CFGR_JAUTO) == RESET)    &&
        (ADC_IS_SOFTWARE_START_REGULAR(hadc)          &&
        (READ_BIT (tmp_cfgr, ADC_CFGR_CONT) == RESET)   )   )   ) {
      /* If End of Sequence is reached, disable interrupts */
      if( __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS)) {
        /* Particular case if injected contexts queue is enabled:             */
        /* when the last context has been fully processed, JSQR is reset      */
        /* by the hardware. Even if no injected conversion is planned to come */
        /* (queue empty, triggers are ignored), it can start again            */
        /* immediately after setting a new context (JADSTART is still set).   */
        /* Therefore, state of HAL ADC injected group is kept to busy.        */
        if(READ_BIT(tmp_cfgr, ADC_CFGR_JQM) == RESET) {
          /* Allowed to modify bits ADC_IT_JEOC/ADC_IT_JEOS only if bit       */
          /* JADSTART==0 (no conversion on going)                             */
          if (ADC_IS_CONVERSION_ONGOING_INJECTED(hadc) == RESET) { /* Disable ADC end of sequence conversion interrupt  */
            __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC | ADC_IT_JEOS);
            /* Set ADC state */
            CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);
            if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_REG_BUSY))
              SET_BIT(hadc->State, HAL_ADC_STATE_READY);
            }
          else {
            /* Update ADC state machine to error */
            SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
            /* Set ADC error code to ADC IP internal error */
            SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
            }
          }
        }
      }

    /* Injected Conversion complete callback */
    /* Note:  HAL_ADCEx_InjectedConvCpltCallback can resort to
              if( __HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_JEOS)) or
              if( __HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_JEOC)) to determine whether
              interruption has been triggered by end of conversion or end of
              sequence.    */
    HAL_ADCEx_InjectedConvCpltCallback(hadc);

    /* Clear injected group conversion flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
  }

  /* ========== Check Analog watchdog 1 flag ========== */
  if (((tmp_isr & ADC_FLAG_AWD1) == ADC_FLAG_AWD1) && ((tmp_ier & ADC_IT_AWD1) == ADC_IT_AWD1)) {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD1);
    /* Level out of window 1 callback */
    HAL_ADC_LevelOutOfWindowCallback(hadc);
    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD1);
    }

  /* ========== Check analog watchdog 2 flag ========== */
  if (((tmp_isr & ADC_FLAG_AWD2) == ADC_FLAG_AWD2) && ((tmp_ier & ADC_IT_AWD2) == ADC_IT_AWD2)) {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD2);
    /* Level out of window 2 callback */
    HAL_ADCEx_LevelOutOfWindow2Callback(hadc);
    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD2);
    }

  /* ========== Check analog watchdog 3 flag ========== */
  if (((tmp_isr & ADC_FLAG_AWD3) == ADC_FLAG_AWD3) && ((tmp_ier & ADC_IT_AWD3) == ADC_IT_AWD3)) {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD3);
    /* Level out of window 3 callback */
    HAL_ADCEx_LevelOutOfWindow3Callback(hadc);
    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD3);
    }

  /* ========== Check Overrun flag ========== */
  if (((tmp_isr & ADC_FLAG_OVR) == ADC_FLAG_OVR) && ((tmp_ier & ADC_IT_OVR) == ADC_IT_OVR)) {
    /* If overrun is set to overwrite previous data (default setting),        */
    /* overrun event is not considered as an error.                           */
    /* (cf ref manual "Managing conversions without using the DMA and without */
    /* overrun ")                                                             */
    /* Exception for usage with DMA overrun event always considered as an     */
    /* error.                                                                 */
    if (hadc->Init.Overrun == ADC_OVR_DATA_PRESERVED)
      overrun_error = 1;
    else {
      /* Check DMA configuration */
      if (ADC_IS_DUAL_CONVERSION_ENABLE(hadc) == RESET) {
        /* Multimode not set or feature not available or ADC independent */
        if (HAL_IS_BIT_SET(hadc->Instance->CFGR, ADC_CFGR_DMAEN))
          overrun_error = 1;
        }
      else {
        /* Multimode (when feature is available) is enabled,
           Common Control Register MDMA bits must be checked. */
        if (ADC_MULTIMODE_DMA_ENABLED(hadc))
          overrun_error = 1;
        }
      }

    if (overrun_error == 1) {
      /* Change ADC state to error state */
      SET_BIT(hadc->State, HAL_ADC_STATE_REG_OVR);
      /* Set ADC error code to overrun */
      SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_OVR);
      /* Error callback */
      /* Note: In case of overrun, ADC conversion data is preserved until     */
      /*       flag OVR is reset.                                             */
      /*       Therefore, old ADC conversion data can be retrieved in         */
      /*       function "HAL_ADC_ErrorCallback()".                            */
      HAL_ADC_ErrorCallback(hadc);
     }

    /* Clear ADC overrun flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
    }

  /* ========== Check Injected context queue overflow flag ========== */
  if (((tmp_isr & ADC_FLAG_JQOVF) == ADC_FLAG_JQOVF) && ((tmp_ier & ADC_IT_JQOVF) == ADC_IT_JQOVF)) {
    /* Change ADC state to overrun state */
    SET_BIT(hadc->State, HAL_ADC_STATE_INJ_JQOVF);
    /* Set ADC error code to Injected context queue overflow */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_JQOVF);
    /* Clear the Injected context queue overflow flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JQOVF);
    /* Error callback */
    HAL_ADCEx_InjectedQueueOverflowCallback(hadc);
    }
  }
//}}}

//{{{
/**
  * @brief  Configure a channel to be assigned to ADC group regular.
  * @note   In case of usage of internal measurement channels:
  *         Vbat/VrefInt/TempSensor.
  *         These internal paths can be disabled using function
  *         HAL_ADC_DeInit().
  * @note   Possibility to update parameters on the fly:
  *         This function initializes channel into ADC group regular,
  *         following calls to this function can be used to reconfigure
  *         some parameters of structure "ADC_ChannelConfTypeDef" on the fly,
  *         without resetting the ADC.
  *         The setting of these parameters is conditioned to ADC state:
  *         Refer to comments of structure "ADC_ChannelConfTypeDef".
  * @param hadc ADC handle
  * @param sConfig Structure of ADC channel assigned to ADC group regular.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_ConfigChannel (ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpOffsetShifted;
  __IO uint32_t wait_loop_index = 0;

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on regular group:                                    */
  /*  - Channel number                                                        */
  /*  - Channel rank                                                          */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
    /* Set ADC group regular sequence: channel on the selected scan sequence rank */
    LL_ADC_REG_SetSequencerRanks (hadc->Instance, sConfig->Rank, sConfig->Channel);

    /* Parameters update conditioned to ADC state:                              */
    /* Parameters that can be updated when ADC is disabled or enabled without   */
    /* conversion on going on regular group:                                    */
    /*  - Channel sampling time                                                 */
    /*  - Channel offset                                                        */
    if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET) {
#if defined(ADC_SMPR1_SMPPLUS)
      /* Manage specific case of sampling time 3.5 cycles replacing 2.5 cyles */
      if(sConfig->SamplingTime == ADC_SAMPLETIME_3CYCLES_5) {
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, LL_ADC_SAMPLINGTIME_2CYCLES_5);

        /* Set ADC sampling time common configuration */
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_3C5_REPL_2C5);
      }
      else {
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfig->Channel, sConfig->SamplingTime);

        /* Set ADC sampling time common configuration */
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_DEFAULT);
      }
#else
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime (hadc->Instance, sConfig->Channel, sConfig->SamplingTime);
#endif

      /* Configure the offset: offset enable/disable, channel, offset value */
      /* Shift the offset with respect to the selected ADC resolution. */
      /* Offset has to be left-aligned on bit 11, the LSB (right bits) are set to 0 */
      tmpOffsetShifted = ADC_OFFSET_SHIFT_RESOLUTION(hadc, sConfig->Offset);
      if (sConfig->OffsetNumber != ADC_OFFSET_NONE) {
        /* Set ADC selected offset number */
        LL_ADC_SetOffset (hadc->Instance, sConfig->OffsetNumber, sConfig->Channel, tmpOffsetShifted);

        }
      else {
        /* Scan each offset register to check if the selected channel is targeted. */
        /* If this is the case, the corresponding offset number is disabled.       */
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_1)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
          LL_ADC_SetOffsetState (hadc->Instance, LL_ADC_OFFSET_1, LL_ADC_OFFSET_DISABLE);
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_2)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
          LL_ADC_SetOffsetState (hadc->Instance, LL_ADC_OFFSET_2, LL_ADC_OFFSET_DISABLE);
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_3)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
          LL_ADC_SetOffsetState (hadc->Instance, LL_ADC_OFFSET_3, LL_ADC_OFFSET_DISABLE);
        if (__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_4)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel))
          LL_ADC_SetOffsetState (hadc->Instance, LL_ADC_OFFSET_4, LL_ADC_OFFSET_DISABLE);
        }
      }

    /* Parameters update conditioned to ADC state:                              */
    /* Parameters that can be updated only when ADC is disabled:                */
    /*  - Single or differential mode                                           */
    /*  - Internal measurement channels: Vbat/VrefInt/TempSensor                */
    if (ADC_IS_ENABLE(hadc) == RESET) {
      /* Set mode single-ended or differential input of the selected ADC channel */
      LL_ADC_SetChannelSingleDiff(hadc->Instance, sConfig->Channel, sConfig->SingleDiff);

      /* Configuration of differential mode */
      if (sConfig->SingleDiff == ADC_DIFFERENTIAL_ENDED)
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, __LL_ADC_DECIMAL_NB_TO_CHANNEL(__LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfig->Channel) + 1), sConfig->SamplingTime);

      /* Management of internal measurement channels: Vbat/VrefInt/TempSensor.  */
      /* If internal channel selected, enable dedicated internal buffers and paths */
      /* Note: these internal measurement paths can be disabled using           */
      /* HAL_ADC_DeInit().                                                      */

      /* Configuration of common ADC parameters                                 */
      /* If the requested internal measurement path has already been enabled,   */
      /* bypass the configuration processing.                                   */
      if (( (sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) &&
            ((LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) & LL_ADC_PATH_INTERNAL_TEMPSENSOR) == 0U)) ||
          ( (sConfig->Channel == ADC_CHANNEL_VBAT)       &&
            ((LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) & LL_ADC_PATH_INTERNAL_VBAT) == 0U))       ||
          ( (sConfig->Channel == ADC_CHANNEL_VREFINT)    &&
            ((LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) & LL_ADC_PATH_INTERNAL_VREFINT) == 0U))
         ) {
        /* Configuration of common ADC parameters (continuation)                */
        /* Software is allowed to change common parameters only when all ADCs   */
        /* of the common group are disabled.                                    */
        if ((ADC_IS_ENABLE(hadc) == RESET)   &&
           (ADC_ANY_OTHER_ENABLED(hadc) == RESET) ) {
          if (sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) {
            if (ADC_TEMPERATURE_SENSOR_INSTANCE(hadc)) {
              LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance), LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)));

              /* Delay for temperature sensor stabilization time */
              /* Wait loop initialization and execution */
              /* Note: Variable divided by 2 to compensate partially          */
              /*       CPU processing cycles.                                 */
              wait_loop_index = (LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / (1000000 * 2)));
              while(wait_loop_index != 0)
                wait_loop_index--;
              }
            }
          else if (sConfig->Channel == ADC_CHANNEL_VBAT) {
            if (ADC_BATTERY_VOLTAGE_INSTANCE(hadc))
              LL_ADC_SetCommonPathInternalCh (__LL_ADC_COMMON_INSTANCE(hadc->Instance), LL_ADC_PATH_INTERNAL_VBAT | LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)));
            }
          else if (sConfig->Channel == ADC_CHANNEL_VREFINT) {
            if (ADC_VREFINT_INSTANCE(hadc))
              LL_ADC_SetCommonPathInternalCh (__LL_ADC_COMMON_INSTANCE(hadc->Instance), LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)));
            }
          }
        /* If the requested internal measurement path has already been          */
        /* enabled and other ADC of the common group are enabled, internal      */
        /* measurement paths cannot be enabled.                                 */
        else {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
          tmp_hal_status = HAL_ERROR;
          }
        }
      }
    }

  /* If a conversion is on going on regular group, no update on regular */
  /* channel could be done on neither of the channel configuration structure parameters */
  else {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
    tmp_hal_status = HAL_ERROR;
    }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
  }
//}}}
//{{{
/**
 ===============================================================================
             ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure channels on injected group
      (+) Configure multimode when multimode feature is available
      (+) Enable or Disable Injected Queue
      (+) Disable ADC voltage regulator
      (+) Enter ADC deep-power-down mode

@endverbatim
  * @{
  */

/**
  * @brief  Configure a channel to be assigned to ADC group injected.
  * @note   Possibility to update parameters on the fly:
  *         This function initializes injected group, following calls to this
  *         function can be used to reconfigure some parameters of structure
  *         "ADC_InjectionConfTypeDef" on the fly, without resetting the ADC.
  *         The setting of these parameters is conditioned to ADC state:
  *         Refer to comments of structure "ADC_InjectionConfTypeDef".
  * @note   In case of usage of internal measurement channels:
  *         Vbat/VrefInt/TempSensor.
  *         These internal paths can be disabled using function
  *         HAL_ADC_DeInit().
  * @note   Caution: For Injected Context Queue use, a context must be fully
  *         defined before start of injected conversion. All channels are configured
  *         consecutively for the same ADC instance. Therefore, the number of calls to
  *         HAL_ADCEx_InjectedConfigChannel() must be equal to the value of parameter
  *         InjectedNbrOfConversion for each context.
  *  - Example 1: If 1 context is intended to be used (or if there is no use of the
  *    Injected Queue Context feature) and if the context contains 3 injected ranks
  *    (InjectedNbrOfConversion = 3), HAL_ADCEx_InjectedConfigChannel() must be
  *    called once for each channel (i.e. 3 times) before starting a conversion.
  *    This function must not be called to configure a 4th injected channel:
  *    it would start a new context into context queue.
  *  - Example 2: If 2 contexts are intended to be used and each of them contains
  *    3 injected ranks (InjectedNbrOfConversion = 3),
  *    HAL_ADCEx_InjectedConfigChannel() must be called once for each channel and
  *    for each context (3 channels x 2 contexts = 6 calls). Conversion can
  *    start once the 1st context is set, that is after the first three
  *    HAL_ADCEx_InjectedConfigChannel() calls. The 2nd context can be set on the fly.
  * @param hadc ADC handle
  * @param sConfigInjected Structure of ADC injected group and ADC channel for
  *         injected group.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel (ADC_HandleTypeDef* hadc, ADC_InjectionConfTypeDef* sConfigInjected)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpOffsetShifted;
  uint32_t wait_loop_index = 0U;

  uint32_t tmp_JSQR_ContextQueueBeingBuilt = 0U;

  __HAL_LOCK(hadc);

  /* Configuration of injected group sequencer:                               */
  /* Hardware constraint: Must fully define injected context register JSQR    */
  /* before make it entering into injected sequencer queue.                   */
  /*                                                                          */
  /* - if scan mode is disabled:                                              */
  /*    * Injected channels sequence length is set to 0x00: 1 channel         */
  /*      converted (channel on injected rank 1)                              */
  /*      Parameter "InjectedNbrOfConversion" is discarded.                   */
  /*    * Injected context register JSQR setting is simple: register is fully */
  /*      defined on one call of this function (for injected rank 1) and can  */
  /*      be entered into queue directly.                                     */
  /* - if scan mode is enabled:                                               */
  /*    * Injected channels sequence length is set to parameter               */
  /*      "InjectedNbrOfConversion".                                          */
  /*    * Injected context register JSQR setting more complex: register is    */
  /*      fully defined over successive calls of this function, for each      */
  /*      injected channel rank. It is entered into queue only when all       */
  /*      injected ranks have been set.                                       */
  /*   Note: Scan mode is not present by hardware on this device, but used    */
  /*   by software for alignment over all STM32 devices.                      */

  if ((hadc->Init.ScanConvMode == ADC_SCAN_DISABLE)  ||
      (sConfigInjected->InjectedNbrOfConversion == 1U)  )
  {
    /* Configuration of context register JSQR:                                */
    /*  - number of ranks in injected group sequencer: fixed to 1st rank      */
    /*    (scan mode disabled, only rank 1 used)                              */
    /*  - external trigger to start conversion                                */
    /*  - external trigger polarity                                           */
    /*  - channel set to rank 1 (scan mode disabled, only rank 1 can be used) */

    if (sConfigInjected->InjectedRank == ADC_INJECTED_RANK_1)
    {
      /* Enable external trigger if trigger selection is different of         */
      /* software start.                                                      */
      /* Note: This configuration keeps the hardware feature of parameter     */
      /*       ExternalTrigInjecConvEdge "trigger edge none" equivalent to    */
      /*       software start.                                                */
      if (sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
      {
         tmp_JSQR_ContextQueueBeingBuilt = (  ADC_JSQR_RK(sConfigInjected->InjectedChannel, ADC_INJECTED_RANK_1)
                                            | (sConfigInjected->ExternalTrigInjecConv & ADC_JSQR_JEXTSEL)
                                            | sConfigInjected->ExternalTrigInjecConvEdge
                                           );
      }
      else
      {
         tmp_JSQR_ContextQueueBeingBuilt = ( ADC_JSQR_RK(sConfigInjected->InjectedChannel, ADC_INJECTED_RANK_1) );
      }

      MODIFY_REG(hadc->Instance->JSQR, ADC_JSQR_FIELDS, tmp_JSQR_ContextQueueBeingBuilt);
      /* For debug and informative reasons, hadc handle saves JSQR setting */
      hadc->InjectionConfig.ContextQueue = tmp_JSQR_ContextQueueBeingBuilt;

    }
  }
  else
  {
    /* Case of scan mode enabled, several channels to set into injected group */
    /* sequencer.                                                             */
    /*                                                                        */
    /* Procedure to define injected context register JSQR over successive     */
    /* calls of this function, for each injected channel rank:                */
    /* 1. Start new context and set parameters related to all injected        */
    /*    channels: injected sequence length and trigger.                     */

    /* if hadc->InjectionConfig.ChannelCount is equal to 0, this is the first */
    /*   call of the context under setting                                    */
    if (hadc->InjectionConfig.ChannelCount == 0U)
    {
      /* Initialize number of channels that will be configured on the context */
      /*  being built                                                         */
      hadc->InjectionConfig.ChannelCount = sConfigInjected->InjectedNbrOfConversion;
      /* Handle hadc saves the context under build up over each HAL_ADCEx_InjectedConfigChannel()
         call, this context will be written in JSQR register at the last call.
         At this point, the context is merely reset  */
       hadc->InjectionConfig.ContextQueue = 0x00000000U;

      /* Configuration of context register JSQR:                              */
      /*  - number of ranks in injected group sequencer                       */
      /*  - external trigger to start conversion                              */
      /*  - external trigger polarity                                         */

      /* Enable external trigger if trigger selection is different of         */
      /* software start.                                                      */
      /* Note: This configuration keeps the hardware feature of parameter     */
      /*       ExternalTrigInjecConvEdge "trigger edge none" equivalent to    */
      /*       software start.                                                */
      if (sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
      {
         tmp_JSQR_ContextQueueBeingBuilt = (  (sConfigInjected->InjectedNbrOfConversion - 1U)
                                            | (sConfigInjected->ExternalTrigInjecConv & ADC_JSQR_JEXTSEL)
                                            | sConfigInjected->ExternalTrigInjecConvEdge
                                           );
      }
      else
      {
        tmp_JSQR_ContextQueueBeingBuilt = ((sConfigInjected->InjectedNbrOfConversion - 1U) );
      }

    }

    /* 2. Continue setting of context under definition with parameter       */
    /*    related to each channel: channel rank sequence                    */
    /* Clear the old JSQx bits for the selected rank */
    tmp_JSQR_ContextQueueBeingBuilt &= ~ADC_JSQR_RK(ADC_SQR3_SQ10, sConfigInjected->InjectedRank);

    /* Set the JSQx bits for the selected rank */
    tmp_JSQR_ContextQueueBeingBuilt |= ADC_JSQR_RK(sConfigInjected->InjectedChannel, sConfigInjected->InjectedRank);

    /* Decrease channel count  */
    hadc->InjectionConfig.ChannelCount--;

    /* 3. tmp_JSQR_ContextQueueBeingBuilt is fully built for this HAL_ADCEx_InjectedConfigChannel()
          call, aggregate the setting to those already built during the previous
          HAL_ADCEx_InjectedConfigChannel() calls (for the same context of course)  */
    hadc->InjectionConfig.ContextQueue |= tmp_JSQR_ContextQueueBeingBuilt;

    /* 4. End of context setting: if this is the last channel set, then write context
        into register JSQR and make it enter into queue                   */
    if (hadc->InjectionConfig.ChannelCount == 0U)
    {
      MODIFY_REG(hadc->Instance->JSQR, ADC_JSQR_FIELDS, hadc->InjectionConfig.ContextQueue);
    }
  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on injected group:                                   */
  /*  - Injected context queue: Queue disable (active context is kept) or     */
  /*    enable (context decremented, up to 2 contexts queued)                 */
  /*  - Injected discontinuous mode: can be enabled only if auto-injected     */
  /*    mode is disabled.                                                     */
  if (ADC_IS_CONVERSION_ONGOING_INJECTED(hadc) == RESET)
  {
    /* If auto-injected mode is disabled: no constraint                       */
    if (sConfigInjected->AutoInjectedConv == DISABLE)
    {
      MODIFY_REG(hadc->Instance->CFGR, ADC_CFGR_JQM | ADC_CFGR_JDISCEN,
                               ADC_CFGR_INJECT_CONTEXT_QUEUE(sConfigInjected->QueueInjectedContext)          |
                               ADC_CFGR_INJECT_DISCCONTINUOUS(sConfigInjected->InjectedDiscontinuousConvMode) );
    }
    /* If auto-injected mode is enabled: Injected discontinuous setting is    */
    /* discarded.                                                             */
    else
    {
            MODIFY_REG(hadc->Instance->CFGR, ADC_CFGR_JQM | ADC_CFGR_JDISCEN,
                               ADC_CFGR_INJECT_CONTEXT_QUEUE(sConfigInjected->QueueInjectedContext) );
    }

  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on regular and injected groups:                      */
  /*  - Automatic injected conversion: can be enabled if injected group       */
  /*    external triggers are disabled.                                       */
  /*  - Channel sampling time                                                 */
  /*  - Channel offset                                                        */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET)
  {
    /* If injected group external triggers are disabled (set to injected      */
    /* software start): no constraint                                         */
    if ((sConfigInjected->ExternalTrigInjecConv == ADC_INJECTED_SOFTWARE_START)
       || (sConfigInjected->ExternalTrigInjecConvEdge == ADC_EXTERNALTRIGINJECCONV_EDGE_NONE))
    {
         if (sConfigInjected->AutoInjectedConv == ENABLE)
         {
           SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO);
         }
         else
         {
           CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO);
         }
    }
    /* If Automatic injected conversion was intended to be set and could not  */
    /* due to injected group external triggers enabled, error is reported.    */
    else
    {
      if (sConfigInjected->AutoInjectedConv == ENABLE)
      {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

        tmp_hal_status = HAL_ERROR;
      }
      else
      {
        CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_JAUTO);
      }
    }

    if (sConfigInjected->InjecOversamplingMode == ENABLE)
    {
      /* Configuration of Injected Oversampler:                                 */
      /*  - Oversampling Ratio                                                  */
      /*  - Right bit shift                                                     */

      /* Enable OverSampling mode */
       MODIFY_REG(hadc->Instance->CFGR2,
                  ADC_CFGR2_JOVSE |
                  ADC_CFGR2_OVSR  |
                  ADC_CFGR2_OVSS,
                  ADC_CFGR2_JOVSE                                  |
                  sConfigInjected->InjecOversampling.Ratio         |
                  sConfigInjected->InjecOversampling.RightBitShift
                 );
    }
    else
    {
      /* Disable Regular OverSampling */
       CLEAR_BIT( hadc->Instance->CFGR2, ADC_CFGR2_JOVSE);
    }

#if defined(ADC_SMPR1_SMPPLUS)
      /* Manage specific case of sampling time 3.5 cycles replacing 2.5 cyles */
      if(sConfigInjected->InjectedSamplingTime == ADC_SAMPLETIME_3CYCLES_5)
      {
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfigInjected->InjectedChannel, LL_ADC_SAMPLINGTIME_2CYCLES_5);

        /* Set ADC sampling time common configuration */
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_3C5_REPL_2C5);
      }
      else
      {
        /* Set sampling time of the selected ADC channel */
        LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfigInjected->InjectedChannel, sConfigInjected->InjectedSamplingTime);

        /* Set ADC sampling time common configuration */
        LL_ADC_SetSamplingTimeCommonConfig(hadc->Instance, LL_ADC_SAMPLINGTIME_COMMON_DEFAULT);
      }
#else
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime(hadc->Instance, sConfigInjected->InjectedChannel, sConfigInjected->InjectedSamplingTime);
#endif

    /* Configure the offset: offset enable/disable, channel, offset value */

    /* Shift the offset with respect to the selected ADC resolution. */
    /* Offset has to be left-aligned on bit 11, the LSB (right bits) are set to 0 */
    tmpOffsetShifted = ADC_OFFSET_SHIFT_RESOLUTION(hadc, sConfigInjected->InjectedOffset);

    if(sConfigInjected->InjectedOffsetNumber != ADC_OFFSET_NONE)
    {
      /* Set ADC selected offset number */
      LL_ADC_SetOffset(hadc->Instance, sConfigInjected->InjectedOffsetNumber, sConfigInjected->InjectedChannel, tmpOffsetShifted);

    }
    else
    {
      /* Scan each offset register to check if the selected channel is targeted. */
      /* If this is the case, the corresponding offset number is disabled.       */
      if(__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_1)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
       LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_1, LL_ADC_OFFSET_DISABLE);
      }
      if(__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_2)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
       LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_2, LL_ADC_OFFSET_DISABLE);
      }
      if(__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_3)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
       LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_3, LL_ADC_OFFSET_DISABLE);
      }
      if(__LL_ADC_CHANNEL_TO_DECIMAL_NB(LL_ADC_GetOffsetChannel(hadc->Instance, LL_ADC_OFFSET_4)) == __LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel))
      {
       LL_ADC_SetOffsetState(hadc->Instance, LL_ADC_OFFSET_4, LL_ADC_OFFSET_DISABLE);
      }
    }

  }

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated only when ADC is disabled:                */
  /*  - Single or differential mode                                           */
  /*  - Internal measurement channels: Vbat/VrefInt/TempSensor                */
  if (ADC_IS_ENABLE(hadc) == RESET)
  {
    /* Set mode single-ended or differential input of the selected ADC channel */
    LL_ADC_SetChannelSingleDiff(hadc->Instance, sConfigInjected->InjectedChannel, sConfigInjected->InjectedSingleDiff);

    /* Configuration of differential mode */
    if (sConfigInjected->InjectedSingleDiff == ADC_DIFFERENTIAL_ENDED)
    {
      /* Set sampling time of the selected ADC channel */
      LL_ADC_SetChannelSamplingTime(hadc->Instance, __LL_ADC_DECIMAL_NB_TO_CHANNEL(__LL_ADC_CHANNEL_TO_DECIMAL_NB(sConfigInjected->InjectedChannel) + 1), sConfigInjected->InjectedSamplingTime);
    }

    /* Management of internal measurement channels: Vbat/VrefInt/TempSensor   */
    /* internal measurement paths enable: If internal channel selected,       */
    /* enable dedicated internal buffers and path.                            */
    /* Note: these internal measurement paths can be disabled using           */
    /* HAL_ADC_DeInit().                                                      */

    /* Configuration of common ADC parameters                                 */
    /* If the requested internal measurement path has already been enabled,   */
    /* bypass the configuration processing.                                   */
      if (( (sConfigInjected->InjectedChannel == ADC_CHANNEL_TEMPSENSOR) &&
            ((LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) & LL_ADC_PATH_INTERNAL_TEMPSENSOR) == 0U)) ||
          ( (sConfigInjected->InjectedChannel == ADC_CHANNEL_VBAT)       &&
            ((LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) & LL_ADC_PATH_INTERNAL_VBAT) == 0U))      ||
          ( (sConfigInjected->InjectedChannel == ADC_CHANNEL_VREFINT)    &&
            ((LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)) & LL_ADC_PATH_INTERNAL_VREFINT) == 0U))
         )
    {
      /* Configuration of common ADC parameters (continuation)                */
      /* Software is allowed to change common parameters only when all ADCs   */
      /* of the common group are disabled.                                    */
      if ((ADC_IS_ENABLE(hadc) == RESET)   &&
         (ADC_ANY_OTHER_ENABLED(hadc) == RESET) )
      {
        if (sConfigInjected->InjectedChannel == ADC_CHANNEL_TEMPSENSOR)
        {
          if (ADC_TEMPERATURE_SENSOR_INSTANCE(hadc))
          {
            LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance), LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)));

            /* Delay for temperature sensor stabilization time */
            /* Compute number of CPU cycles to wait for */
            wait_loop_index = (LL_ADC_DELAY_TEMPSENSOR_STAB_US * (SystemCoreClock / 1000000));
            while(wait_loop_index != 0)
            {
              wait_loop_index--;
            }
          }
        }
        else if (sConfigInjected->InjectedChannel == ADC_CHANNEL_VBAT)
        {
          if (ADC_BATTERY_VOLTAGE_INSTANCE(hadc))
          {
            LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance), LL_ADC_PATH_INTERNAL_VBAT | LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)));
          }
        }
        else if (sConfigInjected->InjectedChannel == ADC_CHANNEL_VREFINT)
        {
          if (ADC_VREFINT_INSTANCE(hadc))
          {
            LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance), LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_GetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(hadc->Instance)));
          }
        }
      }
      /* If the requested internal measurement path has already been enabled  */
      /* and other ADC of the common group are enabled, internal              */
      /* measurement paths cannot be enabled.                                 */
      else
      {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

        tmp_hal_status = HAL_ERROR;
      }
    }

  }

  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}
//}}}

//{{{
/**
  * @brief  Configure the analog watchdog.
  * @note   Possibility to update parameters on the fly:
  *         This function initializes the selected analog watchdog, successive
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "ADC_AnalogWDGConfTypeDef" on the fly, without resetting
  *         the ADC.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_AnalogWDGConfTypeDef".
  * @note   On this STM32 serie, analog watchdog thresholds cannot be modified
  *         while ADC conversion is on going.
  * @param hadc ADC handle
  * @param AnalogWDGConfig Structure of ADC analog watchdog configuration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig (ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t tmpAWDHighThresholdShifted = 0U;
  uint32_t tmpAWDLowThresholdShifted = 0U;

  /* Process locked */
  __HAL_LOCK(hadc);

  /* Parameters update conditioned to ADC state:                              */
  /* Parameters that can be updated when ADC is disabled or enabled without   */
  /* conversion on going on ADC groups regular and injected:                  */
  /*  - Analog watchdog channels                                              */
  /*  - Analog watchdog thresholds                                            */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET)
  {
    /* Analog watchdog configuration */
    if(AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_1)
    {
      /* Configuration of analog watchdog:                                    */
      /*  - Set the analog watchdog enable mode: one or overall group of      */
      /*    channels, on groups regular and-or injected.                      */
      switch(AnalogWDGConfig->WatchdogMode)
      {
        case ADC_ANALOGWATCHDOG_SINGLE_REG:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(AnalogWDGConfig->Channel, LL_ADC_GROUP_REGULAR));
          break;

        case ADC_ANALOGWATCHDOG_SINGLE_INJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(AnalogWDGConfig->Channel, LL_ADC_GROUP_INJECTED));
          break;

        case ADC_ANALOGWATCHDOG_SINGLE_REGINJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, __LL_ADC_ANALOGWD_CHANNEL_GROUP(AnalogWDGConfig->Channel, LL_ADC_GROUP_REGULAR_INJECTED));
          break;

        case ADC_ANALOGWATCHDOG_ALL_REG:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_REG);
          break;

        case ADC_ANALOGWATCHDOG_ALL_INJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_INJ);
          break;

        case ADC_ANALOGWATCHDOG_ALL_REGINJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_ALL_CHANNELS_REG_INJ);
          break;

        default: /* ADC_ANALOGWATCHDOG_NONE */
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, LL_ADC_AWD1, LL_ADC_AWD_DISABLE);
          break;
      }

      /* Shift the offset in function of the selected ADC resolution:         */
      /* Thresholds have to be left-aligned on bit 11, the LSB (right bits)   */
      /* are set to 0                                                         */
      tmpAWDHighThresholdShifted = ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->HighThreshold);
      tmpAWDLowThresholdShifted  = ADC_AWD1THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->LowThreshold);

      /* Set ADC analog watchdog thresholds value of both thresholds high and low */
      LL_ADC_ConfigAnalogWDThresholds(hadc->Instance, AnalogWDGConfig->WatchdogNumber, tmpAWDHighThresholdShifted, tmpAWDLowThresholdShifted);

      /* Update state, clear previous result related to AWD1 */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD1);

      /* Clear flag ADC analog watchdog */
      /* Note: Flag cleared Clear the ADC Analog watchdog flag to be ready  */
      /* to use for HAL_ADC_IRQHandler() or HAL_ADC_PollForEvent()          */
      /* (in case left enabled by previous ADC operations).                 */
      LL_ADC_ClearFlag_AWD1(hadc->Instance);

      /* Configure ADC analog watchdog interrupt */
      if(AnalogWDGConfig->ITMode == ENABLE)
      {
        LL_ADC_EnableIT_AWD1(hadc->Instance);
      }
      else
      {
        LL_ADC_DisableIT_AWD1(hadc->Instance);
      }
    }
    /* Case of ADC_ANALOGWATCHDOG_2 or ADC_ANALOGWATCHDOG_3 */
    else
    {
      switch(AnalogWDGConfig->WatchdogMode)
      {
        case ADC_ANALOGWATCHDOG_SINGLE_REG:
        case ADC_ANALOGWATCHDOG_SINGLE_INJEC:
        case ADC_ANALOGWATCHDOG_SINGLE_REGINJEC:
          /* Update AWD by bitfield to keep the possibility to monitor        */
          /* several channels by successive calls of this function.           */
          if (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_2)
          {
            SET_BIT(hadc->Instance->AWD2CR, (1U << __LL_ADC_CHANNEL_TO_DECIMAL_NB(AnalogWDGConfig->Channel)));
          }
          else
          {
            SET_BIT(hadc->Instance->AWD3CR, (1U << __LL_ADC_CHANNEL_TO_DECIMAL_NB(AnalogWDGConfig->Channel)));
          }
          break;

        case ADC_ANALOGWATCHDOG_ALL_REG:
        case ADC_ANALOGWATCHDOG_ALL_INJEC:
        case ADC_ANALOGWATCHDOG_ALL_REGINJEC:
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, AnalogWDGConfig->WatchdogNumber, LL_ADC_AWD_ALL_CHANNELS_REG_INJ);
          break;

        default: /* ADC_ANALOGWATCHDOG_NONE */
          LL_ADC_SetAnalogWDMonitChannels(hadc->Instance, AnalogWDGConfig->WatchdogNumber, LL_ADC_AWD_DISABLE);
          break;
      }

      /* Shift the thresholds in function of the selected ADC resolution      */
      /* have to be left-aligned on bit 7, the LSB (right bits) are set to 0  */
      tmpAWDHighThresholdShifted = ADC_AWD23THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->HighThreshold);
      tmpAWDLowThresholdShifted  = ADC_AWD23THRESHOLD_SHIFT_RESOLUTION(hadc, AnalogWDGConfig->LowThreshold);

      /* Set ADC analog watchdog thresholds value of both thresholds high and low */
      LL_ADC_ConfigAnalogWDThresholds(hadc->Instance, AnalogWDGConfig->WatchdogNumber, tmpAWDHighThresholdShifted, tmpAWDLowThresholdShifted);

      if (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_2)
      {
        /* Update state, clear previous result related to AWD2 */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD2);

        /* Clear flag ADC analog watchdog */
        /* Note: Flag cleared Clear the ADC Analog watchdog flag to be ready  */
        /* to use for HAL_ADC_IRQHandler() or HAL_ADC_PollForEvent()          */
        /* (in case left enabled by previous ADC operations).                 */
        LL_ADC_ClearFlag_AWD2(hadc->Instance);

        /* Configure ADC analog watchdog interrupt */
        if(AnalogWDGConfig->ITMode == ENABLE)
        {
          LL_ADC_EnableIT_AWD2(hadc->Instance);
        }
        else
        {
          LL_ADC_DisableIT_AWD2(hadc->Instance);
        }
      }
      /* (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_3) */
      else
      {
        /* Update state, clear previous result related to AWD3 */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD3);

        /* Clear flag ADC analog watchdog */
        /* Note: Flag cleared Clear the ADC Analog watchdog flag to be ready  */
        /* to use for HAL_ADC_IRQHandler() or HAL_ADC_PollForEvent()          */
        /* (in case left enabled by previous ADC operations).                 */
        LL_ADC_ClearFlag_AWD3(hadc->Instance);

        /* Configure ADC analog watchdog interrupt */
        if(AnalogWDGConfig->ITMode == ENABLE)
        {
          LL_ADC_EnableIT_AWD3(hadc->Instance);
        }
        else
        {
          LL_ADC_DisableIT_AWD3(hadc->Instance);
        }
      }
    }

  }
  /* If a conversion is on going on ADC group regular or injected, no update  */
  /* could be done on neither of the AWD configuration structure parameters.  */
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

    tmp_hal_status = HAL_ERROR;
  }
  /* Process unlocked */
  __HAL_UNLOCK(hadc);

  /* Return function status */
  return tmp_hal_status;
}

//}}}

//{{{
/**
  * @brief  Stop ADC conversion.
  * @param hadc ADC handle
  * @param ConversionGroup ADC group regular and/or injected.
  *          This parameter can be one of the following values:
  *            @arg @ref ADC_REGULAR_GROUP           ADC regular conversion type.
  *            @arg @ref ADC_INJECTED_GROUP          ADC injected conversion type.
  *            @arg @ref ADC_REGULAR_INJECTED_GROUP  ADC regular and injected conversion type.
  * @retval HAL status.
  */
HAL_StatusTypeDef ADC_ConversionStop (ADC_HandleTypeDef* hadc, uint32_t ConversionGroup) {

  uint32_t tmp_ADC_CR_ADSTART_JADSTART = 0;
  uint32_t tickstart = 0;
  uint32_t Conversion_Timeout_CPU_cycles = 0;

  /* Verification if ADC is not already stopped (on regular and injected      */
  /* groups) to bypass this function if not needed.                           */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc)) {
    /* Particular case of continuous auto-injection mode combined with        */
    /* auto-delay mode.                                                       */
    /* In auto-injection mode, regular group stop ADC_CR_ADSTP is used (not   */
    /* injected group stop ADC_CR_JADSTP).                                    */
    /* Procedure to be followed: Wait until JEOS=1, clear JEOS, set ADSTP=1   */
    /* (see reference manual).                                                */
    if ((HAL_IS_BIT_SET(hadc->Instance->CFGR, ADC_CFGR_JAUTO))
         && (hadc->Init.ContinuousConvMode==ENABLE)
         && (hadc->Init.LowPowerAutoWait==ENABLE)) {
      /* Use stop of regular group */
      ConversionGroup = ADC_REGULAR_GROUP;

      /* Wait until JEOS=1 (maximum Timeout: 4 injected conversions) */
      while(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS) == RESET) {
        if (Conversion_Timeout_CPU_cycles >= (ADC_CONVERSION_TIME_MAX_CPU_CYCLES *4)) {
          /* Update ADC state machine to error */
          SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

          /* Set ADC error code to ADC IP internal error */
          SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

          return HAL_ERROR;
        }
        Conversion_Timeout_CPU_cycles ++;
      }

      /* Clear JEOS */
      __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOS);
    }

    /* Stop potential conversion on going on regular group */
    if (ConversionGroup != ADC_INJECTED_GROUP) {
      /* Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0 */
      if (HAL_IS_BIT_SET(hadc->Instance->CR, ADC_CR_ADSTART) &&
          HAL_IS_BIT_CLR(hadc->Instance->CR, ADC_CR_ADDIS)     ) {
        /* Stop conversions on regular group */
        LL_ADC_REG_StopConversion(hadc->Instance);
      }
    }

    /* Stop potential conversion on going on injected group */
    if (ConversionGroup != ADC_REGULAR_GROUP) {
      /* Software is allowed to set JADSTP only when JADSTART=1 and ADDIS=0 */
      if (HAL_IS_BIT_SET(hadc->Instance->CR, ADC_CR_JADSTART) &&
          HAL_IS_BIT_CLR(hadc->Instance->CR, ADC_CR_ADDIS)      )
        /* Stop conversions on injected group */
        SET_BIT(hadc->Instance->CR, ADC_CR_JADSTP);
      }

    /* Selection of start and stop bits with respect to the regular or injected group */
    switch(ConversionGroup) {
      case ADC_REGULAR_INJECTED_GROUP:
        tmp_ADC_CR_ADSTART_JADSTART = (ADC_CR_ADSTART | ADC_CR_JADSTART);
        break;
      case ADC_INJECTED_GROUP:
        tmp_ADC_CR_ADSTART_JADSTART = ADC_CR_JADSTART;
        break;
      /* Case ADC_REGULAR_GROUP only*/
      default:
        tmp_ADC_CR_ADSTART_JADSTART = ADC_CR_ADSTART;
        break;
      }

    /* Wait for conversion effectively stopped */
    tickstart = HAL_GetTick();
    while((hadc->Instance->CR & tmp_ADC_CR_ADSTART_JADSTART) != RESET) {
      if((HAL_GetTick()-tickstart) > ADC_STOP_CONVERSION_TIMEOUT) {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
        /* Set ADC error code to ADC IP internal error */
        SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
        return HAL_ERROR;
        }
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef ADC_Enable (ADC_HandleTypeDef* hadc) {

  __IO uint32_t wait_loop_index = 0;

  /* ADC enable and wait for ADC ready (in case of ADC is disabled or         */
  /* enabling phase not yet completed: flag ADC ready not yet set).           */
  /* Timeout implemented to not be stuck if ADC cannot be enabled (possible   */
  /* causes: ADC clock not running, ...).                                     */
  if (ADC_IS_ENABLE (hadc) == RESET) {
    /* Check if conditions to enable the ADC are fulfilled */
    if (ADC_ENABLING_CONDITIONS (hadc) == RESET) {
      /* Update ADC state machine to error */
      SET_BIT (hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
      /* Set ADC error code to ADC IP internal error */
      SET_BIT (hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
      return HAL_ERROR;
      }

    /* Enable the ADC peripheral */
    LL_ADC_Enable(hadc->Instance);

    /* Delay for ADC stabilization time */
    /* Wait loop initialization and execution */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles.                                           */
    wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (1000000 * 2)));
    while (wait_loop_index != 0)
      wait_loop_index--;

    /* Wait for ADC effectively enabled */
    uint32_t tickstart = HAL_GetTick();
    while(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == RESET) {
      /*  If ADEN bit is set less than 4 ADC clock cycles after the ADCAL bit
          has been cleared (after a calibration), ADEN bit is reset by the
          calibration logic.
          The workaround is to continue setting ADEN until ADRDY is becomes 1.
          Additionally, ADC_ENABLE_TIMEOUT is defined to encompass this
          4 ADC clock cycle duration */
      /* Note: Test of ADC enabled required due to hardware constraint to     */
      /*       not enable ADC if already enabled.                             */
      if (LL_ADC_IsEnabled(hadc->Instance) == 0)
        LL_ADC_Enable (hadc->Instance);

      if ((HAL_GetTick() - tickstart) > ADC_ENABLE_TIMEOUT) {
        /* Update ADC state machine to error */
        SET_BIT (hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
        /* Set ADC error code to ADC IP internal error */
        SET_BIT (hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
        return HAL_ERROR;
        }
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef ADC_Disable (ADC_HandleTypeDef* hadc) {

  /* Verification if ADC is not already disabled:                             */
  /* Note: forbidden to disable ADC (set bit ADC_CR_ADDIS) if ADC is already  */
  /*       disabled.                                                          */
  if (ADC_IS_ENABLE (hadc) != RESET) {
    /* Check if conditions to disable the ADC are fulfilled */
    if (ADC_DISABLING_CONDITIONS(hadc) != RESET) {
      /* Disable the ADC peripheral */
      LL_ADC_Disable (hadc->Instance);
      __HAL_ADC_CLEAR_FLAG (hadc, (ADC_FLAG_EOSMP | ADC_FLAG_RDY));
      }
    else {
      /* Update ADC state machine to error */
      SET_BIT (hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
      /* Set ADC error code to ADC IP internal error */
      SET_BIT (hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
      return HAL_ERROR;
      }

    /* Wait for ADC effectively disabled */
    uint32_t tickstart = HAL_GetTick();
    while (HAL_IS_BIT_SET(hadc->Instance->CR, ADC_CR_ADEN)) {
      if ((HAL_GetTick() - tickstart) > ADC_DISABLE_TIMEOUT) {
        /* Update ADC state machine to error */
        SET_BIT (hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);
        /* Set ADC error code to ADC IP internal error */
        SET_BIT (hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);
        return HAL_ERROR;
        }
      }
    }

  return HAL_OK;
  }
//}}}

//{{{
void ADC_DMAConvCplt (DMA_HandleTypeDef* hdma) {

  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Update state machine on conversion status if not in error state */
  if(HAL_IS_BIT_CLR(hadc->State, (HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))) {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular     */
    /* by external trigger, continuous mode or scan sequence on going         */
    /* to disable interruption.                                               */
    /* Is it the end of the regular sequence ? */
    if(HAL_IS_BIT_SET(hadc->Instance->ISR, ADC_FLAG_EOS)) {
      /* Are conversions software-triggered ? */
      if(ADC_IS_SOFTWARE_START_REGULAR(hadc)) {
        /* Is CONT bit set ? */
        if(READ_BIT(hadc->Instance->CFGR, ADC_CFGR_CONT) == RESET) {
          /* CONT bit is not set, no more conversions expected */
          CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
          if(HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
            SET_BIT(hadc->State, HAL_ADC_STATE_READY);
          }
        }
      }
    else {
      /* DMA End of Transfer interrupt was triggered but conversions sequence
         is not over. If DMACFG is set to 0, conversions are stopped. */
      if(READ_BIT(hadc->Instance->CFGR, ADC_CFGR_DMACFG) == RESET) {
        /* DMACFG bit is not set, conversions are stopped. */
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);
        if(HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
          SET_BIT(hadc->State, HAL_ADC_STATE_READY);
        }
      }

    /* Conversion complete callback */
    HAL_ADC_ConvCpltCallback(hadc);
    }

  else {
    /* DMA and-or internal error occurred */
    if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
      /* Call HAL ADC Error Callback function */
      HAL_ADC_ErrorCallback(hadc);
    else
      /* Call ADC DMA error callback */
      hadc->DMA_Handle->XferErrorCallback(hdma);
    }
  }
//}}}
//{{{
void ADC_DMAHalfConvCplt (DMA_HandleTypeDef* hdma) {

  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Half conversion callback */
  HAL_ADC_ConvHalfCpltCallback(hadc);
  }
//}}}
//{{{
void ADC_DMAError (DMA_HandleTypeDef* hdma) {

  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Set ADC state */
  SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);

  /* Set ADC error code to DMA error */
  SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_DMA);

  HAL_ADC_ErrorCallback(hadc);
  }
//}}}

//{{{
/*
      (+) Perform the ADC self-calibration for single or differential ending.
      (+) Get calibration factors for single or differential ending.
      (+) Set calibration factors for single or differential ending.

      (+) Start conversion of ADC group injected.
      (+) Stop conversion of ADC group injected.
      (+) Poll for conversion complete on ADC group injected.
      (+) Get result of ADC group injected channel conversion.
      (+) Start conversion of ADC group injected and enable interruptions.
      (+) Stop conversion of ADC group injected and disable interruptions.

      (+) When multimode feature is available, start multimode and enable DMA transfer.
      (+) Stop multimode and disable ADC DMA transfer.
      (+) Get result of multimode conversion.

  * @brief  Perform an ADC automatic self-calibration
  *         Calibration prerequisite: ADC must be disabled (execute this
  *         function before HAL_ADC_Start() or after HAL_ADC_Stop() ).
  * @param  hadc       ADC handle
  * @param  SingleDiff Selection of single-ended or differential input
  *         This parameter can be one of the following values:
  *           @arg @ref ADC_SINGLE_ENDED       Channel in mode input single ended
  *           @arg @ref ADC_DIFFERENTIAL_ENDED Channel in mode input differential ended
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start (ADC_HandleTypeDef* hadc, uint32_t SingleDiff)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;
  uint32_t WaitLoopIndex = 0;

  __HAL_LOCK(hadc);

  /* Calibration prerequisite: ADC must be disabled. */
  /* Disable the ADC (if not already disabled) */
  tmp_hal_status = ADC_Disable(hadc);

  /* Check if ADC is effectively disabled */
  if (tmp_hal_status == HAL_OK) {
    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_BUSY_INTERNAL);

    /* Select calibration mode single ended or differential ended */
    MODIFY_REG(hadc->Instance->CR, ADC_CR_ADCALDIF, SingleDiff);

    /* Start ADC calibration */
    SET_BIT(hadc->Instance->CR, ADC_CR_ADCAL);

    /* Wait for calibration completion */
    while(HAL_IS_BIT_SET(hadc->Instance->CR, ADC_CR_ADCAL)) {
      WaitLoopIndex++;
      if (WaitLoopIndex >= ADC_CALIBRATION_TIMEOUT) {
        /* Update ADC state machine to error */
        ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_ERROR_INTERNAL);
        /* Process unlocked */
        __HAL_UNLOCK(hadc);
        return HAL_ERROR;
        }
      }

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_READY);
    }
  else
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

    /* Note: No need to update variable "tmp_hal_status" here: already set    */
    /*       to state "HAL_ERROR" by function disabling the ADC.              */

  __HAL_UNLOCK(hadc);
  return tmp_hal_status;
  }
//}}}
//{{{
/**
  * @brief  Get the calibration factor.
  * @param hadc ADC handle.
  * @param SingleDiff This parameter can be only:
  *           @arg @ref ADC_SINGLE_ENDED       Channel in mode input single ended
  *           @arg @ref ADC_DIFFERENTIAL_ENDED Channel in mode input differential ended
  * @retval Calibration value.
  */
uint32_t HAL_ADCEx_Calibration_GetValue (ADC_HandleTypeDef* hadc, uint32_t SingleDiff) {

  /* Return the selected ADC calibration value */
  if (SingleDiff == ADC_DIFFERENTIAL_ENDED)
    return ADC_CALFACT_DIFF_GET(hadc->Instance->CALFACT);
  else
    return ((hadc->Instance->CALFACT) & ADC_CALFACT_CALFACT_S);
  }
//}}}
//{{{
/**
  * @brief  Set the calibration factor to overwrite automatic conversion result.
  *         ADC must be enabled and no conversion is ongoing.
  * @param hadc ADC handle
  * @param SingleDiff This parameter can be only:
  *           @arg @ref ADC_SINGLE_ENDED       Channel in mode input single ended
  *           @arg @ref ADC_DIFFERENTIAL_ENDED Channel in mode input differential ended
  * @param CalibrationFactor Calibration factor (coded on 7 bits maximum)
  * @retval HAL state
  */
HAL_StatusTypeDef HAL_ADCEx_Calibration_SetValue (ADC_HandleTypeDef* hadc, uint32_t SingleDiff,
                                                  uint32_t CalibrationFactor) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  __HAL_LOCK(hadc);

  /* Verification of hardware constraints before modifying the calibration    */
  /* factors register: ADC must be enabled, no conversion on going.           */
  if ((ADC_IS_ENABLE(hadc) != RESET)  &&
      (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET)) {
    /* Set the selected ADC calibration value */
    if (SingleDiff == ADC_DIFFERENTIAL_ENDED)
      MODIFY_REG(hadc->Instance->CALFACT, ADC_CALFACT_CALFACT_D, ADC_CALFACT_DIFF_SET(CalibrationFactor));
    else
      MODIFY_REG(hadc->Instance->CALFACT, ADC_CALFACT_CALFACT_S, CalibrationFactor);
    }
  else {
    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
    /* Update ADC error code */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_INTERNAL);

    /* Update ADC state machine to error */
    tmp_hal_status = HAL_ERROR;
    }

  __HAL_UNLOCK(hadc);
  return tmp_hal_status;
  }
//}}}

//{{{
/**
  * @brief  Enable Injected Queue
  * @note   This function resets CFGR register JQDIS bit in order to enable the
  *         Injected Queue. JQDIS can be written only when ADSTART and JDSTART
  *         are both equal to 0 to ensure that no regular nor injected
  *         conversion is ongoing.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_EnableInjectedQueue (ADC_HandleTypeDef* hadc) {

  /* Parameter can be set only if no conversion is on-going                   */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET) {
    CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);
    /* Update state, clear previous result related to injected queue overflow */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_JQOVF);
    return HAL_OK;
    }
  else
    return HAL_ERROR;
  }
//}}}
//{{{
/**
  * @brief  Disable Injected Queue
  * @note   This function sets CFGR register JQDIS bit in order to disable the
  *         Injected Queue. JQDIS can be written only when ADSTART and JDSTART
  *         are both equal to 0 to ensure that no regular nor injected
  *         conversion is ongoing.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_DisableInjectedQueue (ADC_HandleTypeDef* hadc) {

  /* Parameter can be set only if no conversion is on-going                   */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR_INJECTED(hadc) == RESET) {
    SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);
    return HAL_OK;
    }
  else
    return HAL_ERROR;
  }
//}}}

//{{{
/**
  * @brief  Disable ADC voltage regulator.
  * @note   Disabling voltage regulator allows to save power. This operation can
  *         be carried out only when ADC is disabled.
  * @note   To enable again the voltage regulator, the user is expected to
  *         resort to HAL_ADC_Init() API.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_DisableVoltageRegulator (ADC_HandleTypeDef* hadc) {

  /* ADVREGEN can be written only when the ADC is disabled  */
  if (ADC_IS_ENABLE(hadc) == RESET) {
    CLEAR_BIT(hadc->Instance->CR, ADC_CR_ADVREGEN);
    return HAL_OK;
    }
  else
    return HAL_ERROR;
}
//}}}
//{{{
/**
  * @brief  Enter ADC deep-power-down mode
  * @note   This mode is achieved in setting DEEPPWD bit and allows to save power
  *         in reducing leakage currents. It is particularly interesting before
  *         entering stop modes.
  * @note   Setting DEEPPWD automatically clears ADVREGEN bit and disables the
  *         ADC voltage regulator. This means that this API encompasses
  *         HAL_ADCEx_DisableVoltageRegulator(). Additionally, the internal
  *         calibration is lost.
  * @note   To exit the ADC deep-power-down mode, the user is expected to
  *         resort to HAL_ADC_Init() API as well as to relaunch a calibration
  *         with HAL_ADCEx_Calibration_Start() API or to re-apply a previously
  *         saved calibration factor.
  * @param hadc ADC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_ADCEx_EnterADCDeepPowerDownMode (ADC_HandleTypeDef* hadc) {

  /* DEEPPWD can be written only when the ADC is disabled  */
  if (ADC_IS_ENABLE(hadc) == RESET) {
    SET_BIT(hadc->Instance->CR, ADC_CR_DEEPPWD);
    return HAL_OK;
    }
  else
    return HAL_ERROR;
  }
//}}}
