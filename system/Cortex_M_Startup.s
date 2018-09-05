// **********************************************************************
// *                    SEGGER Microcontroller GmbH                     *
// *                        The Embedded Experts                        *
// **********************************************************************
// *                                                                    *
// *            (c) 2014 - 2018 SEGGER Microcontroller GmbH             *
// *            (c) 2001 - 2018 Rowley Associates Limited               *
// *                                                                    *
// *           www.segger.com     Support: support@segger.com           *
// *                                                                    *
// **********************************************************************
// *                                                                    *
// * All rights reserved.                                               *
// *                                                                    *
// * Redistribution and use in source and binary forms, with or         *
// * without modification, are permitted provided that the following    *
// * conditions are met:                                                *
// *                                                                    *
// * - Redistributions of source code must retain the above copyright   *
// *   notice, this list of conditions and the following disclaimer.    *
// *                                                                    *
// * - Neither the name of SEGGER Microcontroller GmbH                  *
// *   nor the names of its contributors may be used to endorse or      *
// *   promote products derived from this software without specific     *
// *   prior written permission.                                        *
// *                                                                    *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
// * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
// * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
// * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
// * DISCLAIMED.                                                        *
// * IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
// * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
// * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
// * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
// * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
// * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
// * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
// * DAMAGE.                                                            *
// *                                                                    *
// **********************************************************************

.macro ISR_HANDLER name=
  .section .vectors, "ax"
  .word \name
  .section .init, "ax"
  .thumb_func
  .weak \name
\name:
1: b 1b /* endless loop */
.endm

.macro ISR_RESERVED
  .section .vectors, "ax"
  .word 0
.endm

  .syntax unified
  .global reset_handler
  .global Reset_Handler
  .equ Reset_Handler, reset_handler

  .section .vectors, "ax"
  .code 16
  .balign 2
  .global _vectors

.macro DEFAULT_ISR_HANDLER name=
  .thumb_func
  .weak \name
\name:
1: b 1b /* endless loop */
.endm

_vectors:
  .word __stack_end__
  .word reset_handler
ISR_HANDLER NMI_Handler
ISR_HANDLER HardFault_Handler
ISR_HANDLER MemManage_Handler
ISR_HANDLER BusFault_Handler
ISR_HANDLER UsageFault_Handler
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SVC_Handler
ISR_HANDLER DebugMon_Handler
ISR_RESERVED
ISR_HANDLER PendSV_Handler
ISR_HANDLER SysTick_Handler
ISR_HANDLER   WWDG_IRQHandler
ISR_HANDLER   PVD_PVM_IRQHandler
ISR_HANDLER   TAMP_STAMP_IRQHandler
ISR_HANDLER   RTC_WKUP_IRQHandler
ISR_HANDLER   FLASH_IRQHandler
ISR_HANDLER   RCC_IRQHandler
ISR_HANDLER   EXTI0_IRQHandler
ISR_HANDLER   EXTI1_IRQHandler
ISR_HANDLER   EXTI2_IRQHandler
ISR_HANDLER   EXTI3_IRQHandler
ISR_HANDLER   EXTI4_IRQHandler
ISR_HANDLER   DMA1_Channel1_IRQHandler
ISR_HANDLER   DMA1_Channel2_IRQHandler
ISR_HANDLER   DMA1_Channel3_IRQHandler
ISR_HANDLER   DMA1_Channel4_IRQHandler
ISR_HANDLER   DMA1_Channel5_IRQHandler
ISR_HANDLER   DMA1_Channel6_IRQHandler
ISR_HANDLER   DMA1_Channel7_IRQHandler
ISR_HANDLER   ADC1_IRQHandler
ISR_HANDLER   CAN1_TX_IRQHandler
ISR_HANDLER   CAN1_RX0_IRQHandler
ISR_HANDLER   CAN1_RX1_IRQHandler
ISR_HANDLER   CAN1_SCE_IRQHandler
ISR_HANDLER   EXTI9_5_IRQHandler
ISR_HANDLER   TIM1_BRK_TIM15_IRQHandler
ISR_HANDLER   TIM1_UP_TIM16_IRQHandler
ISR_HANDLER   TIM1_TRG_COM_TIM17_IRQHandler
ISR_HANDLER   TIM1_CC_IRQHandler
ISR_HANDLER   TIM2_IRQHandler
ISR_HANDLER   TIM3_IRQHandler
ISR_HANDLER   TIM4_IRQHandler
ISR_HANDLER   I2C1_EV_IRQHandler
ISR_HANDLER   I2C1_ER_IRQHandler
ISR_HANDLER   I2C2_EV_IRQHandler
ISR_HANDLER   I2C2_ER_IRQHandler
ISR_HANDLER   SPI1_IRQHandler
ISR_HANDLER   SPI2_IRQHandler
ISR_HANDLER   USART1_IRQHandler
ISR_HANDLER   USART2_IRQHandler
ISR_HANDLER   USART3_IRQHandler
ISR_HANDLER   EXTI15_10_IRQHandler
ISR_HANDLER   RTC_Alarm_IRQHandler
ISR_HANDLER   DFSDM1_FLT3_IRQHandler
ISR_HANDLER   TIM8_BRK_IRQHandler
ISR_HANDLER   TIM8_UP_IRQHandler
ISR_HANDLER   TIM8_TRG_COM_IRQHandler
ISR_HANDLER   TIM8_CC_IRQHandler
ISR_RESERVED
ISR_HANDLER   FMC_IRQHandler
ISR_HANDLER   SDMMC1_IRQHandler
ISR_HANDLER   TIM5_IRQHandler
ISR_HANDLER   SPI3_IRQHandler
ISR_HANDLER   UART4_IRQHandler
ISR_HANDLER   UART5_IRQHandler
ISR_HANDLER   TIM6_DAC_IRQHandler
ISR_HANDLER   TIM7_IRQHandler
ISR_HANDLER   DMA2_Channel1_IRQHandler
ISR_HANDLER   DMA2_Channel2_IRQHandler
ISR_HANDLER   DMA2_Channel3_IRQHandler
ISR_HANDLER   DMA2_Channel4_IRQHandler
ISR_HANDLER   DMA2_Channel5_IRQHandler
ISR_HANDLER   DFSDM1_FLT0_IRQHandler
ISR_HANDLER   DFSDM1_FLT1_IRQHandler
ISR_HANDLER   DFSDM1_FLT2_IRQHandler
ISR_HANDLER   COMP_IRQHandler
ISR_HANDLER   LPTIM1_IRQHandler
ISR_HANDLER   LPTIM2_IRQHandler
ISR_HANDLER   OTG_FS_IRQHandler
ISR_HANDLER   DMA2_Channel6_IRQHandler
ISR_HANDLER   DMA2_Channel7_IRQHandler
ISR_HANDLER   LPUART1_IRQHandler
ISR_HANDLER   OCTOSPI1_IRQHandler
ISR_HANDLER   I2C3_EV_IRQHandler
ISR_HANDLER   I2C3_ER_IRQHandler
ISR_HANDLER   SAI1_IRQHandler
ISR_HANDLER   SAI2_IRQHandler
ISR_HANDLER   OCTOSPI2_IRQHandler
ISR_HANDLER   TSC_IRQHandler
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER   RNG_IRQHandler
ISR_HANDLER   FPU_IRQHandler
ISR_HANDLER   CRS_IRQHandler
ISR_HANDLER   I2C4_ER_IRQHandler
ISR_HANDLER   I2C4_EV_IRQHandler
ISR_HANDLER   DCMI_IRQHandler
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER   DMA2D_IRQHandler
ISR_HANDLER   LTDC_IRQHandler
ISR_HANDLER   LTDC_ER_IRQHandler
ISR_HANDLER   GFXMMU_IRQHandler
ISR_HANDLER   DMAMUX1_OVR_IRQHandler
  .section .vectors, "ax"
_vectors_end:

  .section .init, "ax"
  .balign 2

  .thumb_func
  reset_handler:

#ifndef __NO_SYSTEM_INIT
  ldr r0, =__stack_end__
  mov sp, r0
  bl SystemInit
#endif

#if !defined(__SOFTFP__)
  // Enable CP11 and CP10 with CPACR |= (0xf<<20)
  movw r0, 0xED88
  movt r0, 0xE000
  ldr r1, [r0]
  orrs r1, r1, #(0xf << 20)
  str r1, [r0]
#endif

  b _start

#ifndef __NO_SYSTEM_INIT
  .thumb_func
  .weak SystemInit
SystemInit:
  bx lr
#endif
