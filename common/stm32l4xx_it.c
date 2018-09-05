#include "cmsis_os.h"
#include "stm32l4xx_nucleo_144.h"

void NMI_Handler() {}
void MemManage_Handler() { while (1) { } }
void BusFault_Handler() { while (1) { } }
void UsageFault_Handler() { while (1) { } }
void DebugMon_Handler() { }

void SysTick_Handler() {
  //osSystickHandler();
  HAL_IncTick();
  }
