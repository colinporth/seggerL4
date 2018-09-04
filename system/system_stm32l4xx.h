#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

extern uint32_t SystemCoreClock;             /*!< System Domain1 Clock Frequency  */
extern const uint8_t  AHBPrescTable[16];
extern const uint8_t  APBPrescTable[8];
extern const uint32_t MSIRangeTable[12];

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
