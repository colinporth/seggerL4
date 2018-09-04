// heap.h
#pragma once
#include <string>
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

uint8_t* dtcmAlloc (size_t bytes);
void dtcmFree (void* p);
size_t getDtcmSize();
size_t getDtcmFreeSize();
size_t getDtcmMinFreeSize();

size_t getSramSize();
size_t getSramFreeSize();
size_t getSramMinFreeSize();

uint8_t* sram123Alloc (size_t bytes);
void sram123Free (void* p);
size_t getSram123Size();
size_t getSram123FreeSize();
size_t getSram123MinFreeSize();

uint8_t* sdRamAlloc (size_t size, const std::string& tag);
void sdRamFree (void* p);
size_t getSdRamSize();
size_t getSdRamFreeSize();
size_t getSdRamMinFreeSize();

//{{{
#ifdef __cplusplus
}
#endif
//}}}
