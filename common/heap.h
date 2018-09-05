// heap.h
#pragma once
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

uint8_t* sram1Alloc (size_t bytes);
void sram1Free (void* p);
size_t getSram1Size();
size_t getSram1FreeSize();
size_t getSram1MinFreeSize();

//{{{
#ifdef __cplusplus
}
#endif
//}}}
