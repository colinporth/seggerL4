// heap.cpp
// sram 1    128k  0x20000000 0x00030000
// sram 2     64k  0x20030000 0x00010000
// sram 3    384k  0x20040000 0x00060000
//{{{  includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "heap.h"
//}}}

//{{{
class cHeap {
public:
  cHeap (size_t size, bool debug) : mSize(size), mFreeSize(size), mMinFreeSize(size), mDebug(debug) {}

  virtual size_t getSize() { return mSize; }
  virtual size_t getFreeSize() { return mFreeSize; }
  virtual size_t getMinFreeSize() { return mMinFreeSize; }
  virtual size_t getLargestFreeSize() { return mLargestFreeSize; }

  virtual uint8_t* alloc (size_t size, const std::string& tag) = 0;
  virtual void free (void* ptr) = 0;

protected:
  size_t mSize = 0;
  size_t mFreeSize = 0;
  size_t mMinFreeSize = 0;
  size_t mLargestFreeSize = 0;
  bool mDebug = false;
  };
//}}}
//{{{
class cRtosHeap : public cHeap {
public:
  //{{{
  cRtosHeap (uint32_t start, size_t size, bool debug) : cHeap (size, debug) {

    // Ensure the heap starts on a correctly aligned boundary
    size_t uxAddress = (size_t)start;
    mSize = size;
    if ((uxAddress & mAlignmentMask) != 0) {
      uxAddress += (mAlignment - 1);
      uxAddress &= ~((size_t)mAlignmentMask);
      mSize -= uxAddress - (size_t)start;
      }

    uint8_t* pucAlignedHeap = (uint8_t*)uxAddress;

    // mStart is used to hold a pointer to the first item in the list of free blocks.
    mStart.mNextFreeBlock = (tLink_t*)pucAlignedHeap;
    mStart.mBlockSize = (size_t)0;

    // mEnd is used to mark the end of the list of free blocks and is inserted at the end of the heap space. */
    uxAddress = ((size_t)pucAlignedHeap) + mSize;
    uxAddress -= kHeapStructSize;
    uxAddress &= ~((size_t) mAlignmentMask );
    mEnd = (tLink_t*)uxAddress;
    mEnd->mBlockSize = 0;
    mEnd->mNextFreeBlock = NULL;

    // start with single free block sized to take up entire heap space, minus space taken by mEnd
    tLink_t* firstFreeBlock = (tLink_t*)pucAlignedHeap;
    firstFreeBlock->mBlockSize = uxAddress - (size_t)firstFreeBlock;
    firstFreeBlock->mNextFreeBlock = mEnd;

    // Only one block exists - and it covers the entire usable heap space
    mMinFreeSize = firstFreeBlock->mBlockSize;
    mFreeSize = firstFreeBlock->mBlockSize;
    }
  //}}}

  //{{{
  virtual uint8_t* alloc (size_t size, const std::string& tag) {

    size_t largestBlock = 0;

    vTaskSuspendAll();
    uint8_t* allocAddress = allocBlock (size);
    xTaskResumeAll();

    if (mDebug) {
      printf ("cHeap::alloc size:%d free:%d minFree:%d largest:%d\n",
              size, mFreeSize, mMinFreeSize, largestBlock);

      tLink_t* block = mStart.mNextFreeBlock;
      while (block) {
        if ((block->mBlockSize & kBlockAllocatedBit) == 0)
          printf (" - alloc %p size:%d\n", block, block->mBlockSize);
        else
          printf (" -  free %p size:%d\n", block, block->mBlockSize);
        block = block->mNextFreeBlock;
        }
      }

    if (!allocAddress)
      printf ("****** cHeap::alloc fail size:%d\n", size);
    return allocAddress;
    }
  //}}}
  //{{{
  virtual void free (void* ptr) {

    if (ptr) {
      // memory being freed will have an tLink_t structure immediately before it.
      uint8_t* puc = (uint8_t*)ptr - kHeapStructSize;
      tLink_t* link = (tLink_t*)puc;

      if (mDebug)
        printf ("cHeap::free %p %d\n", ptr, link->mBlockSize & (~kBlockAllocatedBit));

      if (link->mBlockSize & kBlockAllocatedBit) {
        if (link->mNextFreeBlock == NULL) {
          // block is being returned to the heap - it is no longer allocated.
          link->mBlockSize &= ~kBlockAllocatedBit;

          vTaskSuspendAll();
          mFreeSize += link->mBlockSize;
          insertBlockIntoFreeList (link);
          xTaskResumeAll();
          }
        }
      }
    }
  //}}}

private:
  const uint32_t mAlignment = 8;
  const uint32_t mAlignmentMask = 7;
  const size_t kHeapStructSize = 8;
  const size_t kHeapMinimumBlockSize = kHeapStructSize << 1;
  const size_t kBlockAllocatedBit = 0x80000000;

  //{{{  struct tLink_t
  typedef struct A_BLOCK_LINK {
    struct A_BLOCK_LINK* mNextFreeBlock; // The next free block in the list
    size_t mBlockSize;                   // The size of the free block
    } tLink_t;
  //}}}

  //{{{
  uint8_t* allocBlock (size_t size) {

    uint8_t* allocAddress = NULL;
    size_t largestBlock = 0;

    // The wanted size is increased so it can contain a tLink_t structure in addition to the requested amount of bytes
    size += kHeapStructSize;
    // Ensure that blocks are always aligned to the required number of bytes
    if ((size & mAlignmentMask) != 0x00)
      // Byte alignment required. */
      size += (mAlignment - (size & mAlignmentMask));

    if ((size > 0) && (size <= mFreeSize)) {
      // Traverse the list from the start (lowest address) block until one of adequate size is found
      tLink_t* prevBlock = &mStart;
      tLink_t* block = mStart.mNextFreeBlock;
      largestBlock = mStart.mBlockSize;
      while ((block->mBlockSize < size) && (block->mNextFreeBlock != NULL)) {
        prevBlock = block;
        block = block->mNextFreeBlock;
        if (block->mBlockSize > largestBlock)
          largestBlock = block->mBlockSize;
        }

      // If the end marker was reached then a block of adequate size was not found
      if (block != mEnd) {
        // Return the memory space pointed to - jumping over the tLink_t structure at its start
        allocAddress = ((uint8_t*)prevBlock->mNextFreeBlock) + kHeapStructSize;

        //This block is being returned for use so must be taken out of the list of free blocks
        prevBlock->mNextFreeBlock = block->mNextFreeBlock;

        // If the block is larger than required it can be split into two.
        if ((block->mBlockSize - size) > kHeapMinimumBlockSize) {
          // This block is to be split into two
          // Create a new block following the number of bytes requested
          tLink_t* newLink = (tLink_t*)(((uint8_t*)block) + size);

          // Calculate the sizes of two blocks split from the single block
          newLink->mBlockSize = block->mBlockSize - size;
          block->mBlockSize = size;

          // Insert the new block into the list of free blocks
          insertBlockIntoFreeList (newLink);
          }

        mFreeSize -= block->mBlockSize;
        if (mFreeSize < mMinFreeSize)
          mMinFreeSize = mFreeSize;

        // The block is being returned - it is allocated and owned by the application and has no "next" block. */
        block->mBlockSize |= kBlockAllocatedBit;
        block->mNextFreeBlock = NULL;
        }
      }

    return allocAddress;
    }
  //}}}
  //{{{
  void insertBlockIntoFreeList (tLink_t* insertBlock) {

    // iterate through list until block found that has higher address than block being inserted
    tLink_t* blockIt;
    for (blockIt = &mStart; blockIt->mNextFreeBlock < insertBlock; blockIt = blockIt->mNextFreeBlock) {}

    // Do the block being inserted, and the block it is being inserted after make a contiguous block of memory? */
    uint8_t* puc = (uint8_t*)blockIt;
    if ((puc + blockIt->mBlockSize) == (uint8_t*)insertBlock) {
      blockIt->mBlockSize += insertBlock->mBlockSize;
      insertBlock = blockIt;
      }

    // Do the block being inserted, and the block it is being inserted before make a contiguous block of memory?
    puc = (uint8_t*)insertBlock;
    if ((puc + insertBlock->mBlockSize) == (uint8_t*)blockIt->mNextFreeBlock) {
      if (blockIt->mNextFreeBlock != mEnd ) {
        // Form one big block from the two blocks
        insertBlock->mBlockSize += blockIt->mNextFreeBlock->mBlockSize;
        insertBlock->mNextFreeBlock = blockIt->mNextFreeBlock->mNextFreeBlock;
        }
      else
        insertBlock->mNextFreeBlock = mEnd;
      }
    else
      insertBlock->mNextFreeBlock = blockIt->mNextFreeBlock;

    // If the block being inserted plugged a gab, so was merged with the block
    // before and the block after, then it's mNextFreeBlock pointer will have
    // already been set, and should not be set here as that would make it point to itself/
    if (blockIt != insertBlock)
      blockIt->mNextFreeBlock = insertBlock;
    }
  //}}}

  tLink_t mStart;
  tLink_t* mEnd = NULL;
  };
//}}}

// sram1 0x20000000 192k 0x30000
cRtosHeap* mSram1Heap = nullptr;
//{{{
uint8_t* sram1Alloc (size_t size) {
  if (!mSram1Heap)
    mSram1Heap = new cRtosHeap (0x20000000, 0x00030000, true);
  return (uint8_t*)mSram1Heap->alloc (size, "");
  }
//}}}
void sram1Free (void* ptr) { mSram1Heap->free (ptr); }
size_t getSram1Size(){ return mSram1Heap ? mSram1Heap->getSize() : 0 ; }
size_t getSram1FreeSize() { return mSram1Heap ? mSram1Heap->getFreeSize() : 0 ; }
size_t getSram1MinFreeSize() { return mSram1Heap ? mSram1Heap->getMinFreeSize() : 0 ; }

// sram2 - 0x2003000 64k  0x10000
cRtosHeap* mDtcmHeap = nullptr;
//{{{
uint8_t* dtcmAlloc (size_t size) {
  if (!mDtcmHeap)
    mDtcmHeap = new cRtosHeap (0x20030000, 0x00010000, true);
  return (uint8_t*)mDtcmHeap->alloc (size, "");
  }
//}}}
void dtcmFree (void* ptr) { mDtcmHeap->free (ptr); }
size_t getDtcmSize(){ return mDtcmHeap ? mDtcmHeap->getSize() : 0 ; }
size_t getDtcmFreeSize() { return mDtcmHeap ? mDtcmHeap->getFreeSize() : 0 ; }
size_t getDtcmMinFreeSize() { return mDtcmHeap ? mDtcmHeap->getMinFreeSize() : 0 ; }

// sram3 - 0x20040000 384k 0x60000 - system heap/static 32k 0x08000 -  use x20048000  0x80000
cRtosHeap* mSramHeap = nullptr;
//{{{
void* pvPortMalloc (size_t size) {
  if (!mSramHeap)
    mSramHeap = new cRtosHeap (0x20048000, 0x00058000, true);
  return mSramHeap->alloc (size, "");
  }
//}}}
void vPortFree (void* ptr) { mSramHeap->free (ptr); }
size_t getSramSize() { return mSramHeap ? mSramHeap->getSize() : 0 ; }
size_t getSramFreeSize() { return mSramHeap ? mSramHeap->getFreeSize() : 0 ; }
size_t getSramMinFreeSize() { return mSramHeap ? mSramHeap->getMinFreeSize() : 0 ; }

//{{{
//void* operator new (size_t size) {

  //void* allocAddress = malloc (size);
  //printf ("new %p %d\n", allocAddress, size);
  //return allocAddress;
  //}
//}}}
//{{{
//void operator delete (void* ptr) {

  //printf ("free %p\n", ptr);
  //free (ptr);
  //}
//}}}
