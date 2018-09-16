// cLcd.cpp
//{{{  includes
#include "cLcd.h"

#include "math.h"
#include "../common/heap.h"

#include "../freetype/FreeSansBold.h"
#include "cpuUsage.h"

using namespace std;
//}}}

//{{{
struct sCell {
public:
  int16_t getX() { return mPackedCoord & 0xFFFF; }
  int16_t getY() { return mPackedCoord >> 16; }
  cPoint getPoint() { return cPoint (mPackedCoord & 0xFFFF, mPackedCoord >> 16); }

  //{{{
  void set (int16_t x, int16_t y, int c, int a) {

    mPackedCoord = (y << 16) + x;
    mCoverage = c;
    mArea = a;
    }
  //}}}
  //{{{
  void setCoverage (int32_t c, int32_t a) {

    mCoverage = c;
    mArea = a;
    }
  //}}}
  //{{{
  void addCoverage (int32_t c, int32_t a) {

    mCoverage += c;
    mArea += a;
    }
  //}}}

  int32_t mPackedCoord;
  int32_t mCoverage;
  int32_t mArea;
  };
//}}}
//{{{
class cOutline {
public:
  //{{{
  cOutline() {
    mNumCellsInBlock = 2048;
    reset();
    }
  //}}}
  //{{{
  ~cOutline() {

    sram1Free (mSortedCells);

    if (mNumBlockOfCells) {
      sCell** ptr = mBlockOfCells + mNumBlockOfCells - 1;
      while (mNumBlockOfCells--) {
        // free a block of cells
        sram1Free (*ptr);
        ptr--;
        }

      // free pointers to blockOfCells
      sram1Free (mBlockOfCells);
      }
    }
  //}}}

  int32_t getMinX() const { return mMinX8; }
  int32_t getMinY() const { return mMinY8; }
  int32_t getMaxX() const { return mMaxX8; }
  int32_t getMaxY() const { return mMaxY8; }

  uint16_t getNumCells() const { return mNumCells; }
  //{{{
  sCell** getSortedCells() {

    if (!mClosed) {
      lineTo (mCloseX8, mCloseY8);
      mClosed = true;
      }

    // Perform sort only the first time.
    if (mSortRequired) {
      addCurCell();
      if (mNumCells == 0)
        return 0;
      sortCells();
      mSortRequired = false;
      }

    return mSortedCells;
    }
  //}}}

  //{{{
  void reset() {

    mNumCells = 0;
    mCurCell.set (0x7FFF, 0x7FFF, 0, 0);
    mSortRequired = true;
    mClosed = true;

    mMinX8 =  0x7FFFFFFF;
    mMinY8 =  0x7FFFFFFF;
    mMaxX8 = -0x7FFFFFFF;
    mMaxY8 = -0x7FFFFFFF;
    }
  //}}}
  //{{{
  void moveTo (int32_t x8, int32_t y8) {

    if (!mSortRequired)
      reset();

    if (!mClosed)
      lineTo (mCloseX8, mCloseY8);

    setCurCell (x8 >> 8, y8 >> 8);

    mCurX8 = x8;
    mCurY8 = y8;
    mCloseX8 = x8;
    mCloseY8 = y8;
    }
  //}}}
  //{{{
  void lineTo (int32_t x8, int32_t y8) {

    if (mSortRequired && ((mCurX8 != x8) | (mCurY8 != y8))) {
      int c = mCurX8 >> 8;
      if (c < mMinX8)
        mMinX8 = c;
      ++c;
      if (c > mMaxX8)
        mMaxX8 = c;

      c = x8 >> 8;
      if (c < mMinX8)
        mMinX8 = c;
      ++c;
      if (c > mMaxX8)
        mMaxX8 = c;

      renderLine (mCurX8, mCurY8, x8, y8);
      mCurX8 = x8;
      mCurY8 = y8;
      mClosed = false;
      }
    }
  //}}}

private:
  //{{{
  void addCurCell() {

    if (mCurCell.mArea | mCurCell.mCoverage) {
      if ((mNumCells % mNumCellsInBlock) == 0) {
        // use next block of sCells
        uint32_t block = mNumCells / mNumCellsInBlock;
        if (block >= mNumBlockOfCells) {
          // allocate new block
          auto newCellPtrs = (sCell**)sram1Alloc ((mNumBlockOfCells + 1) * sizeof(sCell*));
          if (mBlockOfCells && mNumBlockOfCells) {
            memcpy (newCellPtrs, mBlockOfCells, mNumBlockOfCells * sizeof(sCell*));
            sram1Free (mBlockOfCells);
            }
          mBlockOfCells = newCellPtrs;
          mBlockOfCells[mNumBlockOfCells] = (sCell*)sram1Alloc (mNumCellsInBlock * sizeof(sCell));
          mNumBlockOfCells++;
          printf ("allocated new blockOfCells %d of %d\n", block, mNumBlockOfCells);
          }
        mCurCellPtr = mBlockOfCells[block];
        }

      *mCurCellPtr++ = mCurCell;
      mNumCells++;
      }
    }
  //}}}
  //{{{
  void setCurCell (int16_t x, int16_t y) {

    if (mCurCell.mPackedCoord != (y << 16) + x) {
      addCurCell();
      mCurCell.set (x, y, 0, 0);
      }
   }
  //}}}
  //{{{
  void swapCells (sCell** a, sCell** b) {
    sCell* temp = *a;
    *a = *b;
    *b = temp;
    }
  //}}}
  //{{{
  void sortCells() {

    if (mNumCells == 0)
      return;

    // allocate mSortedCells, a contiguous vector of sCell pointers
    if (mNumCells > mNumSortedCells) {
      sram1Free (mSortedCells);
      mSortedCells = (sCell**)sram1Alloc ((mNumCells + 1) * 4);
      mNumSortedCells = mNumCells;
      }

    // point mSortedCells at sCells
    sCell** blockPtr = mBlockOfCells;
    sCell** sortedPtr = mSortedCells;
    uint16_t numBlocks = mNumCells / mNumCellsInBlock;
    while (numBlocks--) {
      sCell* cellPtr = *blockPtr++;
      unsigned cellInBlock = mNumCellsInBlock;
      while (cellInBlock--)
        *sortedPtr++ = cellPtr++;
      }

    sCell* cellPtr = *blockPtr++;
    unsigned cellInBlock = mNumCells % mNumCellsInBlock;
    while (cellInBlock--)
      *sortedPtr++ = cellPtr++;

    // terminate mSortedCells with nullptr
    mSortedCells[mNumCells] = nullptr;

    // sort it
    qsortCells (mSortedCells, mNumCells);
    }
  //}}}

  //{{{
  void renderScanLine (int32_t ey, int32_t x81, int32_t y81, int32_t x82, int32_t y82) {

    int ex1 = x81 >> 8;
    int ex2 = x82 >> 8;
    int fx1 = x81 & 0xFF;
    int fx2 = x82 & 0xFF;

    // trivial case. Happens often
    if (y81 == y82) {
      setCurCell (ex2, ey);
      return;
      }

    // single cell
    if (ex1 == ex2) {
      int delta = y82 - y81;
      mCurCell.addCoverage (delta, (fx1 + fx2) * delta);
      return;
      }

    // render a run of adjacent cells on the same scanLine
    int p = (0x100 - fx1) * (y82 - y81);
    int first = 0x100;
    int incr = 1;
    int dx8 = x82 - x81;
    if (dx8 < 0) {
      p = fx1 * (y82 - y81);
      first = 0;
      incr = -1;
      dx8 = -dx8;
      }

    int delta8 = p / dx8;
    int mod8 = p % dx8;
    if (mod8 < 0) {
      delta8--;
      mod8 += dx8;
      }

    mCurCell.addCoverage (delta8, (fx1 + first) * delta8);

    ex1 += incr;
    setCurCell (ex1, ey);
    y81 += delta8;
    if (ex1 != ex2) {
      p = 0x100 * (y82 - y81 + delta8);
      int lift8 = p / dx8;
      int rem8 = p % dx8;
      if (rem8 < 0) {
        lift8--;
        rem8 += dx8;
        }

      mod8 -= dx8;
      while (ex1 != ex2) {
        delta8 = lift8;
        mod8 += rem8;
        if (mod8 >= 0) {
          mod8 -= dx8;
          delta8++;
          }

        mCurCell.addCoverage (delta8, 0x100 * delta8);
        y81 += delta8;
        ex1 += incr;
        setCurCell (ex1, ey);
        }
      }

    delta8 = y82 - y81;
    mCurCell.addCoverage (delta8, (fx2 + 0x100 - first) * delta8);
    }
  //}}}
  //{{{
  void renderLine (int32_t x1, int32_t y1, int32_t x2, int32_t y2) {

    int ey1 = y1 >> 8;
    int ey2 = y2 >> 8;
    int fy1 = y1 & 0xFF;
    int fy2 = y2 & 0xFF;

    int x_from, x_to;
    int p, rem, mod, lift, delta, first;

    if (ey1 < mMinY8)
      mMinY8 = ey1;
    if (ey1+1 > mMaxY8)
      mMaxY8 = ey1+1;
    if (ey2   < mMinY8)
      mMinY8 = ey2;
    if (ey2+1 > mMaxY8)
      mMaxY8 = ey2+1;

    int dx = x2 - x1;
    int dy = y2 - y1;

    // everything is on a single cScanLine
    if (ey1 == ey2) {
      renderScanLine (ey1, x1, fy1, x2, fy2);
      return;
      }

    // Vertical line - we have to calculate start and end cell
    // the common values of the area and coverage for all cells of the line.
    // We know exactly there's only one cell, so, we don't have to call renderScanLine().
    int incr  = 1;
    if (dx == 0) {
      int ex = x1 >> 8;
      int two_fx = (x1 - (ex << 8)) << 1;
      first = 0x100;
      if (dy < 0) {
        first = 0;
        incr  = -1;
        }

      x_from = x1;
      delta = first - fy1;
      mCurCell.addCoverage (delta, two_fx * delta);

      ey1 += incr;
      setCurCell (ex, ey1);

      delta = first + first - 0x100;
      int area = two_fx * delta;
      while (ey1 != ey2) {
        mCurCell.setCoverage (delta, area);
        ey1 += incr;
        setCurCell (ex, ey1);
        }

      delta = fy2 - 0x100 + first;
      mCurCell.addCoverage (delta, two_fx * delta);
      return;
      }

    // render several scanLines
    p  = (0x100 - fy1) * dx;
    first = 0x100;
    if (dy < 0) {
      p     = fy1 * dx;
      first = 0;
      incr  = -1;
      dy    = -dy;
      }

    delta = p / dy;
    mod = p % dy;
    if (mod < 0) {
      delta--;
      mod += dy;
      }

    x_from = x1 + delta;
    renderScanLine (ey1, x1, fy1, x_from, first);

    ey1 += incr;
    setCurCell (x_from >> 8, ey1);

    if (ey1 != ey2) {
      p = 0x100 * dx;
      lift  = p / dy;
      rem   = p % dy;
      if (rem < 0) {
        lift--;
        rem += dy;
        }
      mod -= dy;
      while (ey1 != ey2) {
        delta = lift;
        mod  += rem;
        if (mod >= 0) {
          mod -= dy;
          delta++;
          }

        x_to = x_from + delta;
        renderScanLine (ey1, x_from, 0x100 - first, x_to, first);
        x_from = x_to;

        ey1 += incr;
        setCurCell (x_from >> 8, ey1);
        }
      }

    renderScanLine (ey1, x_from, 0x100 - first, x2, fy2);
    }
  //}}}

  //{{{
  void qsortCells (sCell** start, unsigned numCells) {

    sCell**  stack[80];
    sCell*** top;
    sCell**  limit;
    sCell**  base;

    limit = start + numCells;
    base = start;
    top = stack;

    while (true) {
      int len = int(limit - base);

      sCell** i;
      sCell** j;
      sCell** pivot;

      if (len > 9) { // qsort_threshold)
        // we use base + len/2 as the pivot
        pivot = base + len / 2;
        swapCells (base, pivot);

        i = base + 1;
        j = limit - 1;
        // now ensure that *i <= *base <= *j
        if ((*j)->mPackedCoord < (*i)->mPackedCoord)
          swapCells (i, j);
        if ((*base)->mPackedCoord < (*i)->mPackedCoord)
          swapCells (base, i);
        if ((*j)->mPackedCoord < (*base)->mPackedCoord)
          swapCells (base, j);

        while (true) {
          do {
            i++;
            } while ((*i)->mPackedCoord < (*base)->mPackedCoord);
          do {
            j--;
            } while ((*base)->mPackedCoord < (*j)->mPackedCoord);
          if ( i > j )
            break;
          swapCells (i, j);
          }
        swapCells (base, j);

        // now, push the largest sub-array
        if(j - base > limit - i) {
          top[0] = base;
          top[1] = j;
          base   = i;
          }
        else {
          top[0] = i;
          top[1] = limit;
          limit  = j;
          }
        top += 2;
        }
      else {
        // the sub-array is small, perform insertion sort
        j = base;
        i = j + 1;

        for (; i < limit; j = i, i++) {
          for (; (*(j+1))->mPackedCoord < (*j)->mPackedCoord; j--) {
            swapCells (j + 1, j);
            if (j == base)
              break;
            }
          }

        if (top > stack) {
          top  -= 2;
          base  = top[0];
          limit = top[1];
          }
        else
          break;
        }
      }
    }
  //}}}

  uint16_t mNumCellsInBlock = 0;
  uint16_t mNumBlockOfCells = 0;
  uint16_t mNumSortedCells = 0;
  sCell** mBlockOfCells = nullptr;
  sCell** mSortedCells = nullptr;

  uint16_t mNumCells;
  sCell mCurCell;
  sCell* mCurCellPtr = nullptr;

  int32_t mCurX8 = 0;
  int32_t mCurY8 = 0;
  int32_t mCloseX8 = 0;
  int32_t mCloseY8 = 0;
  int32_t mMinX8;
  int32_t mMinY8;
  int32_t mMaxX8;
  int32_t mMaxY8;

  bool mClosed;
  bool mSortRequired;
  };
//}}}
//{{{
class cScanLine {
public:
  //{{{
  class iterator {
  public:
    iterator (const cScanLine& scanLine) :
      mCoverage(scanLine.mCoverage), mCurCount(scanLine.mCounts), mCurStartPtr(scanLine.mStartPtrs) {}

    int next() {
      ++mCurCount;
      ++mCurStartPtr;
      return int(*mCurStartPtr - mCoverage);
      }

    int getNumPix() const { return int(*mCurCount); }
    const uint8_t* getCoverage() const { return *mCurStartPtr; }

  private:
    const uint8_t* mCoverage;
    const uint16_t* mCurCount;
    const uint8_t* const* mCurStartPtr;
    };
  //}}}
  friend class iterator;

  cScanLine() {}
  //{{{
  ~cScanLine() {

    vPortFree (mCounts);
    vPortFree (mStartPtrs);
    vPortFree (mCoverage);
    }
  //}}}

  int16_t getY() const { return mLast.y; }
  int16_t getBaseX() const { return mMinX;  }
  uint16_t getNumSpans() const { return mNumSpans; }
  bool isReady (int16_t y) const { return mNumSpans && (y != mLast.y); }

  //{{{
  void resetSpans() {

    mNumSpans = 0;
    mCurCount = mCounts;
    mCurStartPtr = mStartPtrs;
    mLast = {0x7FFF, 0x7FFF };
    }
  //}}}
  //{{{
  void reset (int16_t minx, int16_t maxx) {

    uint16_t maxLen = maxx - minx + 2;
    if (maxLen > mMaxLen) {
      // increase allocations
      mMaxLen = maxLen;

      vPortFree (mStartPtrs);
      vPortFree (mCounts);
      vPortFree (mCoverage);

      mCoverage = (uint8_t*)pvPortMalloc (maxLen);
      mStartPtrs = (uint8_t**)pvPortMalloc (maxLen * 4);
      mCounts = (uint16_t*)pvPortMalloc (maxLen * 2);
      }

    mMinX = minx;
    resetSpans();
    }
  //}}}

  //{{{
  void addPixel (cPoint p, uint16_t coverage) {

    p.x -= mMinX;

    *(mCoverage + p.x) = coverage;
    if (p.x == mLast.x + 1)
      *mCurCount += 1;
    else {
      *++mCurCount = 1;
      *++mCurStartPtr = mCoverage + p.x;
      mNumSpans++;
      }

    mLast = p;
    }
  //}}}
  //{{{
  void addSpan (cPoint p, uint16_t num, uint16_t coverage) {

    p.x -= mMinX;

    memset (mCoverage + p.x, coverage, num);
    if (p.x == mLast.x + 1)
      (*mCurCount) += (uint16_t)num;
    else {
      *++mCurCount = (uint16_t)num;
      *++mCurStartPtr = mCoverage + p.x;
      mNumSpans++;
      }

    mLast = p;
    mLast.x += num - 1;
    }
  //}}}

private:
  uint8_t* mCoverage = nullptr;
  uint8_t** mStartPtrs = nullptr;
  uint16_t* mCounts = nullptr;

  uint8_t** mCurStartPtr = nullptr;
  uint16_t* mCurCount = nullptr;

  uint16_t mNumSpans = 0;
  int16_t mMinX = 0;
  uint16_t mMaxLen = 0;
  cPoint mLast;
  };
//}}}
//{{{
class cFontChar {
public:
  void* operator new (size_t size) { return pvPortMalloc (size); }
  void operator delete (void* ptr) { vPortFree (ptr); }

  uint8_t* bitmap;
  int16_t left;
  int16_t top;
  int16_t pitch;
  int16_t rows;
  int16_t advance;
  };
//}}}

//{{{  static var inits
cLcd* cLcd::mLcd = nullptr;

static DMA2D_HandleTypeDef DMA2D_Handle;

static cLcd::eDma2dWait mDma2dWait = cLcd::eWaitNone;

static SemaphoreHandle_t mDma2dSem;

static int32_t* gRedLut = nullptr;
static int32_t* gBlueLut = nullptr;
static int32_t* gUGreenLut = nullptr;
static int32_t* gVGreenLut = nullptr;
static uint8_t* gClampLut5 = nullptr;
static uint8_t* gClampLut6 = nullptr;

static cOutline mOutline;
static cScanLine mScanLine;
static uint8_t mGamma[256];

static uint32_t mNumStamps = 0;
//}}}

//{{{
extern "C" { void DMA2D_IRQHandler() {

  uint32_t isr = DMA2D->ISR;
  if (isr & DMA2D_FLAG_TC) {
    DMA2D->IFCR = DMA2D_FLAG_TC;

    portBASE_TYPE taskWoken = pdFALSE;
    if (xSemaphoreGiveFromISR (mDma2dSem, &taskWoken) == pdTRUE)
      portEND_SWITCHING_ISR (taskWoken);
    }
  if (isr & DMA2D_FLAG_TE) {
    printf ("DMA2D_IRQHandler transfer error\n");
    DMA2D->IFCR = DMA2D_FLAG_TE;
    }
  if (isr & DMA2D_FLAG_CE) {
    printf ("DMA2D_IRQHandler config error\n");
    DMA2D->IFCR = DMA2D_FLAG_CE;
    }
  }
}
//}}}

//{{{
cLcd::~cLcd() {
  FT_Done_Face (FTface);
  FT_Done_FreeType (FTlibrary);
  }
//}}}
//{{{
void cLcd::init (const string& title) {

  FT_Init_FreeType (&FTlibrary);
  FT_New_Memory_Face (FTlibrary, (FT_Byte*)freeSansBold, sizeof (freeSansBold), 0, &FTface);
  FTglyphSlot = FTface->glyph;

  mBuffer = (uint16_t*)pvPortMalloc (LCD_WIDTH*LCD_HEIGHT*2);

  mTitle = title;

  __HAL_RCC_DMA2D_CLK_ENABLE();
  vSemaphoreCreateBinary (mDma2dSem);
  HAL_NVIC_SetPriority (DMA2D_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ (DMA2D_IRQn);

  // sw yuv to rgb565
  gRedLut = (int32_t*)dtcmAlloc (256*4);
  gBlueLut = (int32_t*)dtcmAlloc (256*4);
  gUGreenLut = (int32_t*)dtcmAlloc (256*4);
  gVGreenLut = (int32_t*)dtcmAlloc (256*4);

  for (int32_t i = 0; i <= 255; i++) {
    int32_t index = (i * 2) - 256;
    gRedLut[i] = ((((int32_t) ((1.40200 / 2) * (1L << 16))) * index) + ((int32_t) 1 << (16 - 1))) >> 16;
    gBlueLut[i] = ((((int32_t) ((1.77200 / 2) * (1L << 16))) * index) + ((int32_t) 1 << (16 - 1))) >> 16;
    gUGreenLut[i] = (-((int32_t) ((0.71414 / 2) * (1L << 16)))) * index;
    gVGreenLut[i] = (-((int32_t) ((0.34414 / 2) * (1L << 16)))) * index;
    }

  gClampLut5 = dtcmAlloc (256*3);
  gClampLut6 = dtcmAlloc (256*3);
  for (int i = 0; i < 256; i++) {
    gClampLut5[i] = 0;
    gClampLut6[i] = 0;
    }
  for (int i = 256; i < 512; i++) {
    gClampLut5[i] = (i - 256) >> 3;
    gClampLut6[i] = (i - 256) >> 2;
    }
  for (int i = 512; i < 768; i++) {
    gClampLut5[i] = 0x1F;
    gClampLut6[i] = 0x3F;
    }

  // set gamma 1.2 lut
  for (unsigned i = 0; i < 256; i++)
    mGamma[i] = (uint8_t)(pow(double(i) / 255.0, 1.6) * 255.0);

  DMA2D->OPFCCR = DMA2D_OUTPUT_RGB565;
  }
//}}}

// logging
//{{{
void cLcd::setShowInfo (bool show) {
  if (show != mShowInfo) {
    mShowInfo = show;
    mChanged = true;
    }
  }
//}}}
//{{{
void cLcd::info (const sRgba colour, const string& str) {

  uint16_t line = mCurLine++ % kMaxLines;
  mLines[line].mTime = HAL_GetTick();
  mLines[line].mColour = colour;
  mLines[line].mString = str;

  mChanged = true;
  }
//}}}

// dma2d draw
//{{{
void cLcd::rect (const sRgba colour, const cRect& r) {
// should do blending alpha version

  uint32_t rectRegs[4];
  rectRegs[0] = colour.rgb565;                                     // OCOLR
  rectRegs[1] = uint32_t (mBuffer + r.top * getWidth() + r.left);  // OMAR
  rectRegs[2] = getWidth() - r.getWidth();                         // OOR
  rectRegs[3] = (r.getWidth() << 16) | r.getHeight();              // NLR

  ready();
  memcpy ((void*)(&DMA2D->OCOLR), rectRegs, 5*4);
  DMA2D->CR = DMA2D_R2M | DMA2D_CR_START | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE;
  mDma2dWait = eWaitIrq;
  }
//}}}
//{{{
void cLcd::clear (const sRgba colour) {

  cRect r (getSize());
  rect (colour, r);
  }
//}}}
//{{{
void cLcd::rectClipped (const sRgba colour, cRect r) {

  if (r.right <= 0)
    return;
  if (r.bottom <= 0)
    return;

  if (r.left >= getWidth())
    return;
  if (r.left < 0)
    r.left = 0;
  if (r.right > getWidth())
    r.right = getWidth();
  if (r.right <= r.left)
    return;

  if (r.top >= getHeight())
    return;
  if (r.top < 0)
    r.top = 0;
  if (r.bottom > getHeight())
    r.bottom = getHeight();
  if (r.bottom <= r.top)
    return;

  rect (colour, r);
  }
//}}}
//{{{
void cLcd::rectOutline (const sRgba colour, const cRect& r, uint8_t thickness) {

  rectClipped (colour, cRect (r.left, r.top, r.right, r.top+thickness));
  rectClipped (colour, cRect (r.right-thickness, r.top, r.right, r.bottom));
  rectClipped (colour, cRect (r.left, r.bottom-thickness, r.right, r.bottom));
  rectClipped (colour, cRect (r.left, r.top, r.left+thickness, r.bottom));
  }
//}}}
//{{{
void cLcd::ellipse (const sRgba colour, cPoint centre, cPoint radius) {

  if (!radius.x)
    return;
  if (!radius.y)
    return;

  int x1 = 0;
  int y1 = -radius.x;
  int err = 2 - 2*radius.x;
  float k = (float)radius.y / radius.x;

  do {
    rectClipped (colour, cRect (centre.x-(uint16_t)(x1 / k), centre.y + y1,
                                centre.x-(uint16_t)(x1 / k) + 2*(uint16_t)(x1 / k) + 1, centre.y  + y1 + 1));
    rectClipped (colour, cRect (centre.x-(uint16_t)(x1 / k), centre.y  - y1,
                                centre.x-(uint16_t)(x1 / k) + 2*(uint16_t)(x1 / k) + 1, centre.y  - y1 + 1));

    int e2 = err;
    if (e2 <= x1) {
      err += ++x1 * 2 + 1;
      if (-y1 == centre.x && e2 <= y1)
        e2 = 0;
      }
    if (e2 > y1)
      err += ++y1*2 + 1;
    } while (y1 <= 0);
  }
//}}}
//{{{
int16_t cLcd::text (const sRgba colour, uint16_t fontHeight, const string& str, cRect r) {

  ready();
  DMA2D->FGPFCCR = (colour.getA() < 255) ? ((colour.getA() << 24) | 0x20000 | DMA2D_INPUT_A8) : DMA2D_INPUT_A8;
  DMA2D->FGCOLR = (colour.getR() << 16) | (colour.getG() << 8) | colour.getB();

  for (auto ch : str) {
    if ((ch >= 0x20) && (ch <= 0x7F)) {
      auto fontCharIt = mFontCharMap.find ((fontHeight << 8) | ch);
      cFontChar* fontChar = fontCharIt != mFontCharMap.end() ? fontCharIt->second : nullptr;
      if (!fontChar)
        fontChar = loadChar (fontHeight, ch);
      if (fontChar) {
        if (r.left + fontChar->left + fontChar->pitch >= r.right)
          break;
        else if (fontChar->bitmap) {
          auto src = fontChar->bitmap;
          cRect charRect (r.left + fontChar->left, r.top + fontHeight - fontChar->top,
                          r.left + fontChar->left + fontChar->pitch,
                          r.top + fontHeight - fontChar->top + fontChar->rows);

          // simple clips
          if (charRect.top < 0) {
            src += -charRect.top * charRect.getWidth();
            charRect.top = 0;
            }
          if (charRect.bottom > getHeight())
            charRect.bottom = getHeight();

          if ((charRect.left >= 0) && (charRect.bottom > 0) && (charRect.top < getHeight())) {
            uint32_t dstAddr = uint32_t(mBuffer + charRect.top * getWidth() + charRect.left);
            uint32_t stride = getWidth() - charRect.getWidth();

            ready();
            DMA2D->BGPFCCR = DMA2D_INPUT_RGB565;
            DMA2D->BGMAR = dstAddr;
            DMA2D->OMAR = dstAddr;
            DMA2D->BGOR = stride;
            DMA2D->OOR = stride;
            DMA2D->NLR = (charRect.getWidth() << 16) | charRect.getHeight();
            DMA2D->FGMAR = (uint32_t)src;
            DMA2D->FGOR = 0;
            DMA2D->CR = DMA2D_M2M_BLEND | DMA2D_CR_START | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE;
            mDma2dWait = eWaitIrq;
            }
          }
        r.left += fontChar->advance;
        }
      }
    }

  return r.left;
  }
//}}}

// cpu draw
//{{{
void cLcd::grad (const sRgba colTL, const sRgba colTR, const sRgba colBL, const sRgba colBR, const cRect& r) {

  int32_t rTL = colTL.getR() << 16;
  int32_t rTR = colTR.getR() << 16;
  int32_t rBL = colBL.getR() << 16;
  int32_t rBR = colBR.getR() << 16;

  int32_t gTL = colTL.getG() << 16;
  int32_t gTR = colTR.getG() << 16;
  int32_t gBL = colBL.getG() << 16;
  int32_t gBR = colBR.getG() << 16;

  int32_t bTL = colTL.getB() << 16;
  int32_t bTR = colTR.getB() << 16;
  int32_t bBL = colBL.getB() << 16;
  int32_t bBR = colBR.getB() << 16;

  int32_t rl16 = rTL;
  int32_t gl16 = gTL;
  int32_t bl16 = bTL;
  int32_t rGradl16 = (rBL - rTL) / r.getHeight();
  int32_t gGradl16 = (gBL - gTL) / r.getHeight();
  int32_t bGradl16 = (bBL - bTL) / r.getHeight();

  int32_t rr16 = rTR;
  int32_t gr16 = gTR;
  int32_t br16 = bTR;
  int32_t rGradr16 = (rBR - rTR) / r.getHeight();
  int32_t gGradr16 = (gBR - gTR) / r.getHeight();
  int32_t bGradr16 = (bBR - bTR) / r.getHeight();

  auto dst = (uint16_t*)mBuffer + r.top * getWidth() + r.left;
  for (uint16_t y = r.top; y < r.bottom; y++) {
    int32_t rGradx16 = (rr16 - rl16) / r.getWidth();
    int32_t gGradx16 = (gr16 - gl16) / r.getWidth();
    int32_t bGradx16 = (br16 - bl16) / r.getWidth();

    int32_t r16 = rl16;
    int32_t g16 = gl16;
    int32_t b16 = bl16;
    for (uint16_t x = r.left; x < r.right; x++) {
      *dst++ = (b16 >> 16) | ((g16 >> 11) & 0x07E0) | ((r16 >> 5) & 0xF800);
      r16 += rGradx16;
      g16 += gGradx16;
      b16 += bGradx16;
      }
    dst += getWidth() - r.getWidth();

    rl16 += rGradl16;
    gl16 += gGradl16;
    bl16 += bGradl16;

    rr16 += rGradr16;
    gr16 += gGradr16;
    br16 += bGradr16;
    }
  }
//}}}
//{{{
void cLcd::line (const sRgba colour, cPoint p1, cPoint p2) {

  int16_t deltax = abs(p2.x - p1.x); // The difference between the x's
  int16_t deltay = abs(p2.y - p1.y); // The difference between the y's

  cPoint p = p1;
  cPoint inc1 ((p2.x >= p1.x) ? 1 : -1, (p2.y >= p1.y) ? 1 : -1);
  cPoint inc2 = inc1;

  int16_t numAdd = (deltax >= deltay) ? deltay : deltax;
  int16_t den = (deltax >= deltay) ? deltax : deltay;
  if (deltax >= deltay) { // There is at least one x-value for every y-value
    inc1.x = 0;            // Don't change the x when numerator >= denominator
    inc2.y = 0;            // Don't change the y for every iteration
    }
  else {                  // There is at least one y-value for every x-value
    inc2.x = 0;            // Don't change the x for every iteration
    inc1.y = 0;            // Don't change the y when numerator >= denominator
    }

  int16_t num = den / 2;
  int16_t numPix = den;
  for (int16_t pix = 0; pix <= numPix; pix++) {
    pixel (colour, p);
    num += numAdd;     // Increase the numerator by the top of the fraction
    if (num >= den) {   // Check if numerator >= denominator
      num -= den;       // Calculate the new numerator value
      p += inc1;
      }
    p += inc2;
    }
  }
//}}}
//{{{
void cLcd::ellipseOutline (const sRgba colour, cPoint centre, cPoint radius) {

  int x = 0;
  int y = -radius.y;

  int err = 2 - 2 * radius.x;
  float k = (float)radius.y / (float)radius.x;

  do {
    pixel (colour, centre + cPoint (-(int16_t)(x / k), y));
    pixel (colour, centre + cPoint ((int16_t)(x / k), y));
    pixel (colour, centre + cPoint ((int16_t)(x / k), -y));
    pixel (colour, centre + cPoint (- (int16_t)(x / k), - y));

    int e2 = err;
    if (e2 <= x) {
      err += ++x * 2+ 1 ;
      if (-y == x && e2 <= y)
        e2 = 0;
      }
    if (e2 > y)
      err += ++y *2 + 1;
    } while (y <= 0);
  }
//}}}

// agg render
//{{{
void cLcd::aMoveTo (const cPointF& p) {
  mOutline.moveTo (int(p.x * 256.f), int(p.y * 256.f));
  }
//}}}
//{{{
void cLcd::aLineTo (const cPointF& p) {
  mOutline.lineTo (int(p.x * 256.f), int(p.y * 256.f));
  }
//}}}
//{{{
void cLcd::aWideLine (const cPointF& p1, const cPointF& p2, float width) {

  cPointF perp = (p2-p1).perp() * width;
  aMoveTo (p1 + perp);
  aLineTo (p2 + perp);
  aLineTo (p2 - perp);
  aLineTo (p1 - perp);
  }
//}}}
//{{{
void cLcd::aPointedLine (const cPointF& p1, const cPointF& p2, float width) {

  cPointF perp = (p2-p1).perp() * width;
  aMoveTo (p1 + perp);
  aLineTo (p2);
  aLineTo (p1 - perp);
  }
//}}}
//{{{
void cLcd::aEllipse (const cPointF& centre, const cPointF& radius, int steps) {

  // anticlockwise ellipse
  float angle = 0.f;
  float fstep = 360.f / steps;
  aMoveTo (centre + cPointF(radius.x, 0.f));

  angle += fstep;
  while (angle < 360.f) {
    auto radians = angle * 3.1415926f / 180.0f;
    aLineTo (centre + cPointF (cos(radians) * radius.x, sin(radians) * radius.y));
    angle += fstep;
    }
  }
//}}}
//{{{
void cLcd::aEllipseOutline (const cPointF& centre, const cPointF& radius, float width, int steps) {

  float angle = 0.f;
  float fstep = 360.f / steps;
  aMoveTo (centre + cPointF(radius.x, 0.f));

  // clockwise ellipse
  angle += fstep;
  while (angle < 360.f) {
    auto radians = angle * 3.1415926f / 180.0f;
    aLineTo (centre + cPointF (cos(radians) * radius.x, sin(radians) * radius.y));
    angle += fstep;
    }

  // anti clockwise ellipse
  aMoveTo (centre + cPointF(radius.x - width, 0.f));

  angle -= fstep;
  while (angle > 0.f) {
    auto radians = angle * 3.1415926f / 180.0f;
    aLineTo (centre + cPointF (cos(radians) * (radius.x - width), sin(radians) * (radius.y - width)));
    angle -= fstep;
    }
  }
//}}}
//{{{
void cLcd::aRender (const sRgba colour, bool fillNonZero) {

  sCell** sortedCells = mOutline.getSortedCells();
  uint32_t numCells = mOutline.getNumCells();
  if (!numCells)
    return;

  mNumStamps = 0;
  mScanLine.reset (mOutline.getMinX(), mOutline.getMaxX());

  int coverage = 0;
  sCell* cell = *sortedCells++;
  while (true) {
    cPoint p  = cell->getPoint();
    int packedCoord = cell->mPackedCoord;
    int area = cell->mArea;
    coverage += cell->mCoverage;

    // accumulate all start cells
    while ((cell = *sortedCells++) != 0) {
      if (cell->mPackedCoord != packedCoord)
        break;
      area += cell->mArea;
      coverage += cell->mCoverage;
      }

    if (area) {
      uint8_t alpha = calcAlpha ((coverage << 9) - area, fillNonZero);
      if (alpha) {
        if (mScanLine.isReady (p.y)) {
          renderScanLine (&mScanLine, colour);
          mScanLine.resetSpans();
          }
        mScanLine.addPixel (p, mGamma[alpha]);
        }
      p.x++;
      }

    if (!cell)
      break;

    if (cell->getX() > p.x) {
      uint8_t alpha = calcAlpha (coverage << 9, fillNonZero);
      if (alpha) {
        if (mScanLine.isReady (p.y)) {
          renderScanLine (&mScanLine, colour);
          mScanLine.resetSpans();
          }
        mScanLine.addSpan (p, cell->getX() - p.x, mGamma[alpha]);
        }
      }
    }

  if (mScanLine.getNumSpans())
    renderScanLine (&mScanLine, colour);

  //printf ("render cells:%d stamps:%d\n", numCells, mNumStamps);
  }
//}}}

// cTile
//{{{
void cLcd::copy (cTile* tile, cPoint p) {

  uint16_t width = p.x + tile->mWidth > getWidth() ? getWidth() - p.x : tile->mWidth;
  uint16_t height = p.y + tile->mHeight > getHeight() ? getHeight() - p.y : tile->mHeight;

  ready();

  switch (tile->mFormat) {
    case cTile::eRgb565 : DMA2D->FGPFCCR = DMA2D_INPUT_RGB565; break;
    case cTile::eRgb888 : DMA2D->FGPFCCR = DMA2D_INPUT_RGB888; break;
    }

  DMA2D->FGMAR = (uint32_t)tile->mPiccy;
  DMA2D->FGOR = tile->mPitch - width;

  DMA2D->OMAR = uint32_t(mBuffer + (p.y * getWidth()) + p.x);
  DMA2D->OOR = getWidth() > tile->mWidth ? getWidth() - tile->mWidth : 0;

  DMA2D->NLR = (width << 16) | height;

  DMA2D->CR = DMA2D_M2M_PFC | DMA2D_CR_START | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE;
  mDma2dWait = eWaitIrq;
  }
//}}}
//{{{
void cLcd::copy90 (cTile* tile, cPoint p) {

  uint32_t src = (uint32_t)tile->mPiccy;
  uint32_t dst = (uint32_t)mBuffer;

  ready();
  DMA2D->FGPFCCR = DMA2D_INPUT_RGB565;
  DMA2D->FGOR = 0;

  DMA2D->OOR = getWidth() - 1;
  DMA2D->NLR = 0x10000 | (tile->mWidth);

  for (int line = 0; line < tile->mHeight; line++) {
    DMA2D->FGMAR = src;
    DMA2D->OMAR = dst;
    DMA2D->CR = DMA2D_M2M_PFC | DMA2D_CR_START | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE;
    src += tile->mWidth * tile->mComponents;
    dst += 2;

    mDma2dWait = eWaitDone;
    ready();
    }
  }
//}}}
//{{{
void cLcd::size (cTile* tile, const cRect& r) {

  uint32_t xStep16 = ((tile->mWidth - 1) << 16) / (r.getWidth() - 1);
  uint32_t yStep16 = ((tile->mHeight - 1) << 16) / (r.getHeight() - 1);
  __IO uint16_t* dst = (uint16_t*)mBuffer + r.top * getWidth() + r.left;

  if (tile->mFormat == cTile::eRgb565) {
    //{{{  rgb565 size
    for (uint32_t y16 = (tile->mY << 16); y16 < ((tile->mY + r.getHeight()) * yStep16); y16 += yStep16) {
      auto src = (uint16_t*)(tile->mPiccy) + (tile->mY + (y16 >> 16)) * tile->mPitch + tile->mX;
      for (uint32_t x16 = tile->mX << 16; x16 < (tile->mX + r.getWidth()) * xStep16; x16 += xStep16)
        *dst++ = *(src + (x16 >> 16));
      dst += getWidth() - r.getWidth();
      }
    }
    //}}}
  else if (tile->mFormat == cTile::eRgb888) {
    //{{{  rgb888 size
    for (uint32_t y16 = (tile->mY << 16); y16 < ((tile->mY + r.getHeight()) * yStep16); y16 += yStep16) {
      uint8_t* src = tile->mPiccy + ((tile->mY + (y16 >> 16)) * tile->mPitch + tile->mX) * 3;
      for (uint32_t x16 = tile->mX << 16; x16 < (tile->mX + r.getWidth()) * xStep16; x16 += xStep16) {
        uint8_t r = *src++;
        uint8_t g = *src++;
        uint8_t b = *src++;
        *dst++ = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        }
      dst += getWidth() - r.getWidth();
      }
    }
    //}}}
  else {
    // yuv422 size
    for (uint32_t y16 = (tile->mY << 16); y16 < ((tile->mY + r.getHeight()) * yStep16); y16 += yStep16) {
      uint32_t y = y16 >> 16;
      for (uint32_t x16 = tile->mX << 16; x16 < (tile->mX + r.getWidth()) * xStep16; x16 += xStep16) {
        uint32_t x = x16 >> 16;
        uint32_t mcu = ((y/8) * (tile->mWidth/16)) + (x/16);
        uint8_t* mcuPtr = tile->mPiccy + (mcu * 256) + ((y & 0x07) * 8);
        uint8_t* lumPtr = mcuPtr + ((x & 0x08) ? 64 : 0) + (x & 0x07);
        uint8_t* chrPtr = mcuPtr + 128 + ((x/2) & 0x07);
        uint16_t y = *lumPtr + 0x100;
        *dst++ = (gClampLut5[y + *(gRedLut + *(chrPtr + 64))] << 11) |
                 (gClampLut6[y + ((*(gUGreenLut + *chrPtr) + *(gVGreenLut + *(chrPtr + 64))) >> 16)] << 5) |
                  gClampLut5[y + *(gBlueLut + *chrPtr)];
        }
      dst += getWidth() - r.getWidth();
      }
    }
  }
//}}}

//{{{
void cLcd::display (int brightness) {

  mBrightness = brightness;
  TIM4->CCR2 = 50 * brightness;
  }
//}}}
//{{{
void cLcd::start() {
  mStartTime = HAL_GetTick();
  }
//}}}
//{{{
void cLcd::drawInfo() {

  const int kTitleHeight = 20;
  const int kFooterHeight = 14;
  const int kInfoHeight = 12;
  const int kGap = 4;
  const int kSmallGap = 2;

  // draw title
  const cRect titleRect (0,0, getWidth(), kTitleHeight+kGap);
  text (kBlackSemi, kTitleHeight, mTitle, titleRect);
  text (kYellow, kTitleHeight, mTitle, titleRect + cPoint(-2,-2));

  if (mShowInfo) {
    // draw footer
    auto y = getHeight() - kFooterHeight - kGap;
    text (kWhite, kFooterHeight,
          dec(mNumPresents) + ":" + dec (mDrawTime) + ":" + dec (mPresentTime) + " " +
          dec (osGetCPUUsage()) + "%"
          " s:" + dec (getSram1FreeSize()/1000) + ":" + dec (getSram1Size()/1000) +
          " d:" + dec (getDtcmFreeSize()/1000) + ":" + dec (getDtcmSize()/1000) +
          " p:" + dec (getSramFreeSize()/1000) + ":" + dec (getSramMinFreeSize()/1000) + ":" + dec (getSramSize()/1000),
          cRect(0, y, getWidth(), kTitleHeight+kGap));

    // draw log
    y -= kTitleHeight - kGap;
    auto line = mCurLine - 1;
    while ((y > kTitleHeight) && (line >= 0)) {
      int lineIndex = line-- % kMaxLines;
      auto x = text (kGreen, kInfoHeight,
                     dec ((mLines[lineIndex].mTime-mBaseTime) / 1000) + "." +
                     dec ((mLines[lineIndex].mTime-mBaseTime) % 1000, 3, '0'),
                     cRect(0, y, getWidth(), 20));
      text (mLines[lineIndex].mColour, kInfoHeight, mLines[lineIndex].mString,
            cRect (x + kSmallGap, y, getWidth(), 20));
      y -= kInfoHeight + kSmallGap;
      }
    }
  }
//}}}
//{{{
void cLcd::present() {

  // send start position
  volatile uint16_t* dst = (volatile uint16_t*)0x60000000;
  *dst = 0x20;
  *dst = 0;
  *dst = 0x21;
  *dst = 0;
  *dst = 0x22;

  ready();
  mDrawTime = HAL_GetTick() - mStartTime;
  mPresentTime = getPresentTime();
  mLastPresentTime = HAL_GetTick();

  // send data with dma2d
  DMA2D->FGPFCCR = DMA2D_INPUT_RGB565;
  DMA2D->FGMAR = (uint32_t)mBuffer;
  DMA2D->FGOR = 0;
  DMA2D->OMAR = 0x60080000;
  DMA2D->OOR = 0;
  DMA2D->NLR = (320 << 16) | 480;
  DMA2D->CR = DMA2D_M2M_PFC | DMA2D_CR_START | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE;
  mDma2dWait = eWaitIrq;

  mNumPresents++;
  }
//}}}

// private
//{{{
void cLcd::tftInit() {

  //{{{  clocks
  __HAL_RCC_FMC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  //}}}
  //{{{  pb7 adj tim4 PWM
  __HAL_RCC_TIM4_CLK_ENABLE();

  GPIO_InitTypeDef gpio_init_structure;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Alternate = GPIO_AF2_TIM4;
  gpio_init_structure.Pin = GPIO_PIN_7;
  HAL_GPIO_Init (GPIOB, &gpio_init_structure);

  //  config TIM4 chan2 PWM to PD13
  TIM_HandleTypeDef mTimHandle;
  mTimHandle.Instance = TIM4;
  mTimHandle.Init.Period = 5000 - 1;
  mTimHandle.Init.Prescaler = 1;
  mTimHandle.Init.ClockDivision = 0;
  mTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init (&mTimHandle))
    printf ("HAL_TIM_Base_Init failed\n");

  // init timOcInit
  TIM_OC_InitTypeDef timOcInit = {0};
  timOcInit.OCMode = TIM_OCMODE_PWM1;
  timOcInit.OCPolarity = TIM_OCPOLARITY_HIGH;
  timOcInit.OCFastMode = TIM_OCFAST_DISABLE;
  timOcInit.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  timOcInit.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  timOcInit.OCIdleState = TIM_OCIDLESTATE_RESET;
  timOcInit.Pulse = 50 * mBrightness;

  if (HAL_TIM_PWM_ConfigChannel (&mTimHandle, &timOcInit, TIM_CHANNEL_2))
    printf ("HAL_TIM_PWM_ConfigChannel tim4 ch2 fail\n");

  if (HAL_TIM_PWM_Start (&mTimHandle, TIM_CHANNEL_2))
    printf ("HAL_TIM_PWM_Start tim4 ch2 fail\n");
  //}}}

  // gpio config
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Pin = GPIO_PIN_12;
  HAL_GPIO_Init (GPIOD, &gpio_init_structure);
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // resetHi

  gpio_init_structure.Alternate = GPIO_AF12_FMC;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  // gpioD - noe -> rd, nwe -> wr, ne1 ->cs, a18->rs
  gpio_init_structure.Pin = GPIO_PIN_4  | GPIO_PIN_5  | GPIO_PIN_7 | GPIO_PIN_13 |
                            GPIO_PIN_14 | GPIO_PIN_15 |               // d0:d1
                            GPIO_PIN_0  | GPIO_PIN_1  |               // d2:d3
                            GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10;  // d13:d15
  HAL_GPIO_Init (GPIOD, &gpio_init_structure);

  // gpioE
  gpio_init_structure.Pin = GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 |
                            GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; // d4:d12
  HAL_GPIO_Init (GPIOE, &gpio_init_structure);

  //{{{  reset pulse low
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // resetLo
  vTaskDelay (1);
  HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // resetHi
  vTaskDelay (120);
  //}}}

  //  fmc config - SRAM bank1 0x60000000 accessModeA
  #define FMC_NORSRAM_DEVICE           FMC_Bank1_R
  #define FMC_WRITE_OPERATION_ENABLE   FMC_BCRx_WREN
  #define FMC_NORSRAM_MEM_BUS_WIDTH_16 FMC_BCRx_MWID_0

  #define kAddressSetupTime 3 // 33ns @ 120Mhz, 8.3ns
  #define kAddressHoldTime  0
  #define kDataSetupTime    4 // 33ns @ 120Mhz, 8.3ns

  // FMC_NORSRAM_BANK1 - FMC_ACCESS_MODE_A
  FMC_NORSRAM_DEVICE->BTCR [1] = kAddressSetupTime |
                                (kAddressHoldTime << POSITION_VAL(FMC_BTRx_ADDHLD)) |
                                (kDataSetupTime   << POSITION_VAL(FMC_BTRx_DATAST));

  // FMC_NORSRAM_BANK1 - FMC_MEMORY_TYPE_SRAM
  FMC_NORSRAM_DEVICE->BTCR [0] = FMC_NORSRAM_MEM_BUS_WIDTH_16 | FMC_WRITE_OPERATION_ENABLE | FMC_BCRx_MBKEN;

  //{{{  send lcd commands
  sendCommandData (0x01, 0x023C);
  sendCommandData (0x02, 0x0100);
  sendCommandData (0x03, 0x1030);
  sendCommandData (0x08, 0x0808);
  sendCommandData (0x0A, 0x0500);
  sendCommandData (0x0B, 0x0000);
  sendCommandData (0x0C, 0x0770);
  sendCommandData (0x0D, 0x0000);
  sendCommandData (0x0E, 0x0001);
  sendCommandData (0x11, 0x0406);
  sendCommandData (0x12, 0x000E);
  sendCommandData (0x13, 0x0222);
  sendCommandData (0x14, 0x0015);
  sendCommandData (0x15, 0x4277);
  sendCommandData (0x16, 0x0000);

  sendCommandData (0x30, 0x6A50);
  sendCommandData (0x31, 0x00C9);
  sendCommandData (0x32, 0xC7BE);
  sendCommandData (0x33, 0x0003);
  sendCommandData (0x36, 0x3443);
  sendCommandData (0x3B, 0x0000);
  sendCommandData (0x3C, 0x0000);
  sendCommandData (0x2C, 0x6A50);
  sendCommandData (0x2D, 0x00C9);
  sendCommandData (0x2E, 0xC7BE);
  sendCommandData (0x2F, 0x0003);
  sendCommandData (0x35, 0x3443);
  sendCommandData (0x39, 0x0000);
  sendCommandData (0x3A, 0x0000);
  sendCommandData (0x28, 0x6A50);
  sendCommandData (0x29, 0x00C9);
  sendCommandData (0x2A, 0xC7BE);
  sendCommandData (0x2B, 0x0003);
  sendCommandData (0x34, 0x3443);
  sendCommandData (0x37, 0x0000);
  sendCommandData (0x38, 0x0000);
  vTaskDelay (10);

  sendCommandData (0x12, 0x200E);
  vTaskDelay (10);

  sendCommandData (0x12, 0x2003);
  vTaskDelay (10);

  sendCommandData (0x44, 0x013F);
  sendCommandData (0x45, 0x0000);
  sendCommandData (0x46, 0x01DF);
  sendCommandData (0x47, 0x0000);
  sendCommandData (0x20, 0x0000);
  sendCommandData (0x21, 0x013F);
  sendCommandData (0x07, 0x0012);
  vTaskDelay (10);

  sendCommandData (0x07, 0x0017);
  vTaskDelay (10);
  //}}}
  }
//}}}
//{{{
void cLcd::sendCommandData (uint16_t reg, uint16_t data) {

  *((volatile uint16_t*)0x60000000) = reg;
  *((volatile uint16_t*)0x60080000) = data;
  }
//}}}
//{{{
cFontChar* cLcd::loadChar (uint16_t fontHeight, char ch) {

  FT_Set_Pixel_Sizes (FTface, 0, fontHeight);
  FT_Load_Char (FTface, ch, FT_LOAD_RENDER);

  auto fontChar = new cFontChar();
  fontChar->left = FTglyphSlot->bitmap_left;
  fontChar->top = FTglyphSlot->bitmap_top;
  fontChar->pitch = FTglyphSlot->bitmap.pitch;
  fontChar->rows = FTglyphSlot->bitmap.rows;
  fontChar->advance = FTglyphSlot->advance.x / 64;
  fontChar->bitmap = nullptr;

  if (FTglyphSlot->bitmap.buffer) {
    fontChar->bitmap = (uint8_t*)sram1Alloc (FTglyphSlot->bitmap.pitch * FTglyphSlot->bitmap.rows);
    memcpy (fontChar->bitmap, FTglyphSlot->bitmap.buffer, FTglyphSlot->bitmap.pitch * FTglyphSlot->bitmap.rows);
    }

  return mFontCharMap.insert (map<uint16_t, cFontChar*>::value_type (fontHeight<<8 | ch, fontChar)).first->second;
  }
//}}}

//{{{
void cLcd::ready() {

  switch (mDma2dWait) {
    case eWaitDone:
      while (!(DMA2D->ISR & DMA2D_FLAG_TC))
        taskYIELD();
      DMA2D->IFCR = DMA2D_FLAG_TC;
      break;

    case eWaitIrq:
      if (!xSemaphoreTake (mDma2dSem, 5000))
        printf ("cLcd ready take fail\n");
      break;

    case eWaitNone:
      break;
    }

  mDma2dWait = eWaitNone;
  }
//}}}
//{{{
void cLcd::reset() {

  for (auto i = 0; i < kMaxLines; i++)
    mLines[i].clear();

  mBaseTime = HAL_GetTick();
  mCurLine = 0;
  }
//}}}

//{{{
uint8_t cLcd::calcAlpha (int area, bool fillNonZero) const {

  int coverage = area >> 9;
  if (coverage < 0)
    coverage = -coverage;

  if (!fillNonZero) {
    coverage &= 0x1FF;
    if (coverage > 0x100)
      coverage = 0x200 - coverage;
    }

  if (coverage > 0xFF)
    coverage = 0xFF;

  return coverage;
  }
//}}}
//{{{
void cLcd::renderScanLine (cScanLine* scanLine, const sRgba colour) {

  // yclip top
  auto y = scanLine->getY();
  if (y < 0)
    return;

  // yclip bottom
  if (y >= getHeight())
    return;

  ready();
  DMA2D->FGPFCCR = (colour.getA() < 255) ? ((colour.getA() << 24) | 0x20000 | DMA2D_INPUT_A8) : DMA2D_INPUT_A8;
  DMA2D->FGCOLR = (colour.getR() << 16) | (colour.getG() << 8) | colour.getB();;

  int baseX = scanLine->getBaseX();
  uint16_t numSpans = scanLine->getNumSpans();
  cScanLine::iterator span (*scanLine);
  do {
    auto x = baseX + span.next() ;
    auto coverage = (uint8_t*)span.getCoverage();

    // xclip left
    int16_t numPix = span.getNumPix();
    if (x < 0) {
      numPix += x;
      if (numPix <= 0)
        continue;
      coverage -= x;
      x = 0;
      }

    // xclip right
    if (x + numPix >= getWidth()) {
      numPix = getWidth() - x;
      if (numPix <= 0)
        continue;
      }

    mNumStamps++;

    uint32_t dstAddr = uint32_t(mBuffer + y * getWidth() + x);
    uint32_t stride = getWidth() - numPix;

    ready();
    DMA2D->BGPFCCR = DMA2D_INPUT_RGB565;
    DMA2D->BGMAR = dstAddr;
    DMA2D->OMAR = dstAddr;
    DMA2D->BGOR = stride;
    DMA2D->OOR = stride;
    DMA2D->NLR = (numPix << 16) | 1;
    DMA2D->FGMAR = (uint32_t)coverage;
    DMA2D->FGOR = 0;
    DMA2D->CR = DMA2D_M2M_BLEND | DMA2D_CR_START;
    mDma2dWait = eWaitDone;
    } while (--numSpans);
  }
//}}}
