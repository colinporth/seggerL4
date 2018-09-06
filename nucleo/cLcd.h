#pragma once
//{{{  includes
#include "cmsis_os.h"
#include "semphr.h"

#include "../common/utils.h"
#include "../common/cPointRect.h"
#include <map>
#include <vector>

#include "../system/stm32l4xx.h"
#include "../common/heap.h"

#include <ft2build.h>
#include FT_FREETYPE_H
//}}}
//{{{  screen resolution defines
#define LCD_WIDTH        320
#define LCD_HEIGHT       480

#define BOX_HEIGHT         30
#define SMALL_FONT_HEIGHT  12
#define FONT_HEIGHT        26
#define BIG_FONT_HEIGHT    40
//}}}
//{{{
struct sRgba565 {
  sRgba565 (uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) :
    rgb565 (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)), alpha(a) {}

  uint8_t getR() { return (rgb565 & 0xF800) >> 8; }
  uint8_t getG() { return (rgb565 & 0x07E0) >> 3; }
  uint8_t getB() { return (rgb565 & 0x001F) << 3; }
  uint8_t getA() { return alpha; }

  uint16_t rgb565;
  uint8_t alpha;
  };
//}}}
//{{{  colour const
const sRgba565 kBlack (0,0,0);
const sRgba565 kBlackSemi (0,0,0, 128);
const sRgba565 kGrey (128,128,128);
const sRgba565 kWhite (255,255,255);

const sRgba565 kBlue (0,0,255);
const sRgba565 kGreen (0,255,0);
const sRgba565 kRed (255,0,0);

const sRgba565 kCyan (0,255,255);
const sRgba565 kMagenta (255,0,255);
const sRgba565 kYellow (255,255,0);
//}}}

//{{{
class cTile {
public:
  enum eFormat { eRgb565, eRgb888, eYuvMcu422 };

  cTile() {};
  cTile (uint8_t* piccy, eFormat format, uint16_t pitch, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
     : mPiccy(piccy), mFormat(format), mPitch(pitch), mX(x), mY(y), mWidth(width), mHeight(height) {}

  ~cTile () {
    vPortFree (mPiccy);
    mPiccy = nullptr;
    };

  void* operator new (std::size_t size) { return pvPortMalloc (size); }
  void operator delete (void* ptr) { vPortFree (ptr); }

  uint8_t* mPiccy = nullptr;
  uint16_t mComponents = 0;
  uint16_t mPitch = 0;
  uint16_t mX = 0;
  uint16_t mY = 0;
  uint16_t mWidth = 0;
  uint16_t mHeight = 0;
  eFormat mFormat = eRgb565;
  };
//}}}
class cFontChar;
class cScanLine;

class cLcd {
public:
  enum eDma2dWait { eWaitNone, eWaitDone, eWaitIrq };

  cLcd()  { mLcd = this; }
  ~cLcd();

  void init (const std::string& title);
  void tftInit();

  static uint16_t getWidth() { return LCD_WIDTH; }
  static uint16_t getHeight() { return LCD_HEIGHT; }
  static cPoint getSize() { return cPoint (getWidth(), getHeight()); }
  static uint16_t getBoxHeight() { return BOX_HEIGHT; }
  static uint16_t getSmallFontHeight() { return SMALL_FONT_HEIGHT; }
  static uint16_t getFontHeight() { return FONT_HEIGHT; }
  static uint16_t getBigFontHeight() { return BIG_FONT_HEIGHT; }

  uint16_t getBrightness() { return mBrightness; }
  uint32_t getPresentTime() { return HAL_GetTick() - mLastPresentTime; }

  void setShowInfo (bool show);
  void setTitle (const std::string& str) { mTitle = str; mChanged = true; }
  void change() { mChanged = true; }
  //{{{
  bool isChanged() {
    bool wasChanged = mChanged;
    mChanged = false;
    return wasChanged;
    }
  //}}}

  void info (sRgba565 colour, const std::string& str);
  void info (const std::string& str) { info (kWhite, str); }

  void clear (sRgba565 colour);
  void rect (sRgba565 colour, const cRect& r);
  void rectClipped (sRgba565 colour, cRect r);
  void rectOutline (sRgba565 colour, const cRect& r, uint8_t thickness);
  void ellipse (sRgba565 colour, cPoint centre, cPoint radius);
  int text (sRgba565 colour, uint16_t fontHeight, const std::string& str, cRect r);

  void copy (cTile* tile, cPoint p);
  void copy90 (cTile* tile, cPoint p);
  void size (cTile* tile, const cRect& r);

  inline void pixel (sRgba565 colour, cPoint p) { *(mBuffer + p.y * getWidth() + p.x) = colour.rgb565; }
  void grad (sRgba565 colTL, sRgba565 colTR, sRgba565 colBL, sRgba565 colBR, const cRect& r);
  void line (sRgba565 colour, cPoint p1, cPoint p2);
  void ellipseOutline (sRgba565 colour, cPoint centre, cPoint radius);

  // agg anti aliased
  void aMoveTo (const cPointF& p);
  void aLineTo (const cPointF& p);
  void aWideLine (const cPointF& p1, const cPointF& p2, float width);
  void aPointedLine (const cPointF& p1, const cPointF& p2, float width);
  void aEllipseOutline (const cPointF& centre, const cPointF& radius, float width, int steps);
  void aEllipse (const cPointF& centre, const cPointF& radius, int steps);
  void aRender (sRgba565 colour, bool fillNonZero = true);

  void display (int brightness);
  void start();
  void drawInfo();
  void present();

  static cLcd* mLcd;

private:
  void sendData (uint16_t data);
  void sendCommandData (uint16_t reg, uint16_t data);
  cFontChar* loadChar (uint16_t fontHeight, char ch);

  void ready();
  void reset();

  uint8_t calcAlpha (int area, bool fillNonZero) const;
  void renderScanLine (cScanLine* scanLine, sRgba565 colour);

  //{{{  vars
  TIM_HandleTypeDef mTimHandle;
  int mBrightness = 50;

  bool mChanged = true;
  uint16_t* mBuffer = nullptr;

  uint32_t mBaseTime = 0;
  uint32_t mLastPresentTime = 0;
  uint32_t mPresentTime = 0;
  uint32_t mStartTime = 0;
  uint32_t mDrawTime = 0;
  uint32_t mNumPresents = 0;

  // truetype
  std::map<uint16_t, cFontChar*> mFontCharMap;

  FT_Library FTlibrary;
  FT_Face FTface;
  FT_GlyphSlot FTglyphSlot;

  // info
  bool mShowInfo = true;
  std::string mTitle;

  // log
  static const int kMaxLines = 40;
  //{{{
  class cLine {
  public:
    cLine() {}
    ~cLine() {}

    //{{{
    void clear() {
      mTime = 0;
      mColour = kWhite;
      mString = "";
      }
    //}}}

    int mTime = 0;
    sRgba565 mColour = kWhite;
    std::string mString;
    };
  //}}}
  cLine mLines[kMaxLines];
  int mCurLine = 0;
  //}}}
  };
