// cTraceVec.h
#pragma once
#include <vector>
#include "cLcd.h"

class cTraceVec {
public:
  //{{{
  void addTrace (int16_t numSamples, int16_t averageSamples, int16_t chans) {
    for (int i = 0; i < chans; i++)
      mTraces.push_back (new cTrace (numSamples, averageSamples));
    }
  //}}}
  //{{{
  void addSample (int16_t trace, int16_t value) {
    if (trace < mTraces.size())
      mTraces[trace]->addSample (value);
    }
  //}}}
  //{{{
  void trigger() {
    for (auto trace : mTraces)
      trace->trigger();
    }
  //}}}

  //{{{
  void draw (cLcd* lcd, int16_t top, int16_t bottom) {

    int16_t height = (bottom - top - mTraces.size()) / mTraces.size();
    for (auto trace : mTraces) {
      trace->draw (lcd, top, top + height);
      top += height + 1;
      }
    }
  //}}}

private:
  class cTrace {
  public:
    //{{{
    cTrace (int16_t numSamples, int16_t averageSamples)
       : mNumSamples(numSamples), mAverageSamples(averageSamples) {

      mSamples = (int16_t*)malloc (numSamples*2);
      memset (mSamples, 0, numSamples*2);
      }
    //}}}

    //{{{
    void addSample (int16_t value) {

      if (value < mMin)
        mMin = value;
      if (value > mMax)
        mMax = value;

      mSamples[mCurSample++ % mNumSamples] = value;
      }
    //}}}
    //{{{
    void trigger() {

      mCurSample = 0;
      }
    //}}}

    //{{{
    void draw (cLcd* lcd, int16_t top, int16_t bottom) {

      int32_t range = mMax > -mMin ? mMax : -mMin;
      int16_t height = bottom - top;
      auto midy = top + height/2;

      auto grid = range > 10000 ? 5000 : 1000;
      for (int i = (mMax/1000) * grid; i > mMin; i -= grid)  {
        auto gridy = (i * height) / (range * 2);
        lcd->rectClipped (kGrey, cRect(0, midy - gridy, lcd->getWidth(), midy - gridy + 1));
        }

      int32_t sample = mCurSample - (lcd->getWidth() * mAverageSamples);
      for (int i = 0; i < lcd->getWidth(); i++) {
        int32_t value = 0;
        for (int j = 0; j < mAverageSamples; j++) {
          value += sample > 0 ? mSamples[sample % mNumSamples] : 0;
          sample++;
          }
        value = (value * height) / (range * 2 * mAverageSamples);
        if (value > 0)
          lcd->rectClipped (kWhite, cRect (i, midy - value, i+1, midy));
        else
          lcd->rectClipped (kWhite, cRect (i, midy, i+1, midy - value));
        }

      lcd->text (kGreen, cLcd::getFontHeight(), dec (mMax),
                 cRect (lcd->getWidth() - 60, midy - cLcd::getFontHeight()*3/2,
                        lcd->getWidth(), midy - cLcd::getFontHeight()*3/2 + cLcd::getFontHeight()));
      lcd->text (kYellow, cLcd::getFontHeight(), dec (mSamples[(mCurSample-1) % mNumSamples]),
                 cRect (lcd->getWidth() - 60, midy - cLcd::getFontHeight()/2,
                        lcd->getWidth(), midy - cLcd::getFontHeight()/2 + cLcd::getFontHeight()));
      lcd->text (kWhite, cLcd::getFontHeight(), dec (mMin),
                 cRect (lcd->getWidth() - 60, midy + cLcd::getFontHeight()/2,
                        lcd->getWidth(), midy + cLcd::getFontHeight()/2 + cLcd::getFontHeight()));
      }
    //}}}

  private:
    int16_t mNumSamples = 0;
    int16_t mAverageSamples = 1;
    int16_t* mSamples = nullptr;
    int32_t mCurSample = 0;
    int16_t mMin = 0;
    int16_t mMax = 0;
    };

  std::vector<cTrace*> mTraces;
  };
