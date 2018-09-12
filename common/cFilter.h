// cFilter.h
#pragma once

class cFilter {
public:
  //{{{
  cFilter() : mSize(9) {
    mValues = (uint16_t*)pvPortMalloc (mSize*2);
    mSorted = (uint16_t*)pvPortMalloc (mSize*2);
    }
  //}}}

  //{{{
  ~cFilter() {
    vPortFree (mValues);
    vPortFree (mSorted);
    }
  //}}}
  //{{{
  uint16_t getMedianValue (uint16_t value) {

    addValue (value);
    memcpy (mSorted, mValues, mSize * 2);
    return quickSelect (0, mMaxIndex-1, mMaxIndex/2);
    }
  //}}}
  //{{{
  uint16_t getAverageValue (uint16_t value) {

    addValue (value);

    uint32_t sum = 0;
    for (int i = 0; i < mMaxIndex; i++)
      sum += mValues[i];
    return sum / mMaxIndex;
    }
  //}}}
  //{{{
  uint16_t getAverageMedianValue (uint16_t value) {

    addValue (value);

    memcpy (mSorted, mValues, mSize * 2);
    uint16_t median =  quickSelect (0, mMaxIndex-1, mMaxIndex/2);
    uint16_t medianBefore = mMaxIndex > 5 ? quickSelect (0, mMaxIndex-1, (mMaxIndex/2) -1) : median;
    uint16_t medianAfter = mMaxIndex > 5 ? quickSelect (0, mMaxIndex-1, (mMaxIndex/2) + 1) : median;

    return (median + medianBefore + medianAfter) / 3;
    }
  //}}}

  //{{{
  void clear() {
    mCurIndex = 0;
    mMaxIndex = 0;
    }
  //}}}

private:
  //{{{
  void addValue (uint16_t value) {
    mValues[mCurIndex] = value;
    mCurIndex = (mCurIndex + 1) % mSize;
    if (mMaxIndex < mSize)
      mMaxIndex++;
    }
  //}}}
  //{{{
  uint8_t partition (uint8_t p, uint8_t r) {

    uint16_t pivot = mSorted[r];
    while (p < r) {
      while (mSorted[p] < pivot)
        p++;

      while (mSorted[r] > pivot)
        r--;

      if (mSorted[p] == mSorted[r])
        p++;
      else if (p < r) {
        uint16_t swap = mSorted[p];
        mSorted[p] = mSorted[r];
        mSorted[r] = swap;
        }
      }

    return r;
    }
  //}}}
  //{{{
  uint16_t quickSelect (uint8_t p, uint8_t r, uint8_t k) {

    if (p == r)
      return mSorted[p];

    uint8_t j = partition (p, r);
    uint8_t length = j - p + 1;
    if (length == k)
      return mSorted[j];
    else if (k < length)
      return quickSelect (p, j - 1, k);
    else
      return quickSelect (j + 1, r, k - length);
    }
  //}}}

  const uint16_t mSize;
  uint16_t* mValues;
  uint16_t* mSorted;
  uint8_t mCurIndex = 0;
  uint32_t mMaxIndex = 0;
  };
