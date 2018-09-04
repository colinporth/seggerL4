// utils.h
#include "utils.h"

std::string dec (int num, int width, char fill) {

  std::string str;
  bool neg = num < 0;
  if (neg)
    num = -num;

  while ((width > 0) || num) {
    str = char((num % 10) + 0x30) + str;
    num /= 10;
    width--;
    }

  return (neg ? "-" : "") + str;
  }

std::string hex (uint32_t num, int width, char fill) {

  const char kDigitToChar[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  std::string str;

  while ((width > 0) || num) {
    uint8_t digit = num % 16;
    str = kDigitToChar[digit] + str;
    num /= 16;
    width--;
    }

  return str;
  }
