// utils.h
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string>

std::string dec (int num, int width = 0, char fill = '0');
std::string hex (uint32_t num, int width = 0, char fill = '0');
