#pragma once

#include <stdint.h>

struct CubePose {
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t angle = 0;
  bool on_mat = false;
};

