#pragma once

#ifndef _LIBFULLSCREEN_PRIVATE_HPP_
#define _LIBFULLSCREEN_PRIVATE_HPP_

#include "pros/rtos.hpp"
#include "vex/v5.h"

#define STATIC_ASSET(x)                                                     \
  extern "C" {                                                              \
  extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[]; \
  static struct {                                                           \
    uint8_t* buf;                                                           \
    size_t size;                                                            \
  } x = {_binary_static_##x##_start, (size_t)_binary_static_##x##_size};    \
  }

#define DECODE_PNG(x, width, height)             \
  x##_image.data = new uint32_t[width * height]; \
  // [REDACTED]

#define DECODE_BMP(x, width, height)             \
  x##_image.data = new uint32_t[width * height]; \
  // [REDACTED]

// [REDACTED]
// [REDACTED]                
// [REDACTED]
// [REDACTED]          
// [REDACTED]


#endif  
