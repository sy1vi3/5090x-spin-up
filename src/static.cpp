#include "private.hpp"
#include "vex/v5_api.h"
#include "static.h"

STATIC_ASSET(uwu_png)
STATIC_ASSET(black_png)

static v5_image uwu_image;
static v5_image black_image;

void initImages(){
    DECODE_PNG(uwu,360,120);
    DECODE_PNG(black,480,272);
    // [REDACTED]
    // [REDACTED]
    pros::delay(100);
    // [REDACTED]
    // [REDACTED]
}