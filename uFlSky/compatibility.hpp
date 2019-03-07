
#pragma once

#include "config.h"

#ifdef TEENSY_LC

#include <stdint.h>
#include <Arduino.h>

extern "C" void _println(const char * bytes);

#endif
