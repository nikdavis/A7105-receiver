#include "compatibility.hpp"

#ifdef TEENSY_LC

void _println(uint8_t * bytes) {
  Serial.println((String)bytes);
}

#endif
