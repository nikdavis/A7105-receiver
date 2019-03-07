#include <Arduino.h>

//#define TEENSY_LC

#ifdef TEENSY_LC
  #define chipSelectPin 10
  #define ledPin -1
  #define wtrPin -1
#else
  #define chipSelectPin PIN_A6
  #define ledPin PIN_B2
  #define wtrPin PIN_A0
#endif

#define CLOCK_RECOVER 0x05
#define CLOCK_ENABLE 0x02
#define CLOCK_SWITCH 0x04
#define CLOCK_REQ_AVAIL 0x03
#define CLOCK_DISABLE 0x01

#define spiClockSpeed 4000000
#define waitForRxPacketMicros 2000
#define failsafeTimeoutMillis 2000
#define sbusPacketPeriodMs 20

#define setbit(value, i) ((value) |= ((1) << (i)))
#define clearbit(value, i) ((value) &= ~((1) << (i)))
#define togglebit(byte, bit) ((byte) ^= ((1) << (bit)))
#define testbit(byte, bit) (((byte) >> (bit)) & (1))
