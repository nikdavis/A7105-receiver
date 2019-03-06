//#include <pins_arduino.h>
#include <Arduino.h>

#define chipSelectPin PIN_A6
#define ledPin PIN_B2
#define wtrPin PIN_A0

#define CLOCK_RECOVER 0x05
#define CLOCK_ENABLE 0x02
#define CLOCK_SWITCH 0x04
#define CLOCK_REQ_AVAIL 0x03
#define CLOCK_DISABLE 0x01

#define spiClockSpeed 1000000
#define waitForRxPacketMicros 2000
#define failsafeTimeoutMillis 2000

#define setbit(value, i) ((value) |= ((1) << (i)))
#define clearbit(value, i) ((value) &= ~((1) << (i)))
#define togglebit(byte, bit) ((byte) ^= ((1) << (bit)))
#define testbit(byte, bit) (((byte) >> (bit)) & (1))
