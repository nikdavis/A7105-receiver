#include <stdint.h>
#include "config.h"

extern "C" void writeSingleByte(uint8_t value);

extern "C" void writeRegister(uint8_t address, uint8_t value);

extern "C" void writeRegisterBytes(uint8_t address, uint8_t* bytes, int numBytes);

extern "C" uint8_t readRegister(uint8_t address);

extern "C" void readRegisterBytes(uint8_t address, uint8_t* bytes, int numBytes);

extern "C" void readPayloadBytes(uint8_t* bytes, int numBytes);

extern "C" void writePayloadBytes(uint8_t* bytes, int numBytes);
