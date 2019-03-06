#include <stdint.h>
#include "config.h"

void writeSingleByte(uint8_t value);

void writeRegister(uint8_t address, uint8_t value);

void writeRegisterBytes(uint8_t address, uint8_t* bytes, int numBytes);

uint8_t readRegister(uint8_t address);

void readRegisterBytes(uint8_t address, uint8_t* bytes, int numBytes);

void readPayloadBytes(uint8_t* bytes, int numBytes);

void writePayloadBytes(uint8_t* bytes, int numBytes);
