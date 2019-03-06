#include "spi_common.hpp"
#include <SPI.h>

void writeSingleByte(uint8_t value) {
  SPI.beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(value);

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
}

extern "C" void writeRegister(uint8_t address, uint8_t value) {
  SPI.beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(address);
  SPI.transfer(value);

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
}

void writeRegisterBytes(uint8_t address, uint8_t* bytes, int numBytes) {
  SPI.beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(address);

  for (int i = 0; i < numBytes; i++) {
    SPI.transfer(bytes[i]);
  }

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t address) {
  SPI.beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(address | 0x40); //Send register location
  uint8_t result = SPI.transfer(0);

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  return result;
}

void readRegisterBytes(uint8_t address, uint8_t* bytes, int numBytes) {
  SPI.beginTransaction(SPISettings(spiClockSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(address | 0x40);

  for (int i = 0; i < numBytes; i++) {
    bytes[i] = SPI.transfer(0);
  }

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
}

