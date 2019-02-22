/*
 Circuit:
 SCP1000 sensor 
 SCS: pin 10
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
 */

extern "C" {
  #include "sbus.h"
}
#include "spi_common.h"
#include "A7105.h"

static uint8_t rxPacket[38];
static uint8_t sbusPacket[25];

void setup() {
  Serial.begin(9600);

  // start the SPI library:
  SPI.begin();
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  int retval = afhds2a_init();
  Serial.print("Initialization return: ");
  Serial.println(retval);
}

void loop() {
  wait_and_read(rxPacket);
  readChannels(rxPacket, &stick_values);
  buildPacket(sbusPacket, &stick_values);
  Serial.println(stick_values.throttle);
  Serial.println("--------");
  uint8_t * bytes = sbusPacket;
  for(int i = 0; i < 25; i++) {
    Serial.println(bytes[i], BIN);
  }
  Serial.println("--------");
}
