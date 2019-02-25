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

IntervalTimer sbusTimer;
IntervalTimer terminalWriter;
elapsedMillis lastRxPacketReceived;

void setup() {
  Serial.begin(500000);
  Serial1.begin(100000, SERIAL_8E2_TXINV);
  
  // start the SPI library:
  SPI.begin();
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  int retval = afhds2a_init();
  Serial.print("Initialization return: ");
  Serial.println(retval);
  lastRxPacketReceived = 0;
  initDefaults(&stick_values);
  sbusTimer.begin(writeSbusPacket, 7000);
//  terminalWriter.begin(writeValuesToTerminal, 50000);
}

void loop() {
  readFromRx(rxPacket);
  noInterrupts();
  lastRxPacketReceived = 0;
  interrupts();
  readChannels(rxPacket, &stick_values);
  noInterrupts();
  buildPacket(sbusPacket, &stick_values);
  interrupts();
}

//void writeValuesToTerminal() {
//  if(lastRxPacketReceived > 1000) {
//    lastRxPacketReceived = 0;
//    Serial.printf("throttle: %d\n", stick_values.throttle);
//    Serial.printf("roll: %d\n",  stick_values.roll);
//    Serial.printf("pitch: %d\n",  stick_values.pitch);
//    Serial.printf("yaw: %d\n",  stick_values.yaw);
//  }
//}

void writeSbusPacket() {
  // Simple failsafe, turn off throttle
  if(lastRxPacketReceived > 1000) {
    initDefaults(&stick_values);
  }
  Serial1.write(sbusPacket, 25);
}

