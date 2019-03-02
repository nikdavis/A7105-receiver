/*
 Circuit:
 SCP1000 sensor 
 SCS: pin 10
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
 */

#include <HardwareSerial.h>

extern "C" {
  #include "sbus.h"
}
#include "spi_common.h"
#include "A7105.h"

static uint8_t rxPacket[38];
static uint8_t sbusPacket[25];
bool ledState = 0;
bool sendSbusPacket = 1;
bool packetReady = 0;

void setup() {
  Serial.begin(100000);
  // start the SPI library:
  SPI.begin();
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(50);
  digitalWrite(ledPin, HIGH);
  delay(50);
  digitalWrite(ledPin, LOW);
  delay(200);
  int retval = afhds2a_init();
  if(retval == 0) {
    while(1) {
      digitalWrite(ledPin, LOW);
      delay(700);
      digitalWrite(ledPin, HIGH);
      delay(700);
    }
  }
  digitalWrite(ledPin, retval);
  initDefaults(&stick_values);
  setupTimer1FourteenMs();
  setupWtrInterrupt();
  setupRx();
}

void loop() {
  if(packetReady) {
    readFromRx(rxPacket);
    packetReady = 0;
  }
  
  if(sendSbusPacket) {
    writeSbusPacket();
    sendSbusPacket = 0;
  }
}

void writeSbusPacket() {
  readChannels(rxPacket, &stick_values);
  buildPacket(sbusPacket, &stick_values);
  Serial.write(sbusPacket, 25);
}



void setupTimer1FourteenMs() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 557; // 14ms at 8MHz / 256 clock speed + emperical fudge factor
  TCCR1B |= 0b01 << 3; // set CTC for OCR1A
  TCCR1B |= 0b100; // set to 256x prescale
  TCNT1 = 0x0;
  TIMSK1 |= 1 << 1; //OCR1A interrupt enable
  interrupts();
}

ISR(TIMER1_COMPA_vect){
 sendSbusPacket = 1;
}

// Falling edge when RX or TX are complete
void setupWtrInterrupt() {
    noInterrupts();
    pinMode(wtrPin, INPUT);
    PCMSK0 = 0x1; // only enable desired pin for interrupt
    PCICR = 0x1; // enable PCIINT0 pins interrupt
    PCIFR = 0x1;
    interrupts();
}

ISR(PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{    
  bool val = digitalRead(wtrPin);
  digitalWrite(ledPin, ledState);
  ledState = !ledState;
  if(!val) {
    packetReady = 1;
  }
  PCIFR = 0x1;
}


