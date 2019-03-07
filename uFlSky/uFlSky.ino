/*
 Circuit:
 SCP1000 sensor 
 SCS: pin 10
 MOSI: pin 11
 MISO: pin 12
 SCK: pin 13
 */

#include <HardwareSerial.h>
#include <SPI.h>
#include "config.h"

extern "C" {
  #include "sbus.h"
  #include "afhds2a.h"
  #include "rx_a7105.h"
}
//#include "spi_common.h"
//#include "A7105.h"
#include <avr/io.h> 

static uint8_t sbusPacket[25];
static uint8_t rxPacket[38];
extern bool ledState = 0;
bool sendSbusPacket = 1;
bool packetReady = 0;

void setup() {
  bool retval = 0;
  Serial.begin(100000);
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
  retval = flySkyInit();
  if(retval == 0) {
    while(1) {
      digitalWrite(ledPin, LOW);
      delay(700);
      digitalWrite(ledPin, HIGH);
      delay(700);
    }
  }
#ifndef TEENSY_LC
  switchClock(0, 1);
  fixUartBaud();
#endif
  digitalWrite(ledPin, HIGH);
  initDefaults(&stick_values);
//  setupTimer1FourteenMs();
}


void flashLed() {
  digitalWrite(ledPin, 0);
  delay(50);
  digitalWrite(ledPin, 1);
  delay(50);
}

void loop() {
//#ifdef TEENSY_LC
  delayMicroseconds(100);
  A7105IntHandler();
//#endif
  flySkyDataReceived(rxPacket);
}

//void writeSbusPacket() {
//  readChannels(rxPacket, &stick_values);
//  if(STUFF_RSSI) {
//    stuffRssiChannel(&stick_values, rssi);
//  }
//  buildPacket(sbusPacket, &stick_values);
//
//  Serial.write(sbusPacket, 25);
//}

#ifdef TEENSY_LC


#else

void fixUartBaud() {
  LINCR = 0;
  LINBTR = _BV(LDISR) & ~0x1F;
  LINBTR |= 8;
  LINBRR = 9;
  LINCR |= 1 << LENA | 1 << LCMD0 | 1 << LCMD2;
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

//ISR(PCINT0_vect) // handle pin change interrupt for D8 to D13 here
//{    
//  A7105IntHandler();
//  PCIFR = 0x1;
//}

void switchClock(uint8_t clk_number, uint8_t sut) {
  uint8_t previous_clk, temp;
  
  // Disable interrupts
  temp = SREG; asm ("cli");
  
  // Save the current system clock source
  CLKCSR = 1 << CLKCCE;
  CLKCSR = CLOCK_RECOVER;
  previous_clk = CLKSELR & 0x0F;
  
  // Enable the new clock source
  CLKSELR = ((sut << 4 ) & 0x30) | (clk_number & 0x0F);
  CLKCSR = 1 << CLKCCE;
  CLKCSR = CLOCK_ENABLE;

  CLKCSR = 1 << CLKCCE;
  CLKCSR = CLOCK_REQ_AVAIL;
  
  // Wait for clock validity
  while ((CLKCSR & (1 << CLKRDY)) == 0);
  
  // Switch clock source
  CLKCSR = 1 << CLKCCE;
  CLKCSR = CLOCK_SWITCH;
  
  // Wait for effective switching
  while (1){
    CLKCSR = 1 << CLKCCE;
    CLKCSR = CLOCK_RECOVER;
    if ((CLKSELR & 0x0F) == (clk_number & 0x0F)) break;
  }
  
  // Shut down unneeded clock source
  if (previous_clk != (clk_number & 0x0F)) {
    CLKSELR = previous_clk;
    CLKCSR = 1 << CLKCCE;
    CLKCSR = CLOCK_DISABLE;
  }
  // Re-enable interrupts
  SREG = temp;
}

#endif

