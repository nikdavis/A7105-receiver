/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>

#include "config.h"
#include "rx_a7105.h"

static volatile uint32_t timeEvent = 0;
static volatile bool occurEvent = false;

#ifdef TEENSY_LC
  void A7105PauseInt() {
    // TODO
  }
  
  void A7105ResumeInt() {
    // TODO
  }
  
  void A7105Init(uint32_t id)
  {
      pinMode(wtrPin, INPUT);
      A7105PauseInt();
  
      A7105SoftReset();
      A7105WriteID(id);
  }

  void A7105IntHandler()
  {
    // Teensy board does not have GPIO1 / WTR interrupt broken out, so we just need
    // to assume something happens. Can eventually read / check for RX/TX action?
    timeEvent = micros();
    occurEvent = true;
  }
#else // AVR
  void A7105PauseInt() {
    PCICR &= ~0x1;
  }
  
  void A7105ResumeInt() {
    PCICR |= 0x1;
  }
  
  void A7105Init(uint32_t id)
  {
      pinMode(wtrPin, INPUT);
      PCMSK0 = 0x1; // only enable desired pin for interrupt
      PCIFR = 0x1;
      A7105PauseInt();
  
      A7105SoftReset();
      A7105WriteID(id);
  }

  void A7105IntHandler()
  {
    bool val = digitalRead(wtrPin);
    if (val == 0) {
        timeEvent = micros();
        occurEvent = true;
    }
  }
#endif


void A7105Config(const uint8_t *regsTable, uint8_t size)
{
    if (regsTable) {
        unsigned timeout = 1000;

        for (unsigned i = 0; i < size; i++) {
            if (regsTable[i] != 0xFF) {
                A7105WriteReg ((A7105Reg_t)i, regsTable[i]);
            }
        }

        A7105Strobe(A7105_STANDBY);

        A7105WriteReg(A7105_02_CALC, 0x01);

        while ((A7105ReadReg(A7105_02_CALC) != 0) && timeout--) {}

        A7105ReadReg(A7105_22_IF_CALIB_I);

        A7105WriteReg(A7105_24_VCO_CURCAL, 0x13);
        A7105WriteReg(A7105_25_VCO_SBCAL_I, 0x09);
        A7105Strobe(A7105_STANDBY);
    }
}

bool A7105RxTxFinished(uint32_t *timeStamp) {
    bool result = false;

    if (occurEvent) {
        if (timeStamp) {
            *timeStamp = timeEvent;
        }

        occurEvent = false;
        result = true;
    }
    return result;
}

void A7105SoftReset(void)
{
    writeRegister((uint8_t)A7105_00_MODE, 0x00);
}

uint8_t A7105ReadReg(A7105Reg_t reg)
{
    return readRegister((uint8_t)reg);
}

void A7105WriteReg(A7105Reg_t reg, uint8_t data)
{
    writeRegister((uint8_t)reg, data);
}

void A7105Strobe(A7105State_t state)
{
    if (A7105_TX == state || A7105_RX == state) {
        A7105ResumeInt();
    } else {
        A7105PauseInt();
    }

    writeSingleByte((uint8_t)state);
}

void A7105WriteID(uint32_t id)
{
    uint8_t data[4];
    data[0] = (id >> 24) & 0xFF;
    data[1] = (id >> 16) & 0xFF;
    data[2] = (id >> 8) & 0xFF;
    data[3] = (id >> 0) & 0xFF;
    writeRegisterBytes((uint8_t)A7105_06_ID_DATA, &data[0], sizeof(data));
}

uint32_t A7105ReadID(void)
{
    uint32_t id;
    uint8_t data[4];
    readRegisterBytes((uint8_t)A7105_06_ID_DATA, &data[0], sizeof(data));
    id = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3] << 0;
    return id;
}

void A7105ReadFIFO(uint8_t *data, uint8_t num)
{
    if (data) {
        if(num > 64) {
            num = 64;
        }

        A7105Strobe(A7105_RST_RDPTR); /* reset read pointer */
        readRegisterBytes((uint8_t)A7105_05_FIFO_DATA, data, num);
    }
}

void A7105WriteFIFO(uint8_t *data, uint8_t num)
{
    if (data) {
        if(num > 64) {
            num = 64;
        }

        A7105Strobe(A7105_RST_WRPTR); /* reset write pointer */
        writeRegisterBytes((uint8_t)A7105_05_FIFO_DATA, data, num);
    }
}
