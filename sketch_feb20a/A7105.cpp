#include "A7105.h"

uint8_t packet[38];
static int channel = 0;

static const uint8_t AFHDS2A_regs[] = {
    -1  , 0x42 | (1<<5), 0x00, 0x25, 0x00,   -1,   -1, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3c, 0x05, 0x00, 0x50, // 00 - 0f
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f, 0x62, 0x80,   -1,   -1, 0x2a, 0x32, 0xc3, 0x1f, // 10 - 1f
    0x1e,   -1, 0x00,   -1, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00, // 20 - 2f
    0x01, 0x0f // 30 - 31
};

static const uint8_t my_channels[] = {0x74, 0x6B, 0x2A, 0x1D, 0x24, 0x91, 0x50, 0x2F, 0x5A, 0x61, 0x13, 0x3B, 0x8B, 0x49, 0x82, 0x55};

bool packet_valid() {
  uint8_t value = readRegister(0x00);
  // check CRC and FEC for validity, 1 is error
  return !((value & (1 << 6)) || (value & (1 << 5)));
}

void readFromRx(uint8_t * packetBuffer) {
    uint8_t rawPacket[38];
    
    if (channel > 15) { // this code runs once every ~120 msec
        channel = 0;
    }
    
    noInterrupts();
    writeSingleByte(A7105_STANDBY);
    writeRegister(A7105_0F_PLL_I, my_channels[channel]);
    writeSingleByte(A7105_RX);
    interrupts();
    delayMicroseconds(140);
    
    while (1) { // look for data on the current channel
        noInterrupts()
        uint8_t dataNotReady = readRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK;
        interrupts();
        
        if (!dataNotReady) {
          noInterrupts()
          writeSingleByte(A7105_RST_RDPTR);
          readRegisterBytes(0x05, rawPacket, 38);
          channel++;
          interrupts();
          if (packet_valid() && rawPacket[0] == 0x58) {
            memcpy(packetBuffer, rawPacket, 38); 
          }
          break;
       }
    }

//    // If it's likely we missed a packet, then start walking backwards
//    // on the hopping frequencies to converge on the next hopping loop
//    if(elapsedTimeInMicros >= 2000) {
//      channel--;
//      if(channel < 0) {
//        channel = 15;
//      }
//    }
    noInterrupts()
    writeSingleByte(A7105_STANDBY);
    interrupts()
}

//returns success or failure
uint8_t rebind()
{

    Serial.println("Starting binding");
    int msElapsed = 0;

    // binding process
    uint8_t channel = 0;
    const uint8_t allowed_ch[] = {0x8B, 0x0C};
//    const uint8_t allowed_ch[] = {0x0d, 0x8c}; // from deviationtx fw
    uint8_t found_channel = 0;
    
    while (!found_channel) { // scan through the PLL channels, ~10 msec at a time

        if (channel > 1) { // this code runs once every ~120 msec
            channel = 0;
        }

        Serial.print("Starting search on ");
        Serial.println(allowed_ch[channel], HEX);
        writeSingleByte(A7105_STANDBY);
        writeRegister(A7105_0F_PLL_I, allowed_ch[channel]);
        writeSingleByte(A7105_RX);

        while (1) { // look for data on the current channel
//            Serial.println("inner loop");
//            Serial.println(msElapsed);
            if (msElapsed > 10) { // give up on this channel
                channel++;
                msElapsed = 0;
                break;
            }
            if (readRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK) {
              msElapsed++;
              delayMicroseconds(10000);
            }
            else {
              Serial.println("Received data");
                writeSingleByte(A7105_RST_RDPTR);
                readRegisterBytes(0x05, packet, 38);
                found_channel = 1;
                break;
            }
        }
    }

    // we now know our PLL channel
    channel = packet[1];
    writeSingleByte(A7105_STANDBY);

    for(int i = 0; i < 38; i++) {
      Serial.println(packet[i], HEX);
    }

    return 1;
}

void A7105_reset(void)
{
    writeRegister(A7105_reg_mode, 0x00);
}

void A7105_SetPower(int power)
{
    /*
    Power amp is ~+16dBm so:
    TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
    TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
    TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
    TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
    TXPOWER_10mW   = -6dBm  == PAC=1 TBG=5
    TXPOWER_30mW   = 0dBm   == PAC=2 TBG=7
    TXPOWER_100mW  = 1dBm   == PAC=3 TBG=7
    TXPOWER_150mW  = 1dBm   == PAC=3 TBG=7
    */
    uint8_t pac, tbg;
    switch(power) {
        case 0: pac = 0; tbg = 0; break;
        case 1: pac = 0; tbg = 1; break;
        case 2: pac = 0; tbg = 2; break;
        case 3: pac = 0; tbg = 4; break;
        case 4: pac = 1; tbg = 5; break;
        case 5: pac = 2; tbg = 7; break;
        case 6: pac = 3; tbg = 7; break;
        case 7: pac = 3; tbg = 7; break;
        default: pac = 0; tbg = 0; break;
    }
    writeRegister(0x28, (pac << 3) | tbg);
}

int afhds2a_init()
{
    int i;
    int ms = 0;
    uint8_t if_calibration1;
    uint8_t vco_calibration0;
    uint8_t vco_calibration1;

    A7105_ID_write(0x5475c52a);
    for (i = 0; i < 0x33; i++)
        if((int8_t)AFHDS2A_regs[i] != -1)
            writeRegister(i, AFHDS2A_regs[i]);

    writeSingleByte(A7105_STANDBY);

    //IF Filter Bank Calibration
    writeRegister(0x02, 1);
    readRegister(0x02);
    while(ms < 500) {
        if(! readRegister(0x02))
            break;
        
        ms++;
        delayMicroseconds(1000);
    }
    if (ms >= 500)
        return 0;
    if_calibration1 = readRegister(0x22);
    if(if_calibration1 & A7105_MASK_FBCF) {
        //Calibration failed...what do we do?
        return 0;
    }

    //VCO Current Calibration
    writeRegister(0x24, 0x13); //Recomended calibration from A7105 Datasheet

    //VCO Bank Calibration
    writeRegister(0x26, 0x3b); //Recomended limits from A7105 Datasheet

    //VCO Bank Calibrate channel 0?
    //Set Channel
    writeRegister(0x0f, 0); //Should we choose a different channel?
    //VCO Calibration
    writeRegister(0x02, 2);
    ms = 0;
    while(ms < 500) {
        if(! readRegister(0x02))
            break;
        
        ms++;
        delayMicroseconds(1000);
    }
    if (ms >= 500)
        return 0;
    vco_calibration0 = readRegister(0x25);
    if (vco_calibration0 & A7105_MASK_VBCF) {
        //Calibration failed...what do we do?
        return 0;
    }

    //Calibrate channel 0xa0?
    //Set Channel
    writeRegister(0x0f, 0xa0);
    //VCO Calibration
    writeRegister(0x02, 2);
    ms = 0;
    while(ms < 500) {
        if(! readRegister(A7105_02_CALC))
            break;

        ms++;
        delayMicroseconds(1000);
    }
    if (ms >= 500)
        return 0;
    vco_calibration1 = readRegister(0x25);
    if (vco_calibration1 & A7105_MASK_VBCF) {
        //Calibration failed...what do we do?
        return 0;
    }

    //Reset VCO Band calibration
    writeRegister(0x25, 0x0A);
    writeSingleByte(A7105_STANDBY);
    return 1;
}

uint32_t A7105_ID_read(void)
{
    uint32_t ID;
    uint8_t idbytes[4];

    // We could read this straight into ID, but the result would depend on
    // the endiness of the architecture.
    readRegisterBytes(A7105_reg_ID, idbytes, 4);

    ID  = (uint32_t) idbytes[0] << 24;
    ID |= (uint32_t) idbytes[1] << 16;
    ID |= (uint32_t) idbytes[2] << 8;
    ID |= (uint32_t) idbytes[3] << 0;

    return (ID);
}

void A7105_ID_write(uint32_t ID)
{
    uint8_t idbytes[4];

    idbytes[0] = ID >> 24;
    idbytes[1] = ID >> 16;
    idbytes[2] = ID >> 8;
    idbytes[3] = ID >> 0;

    writeRegisterBytes(A7105_reg_ID, idbytes, 4);
}

void A7105_set_channel(uint8_t chnl)
{
    writeRegister(A7105_reg_channel, chnl);
}

void A7105_set_mode(enum A7105_mode mode)
{
    uint8_t rxreg;

    // Register 0x18 always reads 0x00, so we have to write the channel
    // width again.

    // RXSM0 and RXSM1 shall always be set to 1
    // DMG shall always be set to 0
    rxreg = 0x3 << 5;

    if (CHNL_WIDTH)
    {
        setbit(rxreg, 1);
    }

    if (mode == master)
    {
        clearbit(rxreg, 0);
    }
    else
    {
        setbit(rxreg, 0);
    }

    writeRegister(A7105_reg_RX, rxreg);
}

uint8_t A7105_receive_byte(void)
{
    uint8_t data;

    writeSingleByte(A7105_strobe_PLL);
    writeSingleByte(A7105_strobe_RX_reset);
    writeSingleByte(A7105_strobe_RX);

    delay(100);

    data = readRegister(A7105_reg_FIFO_data);
    writeSingleByte(A7105_strobe_standby);

    return (data);
}

void A7105_send_byte(uint8_t data)
{
    writeSingleByte(A7105_strobe_PLL);
    writeSingleByte(A7105_strobe_TX_reset);
    writeRegister(A7105_reg_FIFO_data, data);

    writeSingleByte(A7105_strobe_TX);
    delay(100);
    writeSingleByte(A7105_strobe_standby);
}
