//#include "A7105.h"
//
//static int channel = 0;
//uint8_t rssi = 0;
//bool bound = 0;
//uint32_t rxId = 0x12345678;
//uint32_t txId;
//static uint8_t rxPacket[38];
//bool led = 0;
//uint8_t bindCount = 0;
//
//#define ledPin PIN_B2
//
////static const uint8_t AFHDS2A_regs[] = {
////      -1  , 0x42, 0x00, 0x25, 0x00,   -1,   -1, 0x00, 0x00, 0x00, 0xAA, 0x01, 0x00, 0x05, 0x00, 0x50, // 00 - 0f
////      0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f, 0x62, 0x80,   -1,   -1, 0x2a, 0x32, 0xc3, 0x1f, // 10 - 1f
////      0x1e,   -1, 0x00,   -1, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00, // 20 - 2f
////      0x01, 0x0f // 30 - 31
////};
//
//static const uint8_t AFHDS2A_regs[] = {
//      -1  , 0x42, 0x00, 0x25, 0x00,   -1,   -1, 0x00, 0x00, 0x00, 0xAA, 0x01, 0x00, 0x05, 0x00, 0x50, // 00 - 0f
//      0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80,   -1,   -1, 0x0a, 0x32, 0xc3, 0x1f, // 10 - 1f
//      0x1e,   -1, 0x00,   -1, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00, // 20 - 2f
//      0x01, 0x0f // 30 - 31
//};
//
////      -1, 0x42, 0x00, 0x25, 0x00,   -1,   -1, 0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50
////    0x9E, 0x4B, 0x00, 0x02, 0x16, 0x2B, 0x12, 0x00, 0x62, 0x00, 0x80, 0x00, 0x0A, 0x32, 0xC3, 0x1F
////    0x1E, 0xC3, 0x00,   -1, 0x00, 0x00, 0x3B, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00
////    0x01, 0x0F
//
//static const uint8_t my_channels[] = {0x74, 0x6B, 0x2A, 0x1D, 0x24, 0x91, 0x50, 0x2F, 0x5A, 0x61, 0x13, 0x3B, 0x8B, 0x49, 0x82, 0x55};
//
//static const uint8_t bindChannels[] = { 0x0C, 0x8B };
//
//bool packet_valid() {
//  uint8_t value = readRegister(0x00);
//  // check CRC and FEC for validity, 1 is error
//  return !((value & (1 << 6)) || (value & (1 << 5)));
//}
//
//void setupRx() {
//    uint8_t nextChannel;
//    noInterrupts();
//    writeSingleByte(A7105_STANDBY);
//    if(bound) {
//      if (channel > 15) {
//          channel = channel % 16;
//      }
//      nextChannel = my_channels[channel];
//    } else {
//      nextChannel = bindCount % 2 ? bindChannels[0] : bindChannels[1];
//    }
//    writeRegister(A7105_0F_PLL_I, nextChannel);
//    writeSingleByte(A7105_RX);
//    interrupts();
//}
//
//void readFromRx(uint8_t * packetBuffer) {
//    uint8_t rawPacket[38];
////
////    for(int i = 1; i < 5; i++) { // look for data on the current channel
//        noInterrupts();
//        uint8_t dataNotReady = readRegister(A7105_00_MODE) & A7105_MODE_TRER;
//        interrupts();
//        
//        if (!dataNotReady) {
//          noInterrupts();
//          writeSingleByte(A7105_RST_RDPTR);
//          readRegisterBytes(0x05, rawPacket, 38);
//          channel += 3;
//          interrupts();
//          if (packet_valid() && rawPacket[0] == 0x58) {
//            memcpy(packetBuffer, rawPacket, 38); 
//          }
//          rssi = readRegister(0x1D);
////          break;
//       }
////    }
//
//    if (channel > 15) { // this code runs once every ~120 msec
//        channel = channel % 16;
//    }
//    writeSingleByte(A7105_STANDBY);
//    rssi = readRegister(0x1D);
//    setupRx();
//}
//
//void flySky2AReadAndProcess()
//{
//
//    uint8_t packet[FLYSKY_2A_PAYLOAD_SIZE];
//
//    uint8_t bytesToRead = (bound) ? (9 + 2*FLYSKY_2A_CHANNEL_COUNT) : (11 + FLYSKY_FREQUENCY_COUNT);
//    writeSingleByte(A7105_RST_RDPTR);
//    readRegisterBytes(0x05, packet, bytesToRead);
//    
//    switch (packet[0]) {
//    case FLYSKY_2A_PACKET_RC_DATA:
//    case FLYSKY_2A_PACKET_FS_SETTINGS: // failsafe settings
//    case FLYSKY_2A_PACKET_SETTINGS: // receiver settings
////        if (isValidPacket(rxPacket)) {
////            checkRSSI();
////
////            const flySky2ARcDataPkt_t *rcPacket = (const flySky2ARcDataPkt_t*) rxPacket;
////
////            if (rcPacket->type == FLYSKY_2A_PACKET_RC_DATA) {
////                if (payload) {
////                    memcpy(payload, rcPacket->data, 2*FLYSKY_2A_CHANNEL_COUNT);
////                }
////            }
////        }
//        break;
//
//    case FLYSKY_2A_PACKET_BIND1:
//    case FLYSKY_2A_PACKET_BIND2:
//        if (!bound) {
//            digitalWrite(PIN_B2, HIGH);
////            packet[0] = 0xBC;
////            packet[9] = 0x01;
//
//            flySky2ABindPkt_t *bindPacket = (flySky2ABindPkt_t*) packet;
//
//            if (bindPacket->rfChannelMap[0] != 0xFF) {
//                memcpy(my_channels, bindPacket->rfChannelMap, FLYSKY_FREQUENCY_COUNT); // get TX channels
//            }
//
//            txId = bindPacket->txId;
//            bindPacket->rxId = rxId;
//            memset(bindPacket->rfChannelMap, 0xFF, 26); // erase channelMap and 10 bytes after it
//
//            writeSingleByte(A7105_STANDBY); 
//            writeSingleByte(A7105_RST_WRPTR);
//            writeRegisterBytes(0x05, packet, FLYSKY_2A_PAYLOAD_SIZE);
//            delayMicroseconds(2000);
//            writeSingleByte(A7105_TX);
//            uint8_t modeReg = readRegister(A7105_00_MODE);
//            while (!(((modeReg & A7105_MODE_TRSR) != 0) && ((modeReg & A7105_MODE_TRER) == 0))){
//              modeReg = readRegister(A7105_00_MODE);
//              delayMicroseconds(25);
//            }
//
//            delayMicroseconds(200);
//            digitalWrite(PIN_B2, LOW);
//        }
//        break;
//
//    default:
//        break;
//    }
//    
//    setupRx();
//}
//
//void A7105_SetPower(int power)
//{
//    /*
//    Power amp is ~+16dBm so:
//    TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
//    TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
//    TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
//    TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
//    TXPOWER_10mW   = -6dBm  == PAC=1 TBG=5
//    TXPOWER_30mW   = 0dBm   == PAC=2 TBG=7
//    TXPOWER_100mW  = 1dBm   == PAC=3 TBG=7
//    TXPOWER_150mW  = 1dBm   == PAC=3 TBG=7
//    */
//    uint8_t pac, tbg;
//    switch(power) {
//        case 0: pac = 0; tbg = 0; break;
//        case 1: pac = 0; tbg = 1; break;
//        case 2: pac = 0; tbg = 2; break;
//        case 3: pac = 0; tbg = 4; break;
//        case 4: pac = 1; tbg = 5; break;
//        case 5: pac = 2; tbg = 7; break;
//        case 6: pac = 3; tbg = 7; break;
//        case 7: pac = 3; tbg = 7; break;
//        default: pac = 0; tbg = 0; break;
//    }
//    writeRegister(0x28, (pac << 3) | tbg);
//}
//
//int afhds2a_init()
//{
//    int i;
//    int ms = 0;
//    uint8_t if_calibration1;
//    uint8_t vco_calibration0;
//    uint8_t vco_calibration1;
//    uint16_t timeout = 1000;
//
//    // reset
//    writeRegister(0x00, 0x00);
//    delay(100);
//    A7105_ID_write(0x5475c52a);
//    for (i = 0; i < sizeof(AFHDS2A_regs); i++)
//        if((int8_t)AFHDS2A_regs[i] != -1)
//            writeRegister(i, AFHDS2A_regs[i]);
//
//    writeSingleByte(A7105_STANDBY);
//
//    writeRegister(A7105_02_CALC, 0x01);
//
//    while ((readRegister(A7105_02_CALC) != 0) && timeout--) {}
//
//    readRegister(A7105_22_IF_CALIB_I);
//
//    writeRegister(A7105_24_VCO_CURCAL, 0x13);
//    writeRegister(A7105_25_VCO_SBCAL_I, 0x09);
//
//
//    A7105_SetPower(7);
//    writeSingleByte(A7105_STANDBY);
//    return 1;
//}
//
//uint32_t A7105_ID_read(void)
//{
//    uint32_t ID;
//    uint8_t idbytes[4];
//
//    // We could read this straight into ID, but the result would depend on
//    // the endiness of the architecture.
//    readRegisterBytes(A7105_reg_ID, idbytes, 4);
//
//    ID  = (uint32_t) idbytes[0] << 24;
//    ID |= (uint32_t) idbytes[1] << 16;
//    ID |= (uint32_t) idbytes[2] << 8;
//    ID |= (uint32_t) idbytes[3] << 0;
//
//    return (ID);
//}
//
//void A7105_ID_write(uint32_t ID)
//{
//    uint8_t idbytes[4];
//
//    idbytes[0] = ID >> 24;
//    idbytes[1] = ID >> 16;
//    idbytes[2] = ID >> 8;
//    idbytes[3] = ID >> 0;
//
//    writeRegisterBytes(A7105_reg_ID, idbytes, 4);
//}
//
//void A7105_set_channel(uint8_t chnl)
//{
//    writeRegister(A7105_reg_channel, chnl);
//}
//
//void A7105_set_mode(enum A7105_mode mode)
//{
//    uint8_t rxreg;
//
//    // Register 0x18 always reads 0x00, so we have to write the channel
//    // width again.
//
//    // RXSM0 and RXSM1 shall always be set to 1
//    // DMG shall always be set to 0
//    rxreg = 0x3 << 5;
//
//    if (CHNL_WIDTH)
//    {
//        setbit(rxreg, 1);
//    }
//
//    if (mode == master)
//    {
//        clearbit(rxreg, 0);
//    }
//    else
//    {
//        setbit(rxreg, 0);
//    }
//
//    writeRegister(A7105_reg_RX, rxreg);
//}
//
//uint8_t A7105_receive_byte(void)
//{
//    uint8_t data;
//
//    writeSingleByte(A7105_strobe_PLL);
//    writeSingleByte(A7105_strobe_RX_reset);
//    writeSingleByte(A7105_strobe_RX);
//
//    delay(100);
//
//    data = readRegister(A7105_reg_FIFO_data);
//    writeSingleByte(A7105_strobe_standby);
//
//    return (data);
//}
//
//void A7105_send_byte(uint8_t data)
//{
//    writeSingleByte(A7105_strobe_PLL);
//    writeSingleByte(A7105_strobe_TX_reset);
//    writeRegister(A7105_reg_FIFO_data, data);
//
//    writeSingleByte(A7105_strobe_TX);
//    delay(100);
//    writeSingleByte(A7105_strobe_standby);
//}
