#include "sbus.h"

stick_values_t stick_values;

// Read out 14 channels
void readChannels(uint8_t * rxPacket, stick_values_t * values) {
  int channelOffset = 9;
  for (int i = 0; i < NUMBER_OF_TX_CHANNELS; i++) {
    int index = i * 2 + channelOffset;
    uint16_t rawChannel = rxPacket[index + 1] << 8;
    rawChannel = rawChannel | rxPacket[index];
//    rawChannel = 2048;
//    ((uint16_t*)values)[i] = translateFlyskyChannelValueToSbus(rawChannel);
    ((uint16_t*)values)[i] = 192;
  }
}

// 25 byte packet
void buildPacket(uint8_t * outPacket, stick_values_t * values) {
  memset(outPacket, 0x00, 25);
  outPacket[0] = 0x0F;
  outPacket[24] = 0x00;

  int channelCounter = 0;
  int packetOffset = 1;
  int dataBitCounter = 0;
  uint16_t dataValue = ((uint16_t*)values)[channelCounter];
  // Channels represent middle 22 bytes -- 16 channels * 11 bits. They
  // are ordered by lowest channel and LSB first in LSB / lowest byte
  // of outgoing packet
  for( int i = 1; i < 23; i++) {
    for (int j = 0; j < 8; j++) {
      // test data bit, incr data size limit
      uint16_t bitValue = dataValue & (1 << dataBitCounter);
      dataBitCounter++;

      // set to current bit in outgoing byte
      outPacket[i] = outPacket[i] | (bitValue << j);

      if(dataBitCounter >= 11) {
        channelCounter++;
        dataBitCounter = 0;
        if(channelCounter < NUMBER_OF_TX_CHANNELS) {
          dataValue = ((uint16_t*)values)[channelCounter];
        } else {
          // Fill non-TX generated channels with midpoint
          dataValue = (SBUS_MAX_VALUE - SBUS_MIN_VALUE) / 2;
        }
      }
    }
  }
}

uint16_t translateFlyskyChannelValueToSbus(uint16_t value) {
  int32_t channel = value;
  channel = channel - 1000;
  if (channel < 0) {
    channel = 0;
  }
  
  return channel * 1600 / 1000 + 192;
}
