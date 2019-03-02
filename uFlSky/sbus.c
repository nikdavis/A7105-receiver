#include "sbus.h"

stick_values_t stick_values;

void initDefaults(stick_values_t * values) {
  stick_values.roll = stick_values.pitch = stick_values.yaw = 992;
  stick_values.throttle = 192;
}

// Read out 14 channels
void readChannels(uint8_t * rxPacket, stick_values_t * values) {
  uint8_t channelOffset = 9;
  for (uint8_t i = 0; i < NUMBER_OF_TX_CHANNELS; i++) {
    uint8_t index = i * 2 + channelOffset;
    uint16_t rawChannel = rxPacket[index + 1] << 8;
    rawChannel = rawChannel | rxPacket[index];
    ((uint16_t*)values)[i] = translateFlyskyChannelValueToSbus(rawChannel);
  }
}

// 25 byte packet
void buildPacket(uint8_t * outPacket, stick_values_t * values) {
  memset(outPacket, 0x00, 25);
  outPacket[0] = 0x0F;
  outPacket[24] = 0x00;

  uint8_t channelCounter = 0;
  uint8_t dataBitCounter = 0;
  uint16_t dataValue = ((uint16_t*)values)[channelCounter];
  // Channels represent middle 22 bytes -- 16 channels * 11 bits. They
  // are ordered by lowest channel and LSB first in LSB / lowest byte
  // of outgoing packet
  for( uint8_t i = 1; i < 23; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      // test data bit, incr data size limit
      uint16_t bitValue = dataValue & (1 << dataBitCounter);
      dataBitCounter++;

      // set to current bit in outgoing byte
      if(bitValue) {
        outPacket[i] = outPacket[i] | (1 << j);
      }

      if(dataBitCounter >= 11) {
        channelCounter++;
        dataBitCounter = 0;
        if(channelCounter < NUMBER_OF_TX_CHANNELS) {
          dataValue = ((uint16_t*)values)[channelCounter];
        } else {
          // Fill non-TX generated channels with midpoint
          dataValue = SBUS_MID_VALUE;
        }
      }
    }
  }
}

uint16_t translateFlyskyChannelValueToSbus(uint16_t channel) {
  if(channel < 1000) {
    channel = 0;
  } else {
    channel = channel - 1000;
  }

  // Make sacrifices, correct scaling factor is 1.6, but it runs sooo slow in AVRs
  return channel << 2;
}
