#include "sbus.h"

stick_values_t stick_values;

void initDefaults(stick_values_t * values) {
  stick_values.roll = stick_values.pitch = stick_values.yaw = SBUS_MID_VALUE;
  stick_values.throttle = SBUS_MIN_VALUE;
  for(uint8_t i = 0; i < 10; i++) {
    stick_values.aux[i] = SBUS_MID_VALUE;
  }
}

// Maps RSSI (5 high to 160 low) to 0 - 100 % -- RSSI already linear with dBm
// represents -105 to -45 dBm +/- 6dBm. Max readings can
// represent -45 dBm - 0 dBm. The A7105 doesn't differentiate
// measurements higher than -45 dBm.
uint16_t mapRssiToSbusChannel(uint8_t value) {
  uint8_t maxRssiVal = 160;
  uint8_t minRssiVal = 5;
  if(value > maxRssiVal) {
    value = maxRssiVal;
  }
  if(value < minRssiVal) {
    value = minRssiVal;
  }
  uint16_t output = maxRssiVal - value; // invert scale
  output = output * 31 / 3 + SBUS_MIN_VALUE; // (map range of 155 to 1601.6, offset by min SBUS value)
  return output;
}

void stuffRssiChannel(stick_values_t * values, uint8_t rssi) {
  (*values).aux[RSSI_AUX_CHANNEL-1] = mapRssiToSbusChannel(rssi);
}

// Read out 14 channels
void readChannels(uint8_t * rxPacket, stick_values_t * values) {
  uint8_t channelOffset = 9;
  for (uint8_t i = 0; i < NUMBER_OF_TX_CHANNELS; i++) {
//    uint8_t index = i * 2 + channelOffset;
    // Using BF FW, we get channels directly -- no offset
    uint8_t index = i * 2;
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

uint16_t translateFlyskyChannelValueToSbus(uint16_t value) {
  uint16_t channel = value;
  if(channel < 1000) {
    channel = 0;
  } else {
    channel = channel - 1000;
  }

  return channel * 8 / 5 + 192;
}
