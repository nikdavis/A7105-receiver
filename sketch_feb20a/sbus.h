#define NUMBER_OF_TX_CHANNELS 14
#define SBUS_MIN_VALUE 192
#define SBUS_MAX_VALUE 1792
#define PACKET_PERIOD_MS 14
#define PACKET_PERIOD_HIGH_SPEED_MS 7

#include <stdint.h>
#include <stdio.h>

// Requires AETR (default on FS-I6)
// Flysky default values
struct {
  uint16_t roll, pitch, throttle, yaw;
  uint16_t aux[10];
} typedef stick_values_t;

extern stick_values_t stick_values;

void initDefaults(stick_values_t * values);

void readChannels(uint8_t * rxPacket, stick_values_t * values);

void buildPacket(uint8_t * outPacket, stick_values_t * values);

uint16_t translateFlyskyChannelValueToSbus(uint16_t value);
