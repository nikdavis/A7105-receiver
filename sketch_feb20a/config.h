#define chipSelectPin 10
#define spiClockSpeed 4000000

#define setbit(value, i) ((value) |= ((1) << (i)))
#define clearbit(value, i) ((value) &= ~((1) << (i)))
#define togglebit(byte, bit) ((byte) ^= ((1) << (bit)))
#define testbit(byte, bit) (((byte) >> (bit)) & (1))
