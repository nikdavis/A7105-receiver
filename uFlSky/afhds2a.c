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

#include <string.h>

#include "afhds2a_defs.h"
#include "afhds2a.h"
#include "rx_a7105.h"
#include "config.h"

static const uint8_t flySkyRegs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x19, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00,
    0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
    0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f
};

static const uint8_t flySky2ARegs[] = {
    0xff, 0x62, 0x00, 0x25, 0x00, 0xff, 0xff, 0x00,
    0x00, 0x00, 0xAA, 0x01, 0x00, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f,
    0x62, 0x80, 0xff, 0xff, 0x2a, 0x32, 0xc3, 0x1f,
    0x1e, 0xff, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
    0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f
};

static const uint8_t flySky2ABindChannels[] = {
    0x0D, 0x8C
};

static const uint8_t flySkyRfChannels[16][16] = {
    {   0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
    {   0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
    {   0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
    {   0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
    {   0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
    {   0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
    {   0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
    {   0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
    {   0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
    {   0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
    {   0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {   0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
    {   0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
    {   0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
    {   0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {   0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46}
};

const timings_t flySkyTimings = {.packet = 1500, .firstPacket = 1900, .syncPacket = 2250, .telemetry = 0xFFFFFFFF};
const timings_t flySky2ATimings = {.packet = 3850, .firstPacket = 4850, .syncPacket = 5775, .telemetry = 57000};

//static flySkyConfig_t flySkyConfig = { .txId = 0x773C92F1, .rfChannelMap = {0x74, 0x6B, 0x2A, 0x1D, 0x24, 0x91, 0x50, 0x2F, 0x5A, 0x61, 0x13, 0x3B, 0x8B, 0x49, 0x82, 0x55} };
static flySkyConfig_t flySkyConfig;
static const timings_t *timings = &flySky2ATimings;
static uint32_t timeout = 0;
static uint32_t timeLastPacket = 0;
static uint32_t timeLastBind = 0;
static uint32_t timeTxRequest = 0;
static uint32_t countTimeout = 0;
static uint32_t countPacket = 0;
static uint32_t txId = 0;
static uint32_t rxId = 0;
static bool bound = false;
static bool sendTelemetry = false;
static bool waitTx = false;
static uint16_t errorRate = 0;
static uint16_t rssi_dBm = 0;
static uint8_t rfChannelMap[FLYSKY_FREQUENCY_COUNT] = {0};

bool ledState;

void flashLed() {
  digitalWrite(ledPin, 0);
  delay(50);
  digitalWrite(ledPin, 1);
  delay(50);
}

static uint8_t getNextChannel(uint8_t step)
{
    static uint8_t channel = 0;
    channel = (channel + step) & 0x0F;
    return rfChannelMap[channel];
}

static void flySkyCalculateRfChannels(void)
{
    uint32_t channelRow = txId & 0x0F;
    uint32_t channelOffset = ((txId & 0xF0) >> 4) + 1;

    if (channelOffset > 9) {
        channelOffset = 9; // from sloped soarer findings, bug in flysky protocol
    }

    for (unsigned i = 0; i < FLYSKY_FREQUENCY_COUNT; i++) {
        rfChannelMap[i] = flySkyRfChannels[channelRow][i] - channelOffset;
    }
}

static void resetTimeout(const uint32_t timeStamp)
{
    timeLastPacket = timeStamp;
    timeout = timings->firstPacket;
    countTimeout = 0;
    countPacket++;
}

static void checkTimeout(void)
{
    static uint32_t timeMeasuareErrRate = 0;
    static uint32_t timeLastTelemetry = 0;
    uint32_t time = micros();

    if ((time - timeMeasuareErrRate) > (101 * timings->packet)) {
        errorRate = (countPacket >= 100) ? (0) : (100 - countPacket);
        countPacket = 0;
        timeMeasuareErrRate = time;
    }

    if ((time - timeLastTelemetry) > timings->telemetry) {
        timeLastTelemetry = time;
        sendTelemetry = true;
    }

    if ((time - timeLastPacket) > timeout) {
        uint32_t stepOver = (time - timeLastPacket) / timings->packet;

        timeLastPacket = (stepOver > 1) ? (time) : (timeLastPacket + timeout);

        A7105Strobe(A7105_STANDBY);
        A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(stepOver % FLYSKY_FREQUENCY_COUNT));
        A7105Strobe(A7105_RX);

        if(countTimeout > 31) {
            timeout = timings->syncPacket;
//            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
            // TODO: set RSSI somehow
        } else {
            timeout = timings->packet;
            countTimeout++;
        }
    }
}

static void checkRSSI(void)
{
    static uint8_t buf[FLYSKY_RSSI_SAMPLE_COUNT] = {0};
    static int16_t sum = 0;
    static uint16_t currentIndex = 0;

    uint8_t adcValue = A7105ReadReg(A7105_1D_RSSI_THOLD);

    sum += adcValue;
    sum -= buf[currentIndex];
    buf[currentIndex] = adcValue;
    currentIndex = (currentIndex + 1) % FLYSKY_RSSI_SAMPLE_COUNT;

    rssi_dBm = 50 + sum / (3 * FLYSKY_RSSI_SAMPLE_COUNT); // range about [95...52], -dBm

    int16_t tmp = 2280 - 24 * rssi_dBm; // convert to [0...1023]
//    setRssiDirect(tmp, RSSI_SOURCE_RX_PROTOCOL);
    // TODO: set RSSI somehow
}

static bool isValidPacket(const uint8_t *packet) {
    const flySky2ARcDataPkt_t *rcPacket = (const flySky2ARcDataPkt_t*) packet;
    
    return (rcPacket->rxId == rxId && rcPacket->txId == txId);
}

static void buildAndWriteTelemetry(uint8_t *packet)
{
    if (packet) {
        static uint8_t bytesToWrite = FLYSKY_2A_PAYLOAD_SIZE; // first time write full packet to buffer a7105
        flySky2ATelemetryPkt_t *telemertyPacket = (flySky2ATelemetryPkt_t*) packet;
//        uint16_t voltage = getBatteryVoltage();
        uint16_t voltage = 0;

        telemertyPacket->type = FLYSKY_2A_PACKET_TELEMETRY;

        telemertyPacket->sens[0].type = SENSOR_INT_V;
        telemertyPacket->sens[0].number = 0;
        telemertyPacket->sens[0].valueL = voltage & 0xFF;
        telemertyPacket->sens[0].valueH = (voltage >> 8) & 0xFF;

        telemertyPacket->sens[1].type = SENSOR_RSSI;
        telemertyPacket->sens[1].number = 0;
        telemertyPacket->sens[1].valueL = rssi_dBm & 0xFF;
        telemertyPacket->sens[1].valueH = (rssi_dBm >> 8) & 0xFF;

        telemertyPacket->sens[2].type = SENSOR_ERR_RATE;
        telemertyPacket->sens[2].number = 0;
        telemertyPacket->sens[2].valueL = errorRate & 0xFF;
        telemertyPacket->sens[2].valueH = (errorRate >> 8) & 0xFF;

        memset (&telemertyPacket->sens[3], 0xFF, 4 * sizeof(flySky2ASens_t));

        A7105WriteFIFO(packet, bytesToWrite);

        bytesToWrite = 9 + 3 * sizeof(flySky2ASens_t);// next time write only bytes that could change, the others are already set as 0xFF in buffer a7105
    }
}

static bool flySky2AReadAndProcess(uint8_t *payload, const uint32_t timeStamp)
{
    bool result = false;
    uint8_t packet[FLYSKY_2A_PAYLOAD_SIZE];

    uint8_t bytesToRead = (bound) ? (9 + 2*FLYSKY_2A_CHANNEL_COUNT) : (11 + FLYSKY_FREQUENCY_COUNT);
    A7105ReadFIFO(packet, bytesToRead);

    switch (packet[0]) {
    case FLYSKY_2A_PACKET_RC_DATA:
    case FLYSKY_2A_PACKET_FS_SETTINGS: // failsafe settings
    case FLYSKY_2A_PACKET_SETTINGS: // receiver settings
        if (isValidPacket(packet)) {
            checkRSSI();
            resetTimeout(timeStamp);

            const flySky2ARcDataPkt_t *rcPacket = (const flySky2ARcDataPkt_t*) packet;

            if (rcPacket->type == FLYSKY_2A_PACKET_RC_DATA) {
                if (payload) {
                    memcpy(payload, rcPacket->data, 2*FLYSKY_2A_CHANNEL_COUNT);
                }

//                if (sendTelemetry) {
//                    buildAndWriteTelemetry(packet);
//                    sendTelemetry = false;
//                    timeTxRequest = timeStamp;
//                    waitTx = true;
//                }

                result = true;
            }

            if (!waitTx) {
                A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(3));
            }
        }
        break;

    case FLYSKY_2A_PACKET_BIND1:
    case FLYSKY_2A_PACKET_BIND2:
#ifndef TEENSY_LC
        digitalWrite(ledPin, ledState);
        ledState = !ledState;
//            delay(10);
#endif
        if (!bound) {
            resetTimeout(timeStamp);

            flySky2ABindPkt_t *bindPacket = (flySky2ABindPkt_t*) packet;

            if (bindPacket->rfChannelMap[0] != 0xFF) {
                memcpy(rfChannelMap, bindPacket->rfChannelMap, FLYSKY_FREQUENCY_COUNT); // get TX channels
            }

            txId = bindPacket->txId;
            bindPacket->rxId = rxId;
            memset(bindPacket->rfChannelMap, 0xFF, 26); // erase channelMap and 10 bytes after it

            timeTxRequest = timeLastBind = timeStamp;
            waitTx = true;

            A7105WriteFIFO(packet, FLYSKY_2A_PAYLOAD_SIZE);
        }
#ifndef TEENSY_LC
        digitalWrite(ledPin, ledState);
#endif
        break;

    default:
        break;
    }

    if (!waitTx){
        A7105Strobe(A7105_RX);
    }
    return result;
}

bool flySkyInit(void)
{
    uint8_t startRxChannel;

    timings = &flySky2ATimings;
    rxId = 0x12345678;
    startRxChannel = flySky2ABindChannels[0];
    A7105Init(0x5475c52A);
    A7105Config(flySky2ARegs, sizeof(flySky2ARegs));

    if (flySkyConfig.txId == 0) {
        bound = false;
    } else {
        bound = true;
        txId = flySkyConfig.txId; // load TXID
        memcpy (rfChannelMap, flySkyConfig.rfChannelMap, FLYSKY_FREQUENCY_COUNT);// load channel map
        startRxChannel = getNextChannel(0);
    }
    bound = false;

    A7105WriteReg(A7105_0F_CHANNEL, startRxChannel);
    A7105Strobe(A7105_RX); // start listening

    resetTimeout(micros());

    return true;
}

void flySkySetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
{
    if (rcData && payload) {
        uint32_t channelCount;
        channelCount = FLYSKY_2A_CHANNEL_COUNT;

        for (unsigned i = 0; i < channelCount; i++) {
            rcData[i] = payload[2 * i + 1] << 8 | payload [2 * i + 0];
        }
    }
}

bool flySkyDataReceived(uint8_t *payload)
{
    bool result = false;
    uint32_t timeStamp;

    if (A7105RxTxFinished(&timeStamp)) {
        uint8_t modeReg = A7105ReadReg(A7105_00_MODE);

        if (((modeReg & A7105_MODE_TRSR) != 0) && ((modeReg & A7105_MODE_TRER) == 0)) { // TX complete
            if (bound) {
                A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(3));
            }
            A7105Strobe(A7105_RX);
        } else if ((modeReg & (A7105_MODE_CRCF|A7105_MODE_TRER)) == 0) { // RX complete, CRC pass
            result = flySky2AReadAndProcess(payload, timeStamp);

        } else {
            A7105Strobe(A7105_RX);
        }
    }

    if (waitTx && (micros() - timeTxRequest) > TX_DELAY) {
        A7105Strobe(A7105_TX);
        waitTx = false;
    }

    if (false) {
        bound = false;
        txId = 0;
        memset(rfChannelMap, 0, FLYSKY_FREQUENCY_COUNT);
        uint8_t bindChannel = flySky2ABindChannels[0];
        A7105WriteReg(A7105_0F_CHANNEL, bindChannel);
    }

    if (bound) {
        checkTimeout();
    } else {
        if ((micros() - timeLastBind) > BIND_TIMEOUT && rfChannelMap[0] != 0 && txId != 0) {
            result = true;
            bound = true;
            flySkyConfig.txId = txId; // store TXID
            memcpy (flySkyConfig.rfChannelMap, rfChannelMap, FLYSKY_FREQUENCY_COUNT);// store channel map
//            writeEEPROM();
        }
    }

    return result;
}

