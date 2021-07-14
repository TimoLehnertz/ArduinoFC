#include <Arduino.h>
#include "crossfire.h"

void Crossfire::handle() {
    // Serial.println("handle start");
    while(Serial1.available()) {
        // Serial.println("available start");
        char c = Serial1.read();
        // Serial.println("after read");
        // Serial.write(c);

        static uint8_t crsfFramePosition = 0;

        timeUs_t currentTimeUs = micros();

        if (cmpTimeUs(currentTimeUs, crsfFrameStartAtUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
            // We've received a character after max time needed to complete a frame,
            // so this must be the start of a new frame.
            crsfFramePosition = 0;
        }

        if (crsfFramePosition == 0) {
            crsfFrameStartAtUs = currentTimeUs;
        }
        // assume frame is 5 bytes long until we have received the frame length
        // full frame length includes the length of the address and framelength fields
        const int fullFrameLength = crsfFramePosition < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
        if (crsfFramePosition >= fullFrameLength) {
            continue;
        }
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t) c;
        if (crsfFramePosition >= fullFrameLength) {
            const uint8_t crc = crsfFrameCRC(crsfFrame);
            if (crc != crsfFrame.bytes[fullFrameLength - 1]) {
                Serial.println("CRSF crc missmatch! ");//Serial.print(crc); Serial.print(", got: "); Serial.println(crsfFrame.bytes[fullFrameLength - 1]);
                continue;
            }
            switch (crsfFrame.frame.type) {
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
            // default:
                // crsfFrame.bytes[crsfFramePosition] = 0;
                handleCrsfFrame(crsfFrame, crsfFramePosition - 3);
                break;
            }
            crsfFramePosition = 0;
        }
    }
    // Serial.println("handle end");
}

bool Crossfire::isFailsafe() {
    return micros() - lastRcFrame > CRSF_FAILSAFE_TIMEOUT_US;
}

void Crossfire::handleCrsfFrame(CRSF_Frame_t& frame, int payloadLength) {
    lastRcFrame = micros();
    firstFrameReceived = true;
    chanels = frameToChanels(frame, payloadLength);

    #ifdef CRSF_DEBUG //print out each frame
    Serial.print("received frame of type: "); Serial.print(frame.frame.type);
    Serial.print(", length: "); Serial.print(frame.frame.frameLength);
    Serial.print(" chanels: ");
    for (int i = 0; i < MAX_CHANEL_COUNT; i++) {
        Serial.print(chanels.chanels[i]); Serial.print(",");
    }
    Serial.println();
    #endif
    /**
     * Telemetry
     **/
    if(telemFrequency <= 0) return;
    telemInc++;
    if(telemInc >= telemFrequency) {
        telemInc = 0;
        // printFrame(frame);
        // if(fmQued) {
            // sendFlightMode();
            sendBatteryInfo();
            // sendGpsFrame();
        // }
    }
}

CRSF_TxChanels Crossfire::frameToChanels(CRSF_Frame_t& frame, int payloadLength) {
    CRSF_TxChanels chanels;
    // uint8_t numOfChannels = ((frame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC) * 8 - CRSF_SUBSET_RC_STARTING_CHANNEL_BITS) / CHANEL_BITS;
    uint8_t numOfChannels = 12;
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    uint8_t readByteIndex = 0;
    for (int chanel = 0; chanel < numOfChannels; chanel++) {
        while (bitsMerged < CRSF_SUBSET_RC_RES_BITS_11B) {
            uint8_t readByte = frame.frame.payload[readByteIndex++];
            readValue |= ((uint32_t) readByte) << bitsMerged;
            bitsMerged += 8;
        }
        chanels.chanels[chanel] = readValue & CRSF_SUBSET_RC_RES_MASK_11B;
        readValue >>= CRSF_SUBSET_RC_RES_BITS_11B;
        bitsMerged -= CRSF_SUBSET_RC_RES_BITS_11B;
    }
    return chanels;
}

uint8_t Crossfire::crsfFrameCRC(CRSF_Frame_t &frame) {
     // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, frame.frame.type);
    for (int i = 0; i < frame.frame.frameLength - 2; i++) {
        crc = crc8_dvb_s2(crc, frame.frame.payload[i]);
    }
    return crc;
}

// <Device address><Frame length><Type><Payload><CRC>
// uint8_t Crossfire::crsfFrameCRCTelemetry(CRSF_Frame_t &frame, int payloadLen) {
//      // CRC includes type and payload
    
//     uint8_t crc = crc8_dvb_s2(0, frame.frame.type);

//     for (int i = 0; i < payloadLen; i++) {
//         crc = crc8_dvb_s2(crc, frame.frame.payload[i]);
//     }
//     return crc;
// }

void Crossfire::begin() {
    Serial.println("starting crossfire");
    // uart.begin(9600);
    uart.begin(CRSF_BAUDRATE);
}

void Crossfire::end() {
    uart.end();
}

uint8_t Crossfire::crc8_calc(uint8_t crc, unsigned char a, uint8_t poly) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ poly;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

/**
 * Telemetry
 **/
void Crossfire::updateTelemetryFlightMode(const char* flightMode) {
    fm = flightMode;
}

void Crossfire::sendFlightMode() {
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.frameLength = 0;
    frame.frame.type = CRSF_FRAMETYPE_FLIGHT_MODE ;
    frame.frame.payload[0] = 'M';
    frame.frame.payload[1] = 'O';
    frame.frame.payload[2] = 'I';
    frame.frame.payload[3] = 'N';
    frame.frame.payload[4] = 0; //null terminated
    frame.frame.payload[5] = crsfFrameCRC(frame);
    Serial.write(frame.bytes, 4 + 5);// 4 + payload
    // initializeTelemetryFrame();
    // uart.write((uint8_t) 0); //write zero for frame length, since we don't know it yet
    // uart.write((uint8_t) CRSF_FRAMETYPE_FLIGHT_MODE);
    // uart.print(fm);
    // uart.write((uint8_t) 0); //zero-terminate string
}

// <Device address><Frame length><Type><Payload><CRC>
void Crossfire::sendBatteryInfo() {
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.type = CRSF_FRAMETYPE_BATTERY_SENSOR;
    frame.frame.frameLength = CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
    //voltage
    frame.frame.payload[0] = batAvgCellVoltage >> 8;
    frame.frame.payload[1] = (uint8_t) batAvgCellVoltage;
    //current
    frame.frame.payload[2] = batCurrent >> 8;
    frame.frame.payload[3] = (uint8_t) batCurrent;
    //fuel
    frame.frame.payload[4] = batFuel >> 16;
    frame.frame.payload[5] = batFuel >> 8;
    frame.frame.payload[6] = (uint8_t) batFuel;
    //remaining percent
    frame.frame.payload[7] = batRemainingPercentage;
    // frame.frame.payload[8] = crsfFrameCRCTelemetry(frame, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE);
    frame.frame.payload[8] = crsfFrameCRC(frame);
    sendFrame(frame);
    #ifdef CRSF_DEBUG
    // printFrame(frame);
    #endif
}

void Crossfire::sendGpsFrame() {
    CRSF_Frame_t frame;
    frame.frame.deviceAddress = CRSF_SYNC_BYTE;
    frame.frame.type = CRSF_FRAMETYPE_GPS;
    frame.frame.frameLength = CRSF_FRAME_GPS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;//1 CRC 1 type
    //lat lng
    writeU32BigEndian(&frame.frame.payload[0], gpsLat);
    writeU32BigEndian(&frame.frame.payload[4], gpsLng);
    //groundspeed
    writeU16BigEndian(&frame.frame.payload[8], groundSpeed);
    writeU16BigEndian(&frame.frame.payload[10], groundSpeed); //groundcourse??
    //altitude
    writeU16BigEndian(&frame.frame.payload[12], altitude); //groundcourse??
    frame.frame.payload[14] = satelitesInUse;

    //CRC
    frame.frame.payload[15] = crsfFrameCRC(frame);

    sendFrame(frame);

    Serial.println("gps frame:");
    printFrame(frame);
}

void Crossfire::writeU32BigEndian(uint8_t *dst, uint32_t val) {
    dst[0] = val >> 24;
    dst[1] = val >> 16;
    dst[2] = val >> 8;
    dst[3] = (uint8_t) val;
}

void Crossfire::writeU16BigEndian(uint8_t *dst, uint16_t val) {
    dst[0] = val >> 8;
    dst[0] = (uint8_t) val;
}

void Crossfire::sendFrame(CRSF_Frame_t &frame) {
    const int fullFrameLength = frame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;
    // Serial.println(fullFrameLength);
    return; // write crashes on teensy
    uart.write(frame.bytes, fullFrameLength);
}

void Crossfire::printFrame(CRSF_Frame_t &frame) {
    Serial.println("Frame: --");
    Serial.print(frame.frame.deviceAddress); Serial.println(" addr");
    Serial.print(frame.frame.frameLength); Serial.println(" len");
    Serial.print(frame.frame.type); Serial.println(" t");
    for (int i = 0; i < frame.frame.frameLength - 1; i++) {
        Serial.print(", "); Serial.print(frame.frame.payload[i]);
    }
    Serial.println("<-crc");
    int crc = crsfFrameCRC(frame);
    Serial.print(crc); Serial.println(" calculated crc");
    if(crc != frame.frame.payload[frame.frame.frameLength - 2]) {
        // Serial.println("CRC match");
    // } else {
        Serial.println("CRC MISSMATCH!");
    }
    Serial.println("------");
}

CRSF_TxChanels_Converted Crossfire::getChanelsCoverted() {
    CRSF_TxChanels_Converted conv;
    if(!firstFrameReceived) { //default values
        conv.roll       = 0;
        conv.pitch      = 0;
        conv.throttle   = 0;
        conv.yaw        = 0;
        conv.armed1     = 0;
        conv.armed2     = 0;
        conv.armed3     = 0;
        conv.chanel7    = 0;
        conv.chanel8    = 0;
        conv.chanel9    = 0;
        conv.chanel10   = 0;
        conv.chanel11   = 0;
        conv.chanel12   = 0;
    } else {
        conv.roll = map(chanels.labels.roll, 172.0, 1809.0, -1.0, 1.0);
        conv.pitch = map(chanels.labels.pitch, 172.0, 1809.0, -1.0, 1.0);
        conv.throttle = map((double) chanels.labels.throttle, 172.0, 1809.0, 0.0, 1.0);//Serial.print("throttle: "); Serial.println(chanels.labels.throttle);
        conv.yaw = map(chanels.labels.yaw, 172.0, 1809.0, -1.0, 1.0);
        conv.armed1 = chanels.labels.armed1 > 1800;
        conv.armed2 = chanels.labels.armed2 > 1800;
        conv.armed3 = conv.armed1 && conv.armed2;
        conv.chanel7    = map(chanels.labels.chanel7, 172.0, 1809.0, -1, 1);
        conv.chanel8    = map(chanels.labels.chanel8, 172.0, 1809.0, -1, 1);
        conv.chanel9    = map(chanels.labels.chanel9, 172.0, 1809.0, -1, 1);
        conv.chanel10   = map(chanels.labels.chanel10, 172.0, 1809.0, -1, 1);
        conv.chanel11   = map(chanels.labels.chanel11, 172.0, 1809.0, -1, 1);
        conv.chanel12   = map(chanels.labels.chanel12, 172.0, 1809.0, -1, 1);
    }
    return conv;
}

double Crossfire::map(double x, double in_min, double in_max, double out_min, double out_max) {
    // const double dividend = out_max - out_min;
    // const double divisor = in_max - in_min;
    // const double delta = x - in_min;

    // return (delta * dividend + (divisor / 2)) / divisor + out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}