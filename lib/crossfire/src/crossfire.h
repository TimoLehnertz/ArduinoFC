#pragma once
#include <Arduino.h>

// #ifndef ARDUINO_AS_MBED_LIBRARY
// #include <SoftwareSerial.h>
// #endif
#include <HardwareSerial.h> //esp32

typedef uint32_t timeUs_t;
// typedef uint64_t timeUs_t;

// time difference, 32 bits always sufficient
typedef int32_t timeDelta_t;

#define CRSF_FRAME_LENGTH_ADDRESS 1
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1

#define CRSF_FRAME_LENGTH_TYPE_CRC 2 // length of TYPE and CRC fields combined


#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_FRAME_SIZE_MAX 64 // 62 bytes frame plus 2 bytes frame header(<length><type>)
#define CRSF_PAYLOAD_SIZE_MAX CRSF_FRAME_SIZE_MAX - 6

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED 0x17


#define MAX_CHANEL_COUNT 12
#define CHANEL_BITS 11

#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF

#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS 5

#define crc8_dvb_s2(crc, a) crc8_calc(crc, a, 0xD5)

#define CRSF_BAUDRATE 420000
// #define CRSF_BAUDRATE 416000

#define CRSF_SYNC_BYTE 0xC8

#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_GPS              0x02
// #define CRSF_FRAMETYPE_BATTERY_SENSOR 0x09

#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE 8
#define CRSF_FRAME_GPS_PAYLOAD_SIZE 15

#define CRSF_DEBUG

#define CRSF_FAILSAFE_TIMEOUT_US 100000 //100ms

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }

struct CRSF_TxChanels_Labels {
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t armed1;
    uint16_t armed2;
    uint16_t chanel7;
    uint16_t chanel8;
    uint16_t chanel9;
    uint16_t chanel10;
    uint16_t chanel11;
    uint16_t chanel12;
};

struct CRSF_TxChanels_Converted {
    double roll;
    double pitch;
    double throttle;
    double yaw;
    bool armed1;
    bool armed2;
    bool armed3; //true when both 1 and 2 are true 
    double chanel7;
    double chanel8;
    double chanel9;
    double chanel10;
    double chanel11;
    double chanel12;
};

union CRSF_TxChanels {
    uint16_t chanels[MAX_CHANEL_COUNT];
    CRSF_TxChanels_Labels labels;

    // CRSF_TxChanels() {
    //     labels.roll = 0;
    //     labels.pitch = 0;
    //     labels.throttle = 0;
    //     labels.yaw = 0;
    //     labels.armed1 = false;
    //     labels.armed2 = false;
    // }
};

struct CRSF_FrameDef_t {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
};

union CRSF_Frame_t {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    CRSF_FrameDef_t frame;
};

class Crossfire {
public:

    bool firstFrameReceived = false;
    CRSF_TxChanels chanels; //always set to the latest received chanels
    // for (size_t i = 0; i < MAX_CHANEL_COUNT; i++) {
    //     chanels.chanels[i] = 0;
    // }
    

    #ifdef ARDUINO_AS_MBED_LIBRARY
    Crossfire(UART uart) : uart(uart) {}
    #else
    // Crossfire(int rx, int tx) : uart(rx, tx) {}
    Crossfire(HardwareSerial *h) : uart(h) {}
    #endif

    // states
    void begin();
    void handle();
    void end();

    CRSF_TxChanels_Converted getChanelsCoverted();

    /**
     * Failsafe
     **/
    bool isFailsafe();


    /**
     * telemetry
     **/
    void updateTelemetryFlightMode(const char* mode); //null terminated string probably not more than 5 letters
    
    double map(double x, double in_min, double in_max, double out_min, double out_max);
private:
    #ifdef ARDUINO_AS_MBED_LIBRARY
    UART uart; //uart with wich the receiver is connected
    #else
    HardwareSerial* uart;
    #endif
    CRSF_Frame_t crsfFrame;
    byte payloadLength = 0;
    byte crsfFramePosition = 0;
    timeUs_t crsfFrameStartAtUs = 0;

    uint8_t crsfFrameCRC(CRSF_Frame_t & frame);
    uint8_t crsfFrameCRCTelemetry(CRSF_Frame_t & frame, int len);
    uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);

    void handleCrsfFrame(CRSF_Frame_t&, int);

    CRSF_TxChanels frameToChanels(CRSF_Frame_t& frame, int payloadLength);



    void writeU32BigEndian(uint8_t *dst, uint32_t val);
    void writeU16BigEndian(uint8_t *dst, uint16_t val);

    void printFrame(CRSF_Frame_t &frame);

    /**
     * Failsafe
     * has to be set to cut on rx
     **/
    timeUs_t lastRcFrame = 0;


    /**
     * Telemetry
     **/
    // 0 -> no telemetry
    // 1 telemetry is send between every received frame
    // 10 -> telemetry is send after every tenth received frame
    int telemFrequency = 5;
    int telemInc = 0;

    bool fmQued = false;
    const char* fm = "-";

    /**
     * Battery stats
     **/
    uint16_t batAvgCellVoltage = 42; //42 = 4.2Volts
    uint16_t batCurrent = 10; //10 = 1Ampere
    uint32_t  mahDraw = 100;
    uint8_t  batRemainingPercentage = 66;

    /**
     * GPS stats
     * 
     **/
    int32_t gpsLat = 1000;
    int32_t gpsLng = 1000;
    uint16_t groundSpeed = 100;
    uint16_t gpsHeading = 36500;
    uint16_t altitude = 1000;
    uint8_t satelitesInUse = 5;

    void sendFrame(CRSF_Frame_t &frame);

    void sendFlightMode();
    void sendBatteryInfo();
    void sendGpsFrame();
};