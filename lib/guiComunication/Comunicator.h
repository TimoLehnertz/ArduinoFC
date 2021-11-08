#ifndef comunicator
#define comunicator
#include <ins.h>
#include <FC.h>
#include <Storage.h>
#include <Adafruit_NeoPixel.h>

/**
 * Comunication protocol:
 * 	types of messages:
 * 		FC_DO   - commands like do calibration no response
 * 		FC_GET	- get commands wich the fc has to answer with a post
 * 		FC_RES	- Response on FC_GET
 * 		FC_POST	- 
 * 
 * 	FC_GET in the form of FC_GET_<Command> <id>
 * 		<id> represents an id wich is unique to each get request to be echoed back in the post response can be any string
 * 
 * 	FC_RES in the form of FC_RES <id> <response>
 * 		<id> the requests id
 * 		<response> response string
 * 	
 * 	FC_POST in the form of FC_POST_<command> <body>
 * 		<command> the name of the post command
 * 		<body> the string to be posted
 * 
 * FC_SET in the form of FC_SET_<command> <id> <body>
 * 		<command> the name of the SET command
 * 		<id> the set id. this id will get echoed back to verify succsess
 * 		<body> the string to be Setted
 * 		The FC has to reply with a response to the id echoing the original body to know that everything has gone right
 * 
 * 	
 * 	All messages can have MAXIMUM of 254 bytes in length
 * 
 * @author Timo Lehnertz
 * 
 */

class Comunicator {
public:
    INS* ins;
    SensorInterface* sensors;
    FC* fc;
	Crossfire* crsf;
	Adafruit_NeoPixel* pixels;

    Comunicator(INS* ins, SensorInterface* sensors, FC* fc, Crossfire* crsf, Adafruit_NeoPixel* pixels) : ins(ins), sensors(sensors), fc(fc), crsf(crsf), pixels(pixels) {}

    void begin();
    void handle();
    void end();

    void postSensorData(const char* sensorName, const char* subType, float value);
    void postSensorDataDouble(const char* sensorName, const char* subType, double value);
    void postSensorDataInt(const char* sensorName, const char* subType, uint64_t value);
    void postSensorData(const char* sensorName, PID pid);

	bool motorOverwrite = false;
	int motorFL = 0;//percentage 0 - 100
	int motorFR = 0;//percentage 0 - 100
	int motorBL = 0;//percentage 0 - 100
	int motorBR = 0;//percentage 0 - 100

	uint64_t loopStart = 0;
	uint64_t crsfTime = 0;
	uint64_t sensorsTime = 0;
	uint64_t insTime = 0;
	uint64_t comTime = 0;
	uint64_t chanelsTime = 0;
	uint64_t fcTime = 0;
	uint64_t loopEnd = 0;
	uint64_t maxLoopTime = 0;

	float cpuLoad = 0;
	uint64_t loopTimeUs = 0;
	int actualFreq = 0;

	int loopFreqRate = 1000;
	int loopFreqLevel = 1000;

    void saveEEPROM();
    void readEEPROM();

	void handleCRSFTelem();

	bool useLeds = false;

	bool useCellVoltage = true;

private:

	bool useAccTelem = false;
	bool useGyroTelem = false;
	bool useMagTelem = false;
	bool useBaroTelem = false;
	bool useGpsTelem = false;
	bool useAttiTelem = false;
	bool useVelTelem = false;
	bool useLocTelem = false;
	bool useQuatTelem = false;
	bool useTimingTelem = false;
	bool useRCTelem = false;
	bool useFCTelem = false;
	bool useBatTelem = false;

    char buffer[256];
    byte bufferCount = 0;
    int telemetryFreq = 20; //Hz
    uint64_t telemUs = 1000000 / telemetryFreq;
    uint64_t lastTelem = 0;

	int ledFreq = 30;
	uint32_t lastLED = 0;

    void processSerialLine();
    void post(const char* command, const char* value);
    void post(const char* command, String value);

    void postResponse(char* uid, String body);
    void postResponse(char* uid, Vec3 vec);
    void postResponse(char* uid, const char* body);
    void postResponse(char* uid, float num);
    void postResponse(char* uid, double num);
    void postResponse(char* uid, int val);
    void postResponse(char* uid, bool val);
    void postResponse(char* uid, Matrix3 mat);
    void postResponse(char* uid, PID pid);

    void scheduleTelemetry();
    void postTelemetry();

	void handleLED();
	void drawLedIdle();
};

#endif