#ifndef comunicator
#define comunicator
#include <ins.h>

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
enum FCCommand {
	FC_DO_START_TELEM,
	FC_DO_RESET_INS,
	FC_DO_STOP_TELEM,
	FC_DO_ACC_CALIB,
	FC_DO_GYRO_CALIB,
	FC_DO_MAG_CALIB,
	FC_DO_END_COM,
	FC_GET_ACC_OFFSET,
	FC_GET_GYRO_OFFSET,
	FC_GET_MAG_OFFSET,
	FC_GET_ACC_MUL,
	FC_GET_GYRO_MUL,
	FC_GET_MAG_MUL,
    FC_GET_ACC_LPF,
	FC_GET_GYRO_LPF,
	FC_GET_INS_ACC_INFL,
	FC_POST_SENSOR,
	FC_SET_ACC_OFFSET,
	FC_SET_GYRO_OFFSET,
	FC_SET_MAG_OFFSET,
	FC_SET_ACC_MUL,
	FC_SET_GYRO_MUL,
	FC_SET_MAG_MUL,
    FC_SET_ACC_LPF,
	FC_SET_GYRO_LPF,
	FC_SET_INS_ACC_INFL,
};

class Comunicator {
public:
    INS* ins;
    SensorInterface* sensors;

    Comunicator(INS* ins, SensorInterface* sensors) : ins(ins), sensors(sensors) {}

    void begin();
    void handle();
    void end();

    void postSensorData(const char* sensorName, const char* subType, float value);

private:

    char buffer[256];
    byte bufferCount = 0;
    bool useTelemetry = false;
    int telemetryFreq = 30; //Hz
    uint64_t telemUs = 1000000 / telemetryFreq;
    uint64_t lastTelem = 0;

    void processSerialLine();
    void post(FCCommand command, const char* value);
    void post(FCCommand command, String value);
    void postResponse(char* uid, String body);
    void postResponse(char* uid, float num);

    void scheduleTelemetry();
    void postTelemetry();
};

#endif