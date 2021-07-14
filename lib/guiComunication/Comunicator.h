#pragma once
#include <ins.h>

enum FCCommand {
	ACC,
	GYRO,
	MAG,
	ROT,
	VEL,
	POS,
};

class Comunicator {
public:

    INS* ins;

    Comunicator(INS* ins) : ins(ins) {}

    void begin();
    void handle();
    void end();


private:
    char buffer[256];
    byte bufferCount = 0;

    void processSerialLine();
    void post(FCCommand command, const char* value);
    void serialPost(char* uid, String body);
};