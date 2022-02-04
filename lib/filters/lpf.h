#pragma once
#include <cmath>
#include <maths.h>

class LowPassFilter{
public:
	//constructors
	LowPassFilter();
	LowPassFilter(float iCutOffFrequency, float iDeltaTime);
	//functions
	float update(float input);
	float update(float input, float deltaTime, float cutoffFrequency);
	//get and configure funtions
	float getOutput() const{return output;}
	void reconfigureFilter(float deltaTime, float cutoffFrequency);
	int getCutoffFreq() { return freq; }
protected:
	float output;
	float ePow;
	int freq;
};

class LowPassFilterVec3 : public LowPassFilter {
public:
	//constructors
	LowPassFilterVec3();
	LowPassFilterVec3(float iCutOffFrequency, float iDeltaTime);
	//functions
	Vec3 update(Vec3 input);
	Vec3 update(Vec3 input, float deltaTime, float cutoffFrequency);
	//get and configure funtions
	Vec3 getOutput() const{return output;}
private:
	Vec3 output;
};