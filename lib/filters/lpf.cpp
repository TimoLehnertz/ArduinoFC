#include "lpf.h"

LowPassFilter::LowPassFilter():
	output(0),
	ePow(0){}

LowPassFilter::LowPassFilter(float iCutOffFrequency, float iDeltaTime):
	output(0),
	ePow(1-exp(-iDeltaTime * 2 * PI * iCutOffFrequency)),
	freq(iCutOffFrequency)
{}

float LowPassFilter::update(float input){
	return output += (input - output) * ePow;
}

float LowPassFilter::update(float input, float deltaTime, float cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
	return output += (input - output) * ePow;
}

void LowPassFilter::reconfigureFilter(float deltaTime, float cutoffFrequency){
	ePow = 1-exp(-deltaTime * 2 * PI * cutoffFrequency);
	freq = cutoffFrequency;
}

/**
 * Vec3
 */
LowPassFilterVec3::LowPassFilterVec3(): LowPassFilter() {}

LowPassFilterVec3::LowPassFilterVec3(float iCutOffFrequency, float iDeltaTime): LowPassFilter(iCutOffFrequency, iDeltaTime) {}

Vec3 LowPassFilterVec3::update(Vec3 input){
	return output += (input - output) * ePow;
}

Vec3 LowPassFilterVec3::update(Vec3 input, float deltaTime, float cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
	return output += (input - output) * ePow;
}