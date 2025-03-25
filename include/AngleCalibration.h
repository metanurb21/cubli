#ifndef ANGLE_CALIBRATION_H
#define ANGLE_CALIBRATION_H

#include <Arduino.h>
#include "Tone32.h"

namespace AngleCalibration
{
	void initializeCalibration();
	void angleSetup();
	void calibrateGyro();
	void angleCalc();
	void calculateAngles();
	void readGyroData();
	void readAccelData();
	void writeTo(byte device, byte address, byte value);
	void tone(uint8_t pin, unsigned int frequency, unsigned long duration, uint8_t channel);
	void playNotes(uint16_t note1, uint16_t note2, uint16_t note3, long duration);
	void noTone(uint8_t pin, uint8_t channel);
	void save();
}

#endif // ANGLE_CALIBRATION_H
