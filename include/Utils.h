#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <string>

namespace UTILS
{
	void battVoltage(float voltage);
	void updateBatteryVoltage();
	std::string createMessage(const std::string &text, int32_t value);
	std::string getBatVoltage(const std::string &text, float value);
	uint8_t getRandomNumber();
}
#endif // UTILS_H
