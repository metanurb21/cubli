#include "Utils.h"
#include <string>
#include "BLEHandler.h"
#include "setup.h"

namespace UTILS
{
	// Function to return a random number between 0 and 255
	uint8_t getRandomNumber()
	{
		return random(0, 256); // Upper bound is exclusive, so use 256 to include 255
	}

	void updateBatteryVoltage()
	{
		battVoltage((double)analogRead(VBAT) / 204); // value 204 must be selected by measuring battery voltage!
	}

	void battVoltage(double voltage)
	{
		if (voltage > 8 && voltage <= 9.5)
		{
			digitalWrite(BUZZER, HIGH);
			BLEHandler::sendData("\r\n Battery voltage is low.");
		}
		else
		{
			digitalWrite(BUZZER, LOW);
		}
	}
	std::string createMessage(const std::string &text, int32_t value)
	{
		return "\r\n" + text + std::to_string(value);
	}
}
