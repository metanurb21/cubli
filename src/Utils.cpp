#include "Utils.h"
#include <string>
#include "BLEHandler.h"
#include "setup.h"
#include "AngleCalibration.h" // Include the new angle and calibration header

namespace UTILS
{
	// Function to return a random number between 0 and 255
	uint8_t getRandomNumber()
	{
		return random(0, 256); // Upper bound is exclusive, so use 256 to include 255
	}

	void updateBatteryVoltage()
	{
		// Read the raw ADC value from the VBAT pin
		int rawADC = analogRead(VBAT);

		// Calculate the battery voltage
		// Formula: Vout = (ADC / 4095.0) * Vref * ((R1 + R2) / R2)
		// Vref = 3.3V, R1 = 33k, R2 = 10k
		batteryVoltage = (rawADC / 4095.0) * 3.3 * ((33000.0 + 10000.0) / 10000.0);
		// Pass the calculated voltage to battVoltage function
		battVoltage(batteryVoltage);
	}

	void battVoltage(float voltage)
	{
		if (voltage > 8 && voltage <= 9.5)
		{
			AngleCalibration::playNotes(4186, 4186, 4186, 50);
			BLEHandler::sendData("\r\n Battery voltage is low.");
		}
	}
	std::string createMessage(const std::string &text, int32_t value)
	{
		return "\r\n" + text + std::to_string(value);
	}
	std::string getBatVoltage()
	{
		String volt = String(batteryVoltage, 2);
		std::string message = "\r\nBattery Voltage: " + std::string(volt.c_str()) + "V";
		return message;
	}
}
