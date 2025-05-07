#include "AngleCalibration.h"
#include <Wire.h>
#include <FastLED.h>
#include "StateManager.h" // Include the new state management header
#include "LEDControl.h"
#include "BLEHandler.h"
#include <EEPROM.h>
#include "setup.h"

// Define angle and calibration-related variables here if needed
// extern declarations for shared variables can be added if required

namespace AngleCalibration
{
	void initializeCalibration()
	{
		Serial.println("initializeCalibration");
		angleSetup();
	}

	void angleSetup()
	{
		Serial.println("angleSetup");
		Wire.begin();
		delay(100);
		writeTo(MPU6050, PWR_MGMT_1, 0);
		writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
		writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
		delay(100);
		calibrateGyro();
	}

	void calibrateGyro()
	{
		Serial.println("calibrateGyro");
		for (int i = 0; i < 512; i++)
		{
			angleCalc();
			GyZ_offset_sum += GyZ;
			delay(5);
		}
		GyZ_offset = GyZ_offset_sum >> 9;

		LEDControl::setAllLEDs(CRGB::Crimson, true, 100);

		for (int i = 0; i < 512; i++)
		{
			angleCalc();
			GyY_offset_sum += GyY;
			delay(5);
		}
		GyY_offset = GyY_offset_sum >> 9;

		LEDControl::setAllLEDs(CRGB::Blue, true, 100);

		for (int i = 0; i < 512; i++)
		{
			angleCalc();
			GyX_offset_sum += GyX;
			delay(5);
		}
		GyX_offset = GyX_offset_sum >> 9;

		LEDControl::setAllLEDs(CRGB::YellowGreen, true, 100);

		playNotes(4186, 4186, 4186, 100);
		should_fade_led = true;
	}

	void angleCalc()
	{
		readGyroData();
		readAccelData();

		if (abs(AcX) < 2000)
		{
			AcXc = AcX - offsets.acXv;
			AcYc = AcY - offsets.acYv;
			AcZc = AcZ - offsets.acZv;
		}
		else
		{
			AcXc = AcX - offsets.acXe;
			AcYc = AcY - offsets.acYe;
			AcZc = AcZ - offsets.acZe;
		}
		GyZ -= GyZ_offset;
		GyY -= GyY_offset;
		GyX -= GyX_offset;

		calculateAngles();
	}

	void calculateAngles()
	{
		// Update robot angles using gyroscope data
		robot_angleY += GyY * gyroScaleFactor;
		robot_angleX += GyX * gyroScaleFactor;

		// Calculate accelerometer angles
		Acc_angleY = atan2(AcXc, -AcZc) * radToDeg;
		Acc_angleX = -atan2(AcYc, -AcZc) * radToDeg;

		// Combine gyroscope and accelerometer data using complementary filter
		robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);
		robot_angleX = robot_angleX * Gyro_amount_x + Acc_angleX * (1.0 - Gyro_amount_x);

		StateManager::updateVerticalState();
	}

	void readGyroData()
	{
		Wire.beginTransmission(MPU6050);
		Wire.write(0x43);
		Wire.endTransmission(false);
		Wire.requestFrom(MPU6050, 6, true);
		GyX = Wire.read() << 8 | Wire.read();
		GyY = Wire.read() << 8 | Wire.read();
		GyZ = Wire.read() << 8 | Wire.read();
	}

	void readAccelData()
	{
		Wire.beginTransmission(MPU6050);
		Wire.write(0x3B);
		Wire.endTransmission(false);
		Wire.requestFrom(MPU6050, 6, true);
		AcX = Wire.read() << 8 | Wire.read();
		AcY = Wire.read() << 8 | Wire.read();
		AcZ = Wire.read() << 8 | Wire.read();
	}

	void writeTo(byte device, byte address, byte value)
	{
		Wire.beginTransmission(device);
		Wire.write(address);
		Wire.write(value);
		Wire.endTransmission(true);
	}

	void playNotes(uint16_t note1, uint16_t note2, uint16_t note3, long duration)
	{
		tone(BUZZER, note1, duration, channel);
		delay(duration);
		noTone(BUZZER, channel);
		tone(BUZZER, note2, duration, channel);
		delay(duration);
		noTone(BUZZER, channel);
		tone(BUZZER, note3, duration, channel);
		delay(duration);
		noTone(BUZZER, channel);
	}

	void tone(uint8_t pin, unsigned int frequency, unsigned long duration, uint8_t channel)
	{
		if (ledcRead(channel))
		{
			log_e("Tone channel %d is already in use", ledcRead(channel));
			return;
		}
		ledcAttachPin(pin, channel);
		ledcWriteTone(channel, frequency);
		if (duration)
		{
			delay(duration);
			noTone(pin, channel);
		}
	}

	void noTone(uint8_t pin, uint8_t channel)
	{
		ledcDetachPin(pin);
		ledcWrite(channel, 0);
	}

	void save()
	{
		EEPROM.put(0, offsets);
		EEPROM.commit();
		EEPROM.get(0, offsets);
		if (offsets.ID == 96)
		{
			calibrated = true;
		}
		calibrating = false;
		BLEHandler::sendData("Gyro settings saved, Calibration complete.");
		LEDControl::setAllLEDs(CRGB::BlueViolet, false, 0);
		AngleCalibration::playNotes(4186, 4699, 5274, 100);
		delay(500);
		LEDControl::clearLEDs();
	}
}
