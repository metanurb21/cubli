#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLEHandler.h"
#include "AngleCalibration.h"
#include "setup.h"
#include "Utils.h"

namespace BLEHandler
{
	BLECharacteristic *pBiDirectionalCharacteristic;

	// Callback class to handle incoming data
	class MyCallbacks : public BLECharacteristicCallbacks
	{
		void onWrite(BLECharacteristic *pBiDirectionalCharacteristic)
		{
			std::string receivedData = pBiDirectionalCharacteristic->getValue();
			if (receivedData.length() > 0)
			{
				String command = String(receivedData.c_str());
				command.trim();
				Serial.println(command);
				if (command.equals("c") && !calibrating)
				{
					calibrating = true;
					sendData("\r\n Calibrating on.");
					sendData("\r\n Set the cube on vertex...");
					AngleCalibration::playNotes(4186, 4699, 5274, 100);
					delay(1000);
				}
				if (command.equals("o") && calibrating)
				{
					Serial.println("X: ");
					Serial.println(AcX);
					Serial.println(" Y: ");
					Serial.println(AcY);
					Serial.println(" Z: ");
					Serial.println(AcZ + 16384);
					if (abs(AcX) < 2000 && abs(AcY) < 2000)
					{
						offsets.ID = 96;
						offsets.acXv = AcX;
						offsets.acYv = AcY;
						offsets.acZv = AcZ + 16384;
						sendData("\r\n Vertex OK.");
						sendData("\r\n Set the cube on edge...");
						tone(BUZZER, freq + 800, dure / 3, channel);
						vertex_calibrated = true;
						AngleCalibration::playNotes(4186, 4699, 5274, 100);
						delay(1000);
					}
					else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(AcY) < 2000 && vertex_calibrated)
					{
						// Serial.println("X: ");
						// Serial.println(AcX);
						// Serial.println(" Y: ");
						// Serial.println(AcY);
						// Serial.println(" Z: ");
						// Serial.println(AcZ + 16384);
						// Serial.println("Edge OK.");
						sendData("\r\n Edge OK.");
						offsets.acXe = AcX;
						offsets.acYe = AcY;
						offsets.acZe = AcZ + 16384;
						sendData("\r\n Saving...");
						AngleCalibration::save();

						tone(BUZZER, freq + 800, dure / 3, channel);
						delay(100);
						tone(BUZZER, freq + 1000, dure / 3, channel);
					}
					else
					{
						// Serial.println("X: ");
						// Serial.println(AcX);
						// Serial.println(" Y: ");
						// Serial.println(AcY);
						// Serial.println(" Z: ");
						// Serial.println(AcZ + 16384);
						// Serial.println("The angles are wrong!!!");
						sendData("\r\n The angles are wrong!!!");
						AngleCalibration::playNotes(4186, 4186, 4186, 100);
						delay(300);
					}
				}
				if (command.equals("9"))
				{
					motor_init_spin = 0;
					init_spin = true;
					spin_hold_time = millis() + 100000;
					sendData("\r\n Speed Increase");
				}
				if (command.equals("0"))
				{
					init_spin = false;
					slow_down = false;
					toggle_init_spin = true;
				}
				if (command.equals("s"))
				{
					sendData(UTILS::createMessage("motors_speed_X: ", motors_speed_X).c_str());
				}
			}
		}
	};

	void initBLE(const std::string &deviceName, const std::string &serviceUUID, const std::string &charUUID)
	{
		BLEDevice::init(deviceName);

		BLEServer *pServer = BLEDevice::createServer();
		BLEService *pService = pServer->createService(serviceUUID);

		pBiDirectionalCharacteristic = pService->createCharacteristic(
			charUUID,
			BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

		pBiDirectionalCharacteristic->setCallbacks(new MyCallbacks());
		pService->start();

		BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
		pAdvertising->addServiceUUID(serviceUUID);
		pAdvertising->start();
	}

	void sendData(const std::string &data)
	{
		if (pBiDirectionalCharacteristic != nullptr)
		{
			pBiDirectionalCharacteristic->setValue(data);
			pBiDirectionalCharacteristic->notify();
		}
	}
};
