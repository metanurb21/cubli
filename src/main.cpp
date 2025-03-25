/*
  ESP32 Balance cube by David Stewart, dave@metanurb.com 03/25
  Based on REMRC: https://github.com/remrc/Self-Balancing-Cube/blob/main/README.md
  Some refactoring made to update for my needs, and BLE connection so it can work with my iPhone.
  Added FastLED library for LED control.
  Added ability to initiate spinning of the cube.
  Added 2-way BLE communication.
*/
#include "setup.h"
#include <EEPROM.h>
#include <string>
#include "BLEHandler.h"
#include "LEDControl.h"
#include "MotorControl.h"     // Include the new motor control header
#include "StateManager.h"     // Include the new state management header
#include "AngleCalibration.h" // Include the new angle and calibration header

#define DEVICE_NAME "ESP32_BLE_Cube"
#define SERVICE_UUID "0000FF00-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID "0000FF01-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID_NOTIFY "0000FF02-0000-1000-8000-00805F9B34FB" // New UUID for sending data

void battVoltage(double voltage);
void updateBatteryVoltage();
std::string createMessage(const std::string &text, int32_t value);
uint8_t getRandomNumber();

std::string createMessage(const std::string &text, int32_t value)
{
  return "\r\n" + text + std::to_string(value);
}

void setup()
{
  Serial.begin(115200);
  BLEHandler::initBLE(DEVICE_NAME, SERVICE_UUID, CHAR_UUID);

  EEPROM.begin(EEPROM_SIZE);

  initLEDs();
  MotorControl::initializeMotors(); // Initialize motors using the new MotorControl class
  StateManager::initializeState();  // Initialize state management

  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
  {
    calibrated = true;
  }

  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Blue; // Set LED color
    FastLED.show();
    tone(BUZZER, 2186 + (i * 100), 100, channel);
    delay(80);
  }
  delay(500);
  FastLED.clear();
  FastLED.show();

  AngleCalibration::initializeCalibration(); // Initialize angle and calibration
  BLEHandler::sendData("Hello from Balancing Cube");
}

void loop()
{
  currentT = millis();

  // Handle angle calculations and motor speed updates
  if (currentT - previousT_1 >= loop_time)
  {
    MotorControl::updateMotorSpeeds();
    StateManager::handleCurrentState();
    previousT_1 = currentT;
  }

  // Handle battery voltage and LED calibration indication
  if (currentT - previousT_2 >= 2000)
  {
    updateBatteryVoltage();
    StateManager::handleCalibrationIndication();
    previousT_2 = currentT;
  }
}

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
