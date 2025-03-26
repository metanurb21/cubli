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
#include "Utils.h"            // Include the new utils header

void setup()
{
  Serial.begin(115200);
  BLEHandler::initBLE(DEVICE_NAME, SERVICE_UUID, CHAR_UUID);

  EEPROM.begin(EEPROM_SIZE);

  LEDControl::initLEDs();
  MotorControl::initializeMotors(); // Initialize motors using the new MotorControl class
  StateManager::initializeState();  // Initialize state management

  EEPROM.get(0, offsets);
  if (offsets.ID == 96)
  {
    calibrated = true;
  }

  for (int i = 0; i < NUM_LEDS; i++)
  {
    LEDControl::setLEDColor(i, CRGB::Blue); // Set LED color
    tone(BUZZER, 2186 + (i * 100), 100, channel);
    delay(80);
  }
  delay(500);
  LEDControl::clearLEDs();

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
    UTILS::updateBatteryVoltage();
    StateManager::handleCalibrationIndication();
    previousT_2 = currentT;
  }
}
