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
// // Dabble experimrntal
// #define CUSTOM_SETTINGS
// #define INCLUDE_SENSOR_MODULE
// // #define INCLUDE_TERMINAL_MODULE
// #include <DabbleESP32.h>

// void print_gyro_data();
// float calculateOrientation()
// {
//   Dabble.processInput();
//   float heading = Sensor.getAccelerometerYaxis();
//   return heading;
// }

void resetSpin();

void setup()
{
  Serial.begin(115200);
  BLEHandler::initBLE(DEVICE_NAME, SERVICE_UUID, CHAR_UUID);

  // Dabble.begin("Hello From Balancing Cube");

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

  // handle gyro data for spinning from iPhone
  if (currentT - previousT_3 >= 1000)
  {
    // print_gyro_data();
    resetSpin();
    previousT_3 = currentT;
  }
}

void resetSpin()
{
  if (slow_down_finished)
  {
    init_spin = false;
    toggle_init_spin = true;
    previous_spin_hold_time = 0;
  }
}

// void print_gyro_data()
// {
//   device_heading = calculateOrientation();
//   if (device_heading > 2)
//   {
//     if (device_heading > 3)
//     {
//       init_spin_CCW = true;
//       init_spin_CW = false;
//     }
//     init_spin = true;
//   }
//   else if (device_heading < -2)
//   {
//     if (device_heading < -3)
//     {
//       init_spin_CW = true;
//       init_spin_CCW = false;
//     }
//     init_spin = true;
//   }
//   else
//   {

//     if (slow_down_finished)
//     {
//       init_spin = false;
//       slow_down_finished = false;
//       toggle_init_spin = true;
//       previous_spin_hold_time = 0;
//       motor_init_spin = 0;
//     }
//   }
// }
