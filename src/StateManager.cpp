#include "StateManager.h"
#include <FastLED.h>
#include "MotorControl.h"
#include "AngleCalibration.h"
#include "BLEHandler.h"
#include "setup.h"
#include "LEDControl.h"

// Define state-related variables here if needed
// extern declarations for shared variables can be added if required
// Sate machine
VerticalState currentState = VERTICAL_UNKNOWN;

namespace StateManager
{
	void initializeState()
	{
		currentState = VERTICAL_UNKNOWN;
	}

	void handleCurrentState()
	{
		if (currentState == VERTICAL_VERTEX && calibrated && !calibrating)
		{
			handleVerticalVertexState();
		}
		else if (currentState == VERTICAL_EDGE && calibrated && !calibrating)
		{
			handleVerticalEdgeState();
		}
		else
		{
			MotorControl::XYZ_to_threeWay(0, 0, 0);
			digitalWrite(BRAKE, LOW);
			motors_speed_X = 0;
			motors_speed_Y = 0;
		}
	}

	void handleCalibrationIndication()
	{
		if (!calibrated && !calibrating)
		{
			BLEHandler::sendData("Please calibrate the balancing points.");
			if (!calibrated_leds)
			{
				indicateCalibration();
				calibrated_leds = true;
			}
			else
			{
				calibrated_leds = false;
			}
		}
	}

	void indicateCalibration()
	{
		for (int i = 0; i < NUM_LEDS; i++)
		{
			LEDControl::setLEDColor(i, CRGB::Green);
			tone(BUZZER, 2186 + (i * 100), 100, channel);
			delay(80);
		}
		delay(500);
		LEDControl::clearLEDs();
		AngleCalibration::playNotes(4186, 4186, 4186, 100);
	}

	void updateVerticalState()
	{
		switch (currentState)
		{
		case VERTICAL_UNKNOWN:
			if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4)
			{
				robot_angleX = Acc_angleX;
				robot_angleY = Acc_angleY;
				currentState = VERTICAL_VERTEX;
			}
			else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3)
			{
				robot_angleX = Acc_angleX;
				robot_angleY = Acc_angleY;
				currentState = VERTICAL_EDGE;
			}
			else
			{
				currentState = VERTICAL_NONE;
			}
			break;

		case VERTICAL_VERTEX:
			if (abs(robot_angleX) > 7 || abs(robot_angleY) > 7)
			{
				currentState = VERTICAL_NONE;
			}
			break;

		case VERTICAL_EDGE:
			if (abs(robot_angleX) > 7 || abs(robot_angleY) > 7)
			{
				currentState = VERTICAL_NONE;
			}
			break;

		case VERTICAL_NONE:
			if (off_mode)
			{
				LEDControl::clearLEDs();
				tone(BUZZER, 3186, 100, channel);
				delay(100);
				tone(BUZZER, 2186, 100, channel);
				off_mode = false;
				motor1_speed = 0;
				enc_count1 = 0;
				motor2_speed = 0;
				enc_count2 = 0;
				motor3_speed = 0;
				enc_count3 = 0;
				speed_X = 0;
				speed_Y = 0;
				motors_speed_X = 0;
				motors_speed_Y = 0;
				motors_speed_Z = 0;
				init_spin = false;
				slow_down = false;
				toggle_init_spin = true;
				motor_init_spin = 0;
				previous_spin_hold_time = 0;
			}

			if (abs(AcX) < 2000 && abs(Acc_angleX) < 0.4 && abs(Acc_angleY) < 0.4)
			{
				robot_angleX = Acc_angleX;
				robot_angleY = Acc_angleY;
				currentState = VERTICAL_VERTEX;
				LEDControl::setLEDColor(4, CRGB::Blue);
				tone(BUZZER, 2186, 100, channel);
				off_mode = true;
			}
			else if (abs(AcX) > 7000 && abs(AcX) < 10000 && abs(Acc_angleX) < 0.3)
			{
				robot_angleX = Acc_angleX;
				robot_angleY = Acc_angleY;
				currentState = VERTICAL_EDGE;
				LEDControl::setLEDColor(4, CRGB::Yellow);
				tone(BUZZER, 2186, 100, channel);
				off_mode = true;
			}
			break;
		}
	}

	void handleVerticalVertexState()
	{
		digitalWrite(BRAKE, HIGH);

		// Normalize gyroscope readings
		const float gyroScale = 1.0 / 131.0;
		gyroX = GyX * gyroScale;
		gyroY = GyY * gyroScale;
		gyroZ = GyZ * gyroScale;

		// Apply low-pass filter to gyroscope readings
		gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
		gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;

		// Calculate PWM values using PID control
		int pwm_X = constrain(K1 * robot_angleX + K2 * gyroXfilt + K3 * speed_X + K4 * motors_speed_X, -255, 255);
		int pwm_Y = constrain(K1 * robot_angleY + K2 * gyroYfilt + K3 * speed_Y + K4 * motors_speed_Y, -255, 255);
		int pwm_Z = constrain(zK2 * gyroZ + zK3 * motors_speed_Z, -255, 255);

		// Update motor speeds
		motors_speed_X += speed_X / motor_speed_x_divisor;
		motors_speed_Y += speed_Y / motor_speed_y_divisor;

		// Control motors based on calculated PWM values
		MotorControl::XYZ_to_threeWay(-pwm_X, pwm_Y, -pwm_Z);
	}

	void handleVerticalEdgeState()
	{
		digitalWrite(BRAKE, HIGH);
		gyroX = GyX / 131.0;
		gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;

		int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor3_speed + eK4 * motors_speed_X, -255, 255);

		motors_speed_X += motor3_speed / 5;
		MotorControl::Motor_control(3, pwm_X, motor3_speed, DIR3, PWM3_CH);
	}
}
