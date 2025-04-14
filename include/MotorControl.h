#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

namespace MotorControl
{
	void initializeMotors();
	void Motor_control(int motor_number, int sp, int motor_speed, uint8_t dir_pin, uint8_t pwm_channel);
	void pwmSet(uint8_t channel, uint32_t value);
	void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z);
	void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3);
	void updateMotorSpeeds();
	void IRAM_ATTR ENC1_READ();
	void IRAM_ATTR ENC2_READ();
	void IRAM_ATTR ENC3_READ();
	void updateMotorInitSpin();
	void handleMotor3Control(int &sp, int motor_speed);
	void handleInitSpin();
	void handleSlowDown();
	void handleSpinCompletion();
	void adjustSpeedForSpinDirection(int &sp, int motor_speed);
	void setMotorDirectionAndSpeed(int sp, uint8_t dir_pin, uint8_t pwm_channel);
}

#endif // MOTOR_CONTROL_H
