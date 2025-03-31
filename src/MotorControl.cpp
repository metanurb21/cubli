#include "MotorControl.h"
#include "AngleCalibration.h"
#include "setup.h"

// Define motor-related variables here if needed
// extern declarations for shared variables can be added if required

namespace MotorControl
{
	void initializeMotors()
	{
		// Initialize motor pins and settings
		pinMode(DIR1, OUTPUT);
		pinMode(ENC1_1, INPUT);
		pinMode(ENC1_2, INPUT);
		attachInterrupt(ENC1_1, ENC1_READ, CHANGE);
		attachInterrupt(ENC1_2, ENC1_READ, CHANGE);
		ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
		ledcAttachPin(PWM1, PWM1_CH);
		Motor_control(1, 0, motor1_speed, DIR1, PWM1_CH);

		pinMode(DIR2, OUTPUT);
		pinMode(ENC2_1, INPUT);
		pinMode(ENC2_2, INPUT);
		attachInterrupt(ENC2_1, ENC2_READ, CHANGE);
		attachInterrupt(ENC2_2, ENC2_READ, CHANGE);
		ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
		ledcAttachPin(PWM2, PWM2_CH);
		Motor_control(2, 0, motor2_speed, DIR2, PWM2_CH);

		pinMode(DIR3, OUTPUT);
		pinMode(ENC3_1, INPUT);
		pinMode(ENC3_2, INPUT);
		attachInterrupt(ENC3_1, ENC3_READ, CHANGE);
		attachInterrupt(ENC3_2, ENC3_READ, CHANGE);
		ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
		ledcAttachPin(PWM3, PWM3_CH);
		Motor_control(3, 0, motor3_speed, DIR3, PWM3_CH);

		pinMode(BRAKE, OUTPUT);
		digitalWrite(BRAKE, HIGH);
		pinMode(BUZZER, OUTPUT);
	}

	void Motor_control(int motor_number, int sp, int motor_speed, uint8_t dir_pin, uint8_t pwm_channel)
	{
		if (motor_number == 3 && init_spin)
		{
			if (motor_init_spin < rotate_speed)
			{
				toggle_init_spin ? motor_init_spin++ : motor_init_spin = motor_init_spin;
				toggle_init_spin = !toggle_init_spin;
				spin_hold_time = millis() + 3000;
			}
			else
			{
				current_spin_hold_time = millis();
				if (current_spin_hold_time < spin_hold_time)
				{
					previous_spin_hold_time = current_spin_hold_time;
				}
				else
				{
					slow_down_finished = true;
				}
			}
			if (init_spin_CW)
			{
				sp = sp + (motor_speed - abs(motor_init_spin));
			}
			else if (init_spin_CCW)
			{
				sp = sp + (motor_speed + abs(motor_init_spin));
			}
		}
		else
		{
			sp = sp + motor_speed;
		}

		if (sp < 0)
			digitalWrite(dir_pin, LOW);
		else
		{
			digitalWrite(dir_pin, HIGH);
		}

		pwmSet(pwm_channel, 255 - abs(sp));
	}

	void pwmSet(uint8_t channel, uint32_t value)
	{
		ledcWrite(channel, value);
	}

	void XYZ_to_threeWay(float pwm_X, float pwm_Y, float pwm_Z)
	{
		int16_t m1 = round((0.5 * pwm_X - speed_offset * pwm_Y) / 1.37 + pwm_Z);
		int16_t m2 = round((0.5 * pwm_X + speed_offset * pwm_Y) / 1.37 + pwm_Z);
		int16_t m3 = -pwm_X / 1.37 + pwm_Z;
		Motor_control(1, m1, motor1_speed, DIR1, PWM1_CH);
		Motor_control(2, m2, motor2_speed, DIR2, PWM2_CH);
		Motor_control(3, m3, motor3_speed, DIR3, PWM3_CH);
	}

	void threeWay_to_XY(int in_speed1, int in_speed2, int in_speed3)
	{
		speed_X = ((in_speed3 - (in_speed2 + in_speed1) * 0.5) * 0.5) * 1.81;
		speed_Y = -(-speed_offset * (in_speed2 - in_speed1)) / 1.1;
	}

	void updateMotorSpeeds()
	{
		AngleCalibration::angleCalc();
		motor1_speed = enc_count1;
		enc_count1 = 0;
		motor2_speed = enc_count2;
		enc_count2 = 0;
		motor3_speed = enc_count3;
		enc_count3 = 0;
		threeWay_to_XY(motor1_speed, motor2_speed, motor3_speed);
		motors_speed_Z = motor1_speed + motor2_speed + motor3_speed;
	}

	void IRAM_ATTR ENC1_READ()
	{
		static int state = 0;
		state = (state << 2 | (digitalRead(ENC1_1) << 1) | digitalRead(ENC1_2)) & 0x0f;
		if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b)
		{
			enc_count1++;
		}
		else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07)
		{
			enc_count1--;
		}
	}

	void IRAM_ATTR ENC2_READ()
	{
		static int state = 0;
		state = (state << 2 | (digitalRead(ENC2_1) << 1) | digitalRead(ENC2_2)) & 0x0f;
		if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b)
		{
			enc_count2++;
		}
		else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07)
		{
			enc_count2--;
		}
	}

	void IRAM_ATTR ENC3_READ()
	{
		static int state = 0;
		state = (state << 2 | (digitalRead(ENC3_1) << 1) | digitalRead(ENC3_2)) & 0x0f;
		if (state == 0x02 || state == 0x0d || state == 0x04 || state == 0x0b)
		{
			enc_count3++;
			motor_direction = true;
		}
		else if (state == 0x01 || state == 0x0e || state == 0x08 || state == 0x07)
		{
			enc_count3--;
			motor_direction = false;
		}
	}
}
