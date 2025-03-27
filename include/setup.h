#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>
#include <FastLED.h>
#include <stdint.h>

#define BUZZER 27
#define VBAT 34
#define INT_LED 2
#define BRAKE 26

#define DIR2 15
#define ENC2_1 13
#define ENC2_2 14
#define PWM2 25
#define PWM2_CH 0

#define DIR3 5
#define ENC3_1 16
#define ENC3_2 17
#define PWM3 18
#define PWM3_CH 2

#define DIR1 4
#define ENC1_1 35
#define ENC1_2 33
#define PWM1 32
#define PWM1_CH 1

#define TIMER_BIT 8
#define BASE_FREQ 20000

#define MPU6050 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define accSens 0
#define gyroSens 0

#define EEPROM_SIZE 64
#define NUM_LEDS 9

// BLE
#define DEVICE_NAME "ESP32_BLE_Cube"
#define SERVICE_UUID "0000FF00-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID "0000FF01-0000-1000-8000-00805F9B34FB"
#define CHAR_UUID_NOTIFY "0000FF02-0000-1000-8000-00805F9B34FB" // New UUID for sending data

// Global variables
extern int16_t motor1_speed;
extern int16_t motor2_speed;
extern int16_t motor3_speed;
extern volatile int enc_count1;
extern volatile int enc_count2;
extern volatile int enc_count3;
extern int32_t motors_speed_X;
extern int32_t motors_speed_Y;
extern int32_t motors_speed_Z;
extern float speed_X;
extern float speed_Y;
extern float speed_offset;
extern float robot_angleX;
extern float robot_angleY;
extern float Acc_angleX;
extern float Acc_angleY;
extern float gyroScaleFactor;
extern float Gyro_amount;
extern float Gyro_amount_x;
extern float alpha;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float gyroXfilt;
extern float gyroYfilt;
extern float zK2;
extern float zK3;
extern float K1;
extern float K2;
extern float K3;
extern float K4;
extern float eK1;
extern float eK2;
extern float eK3;
extern float eK4;
extern int motor_speed_x_divisor;
extern int motor_speed_y_divisor;
extern bool calibrated;
extern bool calibrating;
extern bool calibrated_leds;
extern bool off_mode;
extern bool init_spin;
extern bool slow_down;
extern bool slow_down_finished;
extern bool toggle_init_spin;
extern int motor_init_spin;
extern int motor_init_spin_delay;
extern int16_t AcX;
extern int16_t AcY;
extern int16_t AcZ;
extern int16_t AcXc;
extern int16_t AcYc;
extern int16_t AcZc;
extern int16_t GyX;
extern int16_t GyY;
extern int16_t GyZ;
extern int16_t GyX_offset;
extern int16_t GyY_offset;
extern int16_t GyZ_offset;
extern int32_t GyX_offset_sum;
extern int32_t GyY_offset_sum;
extern int32_t GyZ_offset_sum;
extern long currentT;
extern long previousT_1;
extern long previousT_2;
extern long previousT_3;
extern int loop_time;
extern long spin_hold_time;
extern long current_spin_hold_time;
extern long previous_spin_hold_time;
extern int channel;
extern int freq;
extern int dure;
extern float radToDeg;
extern int rotate_speed;

extern bool vertical_vertex;
extern bool vertical_edge;
extern bool vertex_calibrated;
extern bool ledState;
extern bool ledStateSwitch;

extern bool init_spin_CW;
extern bool init_spin_CCW;
extern float device_heading;

// Offset structure
struct Offsets
{
	int ID;
	float acXv;
	float acYv;
	float acZv;
	float acXe;
	float acYe;
	float acZe;
};
extern Offsets offsets;

extern CRGB leds[NUM_LEDS];

#endif // SETUP_H