#include "setup.h"

// Motor speeds and encoder counts
int16_t motor1_speed = 0;
int16_t motor2_speed = 0;
int16_t motor3_speed = 0;
volatile int enc_count1 = 0;
volatile int enc_count2 = 0;
volatile int enc_count3 = 0;
int32_t motors_speed_X = 0;
int32_t motors_speed_Y = 0;
int32_t motors_speed_Z = 0;

// Speed and angle variables
float speed_X = 0;
float speed_Y = 0;
float speed_offset = 1.0;
float robot_angleX = 0;
float robot_angleY = 0;
float Acc_angleX = 0;
float Acc_angleY = 0;
float gyroScaleFactor = loop_time / 1000.0 / 65.536; // 0.001;
float Gyro_amount = 0.98;
float Gyro_amount_x = 0.98;
float alpha = 0.7;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float gyroXfilt = 0;
float gyroYfilt = 0;
float zK2 = 8.00;
float zK3 = 0.30;
float K1 = 180;
float K2 = 30;
float K3 = 1.6;
float K4 = 0.008;
float eK1 = 190;
float eK2 = 31.00;
float eK3 = 2.5;
float eK4 = 0.014;
int motor_speed_x_divisor = 5;
int motor_speed_y_divisor = 5;
int rotate_speed = 180; // 255 max

// Calibration and state variables
bool calibrated = false;
bool calibrating = false;
bool calibrated_leds = false;
bool off_mode = false;
bool init_spin = false;
bool slow_down = false;
bool toggle_init_spin = false;
int motor_init_spin = 0;
int motor_init_spin_delay = 0;

bool vertical_vertex = false;
bool vertical_edge = false;
bool vertex_calibrated = false;
bool ledState = false;
bool ledStateSwitch = false;
bool oscilate = false;

// Accelerometer and gyroscope data
int16_t AcX = 0;
int16_t AcY = 0;
int16_t AcZ = 0;
int16_t AcXc = 0;
int16_t AcYc = 0;
int16_t AcZc = 0;
int16_t GyX = 0;
int16_t GyY = 0;
int16_t GyZ = 0;
int16_t GyX_offset = 0;
int16_t GyY_offset = 0;
int16_t GyZ_offset = 0;
int32_t GyX_offset_sum = 0;
int32_t GyY_offset_sum = 0;
int32_t GyZ_offset_sum = 0;

//
bool init_spin_CW;
bool init_spin_CCW;
float device_heading = 0;
bool slow_down_finished = false;

// Timing variables
long currentT = 0;
long previousT_1 = 0;
long previousT_2 = 0;
long previousT_3 = 0;
int loop_time = 10;
long spin_hold_time = 0;
long current_spin_hold_time = 0;
long previous_spin_hold_time = 0;
bool motor_direction = false;
float batteryVoltage = 0;
int motor_speed_previous = 0;
bool turn_off_leds = false;
long end_hold_time = 0;

// Buzzer and tone variables
int octave = 1;
int channel = 7;
int freq = 2700;
int dure = 500;

// Conversion factor
float radToDeg = 57.2958;

// Offset structure
Offsets offsets = {0, 0, 0, 0, 0, 0, 0};

// LED's
CRGB leds[NUM_LEDS];