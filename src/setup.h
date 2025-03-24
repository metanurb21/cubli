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

float Gyro_amount = 0.996;
float Gyro_amount_x = 0.996;
uint8_t motor_speed_x_divisor = 5;
uint8_t motor_speed_y_divisor = 5;
uint8_t motor_speed_up = 0;
uint8_t motor_speed_up_delay = 0;
bool speed_up = false;
bool slow_down = false;
bool capt_speed = false;
uint8_t capt_speed_val = 0;
uint8_t motor_hold_delay = 0;
bool toggle_speed_up = true;

bool vertical_vertex = false;
bool vertical_edge = false;
bool calibrating = false;
bool vertex_calibrated = false;
bool calibrated = false;
bool calibrated_leds = false;
bool ledState = false;
bool ledStateSwitch = false;

// Sound
uint8_t octave = 1;
uint8_t channel = 7;
unsigned int freq = 2700;
unsigned long dure = 500;

float K1 = 180;
float K2 = 30.00;
float K3 = 1.6;
float K4 = 0.008;
float zK2 = 8.00;
float zK3 = 0.30;

float eK1 = 190;
float eK2 = 31.00;
float eK3 = 2.5;
float eK4 = 0.014;

int loop_time = 15;

float speed_offset = 0.866;

///
struct PIDParams
{
	float Kp;
	float Ki;
	float Kd;
	float prev_error;
	float integral;
};

PIDParams pidParamsX = {180, 1.0, 30.0, 0, 0}; // Example values, adjust as needed
PIDParams pidParamsY = {180, 1.0, 30.0, 0, 0};
PIDParams pidParamsZ = {8.0, 0.0, 0.3, 0, 0};

///
struct OffsetsObj
{
	int ID;
	float acXv;
	float acYv;
	float acZv;
	float acXe;
	float acYe;
	float acZe;
};
OffsetsObj offsets;

float alpha = 0.7;

int16_t AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroY, gyroZ, gyroXfilt, gyroYfilt, gyroZfilt;
float speed_X, speed_Y;

int16_t GyZ_offset = 0;
int16_t GyY_offset = 0;
int16_t GyX_offset = 0;
int32_t GyZ_offset_sum = 0;
int32_t GyY_offset_sum = 0;
int32_t GyX_offset_sum = 0;

float robot_angleX, robot_angleY;
float Acc_angleX, Acc_angleY;
int32_t motors_speed_X;
int32_t motors_speed_Y;
int32_t motors_speed_Z;

long currentT, previousT_1, previousT_2, previousT_3;

volatile int enc_count1 = 0, enc_count2 = 0, enc_count3 = 0;
int16_t motor1_speed;
int16_t motor2_speed;
int16_t motor3_speed;

//

struct KalmanFilter
{
	float Q_angle;	 // Process noise variance for the accelerometer
	float Q_bias;	 // Process noise variance for the gyro bias
	float R_measure; // Measurement noise variance
	float angle;	 // The angle calculated by the Kalman filter
	float bias;		 // The gyro bias calculated by the Kalman filter
	float rate;		 // Unbiased rate calculated from the rate and the calculated bias
	float P[2][2];	 // Error covariance matrix
};

struct KalmanFilter kfX, kfY;

enum VerticalState
{
	VERTICAL_UNKNOWN,
	VERTICAL_VERTEX,
	VERTICAL_EDGE,
	VERTICAL_NONE
};

// Kalman Filter variables
// float Q_angle = 0.001;
// float Q_bias = 0.003;
// float R_measure = 0.03;

// float angleX = 0; // Reset the angle
// float biasX = 0;  // Reset the gyro bias
// float P_00X = 0, P_01X = 0, P_10X = 0, P_11X = 0;

// float angleY = 0; // Reset the angle
// float biasY = 0;  // Reset the gyro bias
// float P_00Y = 0, P_01Y = 0, P_10Y = 0, P_11Y = 0;