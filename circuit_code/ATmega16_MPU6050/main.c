/*
 * ATmega16 Interface with MPU-6050
 * http://www.electronicwings.com
 *
 */ 


// #define F_CPU 8000000UL									/* Define CPU clock Frequency e.g. here its 8MHz */
#define F_CPU 4000000UL									/* Define CPU clock Frequency e.g. here its 1MHz */

#include <avr/io.h>										/* Include AVR std. library file */
#include <util/delay.h>									/* Include delay header file */
#include <inttypes.h>									/* Include integer type header file */
#include <stdlib.h>										/* Include standard library file */
#include <math.h>
#include <string.h>

#include "MPU6050.h"
// #include "USART_RS232_H_file.h"							/* Include USART header file */
#include "FTDI_USART.h"

char float_[10];
char float_buf[6][15];

volatile float Acc_x,Acc_y,Acc_z,Temperature,Gyro_x,Gyro_y,Gyro_z;

// debug
volatile int iteration = 0;
#define PRINT_FREQ 1
int DEBUG = 1;
// #define GYRO_MAX_SAMPLES 1000
#define GYRO_MAX_SAMPLES 200

// PID coefficients
// float Kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
// float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
// float Kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll

float Kp[3] = {3,   0, 0};    
float Ki[3] = {0.2, 0, 0}; 
float Kd[3] = {0,   2, 2};        

// Pulse length
int round_func(double x)
{
	if (x < 0.0)
	return (int)(x - 0.5);
	else
	return (int)(x + 0.5);
}

int pulse_length_to_duty(int pulse_length) {
	return round_func(((float)pulse_length) / 16); // @ 4 MHz
	// return round_func(((float)pulse_length) / 64); // @ 1 MHz
}



#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define STOPPED  0
#define STARTING 1
#define STARTED  2
// ---------------- Receiver variables ---------------------------------------
// Previous state of each channel (HIGH or LOW)
volatile int previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};
// volatile unsigned int pulse_length[4] = {1300, 1300, 1300, 1300};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];


// ----------------------- MPU variables -------------------------------------
#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define PI acos(-1)
#define FREQ 8000
// #define FREQ 1000
#define SSF_GYRO 16.4

// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int gyro_raw[3] = {0,0,0};

// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0,0,0};

// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int acc_raw[3] = {0 ,0 ,0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0,0,0};

// Total 3D acceleration vector in m/s²
long acc_total_vector;

// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

// Init flag set to TRUE after first loop
int initialized = 0;

// ----------------------- MPU variables end -------------------------------------


// ----------------------- Variables for servo signal generation -------------
unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
pulse_length_esc2 = 1000,
pulse_length_esc3 = 1000,
pulse_length_esc4 = 1000;



/**
 * Calibrate MPU6050: take 2000 samples to calculate average offsets.
 * During this step, the quadcopter needs to be static and on a horizontal surface.
 *
 * This function also sends low throttle signal to each ESC to init and prevent them beeping annoyingly.
 *
 * This function might take ~2sec for 2000 samples.
 */
void calibrateMpu6050() {
    int max_samples = GYRO_MAX_SAMPLES;
	// int max_samples = 100;

    for (int i = 0; i < max_samples; i++) {
        MPU6050_Read_RawValue();
		gyro_raw[X] = Gyro_x;
		gyro_raw[Y] = Gyro_y;
		gyro_raw[Z] = Gyro_z;
		acc_raw[X] = Acc_x;
		acc_raw[Y] = Acc_y;
		acc_raw[Z] = Acc_z;
		
		if (i % 100 == 0 && DEBUG) {
			printf("gyro raw: %d, %d, %d \r\n", gyro_raw[X], gyro_raw[Y], gyro_raw[Z]);	
		}

        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];

        // Generate low throttle pulse to init ESC and prevent them beeping
        // PORTD |= B11110000;      // Set pins #4 #5 #6 #7 HIGH
        // delayMicroseconds(1000); // Wait 1000µs
        // PORTD &= B00001111;      // Then set LOW

        // Just wait a bit before next loop
        // delay(3);
		// _delay_ms(3);
    }

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;
	
	if (iteration == PRINT_FREQ && DEBUG)
		printf("gyro offset: %ld, %ld, %ld \r\n", gyro_offset[X], gyro_offset[Y], gyro_offset[Z]);
}

/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles() {
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];

    // Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
if (iteration == PRINT_FREQ && DEBUG) {
	dtostrf(gyro_angle[X], 3, 6, float_buf[0]);
	dtostrf(gyro_angle[Y], 3, 6, float_buf[1]);
	printf("gyro angle: %s, %s \r\n", float_buf[0], float_buf[1]);
}
	//dtostrf(PI, 3, 2, float_);
	//printf("PI: %s \r\n", float_);
}


/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles() {
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
    }

    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
    }
if (iteration == PRINT_FREQ && DEBUG) {
	dtostrf(acc_angle[X], 3, 6, float_buf[0]);
	dtostrf(acc_angle[Y], 3, 6, float_buf[1]);
	printf("acc angle: %s, %s \r\n", float_buf[0], float_buf[1]);
}
}

/**
 * Reset gyro's angles with accelerometer's angles.
 */
void resetGyroAngles() {
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}


/**
 * Calculate real angles from gyro and accelerometer's values
 */
void calculateAngles() {
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized != 0) {
        // Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        // At very first start, init gyro angles with accelerometer angles
        resetGyroAngles();

        initialized = 1;
    }

    // To dampen the pitch and roll angles a complementary filter is used
    measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis
if (iteration == PRINT_FREQ && DEBUG) {
	dtostrf(measures[ROLL], 3, 6, float_buf[0]);
	dtostrf(measures[PITCH], 3, 6, float_buf[1]);
	dtostrf(measures[YAW], 3, 6, float_buf[2]);
	printf("measures angle: %s, %s %s\r\n", float_buf[0], float_buf[1], float_buf[2]);
}

    // Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
if (iteration == PRINT_FREQ && DEBUG) {
	dtostrf(angular_motions[ROLL], 3, 6, float_buf[0]);
	dtostrf(angular_motions[PITCH], 3, 6, float_buf[1]);
	dtostrf(angular_motions[YAW], 3, 6, float_buf[2]);
	printf("angular motions: %s, %s %s\r\n", float_buf[0], float_buf[1], float_buf[2]);
}
}


// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// PID coefficients
// float Kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
// float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
// float Kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll
// ---------------------------------------------------------------------------

/**
 * Calculate the PID set point in °/s
 *
 * @param float angle         Measured angle (in °) on an axis
 * @param int   channel_pulse Pulse length of the corresponding receiver channel
 * @return float
 */
float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float set_point    = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of YAW axis in °/s
 *
 * @param int yaw_pulse      Receiver pulse length of yaw's channel
 * @param int throttle_pulse Receiver pulse length of throttle's channel
 * @return float
 */
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    float set_point = 0;

    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }

    return set_point;
}

/**
 * Calculate PID set points on axis YAW, PITCH, ROLL
 */
void calculateSetPoints() {
    pid_set_points[YAW]   = calculateYawSetPoint(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[mode_mapping[PITCH]]);
    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], pulse_length[mode_mapping[ROLL]]);
if (iteration == PRINT_FREQ && DEBUG) {
	dtostrf(pid_set_points[YAW], 3, 6, float_buf[0]);
	dtostrf(pid_set_points[PITCH], 3, 6, float_buf[1]);
	dtostrf(pid_set_points[ROLL], 3, 6, float_buf[2]);
	printf("pid set points: %s, %s %s\r\n", float_buf[0], float_buf[1], float_buf[2]);
}
	
}

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

/**
 * Customize mapping of controls: set here which command is on which channel and call
 * this function in setup() routine.
 */
void configureChannelMapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

/**
 * Make sure that given value is not over min_value/max_value range.
 *
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 *
 * @return float
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 */
void pidController() {
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = pulse_length[mode_mapping[THROTTLE]];

    // Initialize motor commands with throttle
    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;

    // Do not calculate anything if throttle is 0
    if (throttle >= 1012) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
        yaw_pid   = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid  = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
    }

    // Prevent out-of-range-values
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
	
	if (iteration == PRINT_FREQ && DEBUG) {
		printf("pulse_length_esc: %ld, %ld, %ld, %ld\r\n", 
			pulse_length_esc1, pulse_length_esc2,
			pulse_length_esc3, pulse_length_esc4);
	}
}

/**
 * Calculate errors used by PID controller
 */
void calculateErrors() {
    // Calculate current errors
    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];

    // Calculate sum of errors : Integral coefficients
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Keep values in acceptable range
    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

    // Calculate error delta : Derivative coefficients
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}

// TODO
void applyMotorSpeed() {
	OCR0 = pulse_length_to_duty(pulse_length_esc1);
	OCR2 = pulse_length_to_duty(pulse_length_esc2);
	OCR1A = pulse_length_to_duty(pulse_length_esc3);
	OCR1B = pulse_length_to_duty(pulse_length_esc4);
if (DEBUG) {
	printf("OCR0: %d, OCR1A: %d, OCR1B: %d, OCR2: %d \r\n",
		pulse_length_to_duty(pulse_length_esc1), 
		pulse_length_to_duty(pulse_length_esc3), 
		pulse_length_to_duty(pulse_length_esc4), 
		pulse_length_to_duty(pulse_length_esc2));
}
}




void PWM0_init()
{
	// set fast PWM mode with non-inverted output
	TCCR0  = (1 << WGM00) | (1 << WGM01); // Fast PWM mode is enabled.
	TCCR0 |= (1 << COM01); // Non-inverted wave is generated.
	// TCCR0 |= (1 << CS00); // Prescaler is 1.
	TCCR0 |= (1 << CS00) | (1<<CS01); // Prescaler is 64 - working @ 1 MHz
	// TCCR0 |= (1 << CS02); // Prescaler is 256.
	DDRB  |= (1 << PB3);  // OC0 pin is output.
}


void PWM2_init()
{
	// set fast PWM mode with non-inverted output
	TCCR2  = (1 << WGM20) | (1 << WGM21); // Fast PWM mode is enabled.
	TCCR2 |= (1 << COM21); // Non-inverted wave is generated.
	TCCR2 |= (1 << CS22); // Prescaler is 64 - working @ 1 MHz
	DDRD  |= (1 << PD7);  // OC2 pin is output.
}

void PWM1_init()
{
	// set fast PWM mode with non-inverted output
	TCCR1A  = (1 << WGM10); // Fast PWM mode is enabled, 8-bit.
	TCCR1B  = (1 << WGM12); // Fast PWM mode is enabled, 8-bit.
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Non-inverted wave is generated.
	TCCR1B  |= (1 << CS10) | (1 << CS11); // Prescaler is 64 - working @ 1 MHz
	DDRD   |= (1 << PD5);  // OC1A pin is output.
	DDRD   |= (1 << PD4);  // OC1B pin is output.
}

void applyOCR() {
	int a = 17;
	// scanf("%d", &a);
	printf("OCR set to %d.\r\n", a);
	OCR0 = a;
	OCR2 = a;
	OCR1A = a;
	OCR1B = a;
}

int UART_receive() {
	while((UCSRA & (1<<RXC)) == 0);
	return UDR;
}

volatile int joystick[4] = {150, 150, 150, 150};
	
#define BUF_SIZE 30

volatile int duty_base = 62;
volatile uint8_t a;
volatile int increase = 0;

static  void setup();

void read_rf_module() {
	int rf_read_time = 5;
	while (--rf_read_time) {
		//printf("rf started\r\n");
		char buffer[BUF_SIZE];
		scanf("%29s", buffer);
		buffer[BUF_SIZE-1] = '\0';
		//printf("buffer: %s \r\n", buffer);
		
		char *p;
		char labels[5][4] = {"ROL", "PIT", "YAW", "THR", "STP"};
		p =  strstr(buffer, labels[0]);
		int updated = 0;
		if (p && (p + 3 < buffer + BUF_SIZE)) {
			joystick[ROLL] = *(p + 3);
			updated = 1;
		}
		p =  strstr(buffer, labels[1]);
		if (p && (p + 3 < buffer + BUF_SIZE)) {
			joystick[PITCH] = *(p + 3);
			updated = 1;
		}
		p =  strstr(buffer, labels[2]);
		if (p && (p + 3 < buffer + BUF_SIZE)) {
			joystick[YAW] = *(p + 3);
			updated = 1;
		}
		p =  strstr(buffer, labels[3]);
		if (p && (p + 3 < buffer + BUF_SIZE)) {
			joystick[THROTTLE] = *(p + 3);
			updated = 1;
		}
		p =  strstr(buffer, labels[4]);
		if (p && (p + 3 < buffer + BUF_SIZE)) {
			if(*(p + 3) == '1') {
				printf("Resetting!!!\r\n");
				OCR0 = duty_base;
				OCR2 = duty_base;
				OCR1A = duty_base;
				OCR1B = duty_base;
				_delay_ms(30000);
				setup();
			}
			
		}
		
		if (updated) {
			printf("R %d,P %d,Y %d,T %d\r\n",
			joystick[ROLL], joystick[PITCH], joystick[YAW], joystick[THROTTLE]);
			
		}
		// 8-bit for 256 values, 256 * 4 = 1024; range(1000, 2000) ms
		if (joystick[ROLL] < 20) {
			pulse_length[CHANNEL1] -= 1;
		}
		else if (joystick[ROLL] > 240) {
			pulse_length[CHANNEL1] += 1;
		}
		
		if (joystick[PITCH] < 20) {
			pulse_length[CHANNEL2] -= 1;
		}
		else if (joystick[PITCH] > 240) {
			pulse_length[CHANNEL2] += 1;
		}
		
		if (joystick[YAW] < 20) {
			pulse_length[CHANNEL4] -= 1;
		}
		else if (joystick[YAW] > 240) {
			pulse_length[CHANNEL4] += 1;
		}
		
		if (joystick[THROTTLE] < 20) {
			pulse_length[CHANNEL3] -= 1;
		}
		else if (joystick[THROTTLE] > 240) {
			pulse_length[CHANNEL4] += 1;
		}
		
		// pulse_length[CHANNEL1] = 1000 + joystick[ROLL]*4;
		// pulse_length[CHANNEL2] = 1000 + joystick[PITCH]*4;
		// pulse_length[CHANNEL4] = 1000 + joystick[YAW]*4;
		// pulse_length[CHANNEL3] = 1000 + joystick[THROTTLE]*4; // 1000 - 2000;
		
		
		// _delay_ms(60);
	}
	//printf("radio read\r\n");
}

int battery_voltage = 0;

void ADC_Init()
{
	DDRA = 0x00;		/* Make ADC port as input */
	ADCSRA = 0x87;		/* Enable ADC, fr/128  */
	ADMUX = 0x40;		/* Vref: Avcc, ADC channel: 0 */
}


int ADC_Read(char channel)
{
	int ADC_value;
	
	ADMUX = (0x40) | (channel & 0x07);/* set input channel to read */
	ADCSRA |= (1<<ADSC);	/* start conversion */
	while((ADCSRA &(1<<ADIF))== 0);	/* monitor end of conversion interrupt flag */
	
	ADCSRA |= (1<<ADIF);	/* clear interrupt flag */
	ADC_value = (int)ADCL;	/* read lower byte */
	ADC_value = ADC_value + (int)ADCH*256;/* read higher 2 bits, Multiply with weightage */

	return ADC_value;		/* return digital value */
}

static void setup() {
	ADC_Init();
	I2C_Init();											/* Initialize I2C */
	MPU6050_Init();										/* Initialize MPU6050 */
	// USART_Init(9600);									/* Initialize USART with 9600 baud rate */
	FTDI_init();
	// _delay_ms(5000);
	FTDI_redirect_io();
	
	calibrateMpu6050();
	
	PWM0_init();
	PWM2_init();
	PWM1_init();
	
	// int duty_low = 17;
	// int duty_high = 32;
	// int duty_mid = 25;
	
	// int duty_low = 76;
	OCR0 = duty_base;
	OCR2 = duty_base;
	OCR1A = duty_base;
	OCR1B = duty_base;
	// stops at 62, 76 (start), stop 124
	_delay_ms(100);
	
	
	
	a = duty_base;
	increase = 0;
	printf("setup done\r\n");
	
}




int main()
{
	// char buffer[20];
	setup();
	
	float Xa,Ya,Za,t;
	float Xg=0,Yg=0,Zg=0;
	
	
	while (++iteration) {
		// _delay_ms(1000);
		// _delay_ms(10);
		//if (iteration == PRINT_FREQ) {
			//printf("\r\n--------------------------\r\n");
		//}
		printf("\r\n--------------------------\r\n");
		
		read_rf_module();	
		
	//goto flying_mode;	
		
		
		if (joystick[THROTTLE] < 20) {
			a -= 1;
			increase = 0;
		}
		else if (joystick[THROTTLE] > 240) {
			a += 1;
			increase = 1;
		}
		
		if (increase) {
			if (a > 70 && a < 77) {
				a = 77;
			}
			else if (a < 77) {
				a = 70;
			}			
		}
		else {
			if (a < 77) {
				a = 70;
			}			
		}		
		
		printf("%d ", a);
		
		OCR0 = a;
		OCR2 = a;
		OCR1A = a;
		OCR1B = a;
		
		continue;
		
		//while (1) {
			//char buffer[100];
			//scanf("%90s", buffer);
			//buffer[99] = '\0';
			//printf("buffer: %s \r\n", buffer);
		//}
		//
		//printf("Yes!");
		//_delay_ms(10000);
	flying_mode:
		//battery_voltage = ADC_Read(0);
		//printf("bat[%d]\r\n", battery_voltage);
		
		MPU6050_Read_RawValue();
		gyro_raw[X] = Gyro_x;
		gyro_raw[Y] = Gyro_y;
		gyro_raw[Z] = Gyro_z;
		acc_raw[X] = Acc_x;
		acc_raw[Y] = Acc_y;
		acc_raw[Z] = Acc_z;
		
		 calculateGyroAngles();	
		 calculateAccelerometerAngles();
		
		 calculateAngles();
		 calculateSetPoints();
		 // printf("Here\r\n");
		
		 calculateErrors();
		
		 pidController();
		
		 applyMotorSpeed();
		
		
		if (iteration == PRINT_FREQ) {
			iteration = 0;
			
			//applyOCR();
		}
	}
	

	
	while(1)
	{
		MPU6050_Read_RawValue();
		gyro_raw[X] = Gyro_x;
		gyro_raw[Y] = Gyro_y;
		gyro_raw[Z] = Gyro_z;
		printf("gyro raw: %d, %d, %d \r\n", gyro_raw[X], gyro_raw[Y], gyro_raw[Z]);

		Xa = Acc_x/16384.0;								/* Divide raw value by sensitivity scale factor to get real values */
		Ya = Acc_y/16384.0;
		Za = Acc_z/16384.0;
		
		Xg = Gyro_x/16.4;
		Yg = Gyro_y/16.4;
		Zg = Gyro_z/16.4;

		t = (Temperature/340.00)+36.53;					/* Convert temperature in �/c using formula */


		dtostrf( Xa, 3, 2, float_ );					/* Take values in buffer to send all parameters over USART */
		// sprintf(buffer," Ax = %s g\t",float_);
		// printf(" Ax = %s g  ",float_);
		// USART_SendString(buffer);

		dtostrf( Ya, 3, 2, float_ );
		// sprintf(buffer," Ay = %s g\t",float_);
		//USART_SendString(buffer);
		// printf(" Ay = %s g  ",float_);
		
		dtostrf( Za, 3, 2, float_ );
		// sprintf(buffer," Az = %s g\t",float_);
		//USART_SendString(buffer);
		// printf(" Az = %s g  ",float_);

		dtostrf( t, 3, 2, float_ );
		// sprintf(buffer," T = %s%cC\t",float_,0xF8);           /* 0xF8 Ascii value of degree '�' on serial */
		//USART_SendString(buffer);
		// printf(" T = %s deg C  ",float_);   

		dtostrf( Xg, 3, 2, float_ );
		// sprintf(buffer," Gx = %s%c/s\t",float_,0xF8);
		//USART_SendString(buffer);
		// printf(" Gx = %s deg/s\t",float_);

		dtostrf( Yg, 3, 2, float_ );
		// printf(" Gy = %s deg/s  ",float_);
		//USART_SendString(buffer);
		// sprintf(buffer," Gy = %s%c/s\t",float_,0xF8);
		
		dtostrf( Zg, 3, 2, float_ );
		// sprintf(buffer," Gz = %s%c/s\r\n",float_,0xF8);
		//USART_SendString(buffer);
		// printf(" Gz = %s deg /s\r\n",float_);
		_delay_ms(1000);
	}
}
