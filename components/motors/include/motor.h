#pragma once

#include "driver/ledc.h"

#include "MPU6050.h"
#include "drone_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_15_BIT 	// Set duty resolution to 15 bits
#define LEDC_RES_INT			(15)				// Set duty resolution to 15 bits		
#define LEDC_FREQUENCY          (50) // 50Hz
#define LEDC_PERIOD_US			(1000000 / LEDC_FREQUENCY)

#define FRONT_RIGHT_MOTOR_PIN           33
#define FRONT_LEFT_MOTOR_PIN            32 
#define BACK_RIGHT_MOTOR_PIN            25 
#define BACK_LEFT_MOTOR_PIN             26 

#define MOTOR_MAX_PULSE_PERIOD			2000 // 2ms pulse
#define MOTOR_MIN_PULSE_PERIOD			1000 // 1ms pulse


#define PID_DEFAULT_CONFIG { \
    .Kp = 1.0f, \
    .Ki = 0.0f, \
    .Kd = 0.0f, \
	.roll_err_buffer = 1.0f, \
	.pitch_err_buffer = 1.0f, \
	.roll_int_term = 1.0f, \
	.pitch_int_term = 1.0f \
}

typedef struct {
	ledc_channel_t channel;
	uint8_t gpio;
} motor_handle_t;

typedef struct {
	ledc_timer_t timer;
	motor_handle_t front_right_motor;
	motor_handle_t front_left_motor;
	motor_handle_t back_right_motor;
	motor_handle_t back_left_motor;
} drone_motor_controller_t;

// struct that is used for controlling PID calculation and keeping data between calls
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	// for differentiation calculation
	float roll_err_buffer;
	float pitch_err_buffer;
	// for integration calculation
	float roll_int_term;
	float pitch_int_term;
} PID_state_t;

// PID calculation output used for motor control
typedef struct {
	float roll_correction;
	float pitch_correction;
} PID_angle_correction_t;



drone_err_t motor_init(motor_handle_t *motor, ledc_channel_t channel, uint8_t gpio);


drone_err_t drone_motors_init(drone_motor_controller_t *drone);


drone_err_t motor_set_pulse(motor_handle_t *motor, uint16_t pulse_us);


drone_err_t drone_motors_calibrate(drone_motor_controller_t *drone);


PID_angle_correction_t PID_angle_calculation(angle_data_t angle_data, angle_data_t target_angle, PID_state_t *state, float dt);



