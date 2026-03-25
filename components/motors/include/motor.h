#pragma once

#include "driver/mcpwm_prelude.h"

#include "MPU6050.h"
#include "drone_err.h"



#define FRONT_RIGHT_MOTOR_PIN           33
#define FRONT_LEFT_MOTOR_PIN            32 
#define BACK_RIGHT_MOTOR_PIN            25 
#define BACK_LEFT_MOTOR_PIN             26 

#define MOTOR_TIMEBASE_RESOLUTION_HZ 	1000000  // 1MHz, 1us per tick
#define MOTOR_TIMEBASE_PERIOD        	20000    // 20000 ticks, 20ms
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
	mcpwm_timer_handle_t timer;
	uint32_t resolution_hz;
	uint32_t period_ticks;
} pwm_timebase_t;

typedef struct {
	pwm_timebase_t *timebase;
	mcpwm_oper_handle_t operator;
	mcpwm_cmpr_handle_t comparator;
	mcpwm_gen_handle_t generator;
	int gpio_num;
} motor_handle_t;

typedef struct {
	pwm_timebase_t timebase;
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


// sets motor timer, resolution is in hz and period in ticks
// example: resolution = 1 MHz, period = 20000 ticks -> 1 tick = 1 µs, period = 20000 µs, PWM frequency = 50 Hz
drone_err_t pwm_timer_init(pwm_timebase_t *timebase, uint32_t resolution, uint32_t period);


drone_err_t motor_add_operator(motor_handle_t *motor);


drone_err_t motor_add_comparator(motor_handle_t *motor);


drone_err_t motor_add_generator(motor_handle_t *motor);


drone_err_t motor_init(motor_handle_t *motor, pwm_timebase_t *timebase, int gpio_num);


drone_err_t motor_set_pulse(motor_handle_t *motor, pwm_timebase_t *timebase, uint16_t pulse_length);


PID_angle_correction_t PID_angle_calculation(angle_data_t angle_data, angle_data_t target_angle, PID_state_t *state, float dt);


drone_err_t drone_motors_calibrate(drone_motor_controller_t *drone);


drone_err_t drone_motors_init(drone_motor_controller_t *drone);
