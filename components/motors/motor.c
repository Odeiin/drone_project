#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/mcpwm_prelude.h"

#include "motor.h"
#include "MPU6050.h"
#include "drone_err.h"
#include "motor_err.h"


drone_err_t pwm_timer_init(pwm_timebase_t *timebase, uint32_t resolution, uint32_t period) 
{
	mcpwm_timer_handle_t timer_handle = NULL;
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = resolution,
    .period_ticks = period,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_handle));

	timebase->timer = timer_handle;
	timebase->resolution_hz = resolution;
	timebase->period_ticks = period;
	// if (err != ESP_OK) {
	// 	return err;
	// }

	// return ESP_OK;
	return DRONE_OK;
}


drone_err_t motor_add_operator(motor_handle_t *motor) 
{
  mcpwm_oper_handle_t operator = NULL;
  mcpwm_operator_config_t operator_config = {
    .group_id = 0,
  };

  if (motor->operator != NULL) {
    mcpwm_del_operator(motor->operator);
    motor->operator = NULL;
  }

  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, motor->timebase->timer));

  motor->operator = operator;

  return DRONE_OK;
}


drone_err_t motor_add_comparator(motor_handle_t *motor) 
{
	mcpwm_cmpr_handle_t comparator = NULL;
  mcpwm_comparator_config_t comparator_config = {
    .flags.update_cmp_on_tez = true,
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(motor->operator, &comparator_config, &comparator));

	if (motor->comparator != NULL) {
		mcpwm_del_comparator(motor->comparator);
	}
	motor->comparator = comparator;
	return DRONE_OK;
}

drone_err_t motor_add_generator(motor_handle_t *motor) 
{
	mcpwm_gen_handle_t generator = NULL;
  mcpwm_generator_config_t generator_config = {
    .gen_gpio_num = motor->gpio_num,
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(motor->operator, &generator_config, &generator));

	if (motor->generator != NULL) {
		mcpwm_del_generator(motor->generator);
	}
	motor->generator = generator;
	return DRONE_OK;
}


drone_err_t motor_init(motor_handle_t *motor, pwm_timebase_t *timebase, int gpio_num) 
{
	motor->timebase = timebase;
	motor->gpio_num = gpio_num;
	motor_add_operator(motor);
	motor_add_comparator(motor); 
	motor_add_generator(motor);

  // go high on counter empty
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor->generator,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  // go low on compare threshold
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor->generator,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator, MCPWM_GEN_ACTION_LOW)));

	return DRONE_OK;																														
}

// pulse length is in ticks
drone_err_t motor_set_pulse(motor_handle_t *motor, pwm_timebase_t *timebase, uint16_t pulse_length)
{
  if ((pulse_length > timebase->period_ticks) || (pulse_length == 0)) {
    return MOTOR_INVALID_PULSE_LEN;
  }

  mcpwm_comparator_set_compare_value(motor->comparator, pulse_length);

  return DRONE_OK;
}


PID_angle_correction_t PID_angle_calculation(angle_data_t angle_data, angle_data_t target_angle, PID_state_t *state, float dt) 
{
  float roll_error = target_angle.roll - angle_data.roll;
  float pitch_error = target_angle.pitch - angle_data.pitch;

  // proportional terms
  float roll_prop_term = state->Kp * roll_error;
  float pitch_prop_term = state->Kp * pitch_error;

  // integral terms
  state->roll_int_term += roll_error * dt;
  state->pitch_int_term += pitch_error * dt;

  // differentiation terms
  float roll_diff_term = state->Kd * (roll_error - state->roll_err_buffer) / dt;
  float pitch_diff_term = state->Kd * (pitch_error - state->pitch_err_buffer) / dt;

  // store previous errors for next function call
  state->roll_err_buffer = roll_error;
  state->pitch_err_buffer = pitch_error;

  PID_angle_correction_t correction = {
    .roll_correction = roll_prop_term + (state->Ki * state->roll_int_term) + roll_diff_term,
    .pitch_correction = pitch_prop_term + (state->Ki * state->pitch_int_term) + pitch_diff_term,
  };

  return correction;
}

// ESCs have to calibrated at least once
drone_err_t drone_motors_calibrate(drone_motor_controller_t *drone) 
{
  // set max
  drone_err_t err;
  err = motor_set_pulse(&drone->front_right_motor, &drone->timebase, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->front_left_motor, &drone->timebase, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_right_motor, &drone->timebase, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_left_motor, &drone->timebase, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  // wait 3 secs
  vTaskDelay(pdMS_TO_TICKS(3000));
  // set min
  err = motor_set_pulse(&drone->front_right_motor, &drone->timebase, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->front_left_motor, &drone->timebase, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_right_motor, &drone->timebase, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_left_motor, &drone->timebase, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  // wait 3 secs
  vTaskDelay(pdMS_TO_TICKS(3000));

  return DRONE_OK;
}

// intitialises drone_motor_controller_t with motors and timer
drone_err_t drone_motors_init(drone_motor_controller_t *drone) 
{
  drone_err_t err;
  // init timebase
  err = pwm_timer_init(&drone->timebase, MOTOR_TIMEBASE_RESOLUTION_HZ, MOTOR_TIMEBASE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }

  // init motors
  err = motor_init(&drone->front_right_motor, &drone->timebase, FRONT_RIGHT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_init(&drone->front_left_motor, &drone->timebase, FRONT_LEFT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_init(&drone->back_right_motor, &drone->timebase, BACK_RIGHT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_init(&drone->back_left_motor, &drone->timebase, BACK_LEFT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }

  // enable timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(drone->timebase.timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(drone->timebase.timer, MCPWM_TIMER_START_NO_STOP));

  return DRONE_OK;
}