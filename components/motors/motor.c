#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"

#include "motor.h"
#include "MPU6050.h"
#include "drone_err.h"
#include "motor_err.h"


drone_err_t motor_init(motor_handle_t *motor, ledc_channel_t channel, uint8_t gpio)
{
  ledc_channel_config_t ledc_channel = {
    .speed_mode     = LEDC_MODE,
    .channel        = channel,
    .timer_sel      = LEDC_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = gpio,
    .duty           = 3266,
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  motor->channel = channel;
  motor->gpio = gpio;

  return DRONE_OK;
}

// intitialises drone_motor_controller_t with motors and timer
drone_err_t drone_motors_init(drone_motor_controller_t *drone) 
{
  // timer init
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = LEDC_FREQUENCY,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  drone_err_t err;
  // init motors
  err = motor_init(&drone->front_right_motor, LEDC_CHANNEL_0, FRONT_RIGHT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_init(&drone->front_left_motor, LEDC_CHANNEL_1, FRONT_LEFT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_init(&drone->back_right_motor, LEDC_CHANNEL_2, BACK_RIGHT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_init(&drone->back_left_motor, LEDC_CHANNEL_3, BACK_LEFT_MOTOR_PIN);
  if (err != DRONE_OK) {
    return err;
  }

  return DRONE_OK;
}


drone_err_t motor_set_pulse(motor_handle_t *motor, uint16_t pulse_us)
{
  // might change this, to turn the motor off might possibly have to be less than 1000
  if (pulse_us < 1000 || pulse_us > 2000) {
    return MOTOR_INVALID_PULSE_LEN;
  }

  int duty = (pulse_us * ((1 << LEDC_RES_INT) - 1)) / LEDC_PERIOD_US;

  // set pulse
  ESP_ERROR_CHECK(ledc_set_duty_and_update(LEDC_MODE, motor->channel, duty, 0));
  return DRONE_OK;
}

// ESCs have to calibrated at least once
drone_err_t drone_motors_calibrate(drone_motor_controller_t *drone) 
{
  // set max
  drone_err_t err;
  err = motor_set_pulse(&drone->front_right_motor, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->front_left_motor,MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_right_motor, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_left_motor, MOTOR_MAX_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  // wait 3 secs
  vTaskDelay(pdMS_TO_TICKS(3000));
  // set min
  err = motor_set_pulse(&drone->front_right_motor, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->front_left_motor, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_right_motor, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  err = motor_set_pulse(&drone->back_left_motor, MOTOR_MIN_PULSE_PERIOD);
  if (err != DRONE_OK) {
    return err;
  }
  // wait 3 secs
  vTaskDelay(pdMS_TO_TICKS(3000));

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

