#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "control_protocol.h"
#include "joysticks.h"
#include "drone_err.h"
#include "joysticks_err.h"
#include "esp_err.h"


// joystick ADC1 inputs
const adc_channel_t leftStickY = ADC_CHANNEL_0; // pin 36
const adc_channel_t leftStickX = ADC_CHANNEL_3; // pin 39
const adc_channel_t rightStickX = ADC_CHANNEL_7; // pin 35
const adc_channel_t rightStickY = ADC_CHANNEL_6; // pin 34


// deadzone is applied in both directions
int16_t applyDeadzone(int16_t rawValue, int16_t deadzone) {
  if (deadzone < 0) {
    deadzone = -deadzone;
  }

  if (abs(rawValue) < deadzone) {
    return 0;
  }

  if (rawValue > 0) {
    return rawValue - deadzone;
  } else {
    return rawValue + deadzone;
  }
}

// compare function for sorting
static int cmp_i16(const void *a, const void *b) {
  int16_t ia = *(const int16_t *)a;
  int16_t ib = *(const int16_t *)b;
  return (ia > ib) - (ia < ib);
}


drone_err_t init_joysticks(joystick_handle_t *joysticks) {
  // creates handle to ADC_UNIT_1
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};    
  esp_err_t err = adc_oneshot_new_unit(&init_config1, &joysticks->adc);
	if (err != ESP_OK) {
		return JOYSTICK_INIT_FAIL;
	}

  // config settings
  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
  };

  // set config for joystick inputs
  assert(adc_oneshot_config_channel(joysticks->adc, rightStickY, &config) == ESP_OK); //config settings for adc oneshot
  assert(adc_oneshot_config_channel(joysticks->adc, leftStickY, &config) == ESP_OK);
  assert(adc_oneshot_config_channel(joysticks->adc, leftStickX, &config) == ESP_OK);
  assert(adc_oneshot_config_channel(joysticks->adc, rightStickX, &config) == ESP_OK);

  // init button
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << buttonGPIO),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

  return DRONE_OK;
}

// choosing to use median instead of average as there is sometimes random outlier values
drone_err_t joysticks_calibrate(joystick_handle_t *joysticks) {
  int val;
  // store a list of joystick value for processing
  static int16_t vertical_vals[JOYSTICK_CALIBRATION_SAMPLES]; // static to minimize stack usage
  static int16_t forward_vals[JOYSTICK_CALIBRATION_SAMPLES];
  static int16_t right_vals[JOYSTICK_CALIBRATION_SAMPLES];
  static int16_t turn_vals[JOYSTICK_CALIBRATION_SAMPLES];

  // get samples
  for (int i = 0; i < JOYSTICK_CALIBRATION_SAMPLES; i++) {
    esp_err_t err = adc_oneshot_read(joysticks->adc, rightStickY, &val);
    if (err != ESP_OK) {
      return JOYSTICK_READ_FAIL;
    }
    vertical_vals[i] = (int16_t)val;

    err = adc_oneshot_read(joysticks->adc, rightStickX, &val);
    if (err != ESP_OK) {
      return JOYSTICK_READ_FAIL;
    }
    turn_vals[i] = (int16_t)val;

    err = adc_oneshot_read(joysticks->adc, leftStickY, &val);
    if (err != ESP_OK) {
      return JOYSTICK_READ_FAIL;
    }
    forward_vals[i] = (int16_t)val;

    err = adc_oneshot_read(joysticks->adc, leftStickX, &val);
    if (err != ESP_OK) {
      return JOYSTICK_READ_FAIL;
    }
    right_vals[i] = (int16_t)val;

    vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms delay
  }

  // sort arrays
  qsort(vertical_vals, JOYSTICK_CALIBRATION_SAMPLES, sizeof(vertical_vals[0]), cmp_i16);
  qsort(forward_vals, JOYSTICK_CALIBRATION_SAMPLES, sizeof(forward_vals[0]), cmp_i16);
  qsort(right_vals, JOYSTICK_CALIBRATION_SAMPLES, sizeof(right_vals[0]), cmp_i16);
  qsort(turn_vals, JOYSTICK_CALIBRATION_SAMPLES, sizeof(turn_vals[0]), cmp_i16);

  // joystick centered around median value
  joysticks->center_vertical = vertical_vals[JOYSTICK_CALIBRATION_SAMPLE_CENTER];
  joysticks->center_forward = forward_vals[JOYSTICK_CALIBRATION_SAMPLE_CENTER];
  joysticks->center_right = right_vals[JOYSTICK_CALIBRATION_SAMPLE_CENTER];
  joysticks->center_turn = turn_vals[JOYSTICK_CALIBRATION_SAMPLE_CENTER];
  return DRONE_OK;
}

static inline int16_t apply_deadband(int16_t value, int16_t deadband)
{
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

static inline int16_t limit_slew_rate(int16_t value, int16_t previous, int16_t max_step)
{
  int16_t delta = value - previous;

  if (delta > max_step) {
    return previous + max_step;
  }
    
  if (delta < -max_step) {
    return previous - max_step;
  }
    
  return value;
}

// reads inputs from the controls and returns data in struct
drone_err_t readControls(joystick_handle_t *joysticks) {
	assert(joysticks != NULL);
  ControlData_t joystickData;
  int raw; 

  // right stick up/down
  esp_err_t err = adc_oneshot_read(joysticks->adc, rightStickY, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  int16_t raw_verticalSpeed = applyDeadzone(raw - joysticks->center_vertical, JOYSTICK_DEADZONE);

  // right stick left/right
  err = adc_oneshot_read(joysticks->adc, rightStickX, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  int16_t raw_turnSpeed = applyDeadzone(raw - joysticks->center_turn, JOYSTICK_DEADZONE);

  // left stick up/down
  err = adc_oneshot_read(joysticks->adc, leftStickY, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  int16_t raw_forwardSpeed = applyDeadzone(raw - joysticks->center_forward, JOYSTICK_DEADZONE);

  // left stick left/right
  err = adc_oneshot_read(joysticks->adc, leftStickX, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  int16_t raw_rightSpeed = applyDeadzone(raw - joysticks->center_right, JOYSTICK_DEADZONE);


  // read button input and debounce
  static bool armed = false;
  static int last_state = 1;
  static int64_t last_press_time = 0;

  int state = gpio_get_level(buttonGPIO); 
  int64_t now = esp_timer_get_time();

  // detect falling edge with debounce
  if (last_state == 1 && state == 0) {
    if ((now - last_press_time) > BUTTON_DEBOUNCE_PERIOD_US) {
      armed = !armed; // toggle
      last_press_time = now;
    }
  }

  last_state = state;
  joystickData.button = armed;

  // joystick inputs are currently bad and noisy so im trying to improve the readings below

  // applying low pass filter then deadband to joystick inputs to make jittery inputs effect the drone less
  static float filtered_verticalSpeed = 0.0f;
  static float filtered_forwardSpeed  = 0.0f;
  static float filtered_rightSpeed    = 0.0f;
  static float filtered_turnSpeed     = 0.0f;
  
  filtered_verticalSpeed += JOYSTICK_FILTER_ALPHA * (raw_verticalSpeed - filtered_verticalSpeed);
  filtered_forwardSpeed += JOYSTICK_FILTER_ALPHA * (raw_forwardSpeed - filtered_forwardSpeed);
  filtered_rightSpeed += JOYSTICK_FILTER_ALPHA * (raw_rightSpeed - filtered_rightSpeed);
  filtered_turnSpeed += JOYSTICK_FILTER_ALPHA * (raw_turnSpeed - filtered_turnSpeed);

  filtered_verticalSpeed = apply_deadband(filtered_verticalSpeed, JOYSTICK_DEADBAND);
  filtered_forwardSpeed = apply_deadband(filtered_forwardSpeed, JOYSTICK_DEADBAND);
  filtered_rightSpeed = apply_deadband(filtered_rightSpeed, JOYSTICK_DEADBAND);
  filtered_turnSpeed = apply_deadband(filtered_turnSpeed, JOYSTICK_DEADBAND);
  
  static int16_t prev_vertical = 0;
  static int16_t prev_forward = 0;
  static int16_t prev_right = 0;
  static int16_t prev_turn = 0;

  // slew rate limit
  joystickData.verticalSpeed  = limit_slew_rate(filtered_verticalSpeed, prev_vertical, JOYSTICK_MAX_STEP);
  joystickData.forwardSpeed = limit_slew_rate(filtered_forwardSpeed, prev_forward, JOYSTICK_MAX_STEP);
  joystickData.rightSpeed  = limit_slew_rate(filtered_rightSpeed, prev_right, JOYSTICK_MAX_STEP);
  joystickData.turnSpeed  = limit_slew_rate(filtered_turnSpeed, prev_turn, JOYSTICK_MAX_STEP);

  prev_vertical = joystickData.verticalSpeed;
  prev_forward = joystickData.forwardSpeed;
  prev_right = joystickData.rightSpeed;
  prev_turn = joystickData.turnSpeed;

	joysticks->data = joystickData;	
  return DRONE_OK;
}


