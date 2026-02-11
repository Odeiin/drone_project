#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "control_protocol.h"
#include "joysticks.h"
#include "drone_err.h"
#include "joysticks_err.h"
#include "esp_err.h"


// joystick ADC1 inputs
const adc_channel_t leftStickX = ADC_CHANNEL_0; // pin 36
const adc_channel_t leftStickY = ADC_CHANNEL_3; // pin 39
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
  joystickData.verticalSpeed = applyDeadzone(raw - joysticks->center_vertical, JOYSTICK_DEADZONE);

  // right stick left/right
  err = adc_oneshot_read(joysticks->adc, rightStickX, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  joystickData.turnSpeed = applyDeadzone(raw - joysticks->center_turn, JOYSTICK_DEADZONE);

  // left stick up/down
  err = adc_oneshot_read(joysticks->adc, leftStickY, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  joystickData.forwardSpeed = applyDeadzone(raw - joysticks->center_forward, JOYSTICK_DEADZONE);

  // left stick left/right
  err = adc_oneshot_read(joysticks->adc, leftStickX, &raw);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  joystickData.rightSpeed = applyDeadzone(raw - joysticks->center_right, JOYSTICK_DEADZONE);

  // currently no button on controller
  joystickData.button = 0;

	joysticks->data = joystickData;	
  return DRONE_OK;
}


