#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "joysticks.h"
#include "drone_err.h"
#include "esp_err.h"


drone_err_t init_ADC1 (joystick_handle_t *joysticks) {
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


drone_err_t readControls(joystick_handle_t *joysticks) {
	assert(joysticks != NULL);
  struct ControlData joystickData; 

  // right stick up/down
  esp_err_t err = adc_oneshot_read(joysticks->adc, rightStickY, &joystickData.verticalSpeed); //reads value to joystickData.verticalSpeed
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  // right stick left/right
  err = adc_oneshot_read(joysticks->adc, rightStickX, &joystickData.turnSpeed);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  // left stick up/down
  err = adc_oneshot_read(joysticks->adc, leftStickY, &joystickData.forwardSpeed);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}
  // left stick left/right
  err = adc_oneshot_read(joysticks->adc, leftStickX, &joystickData.rightSpeed);
	if (err != ESP_OK) {
		return JOYSTICK_READ_FAIL;
	}

  // currently no button on controller
  joystickData.button = 0;
  
	// needs to consider race condition
	joysticks->data = joystickData;
	joysticks->dataIsNew = true;
	
  return DRONE_OK;
}


int16_t applyDeadzone(int rawValue, int center, int deadzone) {
  int16_t output = rawValue - center;

  if (abs(output) < deadzone) {
    return 0;
  } 

  if (output > 0) {
    return output - deadzone;
  } else {
    return output + deadzone;
  }
}
