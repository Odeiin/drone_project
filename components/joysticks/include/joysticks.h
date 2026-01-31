#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "control_protocol.h"
#include "drone_err.h"
#include "esp_err.h"

#define CENTER 2048

// joystick ADC1 inputs
extern const adc_channel_t leftStickX;
extern const adc_channel_t leftStickY;
extern const adc_channel_t rightStickX;
extern const adc_channel_t rightStickY;

// button GPIO input
// const int stopButton = 12; // DOESNT EXIST ON PROTOTYPE BOARD

// perhaps un-necessary
typedef struct{
	adc_oneshot_unit_handle_t adc;
  ControlData_t data;
} joystick_handle_t;

// CENTER SHOULD BE CALIBRATED each time the program starts I think, currently has static center
// applies a deadzone to values read from the joysticks as the values arent very accurate
int16_t applyDeadzone(int rawValue, int center, int deadzone);

// reads inputs from the controls and returns data in struct 
drone_err_t readControls(joystick_handle_t *joysticks);

// initialises ADC1 for reading inputs and sets config 
drone_err_t init_ADC1 (joystick_handle_t *joysticks);
