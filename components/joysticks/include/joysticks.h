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

//#define CENTER 2048
#define JOYSTICK_CALIBRATION_SAMPLES 101 // odd number for eas as im finding median
#define JOYSTICK_CALIBRATION_SAMPLE_CENTER 51
#define JOYSTICK_DEADZONE 100
#define JOYSTICK_FILTER_ALPHA 0.05
#define JOYSTICK_DEADBAND 300
#define JOYSTICK_MAX_STEP 100

#define BUTTON_DEBOUNCE_PERIOD_US 300000 

// button GPIO input
#define buttonGPIO 25

// joystick ADC1 inputs
extern const adc_channel_t leftStickX; // left/right
extern const adc_channel_t leftStickY; // forward/back
extern const adc_channel_t rightStickX; // turn
extern const adc_channel_t rightStickY; // vertical

// perhaps un-necessary
typedef struct{
	adc_oneshot_unit_handle_t adc;
  ControlData_t data;
  int16_t center_vertical;
  int16_t center_forward;
  int16_t center_right;
  int16_t center_turn;
} joystick_handle_t;

// initialises ADC1 for reading inputs and sets config 
drone_err_t init_joysticks(joystick_handle_t *joysticks);

// choosing to use median instead of average as there is sometimes random outlier values
drone_err_t joysticks_calibrate(joystick_handle_t *joysticks);

// reads inputs from the controls and returns data in struct 
drone_err_t readControls(joystick_handle_t *joysticks);




