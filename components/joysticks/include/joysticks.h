#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "drone_err.h"
#include "esp_err.h"

// joystick ADC1 inputs
const adc_channel_t leftStickX = ADC_CHANNEL_0; // pin 36
const adc_channel_t leftStickY = ADC_CHANNEL_3; // pin 39
const adc_channel_t rightStickX = ADC_CHANNEL_7; // pin 35
const adc_channel_t rightStickY = ADC_CHANNEL_6; // pin 34

// button GPIO input
// const int stopButton = 12; // DOESNT EXIST ON PROTOTYPE BOARD

// change type to like 16 bit or something
typedef struct {  
  int verticalSpeed; // +ve = Acsend, -ve = Descend     
  int forwardSpeed; // +ve = Forward, -ve = Backwards      
  int rightSpeed; // +ve = Right, -ve = Left
  int turnSpeed; // +ve = clockwise, -ve = anti-clockwise
  bool button; // button input

  bool dataIsNew;
} ControlData_t;


typedef struct{
	adc_oneshot_unit_handle_t adc;
  ControlData_t data;
} joystick_handle_t;


// CENTER SHOULD BE CALIBRATED each time the program starts I think, currently has static center
// applies a deadzone to values read from the joysticks as the values arent very accurate
int16_t applyDeadzone(int rawValue, int center, int deadzone);

// reads inputs from the controls and returns data in struct 
struct ControlData readControls(void);

// initialises ADC1 for reading inputs and sets config 
void init_ADC1 (void);
