#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"


// joystick ADC1 inputs
const adc_channel_t leftStickX = ADC_CHANNEL_0; // pin 36
const adc_channel_t leftStickY = ADC_CHANNEL_3; // pin 39
const adc_channel_t rightStickX = ADC_CHANNEL_7; // pin 35
const adc_channel_t rightStickY = ADC_CHANNEL_6; // pin 34
// button GPIO input
// const int stopButton = 12; // DOESNT EXIST ON PROTOTYPE BOARD

struct ControlData {  
  int verticalSpeed; // +ve = Acsend, -ve = Descend     
  int forwardSpeed; // +ve = Forward, -ve = Backwards      
  int rightSpeed; // +ve = Right, -ve = Left
  int turnSpeed; // +ve = clockwise, -ve = anti-clockwise
  bool button; // button input
};

// --------------- TASKS ----------------------
// RTOS task for getting inputs
void readInputsTask(void *arg); 

// RTOS task for sending data to drone (and maybe server eventually)
void sendDataTask(void *arg);

// --------------- TASKS ----------------------


// CENTER SHOULD BE CALIBRATED each time the program starts I think, currently has static center
// applies a deadzone to values read from the joysticks as the values arent very accurate
int16_t applyDeadzone(int rawValue, int center, int deadzone);

// reads inputs from the controls and returns data in struct 
struct ControlData readControls(void);

// initialises ADC1 for reading inputs and sets config 
void init_ADC1 (void);

