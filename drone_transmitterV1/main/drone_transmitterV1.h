#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"









// --------------- TASKS ----------------------
// RTOS task for getting inputs
void readInputsTask(void *arg); 

// RTOS task for sending data to drone (and maybe server eventually)
void sendDataTask(void *arg);

// --------------- TASKS ----------------------



