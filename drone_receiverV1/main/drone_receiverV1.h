#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"


// --------------- TASKS ----------------------
// RTOS task, receives data from radio
void getDataTask(void *arg);

// RTOS task, gets data from inertial measurement unit (IMU), should read and filter data for use when controlling flight
void imuTask(void *arg);

// --------------- TASKS ----------------------