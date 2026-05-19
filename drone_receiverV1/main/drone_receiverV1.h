#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "NRF24L01.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

// DRONE CONSTANTS

// determines the possible range of angles the drone can be set to
// if you want it to go faster some direction it needs to be able to tilt to a larger angle
#define DRONE_ANGLE_RANGE 5.0f // in degrees
// the range of values that can be input by the controller 
#define CONTROLLER_INPUT_RANGE 2000.0f  // 2000 is currently just the approximate range of the joysticks (-+2000)
// factor used to convert controller input into target angle
#define INPUT_TO_ANGLE_FACTOR (DRONE_ANGLE_RANGE / CONTROLLER_INPUT_RANGE)
// range of values where the controller input is a deadband
#define CONTROLLER_DEADBAND 300

// control input response factors, larger is high response to input
// controls how responsive the drone is to controller input
#define VERT_RESPONSE_FACTOR 0.05
#define TURN_RESPONSE_FACTOR 0.1
#define ROLL_RESPONSE_FACTOR 1.0
#define PITCH_RESPONSE_FACTOR 1.0


// RADIO CONSTANTS
#define RADIO_CE_PIN 4
#define RADIO_CSN_PIN 14
#define RADIO_IRQ_PIN 22

#define RADIO_CHANNEL 50
#define TELEMETRY_PERIOD_US 100000

static const NRF_addr_t RADIO_RX_ADDR = {0x1A, 0x1A, 0x1A, 0x1A, 0x1A};
static const NRF_addr_t RADIO_TX_ADDR = {0x50, 0x50, 0x50, 0x50, 0x50};


// --------------- TASKS ----------------------
// RTOS task, receives data from radio and sends telemetry data
void radioTask(void *arg);

// RTOS task, gets data from inertial measurement unit (IMU), should read and filter data for use when controlling flight
void imuTask(void *arg);


void flight_control_task(void *arg);

// --------------- TASKS ----------------------