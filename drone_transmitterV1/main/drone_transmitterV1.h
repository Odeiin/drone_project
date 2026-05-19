#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "NRF24L01.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"


// RADIO CONSTANTS
#define RADIO_CE_PIN 4
#define RADIO_CSN_PIN 14
#define RADIO_IRQ_PIN 22

#define RADIO_CHANNEL 50
#define TELEMETRY_PERIOD_US 100000

static const NRF_addr_t RADIO_TX_ADDR = {0x1A, 0x1A, 0x1A, 0x1A, 0x1A};
static const NRF_addr_t RADIO_RX_ADDR = {0x50, 0x50, 0x50, 0x50, 0x50};


// --------------- TASKS ----------------------
// RTOS task for getting inputs
void readInputsTask(void *arg); 

// RTOS task for sending data to drone (and maybe server eventually)
void radioTask(void *arg);

// --------------- TASKS ----------------------



