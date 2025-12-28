#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "drone_transmitterV1.h"
#include "NRF24L01.h"
#include "drone_err.h"
#include "NRF_err.h"


// --------- globals -------------------------------

ControlData_t controlDataGlobal = {
  .forwardSpeed = 0,
  .rightSpeed = 0,
  .verticalSpeed = 0,
  .turnSpeed = 0,
  .button = false,
  .dataIsNew = false
};

// --------- globals -------------------------------


void app_main(void)
{

  // i think the reading task should probably be higher priority than the sending data task
  // if the control data isnt updated sending doesnt even matter anyway
  //xTaskCreate(readInputsTask, "reading inputs", 2500, NULL, 1, NULL);

  xTaskCreate(sendDataTask, "sending data", 8192, NULL, 1, NULL); // not sure about mem size

}


void sendDataTask(void *arg) {
  NRF_addr_t txAddr = {0x1A, 0x1A, 0x1A, 0x1A, 0x1A};
  NRF_addr_t rxAddr = {0x50, 0x50, 0x50, 0x50, 0x50};
  NRF_channel_t channel = 50;
  NRF_handle_t radio;  
  drone_err_t err = NRF_init(&radio, rxAddr, txAddr, channel);
  assert(err == DRONE_OK);
  
  //int64_t TXstartTime_us = esp_timer_get_time();  
  for (;;) {

    if (radio.state == standby) {
      ControlData_t packet = controlDataGlobal; // needs to consider race condition

      err = NRF_push_packet(radio, (const uint8_t *)&packet, sizeof(packet)); // verify use of sizeof is correct
      assert(err != NRF_INVALID_PACKET_LEN); 

      // i think the busy wait is the only real way to do this
      err = NRF_pulse_TXmode(radio);
      assert(err == DRONE_OK);
    }

    vTaskDelay(pdMS_TO_TICKS(2)); // 500hz
  }
}


void readInputsTask(void *arg) {
  joystick_handle_t joysticks;
  drone_err_t err = init_ADC1(&joysticks);
  assert(err == DRONE_OK);

  joystick_handle_t
  for (;;) {
    controlDataGlobal = readControls();

    // int f = controlDataGlobal.forwardSpeed;
    // int r = controlDataGlobal.rightSpeed;
    // int v = controlDataGlobal.verticalSpeed;
    // int t = controlDataGlobal.turnSpeed;
    // int b = controlDataGlobal.button;
    //printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);

    // occasionally print stack headroom
    // static int counter = 0;
    // if ((++counter % 200) == 0) { // every ~1 s here
    //   UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
    //   size_t free_bytes = hw * sizeof(StackType_t);
    //   printf("[inputs] stack free min = %u bytes\n", (unsigned)free_bytes);
    // }

    vTaskDelay(pdMS_TO_TICKS(2)) // 500 hz 
  }
}



