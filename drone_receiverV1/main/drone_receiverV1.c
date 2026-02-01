#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "drone_receiverV1.h"
#include "NRF24L01.h"
#include "joysticks.h"
#include "control_protocol.h"

#include "drone_err.h"
#include "NRF_err.h"


// --------- globals -------------------------------


// --------- globals -------------------------------


void app_main(void)
{

  // NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  vTaskDelay(pdMS_TO_TICKS(1000)); 

  xTaskCreate(getDataTask, "receiving data", 8192, NULL, 1, NULL); // not sure about mem size
}


void getDataTask(void *arg) {
  uint8_t ce_pin = 4;
  uint8_t csn_pin = 14;
  uint8_t irq_pin = 22;

  // --------------------
  // you should use IRQ later so remember to configure it as input
  // GPIO init should be moved to an init function when i refactor NRF_init
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << ce_pin),      // Select GPIO 2
    .mode = GPIO_MODE_OUTPUT,            // Set as output
    .pull_up_en = GPIO_PULLUP_DISABLE,  // Disable pull-up
    .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
    .intr_type = GPIO_INTR_DISABLE             // Disable interrupts
  };
  gpio_config(&io_conf);
  // --------------------

  NRF_addr_t rxAddr = {0x1A, 0x1A, 0x1A, 0x1A, 0x1A};
  NRF_addr_t txAddr = {0x50, 0x50, 0x50, 0x50, 0x50};
  NRF_channel_t channel = 50;
  NRF_handle_t radio;  
  
  drone_err_t err = NRF_init(&radio, ce_pin, csn_pin, irq_pin);
  assert(err == DRONE_OK);
  err = NRF_set_address(&radio, rxAddr, txAddr);
  assert(err == DRONE_OK);
  err = NRF_set_packet_length(&radio, sizeof(ControlData_t));
  assert(err == DRONE_OK);
  err = NRF_set_data_rate(&radio, DATA_RATE_250KBPS);
  assert(err == DRONE_OK);
  err = NRF_set_power_level(&radio, MIN);
  assert(err == DRONE_OK);
  err = NRF_set_channel(&radio, channel);
  assert(err == DRONE_OK);

  err = NRF_enter_RXmode(&radio);
  assert(err  == DRONE_OK);
  for (;;) {
    ControlData_t packet = {0};
    err = NRF_read_Fifo(&radio, (uint8_t *)&packet, sizeof(packet));
    if (err == NRF_EMPTY_RXFIFO) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    assert(err == DRONE_OK);

    // int f = packet.forwardSpeed;
    // int r = packet.rightSpeed;
    // int v = packet.verticalSpeed;
    // int t = packet.turnSpeed;
    // int b = packet.button;
    // printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);   
    
    
    // handle data if received properly

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}



