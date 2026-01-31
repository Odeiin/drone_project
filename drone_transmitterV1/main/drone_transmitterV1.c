#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "drone_transmitterV1.h"
#include "NRF24L01.h"
#include "joysticks.h"
#include "control_protocol.h"

#include "drone_err.h"
#include "NRF_err.h"

// --------- globals -------------------------------

// freeRTOS queue for communication between tasks
QueueHandle_t dataQueue;


// ControlData_t controlDataGlobal = {
//   .forwardSpeed = 0,
//   .rightSpeed = 0,
//   .verticalSpeed = 0,
//   .turnSpeed = 0,
//   .button = false,
//   .dataIsNew = false
// };

// --------- globals -------------------------------


void app_main(void)
{
  // --------------------
  // you should use IRQ later so remember to configure it as input
  // GPIO init should be moved to an init function when i refactor NRF_init
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << CE_pin),      // Select GPIO 2
    .mode = GPIO_MODE_OUTPUT,            // Set as output
    .pull_up_en = GPIO_PULLUP_DISABLE,  // Disable pull-up
    .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
    .intr_type = GPIO_INTR_DISABLE             // Disable interrupts
  };
  gpio_config(&io_conf);
  // --------------------

  // NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  vTaskDelay(pdMS_TO_TICKS(1000)); 

  // length 1 queue acts as buffer for control data between tasks
  // only length 1 as only the most recent data matters 
  dataQueue = xQueueCreate(1, sizeof(ControlData_t));
  assert(dataQueue != NULL);

  // i think the reading task should maybe be higher priority than the sending data task
  // if the control data isnt updated sending doesnt even matter anyway
  xTaskCreate(readInputsTask, "reading inputs", 2500, NULL, 1, NULL);

  xTaskCreate(sendDataTask, "sending data", 8192, NULL, 1, NULL); // not sure about mem size

}


void sendDataTask(void *arg) {

  // should make an NRF_responding function or something to check it works at the start of each program

  NRF_addr_t txAddr = {0x1A, 0x1A, 0x1A, 0x1A, 0x1A};
  NRF_addr_t rxAddr = {0x50, 0x50, 0x50, 0x50, 0x50};
  NRF_channel_t channel = 50;
  NRF_handle_t radio;  
  drone_err_t err = NRF_init(&radio, rxAddr, txAddr, channel, sizeof(ControlData_t));
  assert(err == DRONE_OK);
  
  for (;;) {
    // if think this could be necessary to track the radio state if theres another task which used for receiving data but pointless currently
    // probably shouldnt have multiple tasks interacting with the radio though
    // if (radio.state != standby) {
    //   something
    // }


    uint8_t txBuffer[1 + 5];
    uint8_t rxBuffer[1 + 5];
    // read status
    txBuffer[0] = CMD_R_REG | 0x07;
    txBuffer[1] = CMD_NOP;
    SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);
    printf("status %02X, %02X\n", rxBuffer[0], rxBuffer[1]);


    ControlData_t packet;
    if (xQueueReceive(dataQueue, &packet, 0) == errQUEUE_EMPTY) {
      continue;
    }

    int f = packet.forwardSpeed;
    int r = packet.rightSpeed;
    int v = packet.verticalSpeed;
    int t = packet.turnSpeed;
    int b = packet.button;
    printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);

    err = NRF_push_packet(&radio, (const uint8_t *)&packet, sizeof(packet)); // verify use of sizeof is correct
    assert(err != NRF_INVALID_PACKET_LEN); 

    // i think the busy wait is the only real way to do this
    // going to implement a hardware solution to this problem ^
    err = NRF_pulse_TXmode(&radio);
    assert(err == DRONE_OK);

    while(1) {
      // check if theres data in fifo



      // read status
      txBuffer[0] = CMD_R_REG | 0x07;
      txBuffer[1] = CMD_NOP;
      SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);
      
      // if TX_DS = 1
      if ((rxBuffer[1] & 0x20) == 0x20) {
        printf("TX_DS\n");
        // clear bit
        txBuffer[0] = CMD_W_REG | 0x07;
        txBuffer[1] = rxBuffer[1] | 0x20;
        SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);

        break;
      }

      // if MAX_RT = 1
      if ((rxBuffer[1] & 0x10) == 0x10) {
        printf("MAX_RT\n");
        break;
      }

      // read FIFO_STATUS
      txBuffer[0] = CMD_R_REG | 0x17;
      txBuffer[1] = CMD_NOP;
      SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);

      // if TX_EMPTY = 1, so when fifo empties
      if ((rxBuffer[1] & 0x10) == 0x10) {
        printf("TX_EMPTY\n");
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void readInputsTask(void *arg) {
  joystick_handle_t joysticks;
  drone_err_t err = init_ADC1(&joysticks);
  assert(err == DRONE_OK);

  //joystick_handle_t
  for (;;) {
    err  = readControls(&joysticks);
    assert(err == DRONE_OK);

    ControlData_t data = joysticks.data;
    xQueueOverwrite(dataQueue, &data); // only the most recent value matters

    // occasionally print stack headroom
    // static int counter = 0;
    // if ((++counter % 200) == 0) { // every ~1 s here
    //   UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
    //   size_t free_bytes = hw * sizeof(StackType_t);
    //   printf("[inputs] stack free min = %u bytes\n", (unsigned)free_bytes);
    // }

    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}



