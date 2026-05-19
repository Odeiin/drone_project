#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

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
#include "MPU6050.h"
#include "joysticks.h"
#include "control_protocol.h"

#include "drone_err.h"
#include "NRF_err.h"

// --------- globals -------------------------------

// freeRTOS queue for communication between tasks
QueueHandle_t dataQueue;

// --------- globals -------------------------------


void app_main(void)
{

  // I beleive NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  vTaskDelay(pdMS_TO_TICKS(1000)); 

  // length 1 queue acts as buffer for control data between tasks
  // only length 1 as only the most recent data matters 
  dataQueue = xQueueCreate(1, sizeof(ControlData_t));
  assert(dataQueue != NULL);

  // i think the reading task should maybe be higher priority than the sending data task
  // if the control data isnt updated sending doesnt even matter anyway
  xTaskCreate(readInputsTask, "reading inputs", 2500, NULL, 2, NULL);

  xTaskCreate(radioTask, "radio task", 8192, NULL, 1, NULL); // not sure about mem size

}

// maybe make an NRF_responding function or something to check it works at the start of each program
void radioTask(void *arg) 
{
  NRF_handle_t radio;

  // controller data sent to drone
  ControlData_t packet = {0};
  // data from drone for logging
  angle_data_t telemetry_data = {0};

  // initialises radio
  drone_err_t err = NRF_init(&radio, RADIO_CE_PIN, RADIO_CSN_PIN, RADIO_IRQ_PIN);
  assert(err == DRONE_OK);
  err = NRF_default_setup(&radio, RADIO_RX_ADDR, RADIO_TX_ADDR, RADIO_CHANNEL, sizeof(angle_data_t));
  assert(err == DRONE_OK);
  err = NRF_flush_TX(&radio);
  assert(err == DRONE_OK);
  err = NRF_flush_RX(&radio);
  assert(err == DRONE_OK);
  err = NRF_clear_interrupts(&radio);
  assert(err == DRONE_OK);
  
  for (;;) {
    // if queue empty
    if (xQueueReceive(dataQueue, &packet, 0) == errQUEUE_EMPTY) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // checks if any telemetry data was received while this task was delayed
    err = NRF_receive_data(&radio, (uint8_t *)&telemetry_data, sizeof(telemetry_data));
    if (err == DRONE_OK) {
      //printf("angle data: roll -> %f, pitch -> %f\n", telemetry_data.roll, telemetry_data.pitch);
    } else if (err == NRF_EMPTY_RXFIFO) {
      // no telemetry available, this is ok
    } else {
      printf("radio error when receiving: 0x%lx\n", err);
    }

    // briefly enters TX mode and will send control data
    err = NRF_send_data(&radio, (const uint8_t *)&packet, sizeof(packet));
    if (err != DRONE_OK) {
      printf("radio error when sending: 0x%lx\n", err);
    }

    // then will enter RX mode until theres more control data to send 
    err = NRF_receive_data(&radio, (uint8_t *)&telemetry_data, sizeof(telemetry_data));
    if (err == DRONE_OK) {
      //printf("angle data: roll -> %f, pitch -> %f\n", telemetry_data.roll, telemetry_data.pitch);
    } else if (err == NRF_EMPTY_RXFIFO) {
      // no telemetry available, this is ok
    } else {
      printf("radio error when receiving: 0x%lx\n", err);
    }

    // // print status reg    
    // uint8_t txBuffer[2];
    // uint8_t rxBuffer[2];
    
    // txBuffer[0] = CMD_R_REG | 0x07; // read status
    // txBuffer[1] = CMD_NOP;
    // SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);

    // printf("status reg -> %x\n", rxBuffer[1]);




    // int f = packet.forwardSpeed;
    // int r = packet.rightSpeed;
    // int v = packet.verticalSpeed;
    // int t = packet.turnSpeed;
    // int b = packet.button;
    // printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// should add some sort of calibration that centers the values at the start of each function and use a deadzone
void readInputsTask(void *arg) {
  joystick_handle_t joysticks = {0};
  drone_err_t err = init_joysticks(&joysticks);
  assert(err == DRONE_OK);

  err = joysticks_calibrate(&joysticks);
  assert(err == DRONE_OK);
  //joystick_handle_t
  for (;;) {
    err  = readControls(&joysticks);
    assert(err == DRONE_OK);

    // put data in global (thread safe)
    ControlData_t data = joysticks.data;
    xQueueOverwrite(dataQueue, &data); // only the most recent value matters


    // int f = data.forwardSpeed;
    // int r = data.rightSpeed;
    // int v = data.verticalSpeed;
    // int t = data.turnSpeed;
    // int b = data.button;
    // printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);


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



