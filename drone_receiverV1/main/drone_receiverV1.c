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
#include "esp_timer.h"

#include "drone_receiverV1.h"
#include "NRF24L01.h"
#include "joysticks.h"
#include "control_protocol.h"
#include "MPU6050.h"

#include "drone_err.h"
#include "NRF_err.h"


// --------- globals -------------------------------


// --------- globals -------------------------------


void app_main(void) {

  // NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  vTaskDelay(pdMS_TO_TICKS(1000)); 
  xTaskCreate(imuTask, "IMU task", 8192, NULL, 1, NULL); // not sure about mem size, more than this needs though
  xTaskCreate(getDataTask, "receiving data", 8192, NULL, 1, NULL); // not sure about mem size
}

void imuTask(void *arg) 
{
  MPU_handle_t imu;
  // incase theres errors just stays in the loop until init is done, maybe change
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10));
    drone_err_t err = MPU_init(&imu, 1, 2);
    if (err != DRONE_OK) continue;
    err = MPU_set_DLPF(&imu, 5);
    if (err != DRONE_OK) continue;
    err = MPU_gyro_calibrate(&imu);
    if (err != DRONE_OK) continue;
    err = MPU_accel_calibrate(&imu);
    if (err != DRONE_OK) continue;
    break;
  }

  angle_data_t filterAngles = (angle_data_t){0};

  int64_t t1 = esp_timer_get_time();
  int64_t t2 = esp_timer_get_time();
  for (;;) {
    drone_err_t err;
    t2 = esp_timer_get_time(); // time at new angle calculation
    float dt = (t2 - t1) * 1e-6f; // time in seconds
    err = MPU_complementary_filter(&imu, &filterAngles, &filterAngles, dt);
    if (err != DRONE_OK) {
      printf("err\n"); // not really handling yet
    }
    t1 = t2; // time of latest angle data
    
    //printf("filter angles: %f, %f \n", filterAngles.roll, filterAngles.pitch);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void getDataTask(void *arg) 
{
  const uint8_t ce_pin = 4;
  const uint8_t csn_pin = 14;
  const uint8_t irq_pin = 22;

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

    int f = packet.forwardSpeed;
    int r = packet.rightSpeed;
    int v = packet.verticalSpeed;
    int t = packet.turnSpeed;
    int b = packet.button;
    printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);   
    
    
    // handle data if received properly

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


 

