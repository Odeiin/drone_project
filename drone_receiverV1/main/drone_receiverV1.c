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
#include "MPU6050.h"

#include "drone_err.h"
#include "NRF_err.h"


// --------- globals -------------------------------


// --------- globals -------------------------------


void app_main(void)
{

  // NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  vTaskDelay(pdMS_TO_TICKS(1000)); 
  xTaskCreate(imuTask, "IMU task", 8192, NULL, 1, NULL); // not sure about mem size
  // xTaskCreate(getDataTask, "receiving data", 8192, NULL, 1, NULL); // not sure about mem size
}

void imuTask(void *arg) {
  MPU_handle_t imu;
  drone_err_t err;
   MPU_init(&imu);
  // incase theres errors just stays in the loop until init is done, maybe change
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(10));
    drone_err_t err = MPU_set_DLPF(&imu, 5);
    if (err != DRONE_OK) continue;
    err = MPU_set_gyro_range(&imu, 1);
    if (err != DRONE_OK) continue;
    err = MPU_set_accel_range(&imu, 2);
    if (err != DRONE_OK) continue;
    err = MPU_gyro_calibrate(&imu);
    if (err != DRONE_OK) continue;
    break;
  }

  accel_data_t accelData;
  gyro_data_t gyroData;

  for (;;) {
    err = MPU_read_accel(&imu, &accelData);
    err = MPU_read_gyro(&imu, &gyroData);
  
    angle_data_t accelAngles;
    MPU_accel_calc_angles(&imu, &accelData, &accelAngles);

    // accelData.accel_x = accelData.accel_x / 4096;
    // accelData.accel_y = accelData.accel_y / 4096;
    // accelData.accel_z = accelData.accel_z / 4096;

    printf("angle data: %f, %f\n", accelAngles.roll, accelAngles.pitch);
    //printf("accel data: %d, %d, %d\n", accelData.accel_x, accelData.accel_y, accelData.accel_z);
    //printf("gyro data: %d, %d, %d\n", gyroData.gyro_x, gyroData.gyro_y, gyroData.gyro_z);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void getDataTask(void *arg) {
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



