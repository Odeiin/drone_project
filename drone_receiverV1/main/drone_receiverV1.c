#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include "esp_timer.h"

#include "drone_receiverV1.h"
#include "NRF24L01.h"
#include "joysticks.h"
#include "control_protocol.h"
#include "MPU6050.h"
#include "motor.h"

#include "drone_err.h"
#include "NRF_err.h"


// --------- globals -------------------------------
QueueHandle_t angle_data;
QueueHandle_t radio_data;

// --------- globals -------------------------------


void app_main(void) {

  angle_data = xQueueCreate(1, sizeof(angle_data_t));
  assert(angle_data != NULL);

  radio_data = xQueueCreate(1, sizeof(ControlData_t));
  assert(radio_data != NULL);

  // NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  //vTaskDelay(pdMS_TO_TICKS(1000)); 
  //xTaskCreate(imuTask, "IMU task", 8192, NULL, 1, NULL); // not sure about mem size, more than this needs though
  //xTaskCreate(getDataTask, "receiving data", 8192, NULL, 1, NULL); // not sure about mem size
  xTaskCreate(flight_control_task, "test", 8192, NULL, 1, NULL); // not sure about mem size
}


void flight_control_task(void *arg)
{
  drone_err_t err;

  drone_motor_controller_t drone = {0};
  err = drone_motors_init(&drone);
  if (err != DRONE_OK) {
    printf("error: %ld", err);
  }
  err = drone_motors_calibrate(&drone);
  if (err != DRONE_OK) {
    printf("error: %ld", err);
  }









  // int num = 1800;
  // int increment = 10;
  // while (1) {
  //   motor_set_pulse(&drone->front_right_motor, num);

  //   if (num >= 1800) {
  //     increment = -10;
  //   }
  //   if (num < 1200) {
  //     increment = 10;
  //   }

  //   num = num + increment;

  //   vTaskDelay(pdMS_TO_TICKS(10));
  // }








  // flight control pseduocode

  // vertical component of the thrust is:
  // Tvert = Ttotal * cos(pitch_angle) * cos(roll_angle)
  // so to maintain a vertical thrust:
  // Ttotal = baseline/(cos(pitch_angle) * cos(roll_angle))
  // where baseline is the thrust from keeping the drone stable +/- the vertical thrust from the controller
  // this Ttotal = throttle
  // but this assumes im able to increase or decrease the motor speed by specific amounts

  // roll and pitch corrections will come from the PID angle correction

  // i think perhaps vertical thrust is just going to be manually controlled by the remote controller
  // i can maybe reasonablly maintain the vertical thrust but ive currently got no real way of sensing the drones vertical position
  // ill add a barometer later
  

  // m1 = throttle + roll + pitch - yaw
  // m2 = throttle - roll + pitch + yaw
  // m3 = throttle - roll - pitch - yaw
  // m4 = throttle + roll - pitch + yaw



  // ----------------------
  //potentiometer testing pwm
  
  adc_oneshot_unit_handle_t adc;

	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};    
  adc_oneshot_new_unit(&init_config1, &adc);


  // config settings
  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
  };

  assert(adc_oneshot_config_channel(adc, ADC_CHANNEL_6, &config) == ESP_OK);

  int raw;
  while(1) {
    adc_oneshot_read(adc, ADC_CHANNEL_6, &raw);

    raw = 900 + ((raw * 500) / 4095);

    drone_err_t err = motor_set_pulse(&drone.front_right_motor, (uint16_t)raw);
    printf("%ld\n", err);

    vTaskDelay(pdMS_TO_TICKS(10));

  }  
  // ----------------------
  
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
    
    // put data in global (thread safe)
    xQueueOverwrite(angle_data, &filterAngles); // only the most recent value matters
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

    // put data in global (thread safe)
    xQueueOverwrite(radio_data, &packet); // only the most recent value matters

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


 

