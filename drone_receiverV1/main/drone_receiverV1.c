#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

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
QueueHandle_t angle_queue;
QueueHandle_t radio_queue;

// --------- globals -------------------------------

// clamps a value within a given range
static inline int16_t pulse_clamp(int16_t value, int16_t max, int16_t min)
{
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// applies a range where the value becomes 0 (deadband)
static inline int16_t apply_deadband(int16_t value, int16_t deadband)
{
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

void app_main(void) {

  angle_queue = xQueueCreate(1, sizeof(angle_data_t));
  assert(angle_queue != NULL);

  radio_queue = xQueueCreate(1, sizeof(ControlData_t));
  assert(radio_queue != NULL);

  // NRF needs 100ms to settle, id seen waiting longer somewhere, cant hurt
  vTaskDelay(pdMS_TO_TICKS(1000)); 
  xTaskCreate(imuTask, "IMU task", 8192, NULL, 2, NULL); // not sure about mem size, more than this needs though
  xTaskCreate(getDataTask, "receiving data", 8192, NULL, 1, NULL); // not sure about mem size
  xTaskCreate(flight_control_task, "test", 8192, NULL, 3, NULL); // not sure about mem size
}


void flight_control_task(void *arg)
{
  drone_err_t err;
  
  drone_motor_controller_t drone = {0};
  err = drone_motors_init(&drone);
  if (err != DRONE_OK) {
    printf("error: %ld", err);
  }

  // short min pulses at start up to stop motors from going crazy
  for (int i = 0; i < 200; i++) {
    motor_set_pulse(&drone.front_right_motor, MOTOR_MIN_PULSE_PERIOD);
    motor_set_pulse(&drone.front_left_motor, MOTOR_MIN_PULSE_PERIOD);
    motor_set_pulse(&drone.back_right_motor, MOTOR_MIN_PULSE_PERIOD);
    motor_set_pulse(&drone.back_left_motor, MOTOR_MIN_PULSE_PERIOD);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  
  // should only be done once otherwise the motors just go crazy
  // definetly do not uncomment this while the props are on
  // err = drone_motors_calibrate(&drone);
  // if (err != DRONE_OK) {
  //   printf("error: %ld", err);
  // }  

  // currently it seems like I wont be able to get a good vertical velocity reading using the accelerometer (maybe in future)  
  // so I cant really make the drone maintain an altitude, so the current flight control model is the controller input
  // just changes the vertical velocity not the drone height  

  // vertical throttle should change with angle so the drone maintains the set throttle
  // throttle = throttle from controller input / (cos(pitch_angle) * cos(roll_angle))  

  // m1 = throttle + roll + pitch + yaw;
  // m2 = throttle - roll + pitch - yaw;
  // m3 = throttle - roll - pitch + yaw;
  // m4 = throttle + roll - pitch - yaw;

  // control variables
  ControlData_t control_data = {0};
  angle_data_t angle_data = {0};
  angle_data_t target_angle = {0};

  int64_t t1 = esp_timer_get_time();
  int64_t t2 = esp_timer_get_time();
  PID_state_t state = PID_DEFAULT_CONFIG;

  // motor calculation variables
  // addign MOTOR_MIN_PULSE_PERIOD here instead of adding in the motor pulse part is safer is it allows 
  // me to fully stop the drone by decreasing throttle
  int16_t throttle = MOTOR_MIN_PULSE_PERIOD; 
  int16_t yaw = 0;
  int16_t roll = 0;
  int16_t pitch = 0;
  while (1) {
    // wait till theres new angle data
    if (xQueueReceive(angle_queue, &angle_data, pdMS_TO_TICKS(20)) == errQUEUE_EMPTY) {
      continue;
    }
    // update target angle if theres new control data
    if (xQueueReceive(radio_queue, &control_data, 0) != errQUEUE_EMPTY) {
      target_angle.roll  = control_data.rightSpeed  * INPUT_TO_ANGLE_FACTOR;
      target_angle.pitch = control_data.forwardSpeed * INPUT_TO_ANGLE_FACTOR;
    
    }

    printf("target angles: roll -> %f, pitch -> %f  ", target_angle.roll, target_angle.pitch);

    // int f = control_data.forwardSpeed;
    // int r = control_data.rightSpeed;
    // int v = control_data.verticalSpeed;
    // int t = control_data.turnSpeed;
    // int b = control_data.button;
    // printf("f: %d, r: %d, v: %d, t: %d, b: %d   \n", f, r, v, t, b);


    // PID calculations
    t2 = esp_timer_get_time(); // time at new angle calculation
    float dt = (t2 - t1) * 1e-6f; // time in seconds
    PID_angle_correction_t angle_corrections = PID_angle_calculation(angle_data, target_angle, &state, dt);
  
    // motor speed calculations:

    // throttle should slowly increase with control input instead of being set like the others because
    // its awkward to have to hold the joystick at the vertical speed you want, you just slowly increase or decrease using this
    throttle += apply_deadband(control_data.verticalSpeed, CONTROLLER_DEADBAND) * VERT_RESPONSE_FACTOR * dt;
    // using time for throttle calculation aswell
    t1 = t2; // time of latest angle data

    // need to change this currently youre just dividing it every loop
    // throttle should be adjusted based on drone angle so that it maintains its throttle regardless of the drones angle
    //throttle = throttle / (cos(angle_data.pitch * (M_PI / 180.0f)) * cos(angle_data.roll * (M_PI / 180.0f))); // converting to radians

    // printf(" !! throttle value -> %d !! ", throttle);

    yaw = apply_deadband(control_data.turnSpeed, CONTROLLER_DEADBAND) * TURN_RESPONSE_FACTOR;
    roll = angle_corrections.roll_correction * ROLL_RESPONSE_FACTOR;
    pitch = angle_corrections.pitch_correction * PITCH_RESPONSE_FACTOR;

    // testing for bring up
    //yaw = 0;
    //throttle = 0;
    //roll = 0;
    //pitch = 0;

    // motor speed calculation
    int16_t front_right_pulse = throttle - roll + pitch + yaw;
    int16_t front_left_pulse = throttle + roll + pitch - yaw;
    int16_t back_right_pulse = throttle - roll - pitch - yaw;
    int16_t back_left_pulse = throttle + roll - pitch + yaw;

    // clamp values within acceptable motor range
    front_right_pulse = pulse_clamp(front_right_pulse, MOTOR_MAX_PULSE_PERIOD, MOTOR_MIN_PULSE_PERIOD);
    front_left_pulse = pulse_clamp(front_left_pulse, MOTOR_MAX_PULSE_PERIOD, MOTOR_MIN_PULSE_PERIOD);
    back_right_pulse = pulse_clamp(back_right_pulse, MOTOR_MAX_PULSE_PERIOD, MOTOR_MIN_PULSE_PERIOD);
    back_left_pulse = pulse_clamp(back_left_pulse, MOTOR_MAX_PULSE_PERIOD, MOTOR_MIN_PULSE_PERIOD);

    // set motor pulses using calculated values
    motor_set_pulse(&drone.front_right_motor, front_right_pulse);
    motor_set_pulse(&drone.front_left_motor, front_left_pulse);
    motor_set_pulse(&drone.back_right_motor, back_right_pulse);
    motor_set_pulse(&drone.back_left_motor, back_left_pulse);

    // printf("target: roll=%f pitch=%f | angle: roll=%f pitch=%f\n",
    //    target_angle.roll, target_angle.pitch,
    //    angle_data.roll, angle_data.pitch);

    printf("motor variables: throttle -> %d, roll -> %d, pitch -> %d  ", throttle, roll, pitch);
    printf("current angles: roll -> %f, pitch -> %f   ", angle_data.roll, angle_data.pitch);
    printf("PID values: roll -> %f , pitch -> %f\n", angle_corrections.roll_correction, angle_corrections.pitch_correction);
  }

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
    xQueueOverwrite(angle_queue, &filterAngles); // only the most recent value matters
    //printf("filter angles: %f, %f \n", filterAngles.roll, filterAngles.pitch);
    vTaskDelay(pdMS_TO_TICKS(1));
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
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    assert(err == DRONE_OK);

    // put data in global (thread safe)
    xQueueOverwrite(radio_queue, &packet); // only the most recent value matters

    // int f = packet.forwardSpeed;
    // int r = packet.rightSpeed;
    // int v = packet.verticalSpeed;
    // int t = packet.turnSpeed;
    // int b = packet.button;
    // printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);   
    
    // handle data if received properly

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


 

