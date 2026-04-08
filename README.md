**ESP32 Quadcopter (ESP-IDF + FreeRTOS)**

personal drone project with the objective of learning about esp-idf and free rtos aswell as developing my embedded systems programming skills

Currently drone is in a semi working state but it is not stabalising properly on take off, im currently in the process of trying to improve how it flys and reduce oscillations through PID tuning and input filtering

The system uses IMU-based attitude estimation and a FreeRTOS-based control loop to stabilise the drone.

**Features**
ESP32 flight controller using ESP-IDF
MPU6050 IMU driver
Complementary filter for attitude estimation
PID-based roll/pitch stabilisation
custom NRF24L01 driver for wireless control
Custom communication protocol (packed control struct)
ESC control via PWM outputs
Joystick transmitter with deadzone + filtering


**Hardware**

Flight controller:

ESP32
MPU6050 IMU
NRF24L01 radio
4x brushless motors + ESCs

Transmitter:

ESP32
NRF24L01 radio
Dual joystick inputs


**future improvements**

- make it so button from control makes the drone leave startup mode and then also emergency stop once its in operating mode
- add LED to drone which indicates when its in start up mode
- add altitude detection so drone maintains its level instead of having to set the vertical speed using the joysticks
- also add a safety switch on the drone
- have receiver send angle measurement back to transmitter so telemetary data can be read

