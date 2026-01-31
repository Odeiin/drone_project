#pragma once
#include <stdint.h>
#include <stdbool.h>

// change type to like 16 bit or something
typedef struct {  
  int16_t verticalSpeed; // +ve = Acsend, -ve = Descend     
  int16_t forwardSpeed; // +ve = Forward, -ve = Backwards      
  int16_t rightSpeed; // +ve = Right, -ve = Left
  int16_t turnSpeed; // +ve = clockwise, -ve = anti-clockwise
  bool button; // button input
} ControlData_t;