#pragma once
#include "drone_err.h"

#define JOYSTICK_ERR_BASE            0x2000

#define JOYSTICK_INIT_FAIL               (JOYSTICK_ERR_BASE + 1)
#define JOYSTICK_READ_FAIL               (JOYSTICK_ERR_BASE + 2)