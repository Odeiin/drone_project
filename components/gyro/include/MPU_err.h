#pragma once
#include "drone_err.h"

#define MPU_ERR_BASE            0x3000

#define I2C_TIMEOUT             (MPU_ERR_BASE + 1)
#define I2C_FAIL                (MPU_ERR_BASE + 2)
#define MPU_INVALID_ARG         (MPU_ERR_BASE + 3)