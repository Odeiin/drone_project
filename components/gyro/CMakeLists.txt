cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS 
        "MPU6050.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_driver_i2c 
        driver 
        drone_err
)