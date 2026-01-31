cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS
        "joysticks.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_adc
        drone_err
)