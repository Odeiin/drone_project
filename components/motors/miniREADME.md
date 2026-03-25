I was doing motor control using the MCPWM generator but I think theres only 3 MCPWM groups, and I believe each motor needs its own group because they need seperate comparators so they can generate their own PWM signal, last time I checked my drone had 4 motors so Ill try refactoring my motor code to use the LEDC module

recources:

https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_bdc_speed_control/main/mcpwm_bdc_control_example_main.c

https://www.youtube.com/watch?v=XfAt6hNV8XM


