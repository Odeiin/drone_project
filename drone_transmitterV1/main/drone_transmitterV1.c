#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"


#include "drone_transmitterV1.h"



// --------- globals -------------------------------

adc_oneshot_unit_handle_t adc1_handle = NULL;

struct ControlData controlDataGlobal = {
  .forwardSpeed = 0,
  .rightSpeed = 0,
  .verticalSpeed = 0,
  .turnSpeed = 0,
  .button = false
};

// --------- globals -------------------------------


void app_main(void)
{

  // i think the readign task should probably be higher priority than the sending data task
  // if the control data isnt updated sending doesnt even matter anyway
  xTaskCreate(readInputsTask, "reading inputs", 2500, NULL, 1, NULL);

}


void sendDataTask(void *arg) {

}


void readInputsTask(void *arg) {
  for (;;) {
    controlDataGlobal = readControls();

    // int f = controlDataGlobal.forwardSpeed;
    // int r = controlDataGlobal.rightSpeed;
    // int v = controlDataGlobal.verticalSpeed;
    // int t = controlDataGlobal.turnSpeed;
    // int b = controlDataGlobal.button;
    //printf("f: %d, r: %d, v: %d, t: %d, b: %d\n", f, r, v, t, b);

    // occasionally print stack headroom
    // static int counter = 0;
    // if ((++counter % 200) == 0) { // every ~1 s here
    //   UBaseType_t hw = uxTaskGetStackHighWaterMark(NULL);
    //   size_t free_bytes = hw * sizeof(StackType_t);
    //   printf("[inputs] stack free min = %u bytes\n", (unsigned)free_bytes);
    // }

    vTaskDelay(5 / portTICK_PERIOD_MS); // 200 hz 
  }
}


struct ControlData readControls(void) {
  init_ADC1();
  struct ControlData joystickData; 

  // right stick up/down
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, rightStickY, &joystickData.verticalSpeed)); //reads value to joystickData.verticalSpeed
  // right stick left/right
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, rightStickX, &joystickData.turnSpeed));
  // left stick up/down
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, leftStickY, &joystickData.forwardSpeed));
  // left stick left/right
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, leftStickX, &joystickData.rightSpeed));

  // currently no button on controller
  joystickData.button = 0;
  
  return joystickData;
}


void init_ADC1 (void) {
  // only run once
  if (adc1_handle) {
    return;
  }

  // creates handle to ADC_UNIT_1
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};    
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  // config settings
  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12,
  };

  // set config for joystick inputs
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, rightStickY, &config)); //config settings for adc oneshot
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, leftStickY, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, leftStickX, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, rightStickX, &config));
}


int16_t applyDeadzone(int rawValue, int center, int deadzone) {
  int16_t output = rawValue - center;

  if (abs(output) < deadzone) {
    return 0;
  } 

  if (output > 0) {
    return output - deadzone;
  } else {
    return output + deadzone;
  }
}
