#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MPU6050.h"
#include "drone_err.h"
#include "MPU_err.h"
#include "esp_err.h"

void MPU_init(MPU_handle_t *imu) 
{
	i2c_master_bus_handle_t bus_handle; 
	i2c_master_dev_handle_t dev_handle;

	i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
	assert(err == ESP_OK);

	i2c_device_config_t dev_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = MPU_SENSOR_ADDR,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};
	err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
	assert(err == ESP_OK);
	
	imu->bus_handle = bus_handle;
	imu->dev_handle = dev_handle;

	MPU_wakeup(imu);
}


drone_err_t MPU_read(MPU_handle_t *imu, uint8_t reg_addr, uint8_t *data, size_t len) 
{
	esp_err_t err = i2c_master_transmit_receive(imu->dev_handle, &reg_addr, 1, data, len, I2C_TIMEOUT_MS);
	if (err == ESP_OK) return DRONE_OK;
	if (err == ESP_ERR_TIMEOUT) return I2C_TIMEOUT;
	assert(err == ESP_ERR_INVALID_ARG);
	return I2C_FAIL;
}


drone_err_t MPU_write_byte(MPU_handle_t *imu, uint8_t reg_addr, uint8_t data) 
{
	uint8_t write_buf[2] = {reg_addr, data};
	esp_err_t err = i2c_master_transmit(imu->dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
	if (err == ESP_OK) return DRONE_OK;
	if (err == ESP_ERR_TIMEOUT) return I2C_TIMEOUT;
	assert(err == ESP_ERR_INVALID_ARG);
	return I2C_FAIL;
}

// kinda expects the reg_addr to be put in the buffer unlike MPU_write_byte
drone_err_t MPU_write_multi_buffer(MPU_handle_t *imu, i2c_master_transmit_multi_buffer_info_t *data, size_t array_len) 
{
	esp_err_t err = i2c_master_multi_buffer_transmit(imu->dev_handle, data, array_len, I2C_TIMEOUT_MS);
	if (err == ESP_OK) return DRONE_OK;
	if (err == ESP_ERR_TIMEOUT) return I2C_TIMEOUT;
	assert(err == ESP_ERR_INVALID_ARG);
	return I2C_FAIL;
}


drone_err_t MPU_raw_accel(MPU_handle_t *imu, accel_data_t *data) 
{
	uint8_t buf[6];
	drone_err_t err = MPU_read(imu, 59, buf, 6); // read reg 59 - 64
	if (err != DRONE_OK) {
		return err;
	}

	data->accel_x = (int16_t)((buf[0] << 8) | buf[1]); // reg 59 is bits 15:8 so shited left and reg 60 is bits 7:0
	data->accel_y  = (int16_t)((buf[2] << 8) | buf[3]);
	data->accel_z  = (int16_t)((buf[4] << 8) | buf[5]);

	return DRONE_OK;
}

drone_err_t MPU_raw_gyro(MPU_handle_t *imu, gyro_data_t *data) 
{
	uint8_t buf[6];
	drone_err_t err = MPU_read(imu, 67, buf, 6); // read reg 67 - 72
	if (err != DRONE_OK) {
		return err;
	}

	data->gyro_x = (int16_t)((buf[0] << 8) | buf[1]); // reg 67 is bits 15:8 so shited left and reg 68 is bits 7:0
	data->gyro_y  = (int16_t)((buf[2] << 8) | buf[3]);
	data->gyro_z  = (int16_t)((buf[4] << 8) | buf[5]);

	return DRONE_OK;
}

drone_err_t MPU_wakeup(MPU_handle_t *imu)
{
	// uint8_t buf;
	// drone_err_t err = MPU_read(imu, 107, &buf, 1); // read PWR_MGMT_1
	// if (err != DRONE_OK) {
	// 	return err;
	// }

	// buf = buf & 0xBF; // SLEEP -> 0, preserve other bits
	// err = MPU_write_byte(imu, 107, buf);
	// if (err != DRONE_OK) {
	// 	return err;
	// }

	uint8_t buf = 0x00;
	drone_err_t err = MPU_write_byte(imu, 107, buf);
	if (err != DRONE_OK) {
		return err;
	}

	return DRONE_OK;
} 

// read data sheet for DLPF_CFG value descriptions
// need to use a lowpass filter to eliminate alot of the noise that occurs from the vibrations of motors on the drone for example
drone_err_t MPU_set_DLPF(MPU_handle_t *imu, uint8_t DLPF_CFG)
{
	if (DLPF_CFG > 6) {
		return MPU_INVALID_ARG;
	}

	uint8_t buf;
	drone_err_t err = MPU_read(imu, 26, &buf, 1); // read CONFIG
	if (err != DRONE_OK) {
		return err;
	}

	buf = buf & 0xF8;   	// clear DLPF_CFG
	buf = buf | DLPF_CFG; // set DLPF_CFG
	err = MPU_write_byte(imu, 26, buf);
	if (err != DRONE_OK) {
		return err;
	}
	return DRONE_OK;
} 


drone_err_t MPU_set_gyro_range(MPU_handle_t *imu, uint8_t FS_SEL)
{
	if (FS_SEL > 3) {
		return MPU_INVALID_ARG;
	}

	uint8_t buf;
	drone_err_t err = MPU_read(imu, 27, &buf, 1); // read CONFIG
	if (err != DRONE_OK) {
		return err;
	}

	buf = buf & 0xE7;   	// clear FS_SEL
	buf = buf | (FS_SEL << 3); // set FS_SEL
	err = MPU_write_byte(imu, 27, buf);
	if (err != DRONE_OK) {
		return err;
	}
	return DRONE_OK;
}


drone_err_t MPU_set_accel_range(MPU_handle_t *imu, uint8_t AFS_SEL)
{
	if (AFS_SEL > 3) {
		return MPU_INVALID_ARG;
	}

	uint8_t buf;
	drone_err_t err = MPU_read(imu, 28, &buf, 1); // read CONFIG
	if (err != DRONE_OK) {
		return err;
	}

	buf = buf & 0xE7;   	// clear AFS_SEL
	buf = buf | (AFS_SEL << 3); // set AFS_SEL
	err = MPU_write_byte(imu, 28, buf);
	if (err != DRONE_OK) {
		return err;
	}
	return DRONE_OK;
}


drone_err_t MPU_gyro_calibrate(MPU_handle_t *imu)
{
	gyro_data_t gyro_data;

	int gyro_x_total = 0;
	int gyro_y_total = 0;
	int gyro_z_total = 0;
	// collect 100 values
	for (int i = 0; i < MPU_CALIBRATION_SAMPLES; i++) {
		drone_err_t err = MPU_raw_gyro(imu, &gyro_data);
		if (err != DRONE_OK) {
			return err;
		}
			
		gyro_x_total += gyro_data.gyro_x;
		gyro_y_total += gyro_data.gyro_y;
		gyro_z_total += gyro_data.gyro_z;

		vTaskDelay(pdMS_TO_TICKS(10));
	}

	// find averages and set calibration values
	imu->gyro_calibration_vals.gyro_x = gyro_x_total/MPU_CALIBRATION_SAMPLES;
	imu->gyro_calibration_vals.gyro_y = gyro_y_total/MPU_CALIBRATION_SAMPLES;
	imu->gyro_calibration_vals.gyro_z = gyro_z_total/MPU_CALIBRATION_SAMPLES;

	return DRONE_OK;
}

// this function is dumb so far, accel should be calibrated and then modify this function similar to the gyro one
drone_err_t MPU_read_accel(MPU_handle_t *imu, accel_data_t *data) 
{
	drone_err_t err = MPU_raw_accel(imu, data);
		if (err != DRONE_OK) {
		return err;
	}

	return DRONE_OK;
}

drone_err_t MPU_read_gyro(MPU_handle_t *imu, gyro_data_t *data) 
{
	// adjusts values according to calibration
	drone_err_t err = MPU_raw_gyro(imu, data);
	if (err != DRONE_OK) {
		return err;
	}
	data->gyro_x -= imu->gyro_calibration_vals.gyro_x;
	data->gyro_y -= imu->gyro_calibration_vals.gyro_y;
	data->gyro_z -= imu->gyro_calibration_vals.gyro_z;

	return DRONE_OK;
}


void MPU_accel_calc_angles(MPU_handle_t *imu, accel_data_t *data, angle_data_t *angles) 
{
	float accel_x = (float)data->accel_x / 4096; // convert to g's
	float accel_y = (float)data->accel_y / 4096;
	float accel_z = (float)data->accel_z / 4096;

	float roll = atan(accel_y/sqrt(accel_x * accel_x + accel_z * accel_z)) * 1/(3.1415927/180); // calc angle, convert to degrees at the end
	float pitch = -atan(accel_x/sqrt(accel_y * accel_y + accel_z * accel_z)) * 1/(3.1415927/180);

	angles->roll = roll;
	angles->pitch = pitch;
}