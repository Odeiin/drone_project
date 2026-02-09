#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "driver/i2c_master.h"

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
	assert(err != ESP_ERR_INVALID_ARG);
	if (err == ESP_ERR_TIMEOUT) {
		return I2C_TIMEOUT;
	}
	return DRONE_OK;
}


drone_err_t MPU_write_byte(MPU_handle_t *imu, uint8_t reg_addr, uint8_t data) 
{
	uint8_t write_buf[2] = {reg_addr, data};
	esp_err_t err = i2c_master_transmit(imu->dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
	assert(err != ESP_ERR_INVALID_ARG);
	if (err == ESP_ERR_TIMEOUT) {
		return I2C_TIMEOUT;
	}
	return DRONE_OK;
}

// kinda expects the reg_addr to be put in the buffer unlike MPU_write_byte
drone_err_t MPU_write_multi_buffer(MPU_handle_t *imu, i2c_master_transmit_multi_buffer_info_t *data, size_t array_len) 
{
	esp_err_t err = i2c_master_multi_buffer_transmit(imu->dev_handle, data, array_len, I2C_TIMEOUT_MS);
	assert(err != ESP_ERR_INVALID_ARG);
	if (err == ESP_ERR_TIMEOUT) {
		return I2C_TIMEOUT;
	}
	return DRONE_OK;
}


drone_err_t MPU_read_accel(MPU_handle_t *imu, accel_data_t *data) 
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

drone_err_t MPU_read_gyro(MPU_handle_t *imu, gyro_data_t *data) 
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

drone_err_t MPU_wakeup(MPU_handle_t *imu) {
	uint8_t buf;
	drone_err_t err = MPU_read(imu, 107, &buf, 1); // read PWR_MGMT_1
	if (err != DRONE_OK) {
		return err;
	}

	buf = buf & 0xBF; // SLEEP -> 0, preserve other bits
	err = MPU_write_byte(imu, 107, buf);
	if (err != DRONE_OK) {
		return err;
	}
	return DRONE_OK;
} 