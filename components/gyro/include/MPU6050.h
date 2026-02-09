#include "driver/i2c_master.h"

#include "drone_err.h"
#include "MPU_err.h"
#include "esp_err.h"

#define MPU_SENSOR_ADDR 	0x68
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000

#define I2C_TIMEOUT_MS		1000


typedef struct{
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t dev_handle;

} MPU_handle_t;

typedef struct{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} accel_data_t;

typedef struct{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} gyro_data_t;


void MPU_init(MPU_handle_t *imu);

drone_err_t MPU_read(MPU_handle_t *imu, uint8_t reg_addr, uint8_t *data, size_t len);

drone_err_t MPU_write_byte(MPU_handle_t *imu, uint8_t reg_addr, uint8_t data);

drone_err_t MPU_write_multi_buffer(MPU_handle_t *imu, i2c_master_transmit_multi_buffer_info_t *data, size_t array_len);

drone_err_t MPU_read_accel(MPU_handle_t *imu, accel_data_t *data);

drone_err_t MPU_read_gyro(MPU_handle_t *imu, gyro_data_t *data);

drone_err_t MPU_wakeup(MPU_handle_t *imu);