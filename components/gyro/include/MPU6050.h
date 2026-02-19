#include "driver/i2c_master.h"

#include "drone_err.h"
#include "MPU_err.h"
#include "esp_err.h"

#define MPU_SENSOR_ADDR 	0x68
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000

#define I2C_TIMEOUT_MS		1000

#define MPU_CALIBRATION_SAMPLES 100

#define MPU_FILTER_FRACTION 0.98 // must be a value between 0 and 1, this value represents the filter fraction for the gyro

typedef struct{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} accel_data_t;

typedef struct{
	int16_t gyro_x; // roll 
	int16_t gyro_y; // pitch
	int16_t gyro_z; // yaw
} gyro_data_t;

typedef struct{
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t dev_handle;
	gyro_data_t gyro_calibration_vals;
	accel_data_t accel_calibration_vals;
	uint8_t accel_range; // +-g's
	int16_t gyro_range; // +- degrees/s
	int16_t accel_sensitivity; // LSB/g
	float gyro_sensitivity; // LSB/ degrees/s
} MPU_handle_t;

typedef struct{
	float roll;
	float pitch;
} angle_data_t;


drone_err_t MPU_init(MPU_handle_t *imu, uint8_t FS_SEL, uint8_t AFS_SEL);

drone_err_t MPU_read(MPU_handle_t *imu, uint8_t reg_addr, uint8_t *data, size_t len);

drone_err_t MPU_write_byte(MPU_handle_t *imu, uint8_t reg_addr, uint8_t data);

drone_err_t MPU_write_multi_buffer(MPU_handle_t *imu, i2c_master_transmit_multi_buffer_info_t *data, size_t array_len);

drone_err_t MPU_raw_accel(MPU_handle_t *imu, accel_data_t *data);

drone_err_t MPU_raw_gyro(MPU_handle_t *imu, gyro_data_t *data);

drone_err_t MPU_wakeup(MPU_handle_t *imu);

drone_err_t MPU_set_DLPF(MPU_handle_t *imu, uint8_t DLPF_CFG);

drone_err_t MPU_set_gyro_range(MPU_handle_t *imu, uint8_t FS_SEL);

drone_err_t MPU_set_accel_range(MPU_handle_t *imu, uint8_t AFS_SEL);

drone_err_t MPU_gyro_calibrate(MPU_handle_t *imu);

drone_err_t MPU_accel_calibrate(MPU_handle_t *imu);

drone_err_t MPU_read_accel(MPU_handle_t *imu, accel_data_t *data);

drone_err_t MPU_read_gyro(MPU_handle_t *imu, gyro_data_t *data);

drone_err_t MPU_accel_calc_angles(MPU_handle_t *imu, angle_data_t *angles);

drone_err_t MPU_gyro_calc_angles(MPU_handle_t *imu, angle_data_t *prev_angles, angle_data_t *angle_data, float dt);

drone_err_t MPU_complementary_filter(MPU_handle_t *imu, angle_data_t *prev_angles, angle_data_t *angle_data, float dt);