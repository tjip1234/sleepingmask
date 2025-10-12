#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// MPU6050 I2C Configuration
#define MPU6050_I2C_ADDR        0x68  // Default I2C address (AD0 = 0)
#define MPU6050_I2C_ADDR_ALT    0x69  // Alternative address (AD0 = 1)

// MPU6050 Register Addresses
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

// Accelerometer scale options
#define MPU6050_ACCEL_FS_2G     0x00  // ±2g
#define MPU6050_ACCEL_FS_4G     0x08  // ±4g
#define MPU6050_ACCEL_FS_8G     0x10  // ±8g
#define MPU6050_ACCEL_FS_16G    0x18  // ±16g

// Gyroscope scale options
#define MPU6050_GYRO_FS_250     0x00  // ±250 °/s
#define MPU6050_GYRO_FS_500     0x08  // ±500 °/s
#define MPU6050_GYRO_FS_1000    0x10  // ±1000 °/s
#define MPU6050_GYRO_FS_2000    0x18  // ±2000 °/s

// Data structure for MPU6050 readings
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temp_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} mpu6050_data_t;

// Function prototypes
esp_err_t mpu6050_init(i2c_port_t i2c_port, uint8_t dev_addr);
esp_err_t mpu6050_read_raw(mpu6050_data_t *data);
esp_err_t mpu6050_read_all(mpu6050_data_t *data);
void mpu6050_print_data(const mpu6050_data_t *data);
esp_err_t mpu6050_set_accel_range(uint8_t range);
esp_err_t mpu6050_set_gyro_range(uint8_t range);

#endif // MPU6050_H
