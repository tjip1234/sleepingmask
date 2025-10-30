#include "mpu6050.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MPU6050";
static i2c_port_t mpu_i2c_port;
static uint8_t mpu_dev_addr;
static float accel_scale = 16384.0f;  // Default ±2g
static float gyro_scale = 131.0f;     // Default ±250°/s

static esp_err_t mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(mpu_i2c_port, mpu_dev_addr, write_buf, 2, pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read_register(uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(mpu_i2c_port, mpu_dev_addr, &reg, 1, value, 1, pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read_registers(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(mpu_i2c_port, mpu_dev_addr, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

esp_err_t mpu6050_init(i2c_port_t i2c_port, uint8_t dev_addr) {
    mpu_i2c_port = i2c_port;
    mpu_dev_addr = dev_addr;
    esp_err_t ret;
    
    // Check WHO_AM_I register
    uint8_t who_am_i;
    ret = mpu6050_read_register(MPU6050_REG_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    
    if (who_am_i != 0x68) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I value: 0x%02X (expected 0x68)", who_am_i);
    } else {
        ESP_LOGI(TAG, "MPU6050 detected successfully (WHO_AM_I: 0x%02X)", who_am_i);
    }
    
    // Wake up the device (clear sleep bit)
    ret = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set sample rate divider (1kHz / (1 + SMPLRT_DIV))
    ret = mpu6050_write_register(MPU6050_REG_SMPLRT_DIV, 0x07); // 125 Hz
    if (ret != ESP_OK) return ret;
    
    // Set low-pass filter
    ret = mpu6050_write_register(MPU6050_REG_CONFIG, 0x06); // 5Hz bandwidth
    if (ret != ESP_OK) return ret;
    
    // Set accelerometer range to ±2g
    ret = mpu6050_set_accel_range(MPU6050_ACCEL_FS_2G);
    if (ret != ESP_OK) return ret;
    
    // Set gyroscope range to ±250°/s
    ret = mpu6050_set_gyro_range(MPU6050_GYRO_FS_250);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_set_accel_range(uint8_t range) {
    esp_err_t ret = mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG, range);
    if (ret == ESP_OK) {
        switch (range) {
            case MPU6050_ACCEL_FS_2G:  accel_scale = 16384.0f; break;
            case MPU6050_ACCEL_FS_4G:  accel_scale = 8192.0f; break;
            case MPU6050_ACCEL_FS_8G:  accel_scale = 4096.0f; break;
            case MPU6050_ACCEL_FS_16G: accel_scale = 2048.0f; break;
        }
    }
    return ret;
}

esp_err_t mpu6050_set_gyro_range(uint8_t range) {
    esp_err_t ret = mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, range);
    if (ret == ESP_OK) {
        switch (range) {
            case MPU6050_GYRO_FS_250:  gyro_scale = 131.0f; break;
            case MPU6050_GYRO_FS_500:  gyro_scale = 65.5f; break;
            case MPU6050_GYRO_FS_1000: gyro_scale = 32.8f; break;
            case MPU6050_GYRO_FS_2000: gyro_scale = 16.4f; break;
        }
    }
    return ret;
}

esp_err_t mpu6050_read_raw(mpu6050_data_t *data) {
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse raw data (big-endian)
    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return ESP_OK;
}

esp_err_t mpu6050_read_all(mpu6050_data_t *data) {
    esp_err_t ret = mpu6050_read_raw(data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert to physical units
    data->accel_x_g = (float)data->accel_x / accel_scale;
    data->accel_y_g = (float)data->accel_y / accel_scale;
    data->accel_z_g = (float)data->accel_z / accel_scale;
    
    // Temperature in °C = (TEMP_OUT / 340) + 36.53
    data->temp_c = ((float)data->temp_raw / 340.0f) + 36.53f;
    
    data->gyro_x_dps = (float)data->gyro_x / gyro_scale;
    data->gyro_y_dps = (float)data->gyro_y / gyro_scale;
    data->gyro_z_dps = (float)data->gyro_z / gyro_scale;
    
    return ESP_OK;
}

void mpu6050_print_data(const mpu6050_data_t *data) {
    printf("Accel: X=%.2fg Y=%.2fg Z=%.2fg | Gyro: X=%.1f°/s Y=%.1f°/s Z=%.1f°/s | Temp: %.1f°C\n",
           data->accel_x_g, data->accel_y_g, data->accel_z_g,
           data->gyro_x_dps, data->gyro_y_dps, data->gyro_z_dps,
           data->temp_c);
}
