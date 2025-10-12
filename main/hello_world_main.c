/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "max30102.h"
#include "algorithm.h"
#include "ads1292.h"
#include "mpu6050.h"
#include "websocket_server.h"

static const char *TAG = "HR_MONITOR";

// I2C Configuration for MPU6050 (shared bus with MAX30102)
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000

// Running average configuration
#define HR_AVERAGE_WINDOW 10
#define SPO2_AVERAGE_WINDOW 10

typedef struct {
    float hr_readings[HR_AVERAGE_WINDOW];
    float spo2_readings[SPO2_AVERAGE_WINDOW];
    uint8_t hr_index;
    uint8_t spo2_index;
    uint8_t hr_count;
    uint8_t spo2_count;
    float current_hr;
    float current_spo2;
    float temperature;
} monitor_state_t;

static void add_hr_reading(monitor_state_t *state, float hr) {
    state->hr_readings[state->hr_index] = hr;
    state->hr_index = (state->hr_index + 1) % HR_AVERAGE_WINDOW;
    if (state->hr_count < HR_AVERAGE_WINDOW) state->hr_count++;
    state->current_hr = hr;
}

static void add_spo2_reading(monitor_state_t *state, float spo2) {
    state->spo2_readings[state->spo2_index] = spo2;
    state->spo2_index = (state->spo2_index + 1) % SPO2_AVERAGE_WINDOW;
    if (state->spo2_count < SPO2_AVERAGE_WINDOW) state->spo2_count++;
    state->current_spo2 = spo2;
}

static float get_average_hr(monitor_state_t *state) {
    if (state->hr_count == 0) return 0.0f;
    float sum = 0;
    for (int i = 0; i < state->hr_count; i++) {
        sum += state->hr_readings[i];
    }
    return sum / state->hr_count;
}

static float get_average_spo2(monitor_state_t *state) {
    if (state->spo2_count == 0) return 0.0f;
    float sum = 0;
    for (int i = 0; i < state->spo2_count; i++) {
        sum += state->spo2_readings[i];
    }
    return sum / state->spo2_count;
}

void app_main(void)
{
    printf("\n╔════════════════════════════════════════════════════╗\n");
    printf("║   Multi-Sensor Health Monitor with WebSocket     ║\n");
    printf("║   MAX30102 | ADS1292 | MPU6050                   ║\n");
    printf("╚════════════════════════════════════════════════════╝\n\n");
    
    // Initialize WebSocket server (WiFi + HTTP)
    printf("Initializing WiFi and WebSocket server...\n");
    esp_err_t ret = websocket_server_init();
    if (ret != ESP_OK) {
        printf("⚠️  WebSocket server initialization failed (continuing without WiFi)\n");
    } else {
        printf("✓ WebSocket server started - access web interface via browser\n");
    }
    printf("\n");
    
    // Initialize MAX30102
    ret = max30102_init();
    if (ret != ESP_OK) {
        printf("❌ Failed to initialize MAX30102\n");
        return;
    }
    
    max30102_clear_fifo();
    printf("✓ MAX30102 initialized at 100 Hz\n");
    
    // Initialize ADS1292
    ret = ads1292_init();
    if (ret != ESP_OK) {
        printf("❌ Failed to initialize ADS1292\n");
    } else {
        printf("✓ ADS1292 initialized (EOG monitoring)\n");
        ads1292_start_conversion();
    }
    
    // Initialize MPU6050
    ret = mpu6050_init(I2C_MASTER_NUM, MPU6050_I2C_ADDR);
    if (ret != ESP_OK) {
        printf("⚠️  MPU6050 not detected (I2C addr 0x68)\n");
    } else {
        printf("✓ MPU6050 initialized (motion tracking)\n");
    }
    
    printf("\n");
    
    // Initialize monitoring state
    monitor_state_t state = {0};
    
    // Buffers for algorithm processing
    int32_t ir_buffer[ALGORITHM_BUFFER_SIZE];
    int32_t red_buffer[ALGORITHM_BUFFER_SIZE];
    uint16_t buffer_index = 0;
    
    // ADS1292 state
    eog_baseline_t eog_baseline;
    eog_init_baseline(&eog_baseline);
    
    // Counter for periodic sensor updates
    uint32_t loop_count = 0;
    
    printf("Starting monitoring...\n");
    printf("────────────────────────────────────────────────────\n\n");

    while (1) {
        loop_count++;
        if (max30102_is_data_ready()) {
            max30102_sample_t sample;
            ret = max30102_read_single_sample(&sample);
            
            if (ret == ESP_OK) {
                // Add to buffer
                ir_buffer[buffer_index] = (int32_t)sample.ir;
                red_buffer[buffer_index] = (int32_t)sample.red;
                buffer_index++;
                
                // When buffer is full, process
                if (buffer_index >= ALGORITHM_BUFFER_SIZE) {
                    // Read temperature
                    float temp;
                    if (max30102_read_temperature(&temp) == ESP_OK) {
                        state.temperature = temp;
                    }
                    
                    // Use algorithm to process signals
                    algorithm_result_t result;
                    if (algorithm_process_signals(ir_buffer, red_buffer, buffer_index, &result) == ESP_OK) {
                        if (result.valid && result.heart_rate > 30 && result.heart_rate < 200) {
                            add_hr_reading(&state, result.heart_rate);
                            add_spo2_reading(&state, result.spo2);
                            
                            // Print status line
                            printf("\r");
                            printf("HR: %3.0f BPM (avg: %3.0f) │ ", 
                                   state.current_hr, get_average_hr(&state));
                            printf("SpO2: %4.1f%% (avg: %4.1f%%) │ ", 
                                   state.current_spo2, get_average_spo2(&state));
                            printf("Temp: %4.1f°C", state.temperature);
                            fflush(stdout);
                            
                            // Send via WebSocket
                            heartrate_packet_t hr_packet = {
                                .heart_rate = get_average_hr(&state),
                                .spo2 = get_average_spo2(&state),
                                .temperature = state.temperature,
                                .timestamp = esp_timer_get_time() / 1000
                            };
                            websocket_send_heartrate(&hr_packet);
                        }
                    }
                    
                    // Reset buffer
                    buffer_index = 0;
                }
            }
        }
        
        // Read ADS1292 data (EOG) - every iteration
        if (ads1292_data_ready()) {
            ads1292_data_t ads_data;
            if (ads1292_read_data(&ads_data) == ESP_OK) {
                eog_update_baseline(&eog_baseline, &ads_data);
                
                // Print EOG data every 50 samples (~200ms at 250 SPS)
                if (loop_count % 50 == 0) {
                    printf("\n[EOG] ");
                    eog_print_data_with_baseline(&ads_data, &eog_baseline);
                    
                    // Send via WebSocket
                    float ch1_v = ads1292_convert_to_voltage(ads_data.channel1);
                    float ch2_v = ads1292_convert_to_voltage(ads_data.channel2);
                    eeg_packet_t eeg_packet = {
                        .ch1_voltage = ch1_v,
                        .ch2_voltage = ch2_v,
                        .ch1_baseline = eog_baseline.baseline_ch1,
                        .ch2_baseline = eog_baseline.baseline_ch2,
                        .baseline_established = eog_baseline.baseline_established,
                        .timestamp = esp_timer_get_time() / 1000
                    };
                    websocket_send_eeg(&eeg_packet);
                }
            }
        }
        
        // Read MPU6050 data (motion) - every 100ms
        if (loop_count % 10 == 0) {
            mpu6050_data_t mpu_data;
            if (mpu6050_read_all(&mpu_data) == ESP_OK) {
                printf("\n[MPU] ");
                mpu6050_print_data(&mpu_data);
                
                // Send via WebSocket
                mpu_packet_t mpu_packet = {
                    .accel_x = mpu_data.accel_x_g,
                    .accel_y = mpu_data.accel_y_g,
                    .accel_z = mpu_data.accel_z_g,
                    .gyro_x = mpu_data.gyro_x_dps,
                    .gyro_y = mpu_data.gyro_y_dps,
                    .gyro_z = mpu_data.gyro_z_dps,
                    .temp = mpu_data.temp_c,
                    .timestamp = esp_timer_get_time() / 1000
                };
                websocket_send_mpu(&mpu_packet);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
