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
#include "max30102.h"

static const char *TAG = "SLEEPINGMASK";

void app_main(void)
{
    printf("MAX30102 Heart Rate Monitor\n");
    
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("Running on %s chip with %d CPU core(s)\n", CONFIG_IDF_TARGET, chip_info.cores);
    
    if(esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        printf("Flash size: %" PRIu32 "MB\n", flash_size / (uint32_t)(1024 * 1024));
    }

    printf("Free heap: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    printf("\n");

    // Initialize MAX30102
    ESP_LOGI(TAG, "Initializing MAX30102 Heart Rate Sensor...");
    esp_err_t ret = max30102_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MAX30102: %s", esp_err_to_name(ret));
        return;
    }
    
    // Print sensor information once
    max30102_print_sensor_info();

    ret = max30102_clear_fifo();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear FIFO: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "MAX30102 Configuration: %.0f Hz sample rate", MAX30102_SAMPLE_RATE_HZ);
    printf("- Using 300-sample sliding window (3 seconds at 100Hz)\n");
    printf("- Heart rate analysis every 1 second after 1.5 seconds initial collection\n");
    printf("- Fixed sample averaging for proper 100Hz operation\n");
    printf("===========================================================\n");

    max30102_biometrics_t biometrics = {0};
    uint64_t last_log_time_us = 0;
    uint32_t sample_count = 0;

    while (1) {
        if (max30102_is_data_ready()) {
            max30102_sample_t single_sample;
            ret = max30102_read_single_sample(&single_sample);
            if (ret == ESP_OK) {
                sample_count++;
                max30102_calculate_heart_rate(&single_sample, 1, &biometrics);
                const uint64_t now_us = esp_timer_get_time();

                if (biometrics.valid) {
                    ESP_LOGI(TAG, "HR %.1f bpm | SpO2 %.1f%% | quality %.1f | strength %.2f | IR %.0f",
                                biometrics.heart_rate,
                                biometrics.spo2,
                                biometrics.quality_metric,
                                biometrics.signal_strength,
                                biometrics.ir_dc);
                    last_log_time_us = now_us;
                } else {
                    if (now_us - last_log_time_us > 2000000ULL) { // Every 2 seconds during collection
                        ESP_LOGW(TAG, "Collecting samples... %d/300 (IR: %.0f)", 
                                    sample_count > 300 ? 300 : sample_count,
                                    biometrics.ir_dc);
                        last_log_time_us = now_us;
                    }
                }
            } else if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to read sample: %s", esp_err_to_name(ret));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
