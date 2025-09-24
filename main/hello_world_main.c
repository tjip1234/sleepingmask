/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ads1292.h"

static const char *TAG = "SLEEPINGMASK";

void app_main(void)
{
    printf("ADS1292 Sleep Monitoring System\n");
    
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

    // Initialize ADS1292
    ESP_LOGI(TAG, "Initializing ADS1292...");
    esp_err_t ret = ads1292_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADS1292: %s", esp_err_to_name(ret));
        return;
    }

    // Start continuous data conversion
    ESP_LOGI(TAG, "Starting data conversion...");
    ret = ads1292_start_conversion();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start conversion: %s", esp_err_to_name(ret));
        return;
    }

    // Print register configuration
    ads1292_print_registers();
    
    ESP_LOGI(TAG, "EOG Configuration: 250 SPS, Gain=4x (±0.6V range)");
    ESP_LOGI(TAG, "Establishing baseline for EOG detection...");
    ESP_LOGI(TAG, "Press Ctrl+C to stop");
    printf("\nEOG Electrode Placement Tips:\n");
    printf("- Place electrodes around eyes for horizontal/vertical eye movement detection\n");
    printf("- CH1: Horizontal EOG (left-right eye movements)\n");
    printf("- CH2: Vertical EOG (up-down eye movements)\n");
    printf("- Use conductive gel for better signal quality\n");
    printf("===========================================================================\n");

    ads1292_data_t data;
    eog_baseline_t baseline;
    eog_init_baseline(&baseline);
    
    uint32_t sample_count = 0;
    uint32_t movement_count = 0;
    uint32_t last_movement_time = 0;
    
    while (1) {
        // Check if new data is available
        if (ads1292_data_ready()) {
            ret = ads1292_read_data(&data);
            if (ret == ESP_OK) {
                sample_count++;
                
                // Update baseline calculation
                eog_update_baseline(&baseline, &data);
                
                // Print data every 25th sample to reduce spam (250 SPS / 25 = 10 Hz display)
                if (sample_count % 25 == 0) {
                    eog_print_data_with_baseline(&data, &baseline);
                }
                
                
                // Print movement statistics every 1250 samples (5 seconds worth at 250 SPS)
                if (sample_count % 1250 == 0 && baseline.baseline_established) {
                    printf("\n--- 5 Second EOG Statistics ---\n");
                    printf("Total samples: %u, Eye movements detected: %u\n", 
                           (unsigned int)sample_count, (unsigned int)movement_count);
                    printf("Baseline CH1: %.6f V, CH2: %.6f V\n", 
                           baseline.baseline_ch1, baseline.baseline_ch2);
                    printf("Movement rate: %.1f movements/minute\n", 
                           (movement_count * 12.0f)); // 5 seconds * 12 = 1 minute
                    printf("-------------------------------\n\n");
                }
            } else {
                ESP_LOGW(TAG, "Failed to read data: %s", esp_err_to_name(ret));
            }
        }
        
        // Small delay to prevent overwhelming the output (250 SPS = every 4ms)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
