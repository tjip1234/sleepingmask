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
#include "algorithm.h"

static const char *TAG = "SLEEPINGMASK";

// Configure when to run algorithm (once buffer is full)
#define ANALYSIS_INTERVAL_MS    10000   // Run analysis every 10 seconds (more stable)

// Algorithm selection: 0 = Autocorrelation, 1 = FIR-based
#define USE_FIR_ALGORITHM       1

void app_main(void)
{
    printf("===============================================\n");
    printf("MAX30102 Heart Rate & SpO2 Monitor\n");
    printf("===============================================\n");
    
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("Chip: %s with %d CPU core(s)\n", CONFIG_IDF_TARGET, chip_info.cores);
    
    if(esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        printf("Flash: %" PRIu32 "MB\n", flash_size / (uint32_t)(1024 * 1024));
    }
    printf("Free heap: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    printf("===============================================\n\n");

    // Initialize MAX30102
    ESP_LOGI(TAG, "Initializing MAX30102...");
    esp_err_t ret = max30102_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MAX30102: %s", esp_err_to_name(ret));
        return;
    }
    
    max30102_print_sensor_info();
    max30102_clear_fifo();
    
    printf("\n");
    ESP_LOGI(TAG, "Starting data collection...");
    ESP_LOGI(TAG, "Using %s algorithm", USE_FIR_ALGORITHM ? "FIR-based" : "Autocorrelation");
    ESP_LOGI(TAG, "Reading samples and buffering data");
    printf("===============================================\n\n");

    uint32_t total_samples = 0;
    uint64_t last_analysis_time = 0;
    bool first_analysis_done = false;
    
    // Allocate buffers for algorithm processing
    int32_t *ir_analysis_buffer = malloc(ALGORITHM_BUFFER_SIZE * sizeof(int32_t));
    int32_t *red_analysis_buffer = malloc(ALGORITHM_BUFFER_SIZE * sizeof(int32_t));
    
    if (!ir_analysis_buffer || !red_analysis_buffer) {
        ESP_LOGE(TAG, "Failed to allocate analysis buffers");
        free(ir_analysis_buffer);
        free(red_analysis_buffer);
        return;
    }

    while (1) {
        // Read samples from FIFO
        max30102_sample_t samples[32];
        uint8_t num_samples = 32;
        
        ret = max30102_read_fifo(samples, &num_samples);
        if (ret == ESP_OK && num_samples > 0) {
            total_samples += num_samples;
            
            uint64_t now = esp_timer_get_time();
            
            // Run algorithm periodically once we have enough data
            if (total_samples >= ALGORITHM_BUFFER_SIZE && 
                (!first_analysis_done || (now - last_analysis_time >= ANALYSIS_INTERVAL_MS * 1000))) {
                
                // Get buffered samples from MAX30102 driver
                uint16_t buffer_size, samples_available;
                ret = max30102_get_buffered_samples(ir_analysis_buffer, red_analysis_buffer,
                                                    &buffer_size, &samples_available);
                
                if (ret == ESP_OK && samples_available >= ALGORITHM_BUFFER_SIZE) {
                    // Run algorithm on buffered data
                    algorithm_result_t result;
                    
#if USE_FIR_ALGORITHM
                    ret = algorithm_process_signals_fir(ir_analysis_buffer, red_analysis_buffer,
                                                        samples_available, &result);
#else
                    ret = algorithm_process_signals(ir_analysis_buffer, red_analysis_buffer,
                                                    samples_available, &result);
#endif
                    
                    if (ret == ESP_OK) {
                        float actual_rate = max30102_get_actual_sample_rate();
                        
                        printf("\n");
                        printf("═══════════════════════════════════════════\n");
                        printf("  ANALYSIS RESULTS (%s)\n", USE_FIR_ALGORITHM ? "FIR" : "AUTOCORR");
                        printf("═══════════════════════════════════════════\n");
                        printf("  Sample Rate : %.2f Hz\n", actual_rate);
                        printf("  Samples Used: %d\n", samples_available);
                        printf("  Heart Rate  : %d BPM %s\n", result.heart_rate, 
                               result.valid ? "✓" : "✗");
                        printf("  SpO2        : %.1f%% %s\n", result.spo2,
                               result.valid ? "✓" : "✗");
                        printf("  Correlation : %.3f\n", result.correlation);
                        printf("  Quality     : %.3f\n", result.quality);
                        printf("  R0 Value    : %.0f\n", result.r0);
                        printf("  Status      : %s\n", result.valid ? "VALID" : "INVALID");
                        printf("═══════════════════════════════════════════\n\n");
                    }
                }
                
                last_analysis_time = now;
                first_analysis_done = true;
            } else if (!first_analysis_done && total_samples % 100 == 0) {
                // Show progress while collecting initial samples
                ESP_LOGI(TAG, "Collecting samples: %d/%d", total_samples, ALGORITHM_BUFFER_SIZE);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(ir_analysis_buffer);
    free(red_analysis_buffer);
}
