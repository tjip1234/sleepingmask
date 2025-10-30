/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ads1292.h"
#include "eeg_spectral.h"

static const char *TAG = "SLEEPINGMASK";

// Display interval in milliseconds
#define DISPLAY_INTERVAL_MS    100   // Update display every 100ms (10Hz)

// EEG spectral analysis parameters
#define EEG_BUFFER_SIZE        500   // 1 second of data at 500 SPS
#define EEG_BAND_UPDATE_INTERVAL_MS  1000  // Update band power every 1 second

void app_main(void)
{
    printf("\n");
    printf("╔═══════════════════════════════════════════════════╗\n");
    printf("║       ADS1292 Frontal EEG Monitor                ║\n");
    printf("╚═══════════════════════════════════════════════════╝\n");
    printf("  WARNING: Signals clipping at ±2.4V indicate:\n");
    printf("  - DC offset too large (>2.4V between electrodes)\n");
    printf("  - Poor electrode contact or missing reference\n");
    printf("  - Need AC coupling (capacitor on input) or conductive gel\n");
    printf("\n");
    
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("  Chip: %s with %d CPU core(s)\n", CONFIG_IDF_TARGET, chip_info.cores);
    
    if(esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        printf("  Flash: %" PRIu32 "MB\n", flash_size / (uint32_t)(1024 * 1024));
    }
    printf("  Free heap: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    printf("\n");

    // Initialize ADS1292
    ESP_LOGI(TAG, "Initializing ADS1292...");
    esp_err_t ret = ads1292_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADS1292: %s", esp_err_to_name(ret));
        return;
    }
    
    printf("\n");
    ESP_LOGI(TAG, "Starting data acquisition...");
    
    // Start conversion
    ret = ads1292_start_conversion();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start conversion: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize baseline for EEG
    eog_baseline_t baseline;
    eog_init_baseline(&baseline);

    // Initialize EEG spectral analysis
    eeg_spectral_init();

    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════════╗\n");
    printf("║  Sample Rate: 500 SPS  |  Gain: 12x (±200mV optimal for EEG)    ║\n");
    printf("║  Channel 1: Frontal EEG (Fp1) | Channel 2: Frontal EEG (Fp2)    ║\n");
    printf("║  Band Analysis: Delta, Theta, Alpha, Beta, Gamma                 ║\n");
    printf("║  Reference: Internal 2.4V (buffer enabled)                       ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    uint32_t sample_count = 0;
    uint64_t last_display_time = 0;
    uint64_t last_band_update_time = 0;
    ads1292_data_t data;

    // Statistics tracking
    float ch1_min = 0, ch1_max = 0, ch1_sum = 0;
    float ch2_min = 0, ch2_max = 0, ch2_sum = 0;
    uint32_t stats_samples = 0;

    // EEG signal buffers for spectral analysis
    int32_t ch1_buffer[EEG_BUFFER_SIZE];
    int32_t ch2_buffer[EEG_BUFFER_SIZE];
    uint16_t buffer_idx = 0;

    eeg_band_power_t ch1_band_power;
    eeg_band_power_t ch2_band_power;
    memset(&ch1_band_power, 0, sizeof(eeg_band_power_t));
    memset(&ch2_band_power, 0, sizeof(eeg_band_power_t));

    while (1) {
        if (ads1292_data_ready()) {
            ret = ads1292_read_data(&data);
            if (ret == ESP_OK) {
                sample_count++;

                // Update baseline for EEG
                eog_update_baseline(&baseline, &data);

                // Buffer the raw samples for spectral analysis
                ch1_buffer[buffer_idx] = data.channel1;
                ch2_buffer[buffer_idx] = data.channel2;
                buffer_idx = (buffer_idx + 1) % EEG_BUFFER_SIZE;

                // Calculate voltages
                float voltage_ch1 = ads1292_convert_to_voltage(data.channel1);
                float voltage_ch2 = ads1292_convert_to_voltage(data.channel2);

                // Update statistics
                if (stats_samples == 0) {
                    ch1_min = ch1_max = voltage_ch1;
                    ch2_min = ch2_max = voltage_ch2;
                } else {
                    if (voltage_ch1 < ch1_min) ch1_min = voltage_ch1;
                    if (voltage_ch1 > ch1_max) ch1_max = voltage_ch1;
                    if (voltage_ch2 < ch2_min) ch2_min = voltage_ch2;
                    if (voltage_ch2 > ch2_max) ch2_max = voltage_ch2;
                }
                ch1_sum += voltage_ch1;
                ch2_sum += voltage_ch2;
                stats_samples++;

                uint64_t now = esp_timer_get_time();

                // Display data at regular intervals
                if (now - last_display_time >= DISPLAY_INTERVAL_MS * 1000) {
                    float ch1_avg = ch1_sum / stats_samples;
                    float ch2_avg = ch2_sum / stats_samples;
                    float ch1_pp = (ch1_max - ch1_min) * 1000.0f; // Peak-to-peak in mV
                    float ch2_pp = (ch2_max - ch2_min) * 1000.0f;

                    // Clear previous line and print new data
                    printf("\r");
                    printf("Sample: %6lu | ", (unsigned long)sample_count);
                    printf("CH1: %8.3f mV (avg:%7.3f, p-p:%6.2f) | ",
                           voltage_ch1 * 1000.0f, ch1_avg * 1000.0f, ch1_pp);
                    printf("CH2: %8.3f mV (avg:%7.3f, p-p:%6.2f) | ",
                           voltage_ch2 * 1000.0f, ch2_avg * 1000.0f, ch2_pp);
                    printf("Status: 0x%06X", (unsigned int)data.status);

                    // Detect significant EEG activity (alpha waves, artifacts, etc.)
                    if (eog_detect_movement(&data, &baseline, 50.0f)) {
                        printf(" [ACTIVITY]");
                    }

                    fflush(stdout);

                    last_display_time = now;

                    // Reset statistics every second
                    if (stats_samples >= 250) {
                        stats_samples = 0;
                        ch1_sum = ch2_sum = 0;
                    }
                }

                // Calculate band powers periodically
                if (now - last_band_update_time >= EEG_BAND_UPDATE_INTERVAL_MS * 1000) {
                    // Calculate band powers
                    ads1292_calculate_band_power_ch1(ch1_buffer, EEG_BUFFER_SIZE, &ch1_band_power);
                    ads1292_calculate_band_power_ch2(ch2_buffer, EEG_BUFFER_SIZE, &ch2_band_power);

                    // Print band information
                    printf("\n");
                    printf("CH1 (Fp1) - ");
                    eeg_print_band_power(&ch1_band_power);
                    printf("CH2 (Fp2) - ");
                    eeg_print_band_power(&ch2_band_power);
                    printf("\n");

                    last_band_update_time = now;
                }
            }
        }

        // Small delay to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
