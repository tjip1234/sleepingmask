#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Algorithm configuration
#define ALGORITHM_BUFFER_SIZE       512     // Match MAX30102 buffer size
#define ALGORITHM_SAMPLE_RATE_HZ    100     // Expected sample rate in Hz
#define ALGORITHM_MIN_HEART_RATE    40      // Minimum valid heart rate (BPM)
#define ALGORITHM_MAX_HEART_RATE    180     // Maximum valid heart rate (BPM)
#define ALGORITHM_MIN_SPO2          70      // Minimum valid SpO2 (%)
#define ALGORITHM_MAX_SPO2          100     // Maximum valid SpO2 (%)

// Autocorrelation thresholds
#define AUTOCORR_THRESHOLD_HIGH     0.25    // Threshold for good peak
#define AUTOCORR_THRESHOLD_LOW      0.20    // Threshold for acceptable peak

// Algorithm results structure
typedef struct {
    int heart_rate;             // Calculated heart rate in BPM
    double spo2;                // Calculated SpO2 in %
    double correlation;         // Correlation coefficient between red and IR
    double r0;                  // Autocorrelation at lag 0
    double quality;             // Signal quality metric (0-1)
    bool valid;                 // Whether results are valid
} algorithm_result_t;

// Main algorithm functions
esp_err_t algorithm_process_signals(const int32_t *ir_buffer, const int32_t *red_buffer, 
                                     uint16_t buffer_size, algorithm_result_t *result);

// Alternative FIR-based algorithm
esp_err_t algorithm_process_signals_fir(const int32_t *ir_buffer, const int32_t *red_buffer,
                                        uint16_t buffer_size, algorithm_result_t *result);

// Signal preprocessing
void algorithm_smooth_signal(int32_t *buffer, uint16_t size);
void algorithm_remove_dc(int32_t *ir_buffer, int32_t *red_buffer, uint16_t size,
                         int64_t *ir_mean, int64_t *red_mean);
void algorithm_remove_trend(int32_t *buffer, uint16_t size);

// Heart rate calculation
int algorithm_calculate_heart_rate(const int32_t *ir_data, uint16_t size, double *r0);
int algorithm_calculate_heart_rate_fir(const int32_t *ir_data, uint16_t size, double *quality);

// SpO2 calculation
double algorithm_calculate_spo2(const int32_t *ir_data, const int32_t *red_data, 
                                uint16_t size, int64_t ir_mean, int64_t red_mean);

// Signal analysis utilities
double algorithm_correlation(const int32_t *data_red, const int32_t *data_ir, uint16_t size);
double algorithm_rms(const int32_t *data, uint16_t size);
double algorithm_autocorrelation(const int32_t *data, uint16_t size, int32_t lag);

#endif // ALGORITHM_H