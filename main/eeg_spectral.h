#ifndef EEG_SPECTRAL_H
#define EEG_SPECTRAL_H

#include <stdint.h>
#include <stdbool.h>

// EEG frequency bands (Hz)
#define EEG_DELTA_MIN       0.5f
#define EEG_DELTA_MAX       4.0f
#define EEG_THETA_MIN       4.0f
#define EEG_THETA_MAX       8.0f
#define EEG_ALPHA_MIN       8.0f
#define EEG_ALPHA_MAX       13.0f
#define EEG_BETA_MIN        13.0f
#define EEG_BETA_MAX        30.0f
#define EEG_GAMMA_MIN       30.0f
#define EEG_GAMMA_MAX       100.0f

// EEG band power results
typedef struct {
    float delta_power;      // 0.5-4 Hz
    float theta_power;      // 4-8 Hz
    float alpha_power;      // 8-13 Hz
    float beta_power;       // 13-30 Hz
    float gamma_power;      // 30-100 Hz
    float total_power;      // Sum of all bands
    float dominant_freq;    // Frequency with highest power
} eeg_band_power_t;

// Initialize spectral analysis
void eeg_spectral_init(void);

// Calculate band powers using FFT-based approach
// Sample rate should be 250 Hz for ADS1292
void eeg_calculate_band_power(const int32_t *signal, uint16_t signal_size,
                              uint16_t sample_rate_hz, eeg_band_power_t *band_power);

// Calculate power spectrum using Welch's method (more robust)
void eeg_calculate_band_power_welch(const int32_t *signal, uint16_t signal_size,
                                    uint16_t sample_rate_hz, eeg_band_power_t *band_power);

// Simple FFT for power spectrum
typedef struct {
    float *magnitude;
    float *phase;
    uint16_t size;
} fft_result_t;

// Goertzel algorithm for specific frequency bands (computationally efficient)
float eeg_goertzel_band_power(const int32_t *signal, uint16_t signal_size,
                              float freq_min, float freq_max, uint16_t sample_rate_hz);

// Print band powers nicely formatted
void eeg_print_band_power(const eeg_band_power_t *band_power);

#endif // EEG_SPECTRAL_H
