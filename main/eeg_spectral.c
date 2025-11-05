#include "eeg_spectral.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "EEG_SPECTRAL";

void eeg_spectral_init(void) {
    ESP_LOGI(TAG, "EEG spectral analysis initialized");
    ESP_LOGI(TAG, "Bands: Delta(0.5-4Hz), Theta(4-8Hz), Alpha(8-13Hz), Beta(13-30Hz), Gamma(30-100Hz)");
}

/**
 * Goertzel algorithm - efficiently calculates power in a specific frequency band
 * This is much more efficient than full FFT on microcontrollers
 */
static float goertzel_filter(const int32_t *signal, uint16_t signal_size,
                             float normalized_freq)
{
    // normalized_freq = actual_freq / sample_rate
    float w = 2.0f * M_PI * normalized_freq;
    float cos_w = cosf(w);
    float coeff = 2.0f * cos_w;

    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

    // Apply the filter
    for (uint16_t i = 0; i < signal_size; i++) {
        float sample = (float)signal[i];
        s0 = sample + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    // Calculate the power (magnitude squared)
    float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;

    // Normalize by signal length
    power = power / (signal_size * signal_size);

    return power;
}

/**
 * Calculate power in a frequency band using multiple Goertzel filters
 * across the band range
 */
float eeg_goertzel_band_power(const int32_t *signal, uint16_t signal_size,
                              float freq_min, float freq_max, uint16_t sample_rate_hz)
{
    if (!signal || signal_size == 0 || sample_rate_hz == 0) {
        return 0.0f;
    }

    float total_power = 0.0f;
    uint16_t num_freqs = 0;

    // Sample frequencies across the band with 1 Hz resolution
    for (float freq = freq_min; freq <= freq_max; freq += 1.0f) {
        float normalized_freq = freq / (float)sample_rate_hz;

        // Skip if frequency is beyond Nyquist
        if (normalized_freq >= 0.5f) break;

        float power = goertzel_filter(signal, signal_size, normalized_freq);
        total_power += power;
        num_freqs++;
    }

    // Return average power across the band
    if (num_freqs > 0) {
        return total_power / num_freqs;
    }

    return 0.0f;
}

/**
 * Calculate EEG band powers
 */
void eeg_calculate_band_power(const int32_t *signal, uint16_t signal_size,
                              uint16_t sample_rate_hz, eeg_band_power_t *band_power)
{
    if (!signal || !band_power || signal_size == 0) {
        return;
    }

    // Calculate power in each band
    band_power->delta_power = eeg_goertzel_band_power(signal, signal_size,
                                                      EEG_DELTA_MIN, EEG_DELTA_MAX,
                                                      sample_rate_hz);

    band_power->theta_power = eeg_goertzel_band_power(signal, signal_size,
                                                      EEG_THETA_MIN, EEG_THETA_MAX,
                                                      sample_rate_hz);

    band_power->alpha_power = eeg_goertzel_band_power(signal, signal_size,
                                                      EEG_ALPHA_MIN, EEG_ALPHA_MAX,
                                                      sample_rate_hz);

    band_power->beta_power = eeg_goertzel_band_power(signal, signal_size,
                                                     EEG_BETA_MIN, EEG_BETA_MAX,
                                                     sample_rate_hz);

    band_power->gamma_power = eeg_goertzel_band_power(signal, signal_size,
                                                      EEG_GAMMA_MIN, EEG_GAMMA_MAX,
                                                      sample_rate_hz);

    // Calculate total power
    band_power->total_power = band_power->delta_power + band_power->theta_power +
                             band_power->alpha_power + band_power->beta_power +
                             band_power->gamma_power;

    // Find dominant frequency by checking which band has max power
    float max_power = band_power->delta_power;
    band_power->dominant_freq = (EEG_DELTA_MIN + EEG_DELTA_MAX) / 2.0f;

    if (band_power->theta_power > max_power) {
        max_power = band_power->theta_power;
        band_power->dominant_freq = (EEG_THETA_MIN + EEG_THETA_MAX) / 2.0f;
    }
    if (band_power->alpha_power > max_power) {
        max_power = band_power->alpha_power;
        band_power->dominant_freq = (EEG_ALPHA_MIN + EEG_ALPHA_MAX) / 2.0f;
    }
    if (band_power->beta_power > max_power) {
        max_power = band_power->beta_power;
        band_power->dominant_freq = (EEG_BETA_MIN + EEG_BETA_MAX) / 2.0f;
    }
    if (band_power->gamma_power > max_power) {
        max_power = band_power->gamma_power;
        band_power->dominant_freq = (EEG_GAMMA_MIN + EEG_GAMMA_MAX) / 2.0f;
    }
}

/**
 * Welch's method for power spectral density estimation
 * More statistically robust but computationally heavier
 */
void eeg_calculate_band_power_welch(const int32_t *signal, uint16_t signal_size,
                                    uint16_t sample_rate_hz, eeg_band_power_t *band_power)
{
    // For now, use simple Goertzel approach
    // Welch's method would require multiple segments and Hann windowing
    // which is more complex but more robust
    eeg_calculate_band_power(signal, signal_size, sample_rate_hz, band_power);
}

/**
 * Print band power values in a nice format
 */
void eeg_print_band_power(const eeg_band_power_t *band_power)
{
    if (!band_power) return;

    // Calculate relative power percentages
    float delta_pct = (band_power->total_power > 0) ?
                      (band_power->delta_power / band_power->total_power) * 100.0f : 0.0f;
    float theta_pct = (band_power->total_power > 0) ?
                      (band_power->theta_power / band_power->total_power) * 100.0f : 0.0f;
    float alpha_pct = (band_power->total_power > 0) ?
                      (band_power->alpha_power / band_power->total_power) * 100.0f : 0.0f;
    float beta_pct = (band_power->total_power > 0) ?
                     (band_power->beta_power / band_power->total_power) * 100.0f : 0.0f;
    float gamma_pct = (band_power->total_power > 0) ?
                      (band_power->gamma_power / band_power->total_power) * 100.0f : 0.0f;

    printf("EEG Bands [μV²]: δ:%.2e(%.1f%%) θ:%.2e(%.1f%%) α:%.2e(%.1f%%) β:%.2e(%.1f%%) γ:%.2e(%.1f%%) | Dominant:%.1f Hz\n",
           band_power->delta_power, delta_pct,
           band_power->theta_power, theta_pct,
           band_power->alpha_power, alpha_pct,
           band_power->beta_power, beta_pct,
           band_power->gamma_power, gamma_pct,
           band_power->dominant_freq);
}

/**
 * 50Hz/60Hz notch filter using IIR biquad filter
 * This removes power line interference from EEG signals
 * 
 * The filter is a second-order IIR notch filter (biquad)
 * Transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 */
void eeg_notch_filter_50hz(int32_t *signal, uint16_t signal_size,
                           uint16_t sample_rate_hz, float notch_freq_hz, float q_factor)
{
    if (!signal || signal_size == 0 || sample_rate_hz == 0) {
        return;
    }

    // Calculate filter coefficients
    float w0 = 2.0f * M_PI * notch_freq_hz / (float)sample_rate_hz;
    float alpha = sinf(w0) / (2.0f * q_factor);
    float cos_w0 = cosf(w0);

    // Biquad filter coefficients (normalized)
    float b0 = 1.0f;
    float b1 = -2.0f * cos_w0;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;

    // Normalize coefficients
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;

    // State variables for the filter (Direct Form I)
    float x1 = 0.0f, x2 = 0.0f;  // Input history
    float y1 = 0.0f, y2 = 0.0f;  // Output history

    // Apply filter to signal (in-place)
    for (uint16_t i = 0; i < signal_size; i++) {
        float x0 = (float)signal[i];
        
        // Compute output using difference equation
        float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        // Update state variables
        x2 = x1;
        x1 = x0;
        y2 = y1;
        y1 = y0;
        
        // Write filtered output back
        signal[i] = (int32_t)y0;
    }

    ESP_LOGI(TAG, "Applied %dHz notch filter (Q=%.1f) to %d samples @ %dHz",
             (int)notch_freq_hz, q_factor, signal_size, sample_rate_hz);
}
