#include "algorithm.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "ALGORITHM";

// Debug logging control
#define ALGORITHM_DEBUG_LOG 0

// Helper function to calculate linear regression coefficients
static void calculate_linear_regression(double *slope, double *intercept, const int32_t *data, uint16_t size)
{
	double sample_period = 1.0 / ALGORITHM_SAMPLE_RATE_HZ;
	double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
	
	for (uint16_t i = 0; i < size; i++) {
		double x = i * sample_period;
		double y = data[i];
		sum_x += x;
		sum_y += y;
		sum_xy += x * y;
		sum_x2 += x * x;
	}
	
	double n = size;
	*slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
	*intercept = (sum_y - *slope * sum_x) / n;
}

// Main algorithm processing function
esp_err_t algorithm_process_signals(const int32_t *ir_buffer, const int32_t *red_buffer,
									 uint16_t buffer_size, algorithm_result_t *result)
{
	if (!ir_buffer || !red_buffer || !result || buffer_size == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	
	// Initialize result
	memset(result, 0, sizeof(algorithm_result_t));
	
	// Create working copies of the buffers
	int32_t *ir_work = malloc(buffer_size * sizeof(int32_t));
	int32_t *red_work = malloc(buffer_size * sizeof(int32_t));
	
	if (!ir_work || !red_work) {
		free(ir_work);
		free(red_work);
		ESP_LOGE(TAG, "Failed to allocate working buffers");
		return ESP_ERR_NO_MEM;
	}
	
	memcpy(ir_work, ir_buffer, buffer_size * sizeof(int32_t));
	memcpy(red_work, red_buffer, buffer_size * sizeof(int32_t));
	
	// Preprocess signals
	algorithm_smooth_signal(ir_work, buffer_size);
	algorithm_smooth_signal(red_work, buffer_size);
	
	int64_t ir_mean, red_mean;
	algorithm_remove_dc(ir_work, red_work, buffer_size, &ir_mean, &red_mean);
	
	algorithm_remove_trend(ir_work, buffer_size);
	algorithm_remove_trend(red_work, buffer_size);
	
	// Calculate metrics
	result->correlation = algorithm_correlation(red_work, ir_work, buffer_size);
	result->heart_rate = algorithm_calculate_heart_rate(ir_work, buffer_size, &result->r0);
	result->spo2 = algorithm_calculate_spo2(ir_work, red_work, buffer_size, ir_mean, red_mean);
	
	// Calculate quality metric (based on correlation and R0)
	result->quality = fabs(result->correlation) * (result->r0 > 0 ? 1.0 : 0.0);
	
	// Validate results
	result->valid = (result->heart_rate >= ALGORITHM_MIN_HEART_RATE && 
	                 result->heart_rate <= ALGORITHM_MAX_HEART_RATE &&
	                 result->spo2 >= ALGORITHM_MIN_SPO2 && 
	                 result->spo2 <= ALGORITHM_MAX_SPO2 &&
	                 result->quality > 0.2);
	
	// Removed verbose logging - results reported in main application
	
	free(ir_work);
	free(red_work);
	
	return ESP_OK;
}

void algorithm_smooth_signal(int32_t *buffer, uint16_t size)
{
	if (size < 3) return;
	
	int32_t *temp = malloc(size * sizeof(int32_t));
	if (!temp) return;
	
	memcpy(temp, buffer, size * sizeof(int32_t));
	
	// Apply 3-point moving average (skip first and last sample)
	for (uint16_t i = 1; i < size - 1; i++) {
		buffer[i] = (temp[i-1] + temp[i] + temp[i+1]) / 3;
	}
	
	free(temp);
}

void algorithm_remove_dc(int32_t *ir_buffer, int32_t *red_buffer, uint16_t size,
                         int64_t *ir_mean, int64_t *red_mean)
{
	*ir_mean = 0;
	*red_mean = 0;
	
	for (uint16_t i = 0; i < size; i++) {
		*ir_mean += ir_buffer[i];
		*red_mean += red_buffer[i];
	}
	
	*ir_mean /= size;
	*red_mean /= size;
	
	for (uint16_t i = 0; i < size; i++) {
		ir_buffer[i] -= *ir_mean;
		red_buffer[i] -= *red_mean;
	}
}

void algorithm_remove_trend(int32_t *buffer, uint16_t size)
{
	double slope, intercept;
	calculate_linear_regression(&slope, &intercept, buffer, size);
	
	double sample_period = 1.0 / ALGORITHM_SAMPLE_RATE_HZ;
	for (uint16_t i = 0; i < size; i++) {
		double trend = slope * (i * sample_period) + intercept;
		buffer[i] -= (int32_t)trend;
	}
}

double algorithm_correlation(const int32_t *data_red, const int32_t *data_ir, uint16_t size)
{
	double x_mean = 0, y_mean = 0;
	
	// Calculate means
	for (uint16_t i = 0; i < size; i++) {
		x_mean += data_red[i];
		y_mean += data_ir[i];
	}
	x_mean /= size;
	y_mean /= size;
	
	// Calculate covariance and standard deviations
	double covar_xy = 0, sum_x_sq = 0, sum_y_sq = 0;
	for (uint16_t i = 0; i < size; i++) {
		double dx = data_red[i] - x_mean;
		double dy = data_ir[i] - y_mean;
		covar_xy += dx * dy;
		sum_x_sq += dx * dx;
		sum_y_sq += dy * dy;
	}
	
	double sx = sqrt(sum_x_sq / size);
	double sy = sqrt(sum_y_sq / size);
	
	if (sx == 0 || sy == 0) return 0;
	
	return (covar_xy / size) / (sx * sy);
}

double algorithm_calculate_spo2(const int32_t *ir_data, const int32_t *red_data,
                                uint16_t size, int64_t ir_mean, int64_t red_mean)
{
	if (ir_mean == 0 || red_mean == 0) return 95.0;  // Default fallback
	
	double ir_rms = algorithm_rms(ir_data, size);
	double red_rms = algorithm_rms(red_data, size);
	
	// Calculate ratio of ratios (R-value)
	double R = (red_rms / red_mean) / (ir_rms / ir_mean);
	
	// Use standard SpO2 calculation (simplified linear approximation)
	// Real SpO2 requires calibration curves
	double spo2;
	if (R > 0.5 && R < 2.0) {
		spo2 = 110.0 - 25.0 * R;
		// Clamp to reasonable range
		if (spo2 > 100.0) spo2 = 100.0;
		if (spo2 < 70.0) spo2 = 95.0;
	} else {
		spo2 = 95.0;  // Default fallback
	}
	
	return spo2;
}

int algorithm_calculate_heart_rate(const int32_t *ir_data, uint16_t size, double *r0)
{
	// Calculate autocorrelation at lag 0
	double r0_value = algorithm_autocorrelation(ir_data, size, 0);
	*r0 = r0_value;
	
	if (r0_value == 0) {
		ESP_LOGW(TAG, "Autocorrelation R0 is zero");
		return 0;
	}
	
	double sample_period = 1.0 / ALGORITHM_SAMPLE_RATE_HZ;
	int max_lag = (int)(3.0 / sample_period);  // Up to 3 seconds of lag
	if (max_lag > size - 1) max_lag = size - 1;
	
	double best_ratio = 0;
	int best_lag = 0;
	
	// Calculate autocorrelation for different lags
	// At 100Hz: 30 BPM = 200 samples, 60 BPM = 100 samples, 120 BPM = 50 samples, 180 BPM = 33 samples
	int min_lag = (int)(60.0 / (ALGORITHM_MAX_HEART_RATE * sample_period));  // 180 BPM
	int max_lag_search = (int)(60.0 / (ALGORITHM_MIN_HEART_RATE * sample_period));  // 40 BPM
	if (max_lag_search > max_lag) max_lag_search = max_lag;
	
	// Range 1: 50-90 BPM (most likely for resting HR)
	int range1_min = (int)(60.0 / (90 * sample_period));  // 90 BPM
	int range1_max = (int)(60.0 / (50 * sample_period));  // 50 BPM
	for (int lag = range1_min; lag <= range1_max && lag < max_lag; lag++) {
		double r = algorithm_autocorrelation(ir_data, size, lag);
		double ratio = r / r0_value;
		if (ratio > AUTOCORR_THRESHOLD_HIGH && ratio > best_ratio) {
			best_ratio = ratio;
			best_lag = lag;
		}
	}
	
	// Range 2: 40-50 and 90-120 BPM (if no good peak in range 1)
	if (best_ratio < AUTOCORR_THRESHOLD_HIGH) {
		// 90-120 BPM
		int range2a_min = (int)(60.0 / (120 * sample_period));
		int range2a_max = (int)(60.0 / (90 * sample_period));
		for (int lag = range2a_min; lag < range2a_max && lag < max_lag; lag++) {
			double r = algorithm_autocorrelation(ir_data, size, lag);
			double ratio = r / r0_value;
			if (ratio > AUTOCORR_THRESHOLD_HIGH && ratio > best_ratio) {
				best_ratio = ratio;
				best_lag = lag;
			}
		}
		
		// 40-50 BPM
		int range2b_min = (int)(60.0 / (50 * sample_period));
		int range2b_max = (int)(60.0 / (40 * sample_period));
		for (int lag = range2b_min; lag <= range2b_max && lag < max_lag; lag++) {
			double r = algorithm_autocorrelation(ir_data, size, lag);
			double ratio = r / r0_value;
			if (ratio > AUTOCORR_THRESHOLD_HIGH && ratio > best_ratio) {
				best_ratio = ratio;
				best_lag = lag;
			}
		}
	}
	
	// Range 3: Full range with lower threshold
	if (best_ratio < AUTOCORR_THRESHOLD_HIGH) {
		for (int lag = min_lag; lag <= max_lag_search; lag++) {
			double r = algorithm_autocorrelation(ir_data, size, lag);
			double ratio = r / r0_value;
			if (ratio > AUTOCORR_THRESHOLD_LOW && ratio > best_ratio) {
				best_ratio = ratio;
				best_lag = lag;
			}
		}
	}
	
	// Calculate heart rate from best peak
	if (best_lag > 0 && best_ratio >= AUTOCORR_THRESHOLD_LOW) {
		double heart_rate = 60.0 / (best_lag * sample_period);
		// Return valid heart rate
		return (int)(heart_rate + 0.5);  // Round to nearest integer
	}
	
	// No valid peak found
	return 0;
}

double algorithm_autocorrelation(const int32_t *data, uint16_t size, int32_t lag)
{
	if (lag >= size) return 0;
	
	double sum = 0;
	for (uint16_t i = 0; i < (size - lag); i++) {
		sum += (double)data[i] * (double)data[i + lag];
	}
	
	return sum / size;
}

double algorithm_rms(const int32_t *data, uint16_t size)
{
	double sum_sq = 0;
	for (uint16_t i = 0; i < size; i++) {
		sum_sq += (double)data[i] * (double)data[i];
	}
	return sqrt(sum_sq / size);
}

// ============================================================================
// FIR-BASED HEART RATE ALGORITHM
// ============================================================================

#include "FIR-coeff.h"

// Heart rate smoothing
static int last_hr_fir = 0;
static int hr_history[5] = {0};
static int hr_history_idx = 0;

static int smooth_heart_rate(int new_hr)
{
	// Add to history
	hr_history[hr_history_idx] = new_hr;
	hr_history_idx = (hr_history_idx + 1) % 5;
	
	// Calculate median of last 5 readings
	int sorted[5];
	memcpy(sorted, hr_history, sizeof(sorted));
	
	// Simple bubble sort
	for (int i = 0; i < 5; i++) {
		for (int j = i + 1; j < 5; j++) {
			if (sorted[i] > sorted[j]) {
				int temp = sorted[i];
				sorted[i] = sorted[j];
				sorted[j] = temp;
			}
		}
	}
	
	return sorted[2];  // Return median
}

// Downsample from 100Hz to 25Hz by taking every 4th sample
static uint16_t downsample_signal(const int32_t *input, uint16_t input_size, 
                                  float *output, uint16_t decimate_factor)
{
	uint16_t output_size = 0;
	for (uint16_t i = 0; i < input_size; i += decimate_factor) {
		output[output_size++] = (float)input[i];
	}
	return output_size;
}

// Simple FIR filter implementation
static void fir_filter_apply(const float *input, float *output, uint16_t size)
{
	// Initialize output
	for (uint16_t i = 0; i < size; i++) {
		output[i] = 0.0f;
		
		// Apply FIR filter
		for (int j = 0; j < HR_BP_TAPS; j++) {
			int input_idx = i - j;
			if (input_idx >= 0) {
				output[i] += (float)hr_bp[j] * input[input_idx];
			}
		}
	}
}

static int find_peaks(const float *data, uint16_t size, uint16_t *peak_indices, 
                      uint16_t max_peaks, float threshold)
{
	int peak_count = 0;
	
	// Find peaks (local maxima above threshold)
	// Looking for positive peaks in a zero-mean signal
	for (uint16_t i = 2; i < size - 2 && peak_count < max_peaks; i++) {
		// Peak must be above threshold and higher than neighbors
		if (data[i] > threshold && 
		    data[i] > data[i-1] && 
		    data[i] > data[i+1] &&
		    data[i] > data[i-2] &&
		    data[i] > data[i+2]) {
			
			// Ensure minimum spacing between peaks (avoid double-counting)
			bool too_close = false;
			if (peak_count > 0) {
				uint16_t spacing = i - peak_indices[peak_count - 1];
				if (spacing < 6) {  // Minimum 6 samples at 25Hz = 0.24s = 250 BPM max
					too_close = true;
				}
			}
			
			if (!too_close) {
				peak_indices[peak_count++] = i;
			}
		}
	}
	
	return peak_count;
}

static int calculate_hr_from_peaks(const uint16_t *peak_indices, int peak_count, 
                                   double sample_period)
{
	if (peak_count < 3) return 0;  // Need at least 3 peaks for good estimate
	
	// Calculate all intervals
	double intervals[100];
	int interval_count = 0;
	
	ESP_LOGI(TAG, "Peak analysis: %d peaks, sample_period=%.4f s", peak_count, sample_period);
	
	for (int i = 1; i < peak_count; i++) {
		uint16_t sample_diff = peak_indices[i] - peak_indices[i-1];
		double interval = sample_diff * sample_period;
		double bpm = 60.0 / interval;
		
		ESP_LOGI(TAG, "  Peak %d->%d: %d samples = %.3f s = %.1f BPM", 
		         i-1, i, sample_diff, interval, bpm);
		
		// Filter out unrealistic intervals (30-200 BPM range)
		if (bpm >= 30 && bpm <= 200) {
			intervals[interval_count++] = interval;
		}
	}
		
	
	if (interval_count < 2) return 0;
	
	// Calculate median interval (more robust than mean)
	// Simple bubble sort
	for (int i = 0; i < interval_count; i++) {
		for (int j = i + 1; j < interval_count; j++) {
			if (intervals[i] > intervals[j]) {
				double temp = intervals[i];
				intervals[i] = intervals[j];
				intervals[j] = temp;
			}
		}
	}
	
	double median_interval = intervals[interval_count / 2];
	int heart_rate = (int)(60.0 / median_interval + 0.5);
	
	ESP_LOGI(TAG, "Median interval: %.3f s = %d BPM (from %d valid intervals)", 
	         median_interval, heart_rate, interval_count);
	
	return heart_rate;
}

int algorithm_calculate_heart_rate_fir(const int32_t *ir_data, uint16_t size, double *quality)
{
	// Downsample from 100Hz to 25Hz (every 4th sample)
	const uint16_t DECIMATE_FACTOR = 4;
	const double DOWNSAMPLED_RATE = 25.0;  // Hz
	
	uint16_t downsampled_size = (size + DECIMATE_FACTOR - 1) / DECIMATE_FACTOR;
	
	// Allocate buffers
	float *downsampled = malloc(downsampled_size * sizeof(float));
	float *filtered = malloc(downsampled_size * sizeof(float));
	
	if (!downsampled || !filtered) {
		ESP_LOGE(TAG, "Failed to allocate FIR buffers");
		free(downsampled);
		free(filtered);
		*quality = 0.0;
		return 0;
	}
	
	// Downsample the signal
	uint16_t actual_size = downsample_signal(ir_data, size, downsampled, DECIMATE_FACTOR);
	
	ESP_LOGI(TAG, "Downsampled %d -> %d samples (100Hz -> 25Hz)", size, actual_size);
	
	// Apply FIR bandpass filter (0.3-4 Hz for heart rate)
	fir_filter_apply(downsampled, filtered, actual_size);
	
	// Calculate signal statistics for quality assessment (skip filter transient)
	uint16_t skip_samples = HR_BP_TAPS;
	if (skip_samples >= actual_size) skip_samples = actual_size / 4;
	
	float mean = 0, variance = 0;
	for (uint16_t i = skip_samples; i < actual_size; i++) {
		mean += filtered[i];
	}
	mean /= (actual_size - skip_samples);
	
	for (uint16_t i = skip_samples; i < actual_size; i++) {
		float diff = filtered[i] - mean;
		variance += diff * diff;
	}
	variance /= (actual_size - skip_samples);
	float std_dev = sqrtf(variance);
	
	// Quality metric based on signal strength
	*quality = fminf(std_dev / 1000.0, 1.0);  // Normalize
	
	// Find peaks in filtered signal
	// Use more aggressive threshold since signal is already filtered
	float threshold = std_dev * 0.5;  // Simple threshold based on variability
	uint16_t peak_indices[100];
	int peak_count = find_peaks(filtered, actual_size, peak_indices, 100, threshold);
	
	// Log peak positions
	if (peak_count > 0 && peak_count <= 10) {
		ESP_LOGI(TAG, "Peak positions (at 25Hz):");
		for (int i = 0; i < peak_count; i++) {
			ESP_LOGI(TAG, "  Peak %d: sample %d (%.2f s)", 
			         i, peak_indices[i], peak_indices[i] / DOWNSAMPLED_RATE);
		}
	}
	
	ESP_LOGI(TAG, "FIR: Found %d peaks, mean=%.1f, std=%.1f, threshold=%.1f",
	         peak_count, mean, std_dev, threshold);
	
	// Calculate heart rate from peak intervals
	double sample_period = 1.0 / DOWNSAMPLED_RATE;
	int heart_rate = calculate_hr_from_peaks(peak_indices, peak_count, sample_period);
	
	// Apply temporal smoothing
	if (heart_rate > 0) {
		heart_rate = smooth_heart_rate(heart_rate);
	}
	
	free(downsampled);
	free(filtered);
	
	if (heart_rate > 0) {
		ESP_LOGI(TAG, "FIR HR detected: %d BPM (quality: %.3f)", heart_rate, *quality);
	} else {
		ESP_LOGW(TAG, "FIR HR detection failed (peaks: %d)", peak_count);
	}
	
	return heart_rate;
}

esp_err_t algorithm_process_signals_fir(const int32_t *ir_buffer, const int32_t *red_buffer,
                                        uint16_t buffer_size, algorithm_result_t *result)
{
	if (!ir_buffer || !red_buffer || !result || buffer_size == 0) {
		return ESP_ERR_INVALID_ARG;
	}
	
	// Initialize result
	memset(result, 0, sizeof(algorithm_result_t));
	
	// Create working copies of the buffers
	int32_t *ir_work = malloc(buffer_size * sizeof(int32_t));
	int32_t *red_work = malloc(buffer_size * sizeof(int32_t));
	
	if (!ir_work || !red_work) {
		free(ir_work);
		free(red_work);
		ESP_LOGE(TAG, "Failed to allocate working buffers");
		return ESP_ERR_NO_MEM;
	}
	
	memcpy(ir_work, ir_buffer, buffer_size * sizeof(int32_t));
	memcpy(red_work, red_buffer, buffer_size * sizeof(int32_t));
	
	// Preprocess signals (remove DC only, FIR does the filtering)
	int64_t ir_mean, red_mean;
	algorithm_remove_dc(ir_work, red_work, buffer_size, &ir_mean, &red_mean);
	
	// Calculate metrics
	result->correlation = algorithm_correlation(red_work, ir_work, buffer_size);
	
	// Use FIR-based heart rate detection
	double fir_quality;
	result->heart_rate = algorithm_calculate_heart_rate_fir(ir_work, buffer_size, &fir_quality);
	result->r0 = fir_quality * 1000000.0;  // Scale for display consistency
	
	result->spo2 = algorithm_calculate_spo2(ir_work, red_work, buffer_size, ir_mean, red_mean);
	
	// Calculate quality metric
	result->quality = fir_quality * fabs(result->correlation);
	
	// Validate results
	result->valid = (result->heart_rate >= ALGORITHM_MIN_HEART_RATE && 
	                 result->heart_rate <= ALGORITHM_MAX_HEART_RATE &&
	                 result->spo2 >= ALGORITHM_MIN_SPO2 && 
	                 result->spo2 <= ALGORITHM_MAX_SPO2 &&
	                 result->quality > 0.02);  // Lower threshold for FIR
	
	ESP_LOGI(TAG, "FIR Algorithm: HR: %d BPM, SpO2: %.1f%%, Correlation: %.3f, Quality: %.3f, Valid: %s",
	         result->heart_rate, result->spo2, result->correlation, 
	         result->quality, result->valid ? "YES" : "NO");
	
	free(ir_work);
	free(red_work);
	
	return ESP_OK;
}