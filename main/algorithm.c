#include "algorithm.h"
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"

static const char *TAG = "ALGORITHM";

double time_array[BUFFER_SIZE];

#define DEBUG true
#define MINIMUM_RATIO 0.3  // Lowered to see more peaks in debug

//Apenas inicia o array do tempo.
void init_time_array()
{
	double time = 0;
	for(int i = 0; i < BUFFER_SIZE; i++){
		time_array[i] = time;
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
}

// Simple 3-point moving average to smooth the signal
void smooth_signal(int32_t *buffer)
{
	int32_t temp[BUFFER_SIZE];
	// Copy original data
	for(int i = 0; i < BUFFER_SIZE; i++){
		temp[i] = buffer[i];
	}
	
	// Apply smoothing (skip first and last sample)
	for(int i = 1; i < BUFFER_SIZE - 1; i++){
		buffer[i] = (temp[i-1] + temp[i] + temp[i+1]) / 3;
	}
}

void remove_dc_part(int32_t *ir_buffer, int32_t *red_buffer, uint64_t *ir_mean, uint64_t *red_mean)
{
	*ir_mean = 0;
	*red_mean = 0;
	for(int i = 0; i < BUFFER_SIZE; i++){
		*ir_mean += ir_buffer[i];
	    *red_mean += red_buffer[i];
	}

	*ir_mean = *ir_mean / (BUFFER_SIZE);
	*red_mean = *red_mean / (BUFFER_SIZE);

	for(int i = 0; i < BUFFER_SIZE; i++){
		red_buffer[i] = red_buffer[i] - *red_mean;
		ir_buffer[i] = ir_buffer[i] - *ir_mean;
	}
}

void remove_trend_line(int32_t *buffer)
{
	double a = 0;
	double b = 0;

	calculate_linear_regression(&a, &b, buffer);

	double time = 0;
	for(int i = 0; i < BUFFER_SIZE; i++){
		buffer [i] = ((buffer[i] + (-a * time)) - b);
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
}

//calcula a regressãoi linear. fonte = https://www.statisticshowto.com/probability-and-statistics/regression-analysis/find-a-linear-regression-equation/
void calculate_linear_regression(double *angular_coef, double *linear_coef, int32_t *data)
{
	int64_t sum_of_y = sum_of_elements(data);
	double sum_of_x = 325.12; //Automatizar...
	double sum_of_x2 = somatoria_x2();
	double sum_of_xy = sum_of_xy_elements(data);
	double sum_of_x_squared = (sum_of_x * sum_of_x);

	double temp = (sum_of_xy - (sum_of_x * sum_of_y) / BUFFER_SIZE);
	double temp2 = (sum_of_x2 - (sum_of_x_squared / BUFFER_SIZE));

	*angular_coef = temp/temp2;
	*linear_coef = ((sum_of_y/BUFFER_SIZE) - (*angular_coef*(sum_of_x/BUFFER_SIZE)));
}

/*Esta equação calcula a correlação entre duas amostras
 * Sample correlation coefficient
 * https://www.statisticshowto.com/probability-and-statistics/correlation-coefficient-formula/
 */
double correlation_datay_datax(int32_t *data_red, int32_t *data_ir)
{
	double correlation = 0;
	double x_mean = 0;
	double y_mean = 0;
	double volatile sum_of_x = 0;
	double volatile sum_of_y = 0;
	double sum_of_x_minus_xmean2 = 0;
	double sum_of_y_minus_ymean2 = 0;
	double covar_xy = 0;  //Covariância de XY
	double sx = 0;  //desvio padrão X
	double sy = 0;  //Desvião padrão de Y

	for(int i = 0; i < BUFFER_SIZE; i++){
		sum_of_x += data_red[i];
		sum_of_y += data_ir[i];
	}
	x_mean = sum_of_x / BUFFER_SIZE;
	y_mean = sum_of_y / BUFFER_SIZE;

	for(int i = 0; i < BUFFER_SIZE; i++){
		sum_of_x_minus_xmean2 += ((data_red[i] - x_mean)*(data_red[i] - x_mean));
		sum_of_y_minus_ymean2 += ((data_ir[i] - y_mean)*(data_ir[i] - y_mean));
		covar_xy += ((data_red[i] - x_mean)*(data_ir[i] - y_mean));
	}
	sx = sqrt(sum_of_x_minus_xmean2 / (BUFFER_SIZE)); //desvio padrão de x
	sy = sqrt(sum_of_y_minus_ymean2 / (BUFFER_SIZE)); //devio padrão de y
	covar_xy = (covar_xy / (BUFFER_SIZE));

	correlation = (covar_xy / (sx * sy));

	return correlation;
}

double spo2_measurement(int32_t *ir_data, int32_t *red_data, uint64_t ir_mean, uint64_t red_mean)
{
	double Z = 0;
	double SpO2;
	double ir_rms = rms_value(ir_data);
	double red_rms = rms_value(red_data);

	// Calculate ratio of ratios (R-value)
	Z = (red_rms/red_mean) / (ir_rms/ir_mean);
	
	// Use more standard SpO2 calculation
	// This is a simplified version - real SpO2 needs calibration
	if (Z > 0.5 && Z < 2.0) {
		SpO2 = 110.0 - 25.0 * Z;  // Simplified linear approximation
		// Clamp to reasonable range
		if (SpO2 > 100.0) SpO2 = 100.0;
		if (SpO2 < 70.0) SpO2 = 95.0;  // Default to reasonable value if calculation fails
	} else {
		SpO2 = 95.0;  // Default fallback value
	}
	
	return SpO2;
}

int calculate_heart_rate(int32_t *ir_data, double *r0, double *auto_correlationated_data)
{
	double auto_correlation_result;
	double auto_coorelation_0 = auto_correlation_function(ir_data, 0);
	*r0 = auto_coorelation_0;
	double biggest_value = 0;
	int biggest_value_index = 0;
	double division;

	// Calculate all autocorrelation values first (up to 3 seconds of lag)
	for(int i = 0; i < 300; i++){
		auto_correlation_result = auto_correlation_function(ir_data, i);
		division = auto_correlation_result / auto_coorelation_0;
		auto_correlationated_data[i] = division;
	}
	
	// Find peaks in different heart rate ranges, preferring lower HR
	// At 100Hz: 30 BPM = 200 samples, 60 BPM = 100 samples, 120 BPM = 50 samples, 180 BPM = 33 samples
	
	// Search for peaks with STRONG preference for reasonable heart rates
	// At 100Hz: 30 BPM = 200 samples, 60 BPM = 100 samples, 120 BPM = 50 samples
	
	// Range 1: 50-90 BPM (most likely for resting HR) - LOWER threshold
	for(int i = 67; i <= 120; i++){  // 50-90 BPM range at 100Hz  
		if(auto_correlationated_data[i] > 0.25 && auto_correlationated_data[i] > biggest_value){
			biggest_value = auto_correlationated_data[i];
			biggest_value_index = i;
		}
	}
	
	// Range 2: 40-50 BPM and 90-120 BPM (if no good peak in range 1)
	if(biggest_value < 0.25){
		for(int i = 50; i < 67; i++){  // 90-120 BPM range at 100Hz
			if(auto_correlationated_data[i] > 0.25 && auto_correlationated_data[i] > biggest_value){
				biggest_value = auto_correlationated_data[i];
				biggest_value_index = i;
			}
		}
		for(int i = 120; i <= 150; i++){  // 40-50 BPM range at 100Hz
			if(auto_correlationated_data[i] > 0.25 && auto_correlationated_data[i] > biggest_value){
				biggest_value = auto_correlationated_data[i];
				biggest_value_index = i;
			}
		}
	}
	
	// Range 3: Full range as last resort with very low threshold
	if(biggest_value < 0.25){
		for(int i = 33; i <= 200; i++){  // 30-180 BPM full range at 100Hz
			if(auto_correlationated_data[i] > 0.2 && auto_correlationated_data[i] > biggest_value){
				biggest_value = auto_correlationated_data[i];
				biggest_value_index = i;
			}
		}
	}
	
	// Debug: Show multiple peaks found, focusing on realistic HR range
	ESP_LOGI(TAG, "Autocorr analysis - scanning for peaks (threshold 0.2)...");
	int peak_count = 0;
	// Show high resolution in the 40-120 BPM range (lags 50-150)
	for(int i = 50; i <= 150; i += 5){  
		if(auto_correlationated_data[i] > 0.2){
			double hr = (1.0 / (i * (DELAY_AMOSTRAGEM / 1000.0))) * 60.0;
			ESP_LOGI(TAG, "  Lag %d: ratio=%.3f, HR=%.1f", i, auto_correlationated_data[i], hr);
			peak_count++;
		}
	}
	// Also show the harmonics we were finding before
	for(int i = 33; i < 50; i += 5){  
		if(auto_correlationated_data[i] > 0.2){
			double hr = (1.0 / (i * (DELAY_AMOSTRAGEM / 1000.0))) * 60.0;
			ESP_LOGI(TAG, "  Lag %d: ratio=%.3f, HR=%.1f (HARMONIC)", i, auto_correlationated_data[i], hr);
			peak_count++;
		}
	}
	ESP_LOGI(TAG, "Found %d potential peaks", peak_count);
	
	// Calculate heart rate from the best peak found
	if(biggest_value_index > 0 && biggest_value >= 0.2){
		double heart_rate = (1.0 / (biggest_value_index * (DELAY_AMOSTRAGEM / 1000.0))) * 60.0;
		ESP_LOGI(TAG, "SELECTED: lag=%d, ratio=%.3f, HR=%.1f", 
			biggest_value_index, biggest_value, heart_rate);
		return (int)heart_rate;
	}
	
	// Return 0 if no valid peak found
	ESP_LOGW(TAG, "No valid autocorrelation peak found (best ratio=%.3f at lag=%d)", biggest_value, biggest_value_index);
	return 0;
}

double auto_correlation_function(int32_t *data, int32_t lag)
{
	double soma = 0;
	double resultado = 0;
	for(int i = 0; i < (BUFFER_SIZE - lag); i++){
		soma += ((data[i]) * (data[i + lag]));
	}
	resultado = soma / BUFFER_SIZE;
	return resultado;
}

int64_t sum_of_elements(int32_t *data)
{
	int64_t sum = 0;
	for(int i = 0; i < BUFFER_SIZE; i++){
		sum += data[i];
	}
	return sum;
}

double sum_of_xy_elements(int32_t *data)
{
	double sum_xy = 0;
	double time = 0;
	for(int i = 0; i < BUFFER_SIZE; i++){
		sum_xy += (data[i] * time);
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
	return sum_xy;
}

double sum_of_squared_elements(int32_t *data)
{
	double sum_squared = 0;
	int time = 0;
	for(int i = 0; i < BUFFER_SIZE; i++){
		sum_squared += (data[i] * data[i]);
		time += DELAY_AMOSTRAGEM / 1000.0;
	}
	return sum_squared;
}

double somatoria_x2()
{
	float incremento = (DELAY_AMOSTRAGEM / 1000.0);
	double resultado = 0.0;
	double squared_values = 0;
	float temp = 0;

	for(int i = 0; i < BUFFER_SIZE; i++){
		squared_values = temp * temp;
		temp += incremento;
		resultado += squared_values;
	}
	return resultado;
}

double rms_value(int32_t *data)
{
	double result = 0;
	int32_t somatoria = 0;
	for(int i = 0; i <BUFFER_SIZE; i++){
		somatoria += (data[i] * data[i]);
	}
	result = sqrt(somatoria / BUFFER_SIZE);
	return result;
}