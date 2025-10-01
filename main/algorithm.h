#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

#define BUFFER_SIZE 1000  // 10 seconds at 100Hz
#define DELAY_AMOSTRAGEM 10  // 10ms sampling delay (100Hz)

// Function declarations
void init_time_array(void);
void smooth_signal(int32_t *buffer);
void remove_dc_part(int32_t *ir_buffer, int32_t *red_buffer, uint64_t *ir_mean, uint64_t *red_mean);
void remove_trend_line(int32_t *buffer);
void calculate_linear_regression(double *angular_coef, double *linear_coef, int32_t *data);
double correlation_datay_datax(int32_t *data_red, int32_t *data_ir);
double spo2_measurement(int32_t *ir_data, int32_t *red_data, uint64_t ir_mean, uint64_t red_mean);
int calculate_heart_rate(int32_t *ir_data, double *r0, double *auto_correlationated_data);
double auto_correlation_function(int32_t *data, int32_t lag);
int64_t sum_of_elements(int32_t *data);
double sum_of_xy_elements(int32_t *data);
double sum_of_squared_elements(int32_t *data);
double somatoria_x2(void);
double rms_value(int32_t *data);

#endif // ALGORITHM_H