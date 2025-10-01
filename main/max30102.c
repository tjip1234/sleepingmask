#include "max30102.h"
#include "algorithm.h"
// Using autocorrelation algorithm instead of filtering

#include <math.h>
#include <string.h>
#include <sys/param.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_TIMEOUT_MS                  100
#define MAX30102_DEFAULT_RED_CURRENT    0x28
#define MAX30102_DEFAULT_IR_CURRENT     0x28
#define MAX30102_DECIMATION_FACTOR      5

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// No filtering needed - using autocorrelation algorithm

typedef struct {
	// Circular buffer for continuous processing
	int32_t ir_buffer[BUFFER_SIZE];
	int32_t red_buffer[BUFFER_SIZE];
	uint16_t buffer_index;
	bool buffer_full;
	
	// Analysis timing
	uint64_t last_analysis_time;
	uint32_t samples_since_analysis;
	
	// Last calculated values
	int last_heart_rate;
	double last_spo2;
	double r0_value;
	double auto_correlation_data[1000];  // Match buffer size
} hr_processing_state_t;

static const char *TAG = "MAX30102";

static bool s_i2c_initialised = false;
static hr_processing_state_t s_hr_state;

// Static working buffers to avoid stack overflow
static int32_t s_ir_work[BUFFER_SIZE];
static int32_t s_red_work[BUFFER_SIZE];

// Simple heart rate validation
static int validate_heart_rate(hr_processing_state_t *state, int new_hr) {
	// Reject impossible heart rate changes (>40 BPM jump)
	if (state->last_heart_rate > 0 && abs(new_hr - state->last_heart_rate) > 40) {
		ESP_LOGW(TAG, "HR change too large: %d->%d BPM, keeping previous", state->last_heart_rate, new_hr);
		return state->last_heart_rate;
	}
	return new_hr;
}

// No filtering needed - using autocorrelation algorithm

static esp_err_t max30102_i2c_init(void)
{
	if (s_i2c_initialised) {
		return ESP_OK;
	}

	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = MAX30102_SDA_PIN,
		.scl_io_num = MAX30102_SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = MAX30102_I2C_FREQ,
		.clk_flags = 0,
	};

	esp_err_t ret = i2c_param_config(MAX30102_I2C_PORT, &conf);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
		return ret;
	}

	ret = i2c_driver_install(MAX30102_I2C_PORT, conf.mode, 0, 0, 0);
	if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
		s_i2c_initialised = true;
		return ESP_OK;
	}

	if (ret == ESP_ERR_INVALID_STATE) {
		s_i2c_initialised = true;
		return ESP_OK;
	}

	ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
	return ret;
}

static esp_err_t max30102_write(uint8_t reg, const uint8_t *data, size_t len)
{
	uint8_t buffer[1 + MAX(1, len)];
	buffer[0] = reg;
	if (len && data != NULL) {
		memcpy(&buffer[1], data, len);
	}

	return i2c_master_write_to_device(MAX30102_I2C_PORT, MAX30102_I2C_ADDR, buffer, len + 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t max30102_read(uint8_t reg, uint8_t *data, size_t len)
{
	return i2c_master_write_read_device(MAX30102_I2C_PORT, MAX30102_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

// Old FIR functions removed - now using Arduino library FIR implementation

static void max30102_processing_init(hr_processing_state_t *state)
{
	// Initialize autocorrelation algorithm state
	memset(state->ir_buffer, 0, sizeof(state->ir_buffer));
	memset(state->red_buffer, 0, sizeof(state->red_buffer));
	state->buffer_index = 0;
	state->buffer_full = false;
	state->last_analysis_time = 0;
	state->samples_since_analysis = 0;
	state->last_heart_rate = 0;
	state->last_spo2 = 0.0;
	state->r0_value = 0.0;
	memset(state->auto_correlation_data, 0, sizeof(state->auto_correlation_data));
	
	// Initialize time array for algorithm
	init_time_array();
}

// Old FFT-based heart rate calculation functions removed - now using PBA algorithm

esp_err_t max30102_init(void)
{
	ESP_LOGI(TAG, "Initializing I2C on SDA=%d, SCL=%d...", MAX30102_SDA_PIN, MAX30102_SCL_PIN);
	esp_err_t ret = max30102_i2c_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
		return ret;
	}

	// Try to detect device on I2C bus
	ESP_LOGI(TAG, "Scanning for MAX30102 at address 0x%02X...", MAX30102_I2C_ADDR);
	uint8_t test_data = 0;
	ret = max30102_read_register(MAX30102_REG_PART_ID, &test_data);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "No response from MAX30102 sensor. Check wiring and power.");
		ESP_LOGE(TAG, "Expected: SDA=GPIO%d, SCL=GPIO%d, VCC=3.3V, GND=GND", MAX30102_SDA_PIN, MAX30102_SCL_PIN);
		return ret;
	}

	ESP_LOGI(TAG, "Resetting MAX30102...");
	ret = max30102_reset();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "MAX30102 reset failed: %s", esp_err_to_name(ret));
		return ret;
	}

	vTaskDelay(pdMS_TO_TICKS(10));

	ESP_LOGI(TAG, "Checking part ID...");
	ret = max30102_check_part_id();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Part ID check failed: %s", esp_err_to_name(ret));
		return ret;
	}

	ret = max30102_configure_fifo(MAX30102_FIFO_SAMPLE_AVG_1, true, MAX30102_FIFO_ALMOST_FULL_7);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = max30102_configure_spo2(MAX30102_SPO2_ADC_RGE_16384, MAX30102_SPO2_SR_100, MAX30102_SPO2_LEDpw_400);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = max30102_set_led_brightness(MAX30102_DEFAULT_RED_CURRENT, MAX30102_DEFAULT_IR_CURRENT);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = max30102_set_mode(MAX30102_MODE_RED_IR);
	if (ret != ESP_OK) {
		return ret;
	}

	max30102_enable_interrupts(MAX30102_INT_DATA_RDY | MAX30102_INT_A_FULL | MAX30102_INT_ALC_OVF);
	max30102_clear_interrupts();
	max30102_processing_init(&s_hr_state);
	return ESP_OK;
}

esp_err_t max30102_reset(void)
{
	uint8_t data = 0x40; // Reset bit
	esp_err_t ret = max30102_write(MAX30102_REG_MODE_CONFIG, &data, 1);
	if (ret == ESP_OK) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	return ret;
}

esp_err_t max30102_shutdown(bool shutdown)
{
	uint8_t cfg;
	esp_err_t ret = max30102_read_register(MAX30102_REG_MODE_CONFIG, &cfg);
	if (ret != ESP_OK) {
		return ret;
	}

	if (shutdown) {
		cfg |= 0x80;
	} else {
		cfg &= ~0x80;
	}

	return max30102_write_register(MAX30102_REG_MODE_CONFIG, cfg);
}

esp_err_t max30102_set_mode(uint8_t mode)
{
	return max30102_write_register(MAX30102_REG_MODE_CONFIG, mode & 0x07);
}

esp_err_t max30102_set_led_brightness(uint8_t red_brightness, uint8_t ir_brightness)
{
	esp_err_t ret = max30102_write_register(MAX30102_REG_LED1_PULSE_AMPLITUDE, red_brightness);
	if (ret != ESP_OK) {
		return ret;
	}
	return max30102_write_register(MAX30102_REG_LED2_PULSE_AMPLITUDE, ir_brightness);
}

esp_err_t max30102_configure_fifo(uint8_t sample_avg, bool rollover, uint8_t almost_full)
{
	uint8_t config = sample_avg & 0xE0;
	if (rollover) {
		config |= MAX30102_FIFO_ROLLOVER_EN;
	}
	config |= (almost_full & 0x0F);
	return max30102_write_register(MAX30102_REG_FIFO_CONFIG, config);
}

esp_err_t max30102_configure_spo2(uint8_t adc_range, uint8_t sample_rate, uint8_t led_pulse_width)
{
	uint8_t config = (adc_range & 0x60) | (sample_rate & 0x1C) | (led_pulse_width & 0x03);
	return max30102_write_register(MAX30102_REG_SPO2_CONFIG, config);
}

esp_err_t max30102_read_register(uint8_t reg, uint8_t *data)
{
	return max30102_read(reg, data, 1);
}

esp_err_t max30102_write_register(uint8_t reg, uint8_t data)
{
	return max30102_write(reg, &data, 1);
}

esp_err_t max30102_read_fifo(max30102_sample_t *samples, uint8_t *num_samples)
{
	if (!samples || !num_samples || *num_samples == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t available = max30102_get_fifo_samples_available();
	if (available == 0) {
		*num_samples = 0;
		return ESP_OK;
	}

	uint8_t to_read = MIN(available, *num_samples);
	const size_t bytes_needed = to_read * MAX30102_BYTES_PER_SAMPLE;
	uint8_t buffer[MAX30102_BYTES_PER_SAMPLE * MAX30102_FIFO_DEPTH];

	esp_err_t ret = max30102_read(MAX30102_REG_FIFO_DATA, buffer, bytes_needed);
	if (ret != ESP_OK) {
		return ret;
	}

	for (uint8_t i = 0; i < to_read; ++i) {
		const uint8_t *chunk = &buffer[i * MAX30102_BYTES_PER_SAMPLE];
		uint32_t red = ((uint32_t)chunk[0] << 16) | ((uint32_t)chunk[1] << 8) | chunk[2];
		uint32_t ir = ((uint32_t)chunk[3] << 16) | ((uint32_t)chunk[4] << 8) | chunk[5];
		red &= 0x3FFFF;
		ir &= 0x3FFFF;
		samples[i].red = red;
		samples[i].ir = ir;
		samples[i].green = 0;
	}

	*num_samples = to_read;
	return ESP_OK;
}

esp_err_t max30102_read_single_sample(max30102_sample_t *sample)
{
	if (!sample) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t count = 1;
	esp_err_t ret = max30102_read_fifo(sample, &count);
	if (ret != ESP_OK) {
		return ret;
	}
	return (count == 1) ? ESP_OK : ESP_FAIL;
}

esp_err_t max30102_get_sensor_info(void)
{
	uint8_t part_id = 0;
	uint8_t rev_id = 0;
	esp_err_t ret = max30102_read_register(MAX30102_REG_PART_ID, &part_id);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_read_register(MAX30102_REG_REV_ID, &rev_id);
	if (ret != ESP_OK) {
		return ret;
	}

	ESP_LOGI(TAG, "MAX30102 Part ID: 0x%02X Revision: 0x%02X", part_id, rev_id);
	return ESP_OK;
}

esp_err_t max30102_clear_fifo(void)
{
	esp_err_t ret = max30102_write_register(MAX30102_REG_FIFO_WR_PTR, 0x00);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_write_register(MAX30102_REG_FIFO_RD_PTR, 0x00);
	if (ret != ESP_OK) {
		return ret;
	}
	return max30102_write_register(MAX30102_REG_FIFO_OVERFLOW_COUNTER, 0x00);
}

esp_err_t max30102_read_temperature(float *temperature)
{
	if (!temperature) {
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t config = 0x01; // Trigger temperature measurement
	esp_err_t ret = max30102_write_register(MAX30102_REG_TEMP_CONFIG, config);
	if (ret != ESP_OK) {
		return ret;
	}

	vTaskDelay(pdMS_TO_TICKS(40));

	uint8_t temp_int = 0;
	uint8_t temp_frac = 0;
	ret = max30102_read_register(MAX30102_REG_TEMP_INTEGER, &temp_int);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_read_register(MAX30102_REG_TEMP_FRACTION, &temp_frac);
	if (ret != ESP_OK) {
		return ret;
	}

	int8_t signed_int = (int8_t)temp_int;
	*temperature = (float)signed_int + (temp_frac * 0.0625f);
	return ESP_OK;
}

bool max30102_is_data_ready(void)
{
	uint8_t status = 0;
	if (max30102_read_register(MAX30102_REG_INTERRUPT_STATUS_1, &status) != ESP_OK) {
		return false;
	}
	return (status & MAX30102_INT_DATA_RDY) != 0;
}

uint8_t max30102_get_fifo_samples_available(void)
{
	uint8_t wr_ptr = 0;
	uint8_t rd_ptr = 0;
	if (max30102_read_register(MAX30102_REG_FIFO_WR_PTR, &wr_ptr) != ESP_OK) {
		return 0;
	}
	if (max30102_read_register(MAX30102_REG_FIFO_RD_PTR, &rd_ptr) != ESP_OK) {
		return 0;
	}

	if (wr_ptr == rd_ptr) {
		return 0;
	}

	if (wr_ptr > rd_ptr) {
		return wr_ptr - rd_ptr;
	}
	return (32 + wr_ptr) - rd_ptr;
}

// Old PBA algorithm removed - now using autocorrelation algorithm

esp_err_t max30102_calculate_heart_rate(max30102_sample_t *samples, uint8_t num_samples, max30102_biometrics_t *bio)
{
	if (!samples || !bio || num_samples == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	for (uint8_t i = 0; i < num_samples; ++i) {
		const int32_t ir = (int32_t)samples[i].ir;
		const int32_t red = (int32_t)samples[i].red;

		// Skin detection: if raw IR is below 10000, no skin detected
		if (ir < 10000) {
			bio->valid = false;
			bio->heart_rate = 0.0f;
			bio->ir_dc = (float)ir;
			bio->red_dc = (float)red;
			bio->ir_baseline = (float)ir;
			bio->red_baseline = (float)red;
			bio->signal_strength = 0.0f;
			bio->quality_metric = 0.0f;
			bio->spo2 = 0.0f;
			return ESP_OK;  // No skin detected - skip processing
		}

		// Add samples to buffer
		s_hr_state.ir_buffer[s_hr_state.buffer_index] = ir;
		s_hr_state.red_buffer[s_hr_state.buffer_index] = red;
		s_hr_state.buffer_index++;

		if (s_hr_state.buffer_index >= BUFFER_SIZE) {
			s_hr_state.buffer_index = 0;
			s_hr_state.buffer_full = true;
		}

		// Minimal logging - just show progress
		static bool logged_ready = false;
		if (s_hr_state.buffer_index == 500 && !logged_ready) {
			ESP_LOGI(TAG, "Collected %d samples, will output 10-second signal...", s_hr_state.buffer_index);
			logged_ready = true;
		}

		// Process every second once we have enough data (200 samples = 4 seconds at 50Hz)
		uint64_t now_us = esp_timer_get_time();
		bool should_analyze = false;
		
		if (s_hr_state.buffer_index >= 500) {  // Have at least 5 seconds of data
			if (s_hr_state.last_analysis_time == 0) {
				// First analysis
				should_analyze = true;
			} else if (now_us - s_hr_state.last_analysis_time >= 1000000ULL) {  // 1 second elapsed
				should_analyze = true;
			}
		}
		
		if (should_analyze) {
			ESP_LOGI(TAG, "Starting heart rate analysis (samples: %d)...", s_hr_state.buffer_index);
			
			// Use static working buffers to avoid stack overflow
			memcpy(s_ir_work, s_hr_state.ir_buffer, sizeof(s_ir_work));
			memcpy(s_red_work, s_hr_state.red_buffer, sizeof(s_red_work));

			// Smooth the signals to reduce noise
			smooth_signal(s_ir_work);
			smooth_signal(s_red_work);
			
			// Remove DC component and trend
			uint64_t ir_mean, red_mean;
			remove_dc_part(s_ir_work, s_red_work, &ir_mean, &red_mean);
			remove_trend_line(s_ir_work);
			remove_trend_line(s_red_work);

			ESP_LOGI(TAG, "Calculating autocorrelation...");
			// Calculate heart rate using autocorrelation (from algorithm.c)
			double r0;
			int heart_rate = calculate_heart_rate(s_ir_work, &r0, s_hr_state.auto_correlation_data);
			
			ESP_LOGI(TAG, "Calculating SpO2...");
			// Calculate SpO2
			double spo2 = spo2_measurement(s_ir_work, s_red_work, ir_mean, red_mean);
			double correlation = correlation_datay_datax(s_red_work, s_ir_work);

			ESP_LOGI(TAG, "Analysis complete - HR=%d BPM, SpO2=%.1f%%, Correlation=%.3f, R0=%.0f", 
				heart_rate, spo2, correlation, r0);

			// Update results if values are reasonable  
			if (heart_rate > 30 && heart_rate < 200) {
				int validated_hr = validate_heart_rate(&s_hr_state, heart_rate);
				s_hr_state.last_heart_rate = validated_hr;
				bio->heart_rate = (float)validated_hr;
				bio->valid = true;
			} else {
				ESP_LOGW(TAG, "Heart rate %d BPM out of range (30-200), keeping previous value", heart_rate);
				bio->heart_rate = (float)s_hr_state.last_heart_rate;
				bio->valid = (s_hr_state.last_heart_rate > 0);
			}
			
			if (spo2 > 70 && spo2 < 100) {
				s_hr_state.last_spo2 = spo2;
				bio->spo2 = (float)spo2;
			} else {
				bio->spo2 = 98.0f;  // Default reasonable value
			}

			bio->quality_metric = (float)(correlation * 100.0);
			bio->signal_strength = (float)(r0 / 1000000.0);  // Scale R0 appropriately

			// Update analysis timing for continuous operation
			s_hr_state.last_analysis_time = now_us;
			s_hr_state.samples_since_analysis = 0;
		}
	}

	// Set baseline values
	bio->ir_dc = (float)samples[num_samples-1].ir;
	bio->red_dc = (float)samples[num_samples-1].red;
	bio->ir_baseline = bio->ir_dc;
	bio->red_baseline = bio->red_dc;

	return ESP_OK;
}





void max30102_print_biometrics(const max30102_biometrics_t *bio)
{
	if (!bio) {
		return;
	}
	if (bio->valid) {
		printf("Heart Rate: %.1f BPM\n", bio->heart_rate);
	} else {
		printf("Heart Rate: ---\n");
	}
}



esp_err_t max30102_check_part_id(void)
{
	uint8_t part_id = 0;
	esp_err_t ret = max30102_read_register(MAX30102_REG_PART_ID, &part_id);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to read part ID register: %s", esp_err_to_name(ret));
		return ret;
	}

	ESP_LOGI(TAG, "Read part ID: 0x%02X (expected: 0x%02X)", part_id, MAX30102_PART_ID);

	if (part_id != MAX30102_PART_ID) {
		ESP_LOGE(TAG, "Unexpected part ID: 0x%02X, expected: 0x%02X", part_id, MAX30102_PART_ID);
		return ESP_FAIL;
	}

	return ESP_OK;
}

void max30102_print_sensor_info(void)
{
	max30102_get_sensor_info();
}

void max30102_enable_interrupts(uint8_t interrupt_mask)
{
	max30102_write_register(MAX30102_REG_INTERRUPT_ENABLE_1, interrupt_mask);
}

void max30102_clear_interrupts(void)
{
	uint8_t dummy = 0;
	max30102_read_register(MAX30102_REG_INTERRUPT_STATUS_1, &dummy);
	max30102_read_register(MAX30102_REG_INTERRUPT_STATUS_2, &dummy);
}



