#include "max30102.h"

#include <string.h>
#include <sys/param.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_TIMEOUT_MS                  100
#define MAX30102_DEFAULT_RED_CURRENT    0x28
#define MAX30102_DEFAULT_IR_CURRENT     0x28
#define MAX30102_TARGET_SAMPLE_RATE     100  // Target Hz for output
#define MAX30102_BUFFER_SIZE            512  // Buffer size for samples

// Sample rate reporting flag
#define MAX30102_LOG_SAMPLE_RATE        0  // Disable sample rate logging

typedef struct {
	// Circular buffer for raw samples
	int32_t ir_buffer[MAX30102_BUFFER_SIZE];
	int32_t red_buffer[MAX30102_BUFFER_SIZE];
	uint16_t buffer_index;
	bool buffer_full;
	
	// Sample rate tracking
	uint64_t first_sample_time;
	uint64_t last_sample_time;
	uint32_t total_samples;
	uint32_t samples_in_buffer;
	float actual_sample_rate;
} max30102_buffer_state_t;

static const char *TAG = "MAX30102";

static bool s_i2c_initialised = false;
static max30102_buffer_state_t s_buffer_state;

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

static void max30102_buffer_init(void)
{
	memset(&s_buffer_state, 0, sizeof(s_buffer_state));
	s_buffer_state.first_sample_time = 0;
	s_buffer_state.last_sample_time = 0;
	s_buffer_state.total_samples = 0;
	s_buffer_state.samples_in_buffer = 0;
	s_buffer_state.actual_sample_rate = 0.0f;
}

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
	max30102_buffer_init();
	
	ESP_LOGI(TAG, "MAX30102 initialized successfully");
	ESP_LOGI(TAG, "Target sample rate: %d Hz, Buffer size: %d samples", 
	         MAX30102_TARGET_SAMPLE_RATE, MAX30102_BUFFER_SIZE);
	
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

	uint64_t now_us = esp_timer_get_time();

	for (uint8_t i = 0; i < to_read; ++i) {
		const uint8_t *chunk = &buffer[i * MAX30102_BYTES_PER_SAMPLE];
		uint32_t red = ((uint32_t)chunk[0] << 16) | ((uint32_t)chunk[1] << 8) | chunk[2];
		uint32_t ir = ((uint32_t)chunk[3] << 16) | ((uint32_t)chunk[4] << 8) | chunk[5];
		red &= 0x3FFFF;
		ir &= 0x3FFFF;
		samples[i].red = red;
		samples[i].ir = ir;
		
		// Store in buffer
		s_buffer_state.ir_buffer[s_buffer_state.buffer_index] = (int32_t)ir;
		s_buffer_state.red_buffer[s_buffer_state.buffer_index] = (int32_t)red;
		s_buffer_state.buffer_index++;
		s_buffer_state.total_samples++;
		
		if (s_buffer_state.buffer_index >= MAX30102_BUFFER_SIZE) {
			s_buffer_state.buffer_index = 0;
			s_buffer_state.buffer_full = true;
			
			// Calculate and log actual sample rate when buffer fills
			if (s_buffer_state.first_sample_time > 0) {
				uint64_t time_diff_us = now_us - s_buffer_state.first_sample_time;
				if (time_diff_us > 0) {
					s_buffer_state.actual_sample_rate = (float)(MAX30102_BUFFER_SIZE * 1000000ULL) / (float)time_diff_us;
					
#if MAX30102_LOG_SAMPLE_RATE
					ESP_LOGI(TAG, "Buffer full - Actual sample rate: %.2f Hz (target: %d Hz)", 
					         s_buffer_state.actual_sample_rate, MAX30102_TARGET_SAMPLE_RATE);
#endif
				}
			}
			s_buffer_state.first_sample_time = now_us;
		}
		
		// Track first sample time
		if (s_buffer_state.total_samples == 1) {
			s_buffer_state.first_sample_time = now_us;
		}
		s_buffer_state.last_sample_time = now_us;
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

esp_err_t max30102_get_buffered_samples(int32_t *ir_buffer, int32_t *red_buffer,
                                        uint16_t *buffer_size, uint16_t *samples_available)
{
	if (!ir_buffer || !red_buffer || !buffer_size || !samples_available) {
		return ESP_ERR_INVALID_ARG;
	}
	
	*buffer_size = MAX30102_BUFFER_SIZE;
	
	if (s_buffer_state.buffer_full) {
		// Buffer is full, copy entire buffer starting from current position
		*samples_available = MAX30102_BUFFER_SIZE;
		uint16_t first_part = MAX30102_BUFFER_SIZE - s_buffer_state.buffer_index;
		
		// Copy from current index to end
		memcpy(ir_buffer, &s_buffer_state.ir_buffer[s_buffer_state.buffer_index], 
		       first_part * sizeof(int32_t));
		memcpy(red_buffer, &s_buffer_state.red_buffer[s_buffer_state.buffer_index],
		       first_part * sizeof(int32_t));
		
		// Copy from beginning to current index
		memcpy(&ir_buffer[first_part], s_buffer_state.ir_buffer,
		       s_buffer_state.buffer_index * sizeof(int32_t));
		memcpy(&red_buffer[first_part], s_buffer_state.red_buffer,
		       s_buffer_state.buffer_index * sizeof(int32_t));
	} else {
		// Buffer not full yet, copy what we have
		*samples_available = s_buffer_state.buffer_index;
		memcpy(ir_buffer, s_buffer_state.ir_buffer, 
		       s_buffer_state.buffer_index * sizeof(int32_t));
		memcpy(red_buffer, s_buffer_state.red_buffer,
		       s_buffer_state.buffer_index * sizeof(int32_t));
	}
	
	return ESP_OK;
}

float max30102_get_actual_sample_rate(void)
{
	return s_buffer_state.actual_sample_rate;
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



