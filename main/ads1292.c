#include "ads1292.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ADS1292";
static spi_device_handle_t spi_handle;

// Convert 24-bit two's complement to 32-bit signed integer
static int32_t convert_24bit_to_32bit(uint32_t data24) {
    if (data24 & 0x800000) {
        // Negative number - sign extend
        return (int32_t)(data24 | 0xFF000000);
    } else {
        // Positive number
        return (int32_t)(data24 & 0x00FFFFFF);
    }
}

esp_err_t ads1292_init(void) {
    esp_err_t ret;

    // Configure GPIO pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ADS1292_START_PIN) | (1ULL << ADS1292_PWDN_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,  // Enable pull-up on PWDN to help it stay HIGH
    };
    gpio_config(&io_conf);

    // Configure DRDY pin as input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ADS1292_DRDY_PIN);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Power up the ADS1292 (AFTER GPIO is configured)
    ESP_LOGI(TAG, "Powering up ADS1292: Setting PWDN (GPIO %d) HIGH...", ADS1292_PWDN_PIN);
    gpio_set_level(ADS1292_PWDN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for power-up - increased to 200ms

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = ADS1292_MISO_PIN,
        .mosi_io_num = ADS1292_MOSI_PIN,
        .sclk_io_num = ADS1292_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    ret = spi_bus_initialize(ADS1292_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = ADS1292_SPI_FREQ,
        .mode = 1, // CPOL=0, CPHA=1 (SPI Mode 1)
        .spics_io_num = ADS1292_CS_PIN,
        .queue_size = 7,
        .flags = 0, // Full duplex mode
        .cs_ena_pretrans = 2,  // CS enable hold time
        .cs_ena_posttrans = 2,  // CS disable wait time
    };

    ret = spi_bus_add_device(ADS1292_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reset the ADS1292
    uint8_t reset_cmd = ADS1292_CMD_RESET;
    uint8_t dummy_rx;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &reset_cmd,
        .rx_buffer = &dummy_rx,
    };
    ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send reset command: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Reset command sent, waiting 200ms...");
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for reset - increased from 100ms

    // Stop data conversion initially
    uint8_t stop_cmd = ADS1292_CMD_SDATAC;
    trans.tx_buffer = &stop_cmd;
    trans.rx_buffer = &dummy_rx;
    ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SDATAC command: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure registers
    ads1292_write_register(ADS1292_REG_CONFIG1, ADS1292_CONFIG1_VAL);
    ads1292_write_register(ADS1292_REG_CONFIG2, ADS1292_CONFIG2_VAL);
    ads1292_write_register(ADS1292_REG_CH1SET, ADS1292_CH1SET_VAL);
    ads1292_write_register(ADS1292_REG_CH2SET, ADS1292_CH2SET_VAL);
    ads1292_write_register(ADS1292_REG_RLD_SENS, ADS1292_RLD_SENS_VAL);
    ads1292_write_register(ADS1292_REG_LOFF_SENS, ADS1292_LOFF_SENS_VAL);

    // Verify device ID
    uint8_t device_id;
    ret = ads1292_read_register(ADS1292_REG_ID, &device_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADS1292 Device ID: 0x%02X", device_id);
        if ((device_id & 0xF8) != 0x50) {
            ESP_LOGW(TAG, "Unexpected device ID. Expected 0x5X, got 0x%02X", device_id);
        }
    } else {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }

    // Print all register values for debugging
    ads1292_print_registers();

    ESP_LOGI(TAG, "ADS1292 initialized successfully");
    ESP_LOGI(TAG, "Sample Rate: 500 SPS, Gain: 12x (±200mV range - optimal for EEG)");
    ESP_LOGI(TAG, "Reference Buffer: ENABLED (internal 2.4V reference)");
    ESP_LOGI(TAG, "CONFIG: CH1SET=0x%02X, CH2SET=0x%02X, CONFIG1=0x%02X, CONFIG2=0x%02X",
             ADS1292_CH1SET_VAL, ADS1292_CH2SET_VAL, ADS1292_CONFIG1_VAL, ADS1292_CONFIG2_VAL);
    return ESP_OK;
}

esp_err_t ads1292_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx_data[3] = {
        ADS1292_CMD_WREG | reg,  // Write register command + register address (0x40 | reg)
        0x00,                    // Number of registers to write - 1 (always 0 for single register)
        value                    // Data to write
    };
    uint8_t rx_data[3] = {0}; // Receive buffer for full-duplex

    spi_transaction_t trans = {
        .length = 24,  // 3 bytes * 8 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    ESP_LOGD(TAG, "WREG: reg=0x%02X val=0x%02X, cmd_byte=0x%02X", reg, value, tx_data[0]);

    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "WREG response: 0x%02X 0x%02X 0x%02X", rx_data[0], rx_data[1], rx_data[2]);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Increased delay for register operations
    return ret;
}

esp_err_t ads1292_read_register(uint8_t reg, uint8_t *value) {
    // First send the read command
    uint8_t cmd_data[2] = {
        ADS1292_CMD_RREG | reg,  // Read register command + register address (0x20 | reg)
        0x00                     // Number of registers to read - 1 (always 0 for single register)
    };

    spi_transaction_t cmd_trans = {
        .length = 16,
        .tx_buffer = cmd_data,
        .rx_buffer = NULL,
    };

    ESP_LOGD(TAG, "RREG: reg=0x%02X, cmd_byte=0x%02X", reg, cmd_data[0]);

    esp_err_t ret = spi_device_transmit(spi_handle, &cmd_trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read command for register 0x%02X: %s", reg, esp_err_to_name(ret));
        return ret;
    }

    // Delay to allow ADS1292 to prepare data (increased from 1ms)
    vTaskDelay(pdMS_TO_TICKS(10));

    // Now read the data
    uint8_t dummy_tx = 0x00;
    uint8_t rx_data = 0;

    spi_transaction_t data_trans = {
        .length = 8,
        .tx_buffer = &dummy_tx,
        .rx_buffer = &rx_data,
    };

    ret = spi_device_transmit(spi_handle, &data_trans);
    if (ret == ESP_OK) {
        *value = rx_data;
        ESP_LOGD(TAG, "RREG response: 0x%02X", rx_data);
    } else {
        ESP_LOGE(TAG, "Failed to read data for register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ads1292_start_conversion(void) {
    // Start continuous data conversion
    uint8_t rdatac_cmd = ADS1292_CMD_RDATAC;
    uint8_t dummy_rx;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &rdatac_cmd,
        .rx_buffer = &dummy_rx,
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send RDATAC command: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Start conversion
    gpio_set_level(ADS1292_START_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Started continuous data conversion");
    return ESP_OK;
}

esp_err_t ads1292_stop_conversion(void) {
    // Stop conversion
    gpio_set_level(ADS1292_START_PIN, 0);
    
    // Stop continuous data conversion
    uint8_t sdatac_cmd = ADS1292_CMD_SDATAC;
    uint8_t dummy_rx;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &sdatac_cmd,
        .rx_buffer = &dummy_rx,
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SDATAC command: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Stopped data conversion");
    return ESP_OK;
}

bool ads1292_data_ready(void) {
    return (gpio_get_level(ADS1292_DRDY_PIN) == 0);
}

esp_err_t ads1292_read_data(ads1292_data_t *data) {
    if (!ads1292_data_ready()) {
        return ESP_ERR_NOT_FOUND; // Data not ready
    }

    uint8_t tx_data[9] = {0}; // Dummy bytes to send
    uint8_t rx_data[9]; // 3 bytes status + 3 bytes CH1 + 3 bytes CH2
    memset(rx_data, 0, sizeof(rx_data));

    spi_transaction_t trans = {
        .length = 72, // 9 bytes * 8 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Parse the received data
    data->status = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];
    
    uint32_t ch1_raw = (rx_data[3] << 16) | (rx_data[4] << 8) | rx_data[5];
    uint32_t ch2_raw = (rx_data[6] << 16) | (rx_data[7] << 8) | rx_data[8];
    
    data->channel1 = convert_24bit_to_32bit(ch1_raw);
    data->channel2 = convert_24bit_to_32bit(ch2_raw);

    return ESP_OK;
}

float ads1292_convert_to_voltage(int32_t raw_value) {
    // ADS1292 with 2.4V reference, gain of 12, 24-bit resolution
    // Voltage = (raw_value * VREF) / (Gain * 2^23)
    // VREF = 2.4V, Gain = 12, 2^23 = 8388608
    // With gain=12, input range is ±200mV (ideal for EEG)
    return (float)raw_value * 2.4f / (12.0f * 8388608.0f);
}void ads1292_print_data(const ads1292_data_t *data) {
    float voltage_ch1 = ads1292_convert_to_voltage(data->channel1);
    float voltage_ch2 = ads1292_convert_to_voltage(data->channel2);
    
    printf("Status: 0x%06X, CH1: %d (%.6f V), CH2: %d (%.6f V)\n", 
           (unsigned int)data->status, 
           (int)data->channel1, voltage_ch1,
           (int)data->channel2, voltage_ch2);
}

esp_err_t ads1292_set_gain(uint8_t gain_setting) {
    // Stop continuous conversion first
    uint8_t sdatac_cmd = ADS1292_CMD_SDATAC;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &sdatac_cmd,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Update CH1SET register with new gain
    ret = ads1292_write_register(ADS1292_REG_CH1SET, gain_setting);
    if (ret != ESP_OK) return ret;
    
    // Restart continuous conversion
    uint8_t rdatac_cmd = ADS1292_CMD_RDATAC;
    trans.tx_buffer = &rdatac_cmd;
    ret = spi_device_transmit(spi_handle, &trans);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "Updated gain setting to 0x%02X", gain_setting);
    return ret;
}

void ads1292_print_registers(void) {
    uint8_t reg_value;
    const char* reg_names[] = {
        "ID", "CONFIG1", "CONFIG2", "LOFF", "CH1SET", "CH2SET",
        "RLD_SENS", "LOFF_SENS", "LOFF_STAT", "RESP1", "RESP2", "GPIO"
    };

    ESP_LOGI(TAG, "=== ADS1292 Register Values ===");
    for (int i = 0; i < 12; i++) {
        if (ads1292_read_register(i, &reg_value) == ESP_OK) {
            if (i == ADS1292_REG_CH1SET || i == ADS1292_REG_CH2SET) {
                uint8_t gain_bits = (reg_value >> 4) & 0x07;
                uint8_t mux_bits = reg_value & 0x0F;
                uint8_t pd_bit = (reg_value >> 7) & 0x01;
                const char* gain_str[] = {"1", "1", "2", "3", "4", "8", "12", "12"};
                ESP_LOGI(TAG, "%s (0x%02X): 0x%02X | PD:%d Gain:%sx MUX:0x%X",
                         reg_names[i], i, reg_value, pd_bit, gain_str[gain_bits], mux_bits);
            } else {
                ESP_LOGI(TAG, "%s (0x%02X): 0x%02X", reg_names[i], i, reg_value);
            }
        }
    }
}

// EOG-specific functions
void eog_init_baseline(eog_baseline_t *baseline) {
    baseline->baseline_ch1 = 0.0f;
    baseline->baseline_ch2 = 0.0f;
    baseline->sample_count = 0;
    baseline->baseline_established = false;
}

void eog_update_baseline(eog_baseline_t *baseline, const ads1292_data_t *data) {
    float voltage_ch1 = ads1292_convert_to_voltage(data->channel1);
    float voltage_ch2 = ads1292_convert_to_voltage(data->channel2);
    
    if (baseline->sample_count < 1000) { // Establish baseline over first 1000 samples (4 seconds at 250 SPS)
        baseline->baseline_ch1 = (baseline->baseline_ch1 * baseline->sample_count + voltage_ch1) / (baseline->sample_count + 1);
        baseline->baseline_ch2 = (baseline->baseline_ch2 * baseline->sample_count + voltage_ch2) / (baseline->sample_count + 1);
        baseline->sample_count++;
        
        if (baseline->sample_count == 1000) {
            baseline->baseline_established = true;
            ESP_LOGI(TAG, "Baseline established: CH1=%.6f V, CH2=%.6f V", baseline->baseline_ch1, baseline->baseline_ch2);
        }
    } else {
        // Slowly adapt baseline (very slow adaptation to track long-term drift)
        baseline->baseline_ch1 = baseline->baseline_ch1 * 0.9999f + voltage_ch1 * 0.0001f;
        baseline->baseline_ch2 = baseline->baseline_ch2 * 0.9999f + voltage_ch2 * 0.0001f;
    }
}

void eog_print_data_with_baseline(const ads1292_data_t *data, const eog_baseline_t *baseline) {
    float voltage_ch1 = ads1292_convert_to_voltage(data->channel1);
    float voltage_ch2 = ads1292_convert_to_voltage(data->channel2);
    
    if (baseline->baseline_established) {
        float diff_ch1 = voltage_ch1 - baseline->baseline_ch1;
        float diff_ch2 = voltage_ch2 - baseline->baseline_ch2;
        
        printf("EOG - CH1: %.6f V (Δ%.6f V), CH2: %.6f V (Δ%.6f V), Status: 0x%06X\n", 
               voltage_ch1, diff_ch1, voltage_ch2, diff_ch2, (unsigned int)data->status);
    } else {
        printf("Establishing baseline... (%u/1000) CH1: %.6f V, CH2: %.6f V\n", 
               (unsigned int)baseline->sample_count, voltage_ch1, voltage_ch2);
    }
}

bool eog_detect_movement(const ads1292_data_t *data, const eog_baseline_t *baseline, float threshold_mv) {
    if (!baseline->baseline_established) {
        return false;
    }

    float voltage_ch1 = ads1292_convert_to_voltage(data->channel1);
    float voltage_ch2 = ads1292_convert_to_voltage(data->channel2);

    float diff_ch1 = fabsf(voltage_ch1 - baseline->baseline_ch1);
    float diff_ch2 = fabsf(voltage_ch2 - baseline->baseline_ch2);

    float threshold_v = threshold_mv / 1000.0f; // Convert mV to V

    return (diff_ch1 > threshold_v) || (diff_ch2 > threshold_v);
}

// EEG spectral analysis wrapper functions
void ads1292_calculate_band_power_ch1(const int32_t *ch1_buffer, uint16_t buffer_size, eeg_band_power_t *band_power) {
    if (!ch1_buffer || !band_power) {
        return;
    }
    eeg_calculate_band_power(ch1_buffer, buffer_size, 500, band_power);  // 500 SPS
}

void ads1292_calculate_band_power_ch2(const int32_t *ch2_buffer, uint16_t buffer_size, eeg_band_power_t *band_power) {
    if (!ch2_buffer || !band_power) {
        return;
    }
    eeg_calculate_band_power(ch2_buffer, buffer_size, 500, band_power);  // 500 SPS
}
