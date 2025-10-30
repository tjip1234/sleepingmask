#ifndef ADS1292_H
#define ADS1292_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "eeg_spectral.h"

// ADS1292 Pin Definitions (adjust these according to your wiring)
#define ADS1292_CS_PIN      GPIO_NUM_10
#define ADS1292_DRDY_PIN    GPIO_NUM_19
#define ADS1292_START_PIN   GPIO_NUM_3
#define ADS1292_PWDN_PIN    GPIO_NUM_1

// SPI Configuration
#define ADS1292_SPI_HOST    SPI2_HOST
#define ADS1292_MOSI_PIN    GPIO_NUM_7
#define ADS1292_MISO_PIN    GPIO_NUM_2
#define ADS1292_SCLK_PIN    GPIO_NUM_6
#define ADS1292_SPI_FREQ    500000   // 500 kHz - slower for reliable communication

// ADS1292 Commands
#define ADS1292_CMD_WAKEUP  0x02
#define ADS1292_CMD_STANDBY 0x04
#define ADS1292_CMD_RESET   0x06
#define ADS1292_CMD_START   0x08
#define ADS1292_CMD_STOP    0x0A
#define ADS1292_CMD_RDATAC  0x10
#define ADS1292_CMD_SDATAC  0x11
#define ADS1292_CMD_RDATA   0x12
#define ADS1292_CMD_RREG    0x20
#define ADS1292_CMD_WREG    0x40

// ADS1292 Register Addresses
#define ADS1292_REG_ID      0x00
#define ADS1292_REG_CONFIG1 0x01
#define ADS1292_REG_CONFIG2 0x02
#define ADS1292_REG_LOFF    0x03
#define ADS1292_REG_CH1SET  0x04
#define ADS1292_REG_CH2SET  0x05
#define ADS1292_REG_RLD_SENS 0x06
#define ADS1292_REG_LOFF_SENS 0x07
#define ADS1292_REG_LOFF_STAT 0x08
#define ADS1292_REG_RESP1   0x09
#define ADS1292_REG_RESP2   0x0A
#define ADS1292_REG_GPIO    0x0B

// Configuration Values for EEG (Frontal electrodes)
#define ADS1292_CONFIG1_VAL 0x02  // Continuous conversion, 500 SPS (better for EEG)
#define ADS1292_CONFIG2_VAL 0xA0  // Reference buffer ENABLED (bit 5=1), lead-off comparators disabled, internal reference
                                   // Binary: 10100000 = bit7(required) + bit5(ref buffer) + bit4-0(reference config)
#define ADS1292_CH1SET_VAL  0x60  // Channel 1: Powered ON, Gain = 12 (±200mV range, ideal for EEG)
#define ADS1292_CH2SET_VAL  0x60  // Channel 2: Powered ON, Gain = 12 (±200mV range, ideal for EEG)
#define ADS1292_RLD_SENS_VAL 0x00 // RLD disabled
#define ADS1292_LOFF_SENS_VAL 0x00 // Lead-off detection disabled for higher sensitivity

// Alternative gain configurations
#define ADS1292_CH1SET_GAIN_1   0x00  // Gain = 1 (±2.4V range)
#define ADS1292_CH1SET_GAIN_2   0x10  // Gain = 2 (±1.2V range)
#define ADS1292_CH1SET_GAIN_3   0x20  // Gain = 3 (±0.8V range)
#define ADS1292_CH1SET_GAIN_4   0x30  // Gain = 4 (±0.6V range)
#define ADS1292_CH1SET_GAIN_6   0x40  // Gain = 6 (±0.4V range)
#define ADS1292_CH1SET_GAIN_8   0x50  // Gain = 8 (±0.3V range)
#define ADS1292_CH1SET_GAIN_12  0x60  // Gain = 12 (±0.2V range)

// Data structure for ADS1292 readings
typedef struct {
    uint32_t status;
    int32_t channel1;
    int32_t channel2;
} ads1292_data_t;

// EOG-specific data structure
typedef struct {
    float baseline_ch1;
    float baseline_ch2;
    uint32_t sample_count;
    bool baseline_established;
} eog_baseline_t;

// Function prototypes
esp_err_t ads1292_init(void);
esp_err_t ads1292_start_conversion(void);
esp_err_t ads1292_stop_conversion(void);
esp_err_t ads1292_read_data(ads1292_data_t *data);
bool ads1292_data_ready(void);
esp_err_t ads1292_write_register(uint8_t reg, uint8_t value);
esp_err_t ads1292_read_register(uint8_t reg, uint8_t *value);
void ads1292_print_data(const ads1292_data_t *data);
float ads1292_convert_to_voltage(int32_t raw_value);
void ads1292_print_registers(void);
esp_err_t ads1292_set_gain(uint8_t gain_setting);

// EOG-specific functions
void eog_init_baseline(eog_baseline_t *baseline);
void eog_update_baseline(eog_baseline_t *baseline, const ads1292_data_t *data);
void eog_print_data_with_baseline(const ads1292_data_t *data, const eog_baseline_t *baseline);
bool eog_detect_movement(const ads1292_data_t *data, const eog_baseline_t *baseline, float threshold_mv);

// EEG spectral analysis functions
void ads1292_calculate_band_power_ch1(const int32_t *ch1_buffer, uint16_t buffer_size, eeg_band_power_t *band_power);
void ads1292_calculate_band_power_ch2(const int32_t *ch2_buffer, uint16_t buffer_size, eeg_band_power_t *band_power);

#endif // ADS1292_H
