#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

// MAX30102 I2C Configuration
#define MAX30102_I2C_ADDR           0x57    // 7-bit I2C address
#define MAX30102_I2C_PORT           I2C_NUM_0
#define MAX30102_SDA_PIN            GPIO_NUM_4
#define MAX30102_SCL_PIN            GPIO_NUM_5
#define MAX30102_I2C_FREQ           400000  // 400kHz

// MAX30102 Register Addresses
#define MAX30102_REG_INTERRUPT_STATUS_1     0x00
#define MAX30102_REG_INTERRUPT_STATUS_2     0x01
#define MAX30102_REG_INTERRUPT_ENABLE_1     0x02
#define MAX30102_REG_INTERRUPT_ENABLE_2     0x03
#define MAX30102_REG_FIFO_WR_PTR            0x04
#define MAX30102_REG_FIFO_OVERFLOW_COUNTER  0x05
#define MAX30102_REG_FIFO_RD_PTR            0x06
#define MAX30102_REG_FIFO_DATA              0x07
#define MAX30102_REG_FIFO_CONFIG            0x08
#define MAX30102_REG_MODE_CONFIG            0x09
#define MAX30102_REG_SPO2_CONFIG            0x0A
#define MAX30102_REG_LED1_PULSE_AMPLITUDE   0x0C  // Red LED
#define MAX30102_REG_LED2_PULSE_AMPLITUDE   0x0D  // IR LED
#define MAX30102_REG_LED3_PULSE_AMPLITUDE   0x0E  // Green LED (if available)
#define MAX30102_REG_LED4_PULSE_AMPLITUDE   0x0F  // Not used
#define MAX30102_REG_MULTI_LED_CTRL_1       0x11
#define MAX30102_REG_MULTI_LED_CTRL_2       0x12
#define MAX30102_REG_TEMP_INTEGER           0x1F
#define MAX30102_REG_TEMP_FRACTION          0x20
#define MAX30102_REG_TEMP_CONFIG            0x21
#define MAX30102_REG_PROX_INT_THRESH        0x30
#define MAX30102_REG_REV_ID                 0xFE
#define MAX30102_REG_PART_ID                0xFF

// Configuration Values
#define MAX30102_PART_ID                    0x15

// Mode Configuration
#define MAX30102_MODE_SHUTDOWN              0x00
#define MAX30102_MODE_RED_ONLY              0x02  // Heart Rate mode
#define MAX30102_MODE_RED_IR                0x03  // SpO2 mode
#define MAX30102_MODE_GREEN_RED_IR          0x07  // Multi-LED mode

// FIFO Configuration
#define MAX30102_FIFO_SAMPLE_AVG_1          0x00
#define MAX30102_FIFO_SAMPLE_AVG_2          0x20
#define MAX30102_FIFO_SAMPLE_AVG_4          0x40
#define MAX30102_FIFO_SAMPLE_AVG_8          0x60
#define MAX30102_FIFO_SAMPLE_AVG_16         0x80
#define MAX30102_FIFO_SAMPLE_AVG_32         0xA0

#define MAX30102_FIFO_ROLLOVER_EN           0x10
#define MAX30102_FIFO_ALMOST_FULL_17        0x00
#define MAX30102_FIFO_ALMOST_FULL_15        0x01
#define MAX30102_FIFO_ALMOST_FULL_13        0x02
#define MAX30102_FIFO_ALMOST_FULL_11        0x03
#define MAX30102_FIFO_ALMOST_FULL_9         0x04
#define MAX30102_FIFO_ALMOST_FULL_7         0x05
#define MAX30102_FIFO_ALMOST_FULL_5         0x06
#define MAX30102_FIFO_ALMOST_FULL_3         0x07
#define MAX30102_FIFO_ALMOST_FULL_1         0x0F

// SpO2 Configuration
#define MAX30102_SPO2_ADC_RGE_2048          0x00
#define MAX30102_SPO2_ADC_RGE_4096          0x20
#define MAX30102_SPO2_ADC_RGE_8192          0x40
#define MAX30102_SPO2_ADC_RGE_16384         0x60

#define MAX30102_SPO2_SR_50                 0x00
#define MAX30102_SPO2_SR_100                0x04
#define MAX30102_SPO2_SR_200                0x08
#define MAX30102_SPO2_SR_400                0x0C
#define MAX30102_SPO2_SR_800                0x10
#define MAX30102_SPO2_SR_1000               0x14
#define MAX30102_SPO2_SR_1600               0x18
#define MAX30102_SPO2_SR_3200               0x1C

#define MAX30102_SPO2_LEDpw_200             0x00
#define MAX30102_SPO2_LEDpw_400             0x01
#define MAX30102_SPO2_LEDpw_800             0x02
#define MAX30102_SPO2_LEDpw_1600            0x03

// Interrupt Enable/Status
#define MAX30102_INT_A_FULL                 0x80
#define MAX30102_INT_DATA_RDY               0x40
#define MAX30102_INT_ALC_OVF                0x20
#define MAX30102_INT_PROX_INT               0x10
#define MAX30102_INT_PWR_READY              0x01

#define MAX30102_INT_TEMP_RDY               0x02

// FIFO Data Structure
#define MAX30102_FIFO_DEPTH                 32
#define MAX30102_BYTES_PER_SAMPLE           6   // 3 bytes Red + 3 bytes IR

#define MAX30102_SAMPLE_RATE_HZ             100.0f
#define MAX30102_WARMUP_SECONDS             1.0f
#define MAX30102_WINDOW_SIZE                128
#define MAX30102_MAX_HR_WINDOWS             8
#define MAX30102_DC_MIN                     2000.0f
#define MAX30102_DC_MAX                     200000.0f
#define MAX30102_DC_JUMP_RATIO              0.30f
#define MAX30102_MIN_QUALITY_RATIO          0.05f
#define MAX30102_MIN_PEAK_MAG               0.5f

// Data structures
typedef struct {
    uint32_t red;
    uint32_t ir;
    uint32_t green;
} max30102_sample_t;

typedef struct {
    max30102_sample_t samples[MAX30102_FIFO_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} max30102_fifo_t;

typedef struct {
    float heart_rate;
    float spo2;
    bool valid;
    uint64_t last_beat_time;
    float ir_baseline;
    float red_baseline;
    float quality_metric;
    float signal_strength;
    float ir_dc;
    float red_dc;
} max30102_biometrics_t;

// Function prototypes
esp_err_t max30102_init(void);
esp_err_t max30102_reset(void);
esp_err_t max30102_shutdown(bool shutdown);
esp_err_t max30102_set_mode(uint8_t mode);
esp_err_t max30102_set_led_brightness(uint8_t red_brightness, uint8_t ir_brightness);
esp_err_t max30102_configure_fifo(uint8_t sample_avg, bool rollover, uint8_t almost_full);
esp_err_t max30102_configure_spo2(uint8_t adc_range, uint8_t sample_rate, uint8_t led_pulse_width);

esp_err_t max30102_read_register(uint8_t reg, uint8_t *data);
esp_err_t max30102_write_register(uint8_t reg, uint8_t data);
esp_err_t max30102_read_fifo(max30102_sample_t *samples, uint8_t *num_samples);
esp_err_t max30102_read_single_sample(max30102_sample_t *sample);
esp_err_t max30102_get_sensor_info(void);
esp_err_t max30102_clear_fifo(void);

esp_err_t max30102_read_temperature(float *temperature);
bool max30102_is_data_ready(void);
uint8_t max30102_get_fifo_samples_available(void);

// Heart rate calculation functions
esp_err_t max30102_calculate_heart_rate(max30102_sample_t *samples, uint8_t num_samples, max30102_biometrics_t *bio);
void max30102_print_biometrics(const max30102_biometrics_t *bio);

// Utility functions
esp_err_t max30102_check_part_id(void);
void max30102_print_sensor_info(void);
void max30102_enable_interrupts(uint8_t interrupt_mask);
void max30102_clear_interrupts(void);

#endif // MAX30102_H
