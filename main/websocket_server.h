#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include "esp_err.h"
#include "max30102.h"
#include "ads1292.h"
#include "mpu6050.h"

// WebSocket server configuration
#define WS_SERVER_PORT 80

// Data packet structures for JSON streaming
typedef struct {
    float heart_rate;
    float spo2;
    float temperature;
    uint64_t timestamp;
} heartrate_packet_t;

typedef struct {
    float ch1_voltage;
    float ch2_voltage;
    float ch1_baseline;
    float ch2_baseline;
    bool baseline_established;
    uint64_t timestamp;
} eeg_packet_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
    uint64_t timestamp;
} mpu_packet_t;

// Function prototypes
esp_err_t websocket_server_init(void);
esp_err_t websocket_send_heartrate(const heartrate_packet_t *data);
esp_err_t websocket_send_eeg(const eeg_packet_t *data);
esp_err_t websocket_send_mpu(const mpu_packet_t *data);
void websocket_server_stop(void);

#endif // WEBSOCKET_SERVER_H
