#include "websocket_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <string.h>
#include <sys/socket.h>

// WebSocket support check - Force disabled for ESP-IDF 5.5.1
#define WS_SUPPORTED 0

static const char *TAG = "WS_SERVER";
static httpd_handle_t server = NULL;

// Shared buffer for SSE data streaming
static char sse_hr_buffer[256];
static char sse_eeg_buffer[300];
static char sse_mpu_buffer[300];
static bool sse_hr_ready = false;
static bool sse_eeg_ready = false;
static bool sse_mpu_ready = false;

// WiFi configuration - UPDATE THESE WITH YOUR CREDENTIALS
#define WIFI_SSID      "---"
#define WIFI_PASS      "---"
#define MAX_RETRY      5

static int s_retry_num = 0;

// Event handler for WiFi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to AP... (%d/%d)", s_retry_num, MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "Failed to connect to WiFi");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
    }
}

static esp_err_t wifi_init_sta(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete. Connecting to SSID: %s", WIFI_SSID);
    return ESP_OK;
}

// Server-Sent Events (SSE) handler
static esp_err_t sse_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // Send initial connection message
    const char *conn_msg = "data: {\"type\":\"connected\"}\n\n";
    httpd_resp_send_chunk(req, conn_msg, strlen(conn_msg));
    
    ESP_LOGI(TAG, "SSE client connected");
    
    // Keep connection alive and send buffered data
    char buffer[512];
    while (1) {
        // Send heart rate data if available
        if (sse_hr_ready) {
            snprintf(buffer, sizeof(buffer), "data: %s\n\n", sse_hr_buffer);
            if (httpd_resp_send_chunk(req, buffer, strlen(buffer)) != ESP_OK) {
                ESP_LOGI(TAG, "SSE client disconnected");
                break;
            }
            sse_hr_ready = false;
        }
        
        // Send EEG data if available
        if (sse_eeg_ready) {
            snprintf(buffer, sizeof(buffer), "data: %s\n\n", sse_eeg_buffer);
            if (httpd_resp_send_chunk(req, buffer, strlen(buffer)) != ESP_OK) {
                ESP_LOGI(TAG, "SSE client disconnected");
                break;
            }
            sse_eeg_ready = false;
        }
        
        // Send MPU data if available
        if (sse_mpu_ready) {
            snprintf(buffer, sizeof(buffer), "data: %s\n\n", sse_mpu_buffer);
            if (httpd_resp_send_chunk(req, buffer, strlen(buffer)) != ESP_OK) {
                ESP_LOGI(TAG, "SSE client disconnected");
                break;
            }
            sse_mpu_ready = false;
        }
        
        // Send keepalive every second
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    httpd_resp_send_chunk(req, NULL, 0); // End response
    return ESP_OK;
}

// HTML page handler
static const char *html_page = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<title>Health Monitor</title>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<style>"
"body { font-family: Arial; margin: 20px; background: #1e1e1e; color: #fff; }"
".container { max-width: 1200px; margin: 0 auto; }"
".sensor-box { background: #2d2d2d; border-radius: 8px; padding: 20px; margin: 10px 0; border-left: 4px solid; }"
".heartrate { border-color: #e74c3c; }"
".eeg { border-color: #3498db; }"
".mpu { border-color: #2ecc71; }"
"h1 { text-align: center; }"
"h2 { margin-top: 0; }"
".value { font-size: 24px; font-weight: bold; margin: 10px 0; }"
".timestamp { font-size: 12px; color: #888; }"
".status { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }"
".status.connected { background: #2ecc71; }"
".status.disconnected { background: #e74c3c; }"
"</style>"
"</head>"
"<body>"
"<div class='container'>"
"<h1>Real-Time Health Monitor</h1>"
"<p><span class='status' id='status'></span><span id='status-text'>Connecting...</span></p>"
""
"<div class='sensor-box heartrate'>"
"<h2>Heart Rate & SpO2</h2>"
"<div class='value'>HR: <span id='hr'>--</span> BPM</div>"
"<div class='value'>SpO2: <span id='spo2'>--</span>%</div>"
"<div class='value'>Temp: <span id='temp'>--</span>°C</div>"
"<div class='timestamp'>Last update: <span id='hr-time'>--</span></div>"
"</div>"
""
"<div class='sensor-box eeg'>"
"<h2>EEG/EOG (ADS1292)</h2>"
"<div class='value'>CH1: <span id='ch1'>--</span> mV (Δ<span id='ch1-delta'>--</span>)</div>"
"<div class='value'>CH2: <span id='ch2'>--</span> mV (Δ<span id='ch2-delta'>--</span>)</div>"
"<div class='timestamp'>Baseline: <span id='baseline-status'>Establishing...</span></div>"
"<div class='timestamp'>Last update: <span id='eeg-time'>--</span></div>"
"</div>"
""
"<div class='sensor-box mpu'>"
"<h2>Motion (MPU6050)</h2>"
"<div class='value'>Accel: X=<span id='ax'>--</span>g Y=<span id='ay'>--</span>g Z=<span id='az'>--</span>g</div>"
"<div class='value'>Gyro: X=<span id='gx'>--</span>°/s Y=<span id='gy'>--</span>°/s Z=<span id='gz'>--</span>°/s</div>"
"<div class='value'>Temp: <span id='mpu-temp'>--</span>°C</div>"
"<div class='timestamp'>Last update: <span id='mpu-time'>--</span></div>"
"</div>"
"</div>"
""
"<script>"
"var eventSource = new EventSource('/events');"
"var statusDot = document.getElementById('status');"
"var statusText = document.getElementById('status-text');"
""
"eventSource.onopen = function() {"
"  statusDot.className = 'status connected';"
"  statusText.textContent = 'Connected';"
"};"
""
"eventSource.onerror = function() {"
"  statusDot.className = 'status disconnected';"
"  statusText.textContent = 'Disconnected';"
"};"
""
"eventSource.onmessage = function(event) {"
"  var data = JSON.parse(event.data);"
"  var now = new Date().toLocaleTimeString();"
"  "
"  if (data.type === 'heartrate') {"
"    document.getElementById('hr').textContent = data.heart_rate.toFixed(1);"
"    document.getElementById('spo2').textContent = data.spo2.toFixed(1);"
"    document.getElementById('temp').textContent = data.temperature.toFixed(1);"
"    document.getElementById('hr-time').textContent = now;"
"  } else if (data.type === 'eeg') {"
"    document.getElementById('ch1').textContent = (data.ch1_voltage * 1000).toFixed(3);"
"    document.getElementById('ch2').textContent = (data.ch2_voltage * 1000).toFixed(3);"
"    document.getElementById('ch1-delta').textContent = ((data.ch1_voltage - data.ch1_baseline) * 1000).toFixed(3);"
"    document.getElementById('ch2-delta').textContent = ((data.ch2_voltage - data.ch2_baseline) * 1000).toFixed(3);"
"    document.getElementById('baseline-status').textContent = data.baseline_established ? 'Established' : 'Establishing...';"
"    document.getElementById('eeg-time').textContent = now;"
"  } else if (data.type === 'mpu') {"
"    document.getElementById('ax').textContent = data.accel_x.toFixed(2);"
"    document.getElementById('ay').textContent = data.accel_y.toFixed(2);"
"    document.getElementById('az').textContent = data.accel_z.toFixed(2);"
"    document.getElementById('gx').textContent = data.gyro_x.toFixed(1);"
"    document.getElementById('gy').textContent = data.gyro_y.toFixed(1);"
"    document.getElementById('gz').textContent = data.gyro_z.toFixed(1);"
"    document.getElementById('mpu-temp').textContent = data.temp.toFixed(1);"
"    document.getElementById('mpu-time').textContent = now;"
"  }"
"};"
"</script>"
"</body>"
"</html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

esp_err_t websocket_server_init(void)
{
    // Initialize WiFi
    ESP_ERROR_CHECK(wifi_init_sta());
    
    // Wait for WiFi connection
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Start HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WS_SERVER_PORT;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register SSE (Server-Sent Events) handler
        httpd_uri_t sse_uri = {
            .uri       = "/events",
            .method    = HTTP_GET,
            .handler   = sse_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &sse_uri);
        ESP_LOGI(TAG, "HTTP server with SSE started on port %d", WS_SERVER_PORT);
        
        // Register index page handler
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &index_uri);
        
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to start WebSocket server");
    return ESP_FAIL;
}

esp_err_t websocket_send_heartrate(const heartrate_packet_t *data)
{
    if (server == NULL) return ESP_FAIL;
    
    snprintf(sse_hr_buffer, sizeof(sse_hr_buffer),
             "{\"type\":\"heartrate\",\"heart_rate\":%.1f,\"spo2\":%.1f,\"temperature\":%.1f,\"timestamp\":%llu}",
             data->heart_rate, data->spo2, data->temperature, data->timestamp);
    sse_hr_ready = true;
    
    return ESP_OK;
}

esp_err_t websocket_send_eeg(const eeg_packet_t *data)
{
    if (server == NULL) return ESP_FAIL;
    
    snprintf(sse_eeg_buffer, sizeof(sse_eeg_buffer),
             "{\"type\":\"eeg\",\"ch1_voltage\":%.6f,\"ch2_voltage\":%.6f,"
             "\"ch1_baseline\":%.6f,\"ch2_baseline\":%.6f,\"baseline_established\":%s,\"timestamp\":%llu}",
             data->ch1_voltage, data->ch2_voltage,
             data->ch1_baseline, data->ch2_baseline,
             data->baseline_established ? "true" : "false",
             data->timestamp);
    sse_eeg_ready = true;
    
    return ESP_OK;
}

esp_err_t websocket_send_mpu(const mpu_packet_t *data)
{
    if (server == NULL) return ESP_FAIL;
    
    snprintf(sse_mpu_buffer, sizeof(sse_mpu_buffer),
             "{\"type\":\"mpu\",\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f,"
             "\"gyro_x\":%.1f,\"gyro_y\":%.1f,\"gyro_z\":%.1f,\"temp\":%.1f,\"timestamp\":%llu}",
             data->accel_x, data->accel_y, data->accel_z,
             data->gyro_x, data->gyro_y, data->gyro_z,
             data->temp, data->timestamp);
    sse_mpu_ready = true;
    
    return ESP_OK;
}

void websocket_server_stop(void)
{
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
}
