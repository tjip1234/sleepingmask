idf_component_register(SRCS "websocket_server.c" "mpu6050.c" "algorithm.c" "max30102.c" "hello_world_main.c" "ads1292.c"
                       PRIV_REQUIRES spi_flash driver esp_timer esp_wifi esp_netif esp_http_server nvs_flash esp_event
                       INCLUDE_DIRS "")
