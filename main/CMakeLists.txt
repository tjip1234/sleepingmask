idf_component_register(SRCS "eeg_spectral.c" "algorithm.c" "max30102.c" "hello_world_main.c" "ads1292.c"
                       PRIV_REQUIRES spi_flash driver esp_timer
                       INCLUDE_DIRS "")
