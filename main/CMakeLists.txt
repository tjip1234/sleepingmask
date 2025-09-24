idf_component_register(SRCS "hello_world_main.c" "ads1292.c"
                       PRIV_REQUIRES spi_flash driver
                       INCLUDE_DIRS "")
