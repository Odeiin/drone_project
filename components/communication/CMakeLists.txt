idf_component_register(
    SRCS "NRF24L01.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi driver
)