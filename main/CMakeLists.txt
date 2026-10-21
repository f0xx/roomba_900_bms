idf_component_register(
    SRCS "main.c" "smbus.c" "bms.c" "sha1.c" "tft.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_spi esp_driver_gpio freertos log
)
