idf_component_register(
    SRCS "rgb_led.cpp" "pattern.cpp"
    INCLUDE_DIRS "include"
    REQUIRES espressif__led_strip esp_common
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
)
