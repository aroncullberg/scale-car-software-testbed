idf_component_register(
    SRCS "rgb_led.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES freertos
    REQUIRES driver espressif__led_strip
)
