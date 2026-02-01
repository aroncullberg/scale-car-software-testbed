idf_component_register(
        SRCS "rclink.cpp"
             "backends/elrs.cpp"
             "backends/sbus.cpp"
        INCLUDE_DIRS "include"
        PRIV_REQUIRES freertos aroncullberg__ESP_CRSF
        REQUIRES esp_driver_uart driver rgb_led
)