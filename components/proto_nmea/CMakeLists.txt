idf_component_register(
        SRCS "nmea_driver.cpp"
             "ubx.cpp"
        INCLUDE_DIRS
            "include"
        REQUIRES andyfinity__TinyGPSPlus-ESPIDF esp_driver_uart esp_driver_gpio freertos nav
        PRIV_REQUIRES esp_timer shared
)
