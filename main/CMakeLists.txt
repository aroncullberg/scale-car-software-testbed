idf_component_register(
    SRCS "main.cpp" "sensors.cpp" "storage_check.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp32-rclink vehicle data_capture esp_driver_gpio esp_driver_spi
                  proto_nmea imu power_monitor storage esp_hw_support recording web
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
