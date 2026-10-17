idf_component_register(
    SRCS "card.cpp" "read_write_test.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio sdmmc
    PRIV_REQUIRES fatfs esp_driver_sdspi esp_timer freertos
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
