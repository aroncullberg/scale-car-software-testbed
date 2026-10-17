idf_component_register(
    SRCS "ulog.cpp" "logger.cpp"
    INCLUDE_DIRS "include"
    REQUIRES vehicle power_monitor imu nav freertos
    PRIV_REQUIRES esp_timer storage
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
