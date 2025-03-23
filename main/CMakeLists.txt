idf_component_register(
    SRCS 
        "main.cpp"
        "dshot_rmt_encoder.c"
        "DShotRMT.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES         
        "esp_timer"
        "freertos"
#        "data_pool"
#        "SBUS"
#        "GPS"
#        "imu"
#        "dshot_controller"
#        "vdc"
#        "log_monitor"
#        "servo"
#        "ConfigManager"
#        "nvs_flash"
        "driver"
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)