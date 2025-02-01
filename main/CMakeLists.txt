idf_component_register(
    SRCS 
        "main.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES         
        "esp_timer"
        "freertos"
        "data_pool"
        "SBUS"
        "GPS"
        "imu"
)


target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)