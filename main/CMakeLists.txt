idf_component_register(
    SRCS 
        "main.cpp"
        "sbus.cpp"
        "DShotRMT.cpp"
    INCLUDE_DIRS 
        "include"
        "."
    REQUIRES         
        "driver"      # For MCPWM and UART
        "esp_timer"   # For timestamp functions
        "freertos"    # For FreeRTOS functionality
        "nvs_flash"   # For NVS initialization
        "hal"         # Hardware Abstraction Layer for GPIO

)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)