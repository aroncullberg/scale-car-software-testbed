idf_component_register(
    SRCS 
        "main.c"
        "pwm_input.c"
        "dshot_esc_encoder.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver      # For MCPWM
        freertos    # For FreeRTOS functionality
        esp_common  # For ESP error codes
)