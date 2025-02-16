idf_component_register(
    SRCS 
        "system_monitor.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        "shared"
        "freertos"
        "esp_common"
        "esp_common"
        "esp_wifi"
        "driver"            # For hardware info
        "esp_system"        # For system functions
        "esp_timer"         # For high-resolution timing
        "esp_wifi"          # For wifi stats
        "freertos"          # For task stats
        "heap"             # For heap statistics
)