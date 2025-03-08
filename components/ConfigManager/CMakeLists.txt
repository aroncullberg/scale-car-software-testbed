idf_component_register(
        SRCS
        "config_manager.cpp"
        INCLUDE_DIRS
        "."
        REQUIRES
        "nvs_flash"  # For NVS functionality
        "freertos"   # For task synchronization
        "esp_common" # For common ESP functionality
        "driver"     # For common driver access
        "esp_hw_support" # For hardware-specific support
)