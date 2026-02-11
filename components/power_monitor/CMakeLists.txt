idf_component_register(
    SRCS "power_monitor.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_adc driver telemetry
)
