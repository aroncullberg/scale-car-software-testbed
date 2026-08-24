idf_component_register(
    SRCS "dshot_driver.cpp" "motor.cpp" "dshot_rmt_encoder.c"
    INCLUDE_DIRS "include" "."
    REQUIRES driver blackboard
    PRIV_REQUIRES esp_timer
)
