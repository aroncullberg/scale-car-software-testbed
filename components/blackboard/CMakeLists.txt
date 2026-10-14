idf_component_register(
    SRCS "blackboard.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos
    PRIV_REQUIRES esp_ringbuf esp_timer
)
