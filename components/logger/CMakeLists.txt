idf_component_register(
    SRCS "logger.cpp" "sinks/stdout_sink.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES freertos blackboard
)
