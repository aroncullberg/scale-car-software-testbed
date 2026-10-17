idf_component_register(
    SRCS "task.cpp"
    INCLUDE_DIRS "include"
    REQUIRES vehicle power_monitor esp32-rclink freertos
    PRIV_REQUIRES rc_telemetry
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
