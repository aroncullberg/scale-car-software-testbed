idf_component_register(
    SRCS "publisher.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp32-dshot esp32-rclink power_monitor
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
