idf_component_register(
    SRCS 
        "main.cpp"
    INCLUDE_DIRS 
        "include"
        "."
    REQUIRES         
        "esp_timer"
        "freertos"
        "data_pool"
        "SBUS"
)


target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)