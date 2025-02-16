idf_component_register(
    SRCS 
        "gps.cpp"
    INCLUDE_DIRS 
        "."
        "tinygps-espidf/include"
        "tinygps-espidf/TinyGPSPlus/src"
    REQUIRES         
        "driver"
        "freertos"
        "esp_common"
        "data_pool"
        "shared"
)

add_subdirectory(tinygps-espidf)

target_link_libraries(${COMPONENT_LIB} PRIVATE tinygps-espidf)
