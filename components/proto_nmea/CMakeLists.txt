idf_component_register(
        SRCS "nmea_driver.cpp"
        INCLUDE_DIRS
            "include"
        REQUIRES andyfinity__TinyGPSPlus-ESPIDF
        PRIV_REQUIRES driver freertos nav shared telemetry
)

