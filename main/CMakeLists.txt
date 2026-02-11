idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        rc
        servo
        vehicle
        power_monitor
        nav
        proto_nmea
        imu
        telemetry
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)

