idf_component_register(
    SRCS
        "main.cpp"
        "pin_test.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        rc
        servo
        motor
        vehicle
        power_monitor
        nav
        proto_nmea
        imu
        rgb_led
        blackboard
        logger
        tcp_sink
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -fno-rtti
    -fno-exceptions
)
