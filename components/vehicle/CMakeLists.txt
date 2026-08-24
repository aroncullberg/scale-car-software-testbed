idf_component_register(
    SRCS "vehicle_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES rc servo motor nav imu rgb_led power_monitor blackboard
)
