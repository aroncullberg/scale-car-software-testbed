idf_component_register(
    SRCS "vehicle_controller.cpp"
    INCLUDE_DIRS "include"
#        REQUIRES rc servo aroncullberg__EscDriver waypoint_storage nav imu
        REQUIRES rc servo aroncullberg__EscDriver nav
)
