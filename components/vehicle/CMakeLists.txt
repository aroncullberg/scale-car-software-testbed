idf_component_register(
    SRCS "controller.cpp" "motor.cpp" "servos.cpp" "yaw_rate_controller.cpp" "lights.cpp" "passthrough_usb.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp32-dshot esp32-rclink esp32-esc-passthrough servo rgb_led freertos log
    PRIV_REQUIRES imu esp_timer esp_driver_usb_serial_jtag esp_hw_support vfs
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
