#include "vehicle/controller.hpp"

#include "esp_timer.h"

namespace vehicle {

void Controller::control_lights(const VehicleCommand& command, bool command_available) {
    led::Command lighting;
    lighting.light_level = command.light_level;
    lighting.armed = motors_[0].armed();
    lighting.rc_available = command_available;
    lighting.left_blinker_pressed = command.left_blinker_pressed;
    lighting.right_blinker_pressed = command.right_blinker_pressed;
    const auto frame = light_pattern_.update(lighting, esp_timer_get_time());
    ESP_ERROR_CHECK(lights_.update(frame));
}

} // namespace vehicle
