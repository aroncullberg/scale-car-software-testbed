#pragma once

#include <cstddef>
#include <cstdint>
#include "driver/gpio.h"
#include "led/frame.hpp"

namespace vehicle {

inline constexpr std::size_t motor_count = 4;

enum class Direction { Forward, Reverse };
enum class Mounting { Normal, Mirrored };
enum class SteeringMode { Direct, YawRate };

constexpr Direction opposite(Direction direction) {
    return direction == Direction::Forward ? Direction::Reverse : Direction::Forward;
}

struct MotorConfig {
    gpio_num_t gpio;
    Mounting mounting = Mounting::Normal;
};

struct VehicleCommand {
    float throttle = 0.0f;
    float brake = 0.0f;
    Direction direction = Direction::Forward;
    bool arm = false;
    float steering = 0.0f;
    float steering_center_adjustment = 0.0f;
    float steering_range = 1.0f;
    SteeringMode steering_mode = SteeringMode::Direct;
    float front_aero = 0.5f;
    float rear_aero = 0.5f;
    led::LightLevel light_level = led::LightLevel::Off;
    bool left_blinker_pressed = false;
    bool right_blinker_pressed = false;
};

} // namespace vehicle
