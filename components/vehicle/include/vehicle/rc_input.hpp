#pragma once

#include <cmath>
#include "rclink/types.hpp"
#include "vehicle/types.hpp"

namespace vehicle {

inline VehicleCommand command_from_rc(const rclink::Channels& channels) {
    VehicleCommand command;
    command.throttle = channels.ch1.unit();
    command.brake = std::abs(channels.ch3.bipolar());
    command.direction = channels.ch7.high() ? Direction::Reverse : Direction::Forward;
    command.arm = channels.ch5.high();
    command.steering = channels.ch2.unit() * 2.0f - 1.0f;
    command.steering_center_adjustment = channels.ch4.unit() * 2.0f - 1.0f;
    command.steering_range = channels.ch6.unit();
    command.steering_mode = channels.ch14.high() ? SteeringMode::Direct : SteeringMode::YawRate;
    command.front_aero = channels.ch8.unit();
    command.rear_aero = channels.ch9.unit();
    command.light_level = channels.ch10.low() ? led::LightLevel::Off :
                          channels.ch10.high() ? led::LightLevel::High : led::LightLevel::Low;
    command.left_blinker_pressed = channels.ch11.high();
    command.right_blinker_pressed = channels.ch12.high();
    return command;
}

} // namespace vehicle
