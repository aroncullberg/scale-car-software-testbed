#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <optional>
#include "dshot/telemetry.hpp"

namespace data_capture {

inline std::optional<uint32_t> ground_speed_cms(
    const std::array<dshot::Telemetry, 4>& motors, float wheel_diameter_mm,
    float motor_revolutions_per_wheel) {
    if (!(wheel_diameter_mm > 0.0f) || !(motor_revolutions_per_wheel > 0.0f) ||
        !std::isfinite(wheel_diameter_mm) || !std::isfinite(motor_revolutions_per_wheel))
        return std::nullopt;
    uint64_t total_rpm = 0;
    unsigned valid_motors = 0;
    for (const auto& motor : motors) {
        if (!motor.available || (motor.speed.state != dshot::ErpmState::Moving &&
                                 motor.speed.state != dshot::ErpmState::Stopped)) continue;
        total_rpm += motor.speed.rpm;
        ++valid_motors;
    }
    if (!valid_motors) return std::nullopt;
    constexpr double pi = 3.14159265358979323846;
    const double speed = double(total_rpm) / valid_motors * pi * wheel_diameter_mm /
                         (600.0 * motor_revolutions_per_wheel);
    return static_cast<uint32_t>(std::llround(std::min(speed,
        double(std::numeric_limits<uint32_t>::max()))));
}

} // namespace data_capture
