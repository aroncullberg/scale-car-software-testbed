#pragma once

#include <array>
#include "vehicle/types.hpp"
#include "vehicle/yaw_rate_controller.hpp"

namespace vehicle {
struct ControlSample {
    uint64_t timestamp_us = 0;
    VehicleCommand command{};
    YawRateTerms steering{};
    YawRateConfig tuning{};
    uint32_t tuning_revision = 0;
    std::array<uint16_t, motor_count> motor_commands{};
    float gyro_z_rads = 0;
    uint32_t gyro_age_us = 0;
    bool gyro_valid = false;
    bool rc_usable = false;
    bool armed = false;
};
using ControlObserver = void (*)(void*, const ControlSample&);
}
