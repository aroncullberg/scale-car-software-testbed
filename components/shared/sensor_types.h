#pragma once

#include <array>
#include <cstdint>

namespace sensor
{
    struct Servo {
        static constexpr uint32_t US_FAILSAFE = 1500;
        static constexpr int16_t FAILSAFE_POSITION = 1000;
        static constexpr int16_t MIN_POSITION = 0;
        static constexpr int16_t MAX_POSITION = 2000;
        static constexpr int16_t NEUTRAL_POSITION = 1000;
    };

    struct Motor {
        static constexpr int16_t FAILSAFE_THROTTLE = 0;
    };


    struct eRPMData // TODO: This should be a union with the throttle value
    {
        struct FrontRight {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } front_right;

        struct FrontLeft {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } front_left;

        struct RearLeft {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } rear_left;

        struct RearRight {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } rear_right;

        int16_t throttle_value{0};
    };
} // Namespace sensor
