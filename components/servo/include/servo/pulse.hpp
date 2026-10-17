#pragma once

#include <algorithm>
#include <cstdint>

namespace servo {

struct OutputConfig {
    int gpio{-1};
    uint16_t minimum_us{1000};
    uint16_t center_us{1500};
    uint16_t maximum_us{2000};
    bool reversed{false};
};

// Positions use the receiver's calibrated range: 0 = min, 1000 = center, 2000 = max.
constexpr uint16_t pulse_for_position(const OutputConfig &config, uint16_t position) {
    uint32_t value = std::min<uint16_t>(position, 2000);
    if (config.reversed) {
        value = 2000 - value;
    }
    if (value <= 1000) {
        return config.minimum_us +
               ((config.center_us - config.minimum_us) * value + 500) / 1000;
    }
    return config.center_us +
           ((config.maximum_us - config.center_us) * (value - 1000) + 500) / 1000;
}

// Round to the nearest hardware PWM tick, using 2^resolution ticks per period.
constexpr uint32_t duty_for_pulse(uint16_t pulse_us, uint32_t frequency_hz,
                                  uint8_t resolution_bits) {
    return (uint64_t(pulse_us) * frequency_hz * (uint64_t{1} << resolution_bits) + 500000) /
           1000000;
}

} // namespace servo
