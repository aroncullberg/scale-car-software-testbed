#pragma once

#include <array>
#include <cstdint>

namespace led {

enum class LightLevel { Off, Low, High };

struct Color {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;

    constexpr bool operator==(const Color& other) const {
        return red == other.red && green == other.green && blue == other.blue;
    }
};

struct Frame {
    std::array<Color, 14> front{};
    std::array<Color, 34> rear{};
    Color status{};

    bool operator==(const Frame& other) const {
        return front == other.front && rear == other.rear && status == other.status;
    }
};

} // namespace led
