#pragma once

#include "led/frame.hpp"

namespace led {

struct Command {
    LightLevel light_level = LightLevel::Off;
    bool armed = false;
    bool rc_available = false;
    bool left_blinker_pressed = false;
    bool right_blinker_pressed = false;
};

class Pattern {
public:
    Frame update(const Command& command, int64_t now_us);

private:
    class Blinker {
    public:
        void update(bool pressed, bool enabled, int64_t now_us);
        bool illuminated(int64_t now_us) const;

    private:
        bool was_pressed_ = false;
        int64_t started_at_us_ = -1;
    };

    Blinker left_;
    Blinker right_;
};

} // namespace led
