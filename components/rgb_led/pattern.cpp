#include "led/pattern.hpp"

namespace led {
namespace {
constexpr int64_t blink_on_us = 500000;
constexpr int64_t blink_off_us = 250000;
constexpr int64_t blink_period_us = blink_on_us + blink_off_us;
constexpr int blink_count = 3;
constexpr int64_t blink_duration_us = blink_count * blink_period_us;
}

void Pattern::Blinker::update(bool pressed, bool enabled, int64_t now_us) {
    if (!enabled) started_at_us_ = -1;
    else if (pressed && !was_pressed_) started_at_us_ = now_us;
    was_pressed_ = pressed;
}

bool Pattern::Blinker::illuminated(int64_t now_us) const {
    return started_at_us_ >= 0 && now_us - started_at_us_ < blink_duration_us &&
           (now_us - started_at_us_) % blink_period_us < blink_on_us;
}

Frame Pattern::update(const Command& command, int64_t now_us) {
    const uint8_t brightness = command.light_level == LightLevel::High ? 255 :
                               command.light_level == LightLevel::Low ? 120 : 0;
    const bool blinkers_enabled = brightness != 0 && command.rc_available;
    left_.update(command.left_blinker_pressed, blinkers_enabled, now_us);
    right_.update(command.right_blinker_pressed, blinkers_enabled, now_us);
    Frame frame;
    if (brightness == 0) return frame;

    const Color front_color = command.armed ? Color{brightness, brightness, brightness} :
        Color{brightness, uint8_t((unsigned(brightness) + 2) / 4),
                          uint8_t((unsigned(brightness) + 1) / 2)};
    const Color orange{brightness, uint8_t((unsigned(brightness) + 2) / 4), 0};
    for (std::size_t i = 0; i < frame.front.size(); ++i) {
        const auto& blinker = i < 7 ? left_ : right_;
        frame.front[i] = blinker.illuminated(now_us) ? orange : front_color;
    }

    if (command.armed || !command.rc_available) {
        frame.rear.fill(Color{brightness, 0, 0});
    } else {
        const auto step = unsigned(now_us / 100000 % 66);
        const auto position = step <= 33 ? step : 66 - step;
        frame.rear[position] = {brightness, 0, 0};
        if (position > 0) frame.rear[position - 1] = {uint8_t(brightness / 3), 0, 0};
        if (position < 33) frame.rear[position + 1] = {uint8_t(brightness / 3), 0, 0};
    }
    const auto status_brightness = uint8_t(brightness / 3);
    frame.status = !command.rc_available ? Color{status_brightness, 0, 0} :
                   command.armed ? Color{0, status_brightness, 0} :
                                   Color{0, 0, status_brightness};
    return frame;
}

} // namespace led
