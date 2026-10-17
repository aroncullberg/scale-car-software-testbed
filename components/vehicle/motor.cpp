#include "vehicle/motor.hpp"

#include <algorithm>

namespace vehicle {

Motor::Motor(dshot::Driver& driver, MotorConfig config)
    : config_(config), link_(driver, config.gpio) {}

esp_err_t Motor::begin() {
    suspend();
    const auto error = link_.begin();
    return error == ESP_OK ? write(0) : error;
}

esp_err_t Motor::disarm() {
    armed_ = false;
    brake_release_neutral_pending_ = false;
    return write(0);
}

esp_err_t Motor::drive(float throttle, Direction direction) {
    if (brake_release_neutral_pending_) {
        const auto error = write(0);
        if (error == ESP_OK) brake_release_neutral_pending_ = false;
        return error;
    }
    return write_motor_output(throttle, direction);
}

esp_err_t Motor::brake(float brake_amount, Direction selected_direction) {
    const auto error = write_motor_output(brake_amount, opposite(selected_direction));
    if (error == ESP_OK) brake_release_neutral_pending_ = armed_ && brake_amount > 0.0f;
    return error;
}

esp_err_t Motor::write_motor_output(float output_fraction, Direction direction) {
    if (!armed_ || !(output_fraction > 0.0f)) return write(0);
    const auto offset = static_cast<std::uint16_t>(std::min(output_fraction, 1.0f) * 999.0f);
    const bool high_half = (direction == Direction::Forward) !=
                           (config_.mounting == Mounting::Mirrored);
    // AM32 car mode: opposite-direction input applies the ESC's own brake.
    // Zero remains neutral.
    return write((high_half ? 1048 : 48) + offset);
}

esp_err_t Motor::write(std::uint16_t payload) {
    const auto error = link_.set(payload);
    if (error == ESP_OK) payload_ = payload;
    return error;
}

} // namespace vehicle
