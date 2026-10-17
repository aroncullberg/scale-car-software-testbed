#include "vehicle/controller.hpp"

#include <algorithm>
#include <cmath>
#include "esp_timer.h"
#include "imu.h"

namespace vehicle {

esp_err_t Controller::start_servos() {
    yaw_rate_controller_.reset();
    const std::array outputs{config_.servos.steering, config_.servos.rear_aero,
                            config_.servos.front_aero1, config_.servos.front_aero2};
    return servos_.start(outputs.data(), outputs.size());
}

float Controller::steering_output(const VehicleCommand& command, float minimum_steering,
                                  float maximum_steering) {
    const float direct_steering = std::clamp(command.steering, -1.0f, 1.0f) *
                                  std::clamp(command.steering_range, 0.0f, 1.0f);
    if (command.steering_mode == SteeringMode::Direct || command.direction == Direction::Reverse) {
        yaw_rate_controller_.reset();
        return std::clamp(direct_steering, minimum_steering, maximum_steering);
    }

    const auto gyro = imu::IMU::instance().samples().gyro;
    const auto now_us = esp_timer_get_time();
    const bool gyro_fresh = gyro.valid && std::isfinite(gyro.z_rads) &&
        gyro.timestamp_us <= uint64_t(now_us) &&
        uint64_t(now_us) - gyro.timestamp_us <= yaw_rate_controller_.config().maximum_gyro_age_us;
    if (!gyro_fresh) {
        yaw_rate_controller_.reset();
        return std::clamp(direct_steering, minimum_steering, maximum_steering);
    }

    bool motors_moving = false;
    for (const auto& motor : motor_telemetry()) {
        if (motor.available && motor.speed.state == dshot::ErpmState::Moving && motor.speed.rpm > 0)
            motors_moving = true;
    }
    const float requested_yaw_rate_rads = std::clamp(command.steering, -1.0f, 1.0f) *
        yaw_rate_controller_.config().maximum_yaw_rate_rads *
        (yaw_rate_controller_.config().positive_steering_turns_left ? 1.0f : -1.0f);
    const auto output = yaw_rate_controller_.update(requested_yaw_rate_rads, gyro.z_rads, now_us,
                                                   minimum_steering, maximum_steering, motors_moving);
    steering_terms_ = yaw_rate_controller_.terms();
    steering_terms_.gyro_age_us = uint64_t(now_us) - gyro.timestamp_us;
    return output;
}

void Controller::control_servos(const VehicleCommand& command, bool command_available) {
    steering_terms_ = {};
    if (!command_available) {
        yaw_rate_controller_.reset();
        ESP_ERROR_CHECK(servos_.center());
        return;
    }

    const auto& steering = config_.servos.steering;
    const float positive_travel_us = steering.reversed ?
        steering.center_us - steering.minimum_us : steering.maximum_us - steering.center_us;
    const float negative_travel_us = steering.reversed ?
        steering.maximum_us - steering.center_us : steering.center_us - steering.minimum_us;
    const float center_adjustment_us = std::clamp(
        std::clamp(command.steering_center_adjustment, -1.0f, 1.0f) *
        config_.servos.steering_center_adjustment_us, -negative_travel_us, positive_travel_us);
    const float steering_range = std::clamp(command.steering_range, 0.0f, 1.0f);
    const float minimum_steering = std::max(-steering_range,
        (-negative_travel_us - center_adjustment_us) / negative_travel_us);
    const float maximum_steering = std::min(steering_range,
        (positive_travel_us - center_adjustment_us) / positive_travel_us);
    const float steering_command = steering_output(command, minimum_steering, maximum_steering);
    steering_terms_.output = steering_command;
    const auto steering_position = static_cast<uint16_t>(std::lround(
        (steering_command + 1.0f) * 1000.0f));
    const float steering_offset = int(servo::pulse_for_position(steering, steering_position)) -
                                  int(steering.center_us);
    const float center_adjustment = center_adjustment_us * (steering.reversed ? -1.0f : 1.0f);
    const int steering_pulse = std::lround(steering.center_us + center_adjustment +
                                         steering_offset);
    ESP_ERROR_CHECK(servos_.write_pulse_us(0, std::clamp(steering_pulse,
        int(steering.minimum_us), int(steering.maximum_us))));

    const auto front_position = static_cast<uint16_t>(std::lround(
        std::clamp(command.front_aero, 0.0f, 1.0f) * 2000.0f));
    const auto rear_position = static_cast<uint16_t>(std::lround(
        std::clamp(command.rear_aero, 0.0f, 1.0f) * 2000.0f));
    ESP_ERROR_CHECK(servos_.write(1, rear_position));
    ESP_ERROR_CHECK(servos_.write(2, front_position));
    ESP_ERROR_CHECK(servos_.write(3, front_position));
}

} // namespace vehicle
