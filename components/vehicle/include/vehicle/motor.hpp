#pragma once

#include "dshot/link.hpp"
#include "vehicle/types.hpp"

namespace vehicle {

class Controller;

// Call from the control task. The shared DShot driver owns frame timing.
class Motor {
public:
    Motor(dshot::Driver& driver, MotorConfig config);

    esp_err_t begin();
    // Enables application output; this is not an acknowledgement from the ESC.
    void arm() { armed_ = true; }
    esp_err_t disarm();
    esp_err_t drive(float throttle, Direction direction);
    esp_err_t brake(float brake_amount, Direction selected_direction);

    bool armed() const { return armed_; }
    std::uint16_t payload() const { return payload_; }
    const MotorConfig& config() const { return config_; }
    dshot::Telemetry telemetry() const { return link_.telemetry(); }

private:
    friend class Controller;
    // The transport has already stopped/closed the link during a handoff.
    void suspend() { armed_ = false; payload_ = 0; brake_release_neutral_pending_ = false; }
    esp_err_t write_motor_output(float output_fraction, Direction direction);
    esp_err_t write(std::uint16_t payload);

    const MotorConfig config_;
    dshot::Link link_;
    bool armed_ = false;
    std::uint16_t payload_ = 0;
    bool brake_release_neutral_pending_ = false;
};

} // namespace vehicle
