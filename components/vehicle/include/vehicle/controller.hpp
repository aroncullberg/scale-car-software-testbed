#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include "dshot/driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rclink/receiver.hpp"
#include "servo/controller.hpp"
#include "rgb_led.h"
#include "led/pattern.hpp"
#include "vehicle/detail/esc_passthrough_owner.hpp"
#include "vehicle/motor.hpp"
#include "vehicle/yaw_rate_controller.hpp"
#include "vehicle/control_sample.hpp"

namespace vehicle {

struct ServoConfig {
    servo::OutputConfig steering{47, 500, 1500, 2500, true};
    servo::OutputConfig rear_aero{4, 1250, 1775, 2300, true};
    servo::OutputConfig front_aero1{6, 700, 1600, 2500, false};
    servo::OutputConfig front_aero2{7, 500, 1400, 2300, true};
    unsigned steering_center_adjustment_us = 200;
};

struct Config {
    std::array<MotorConfig, motor_count> motors;
    dshot::Config dshot;
    ServoConfig servos{};
    YawRateConfig yaw_rate{};
};

// Owns the control task, motor links and USB passthrough handoff.
// Call start()/stop() from the application startup/shutdown task.
class Controller {
public:
    Controller(const Config& config, rclink::Receiver& receiver);
    ~Controller();
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    esp_err_t start();
    void stop();

    const Motor& motor(std::size_t index) const { return motors_[index]; }
    std::array<dshot::Telemetry, motor_count> motor_telemetry() const;
    YawRateConfig tuning() const;
    esp_err_t set_tuning(const YawRateConfig& config);
    esp_err_t set_control_observer(ControlObserver observer, void* context);

private:
    static void control_task_entry(void* argument);
    void control_task();
    void control_motors(const VehicleCommand& command, bool command_available);
    void control_servos(const VehicleCommand& command, bool command_available);
    void control_lights(const VehicleCommand& command, bool command_available);
    esp_err_t start_servos();
    float steering_output(const VehicleCommand& command, float minimum_steering,
                          float maximum_steering);
    bool apply_motor_output_restrictions(const VehicleCommand& command, bool command_available);
    void close_outputs();

    const Config config_;
    rclink::Receiver& receiver_;
    dshot::Driver driver_;
    std::array<Motor, motor_count> motors_;
    servo::Controller servos_;
    YawRateController yaw_rate_controller_;
    led::Controller lights_;
    led::Pattern light_pattern_;
    mutable std::mutex motor_mutex_;
    mutable std::mutex tuning_mutex_;
    YawRateConfig pending_tuning_;
    uint32_t tuning_revision_ = 0;
    uint32_t applied_tuning_revision_ = 0;
    YawRateTerms steering_terms_{};
    ControlObserver control_observer_ = nullptr;
    void* control_observer_context_ = nullptr;
    bool motors_started_ = false;
    detail::EscPassthroughOwner passthrough_owner_;
    esc_passthrough_t* passthrough_ = nullptr;
    bool usb_was_connected_ = false;
    TaskHandle_t task_ = nullptr;
    std::atomic<bool> stop_requested_{false};
    StaticSemaphore_t stopped_storage_;
    SemaphoreHandle_t stopped_;
};

} // namespace vehicle
