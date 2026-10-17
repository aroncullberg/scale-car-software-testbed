#include "vehicle/controller.hpp"

#include "esp_timer.h"
#include "passthrough_usb.hpp"
#include "vehicle/rc_input.hpp"
#include "imu.h"
#include <algorithm>
#include <cmath>

namespace vehicle {

Controller::Controller(const Config& config, rclink::Receiver& receiver)
    : config_(config), receiver_(receiver), driver_(config.dshot),
      motors_{{{driver_, config.motors[0]}, {driver_, config.motors[1]},
               {driver_, config.motors[2]}, {driver_, config.motors[3]}}},
      yaw_rate_controller_(config.yaw_rate),
      pending_tuning_(config.yaw_rate),
      passthrough_owner_(driver_,
          {&motors_[0].link_, &motors_[1].link_, &motors_[2].link_, &motors_[3].link_},
          esp_timer_get_time), stopped_(xSemaphoreCreateBinaryStatic(&stopped_storage_)) {}

Controller::~Controller() {
    stop();
    vSemaphoreDelete(stopped_);
}

esp_err_t Controller::start() {
    if (task_) return ESP_ERR_INVALID_STATE;
    auto error = detail::start_passthrough(config_.motors, passthrough_owner_.callbacks(),
                                          &passthrough_);
    if (error != ESP_OK) return error;
    error = start_servos();
    if (error == ESP_OK) {
        light_pattern_ = {};
        error = lights_.start();
    }
    if (error == ESP_OK) {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        for (auto& motor : motors_) {
            error = motor.begin();
            if (error != ESP_OK) break;
        }
        if (error == ESP_OK) error = driver_.start();
        motors_started_ = error == ESP_OK;
    }
    if (error == ESP_OK) {
        stop_requested_.store(false);
        if (xTaskCreatePinnedToCore(control_task_entry, "vehicle_control", 8192, this,
                                    5, &task_, 1) != pdPASS) error = ESP_ERR_NO_MEM;
    }
    if (error != ESP_OK) close_outputs();
    return error;
}

void Controller::stop() {
    if (!task_) return;
    stop_requested_.store(true);
    // Let the owner task finish any parser/ESC operation and release its IO.
    xSemaphoreTake(stopped_, portMAX_DELAY);
    task_ = nullptr;
}

void Controller::control_task_entry(void* argument) {
    static_cast<Controller*>(argument)->control_task();
}

YawRateConfig Controller::tuning() const {
    std::lock_guard<std::mutex> lock(tuning_mutex_);
    return pending_tuning_;
}

esp_err_t Controller::set_tuning(const YawRateConfig& config) {
    if (!config.valid()) return ESP_ERR_INVALID_ARG;
    std::lock_guard<std::mutex> lock(tuning_mutex_);
    pending_tuning_ = config;
    ++tuning_revision_;
    return ESP_OK;
}

esp_err_t Controller::set_control_observer(ControlObserver observer, void* context) {
    if (task_) return ESP_ERR_INVALID_STATE;
    control_observer_ = observer;
    control_observer_context_ = context;
    return ESP_OK;
}

std::array<dshot::Telemetry, motor_count> Controller::motor_telemetry() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    std::array<dshot::Telemetry, motor_count> telemetry{};
    if (!motors_started_ || passthrough_owner_.owns_motor_pins() ||
        passthrough_owner_.motor_fault_latched()) return telemetry;
    for (std::size_t i = 0; i < motor_count; ++i) telemetry[i] = motors_[i].telemetry();
    return telemetry;
}

void Controller::control_task() {
    TickType_t last_wake = xTaskGetTickCount();
    while (!stop_requested_.load()) {
        {
            std::lock_guard<std::mutex> lock(tuning_mutex_);
            if (applied_tuning_revision_ != tuning_revision_) {
                yaw_rate_controller_.configure(pending_tuning_);
                applied_tuning_revision_ = tuning_revision_;
            }
        }
        ControlSample sample;
        VehicleCommand command;
        bool command_available;
        {
            std::lock_guard<std::mutex> lock(motor_mutex_);
            detail::pump_passthrough(passthrough_, usb_was_connected_);
            const auto rc = receiver_.snapshot();
            command = command_from_rc(rc.channels);
            command_available = rc.usable;
            control_motors(command, command_available);
            for (size_t i = 0; i < motor_count; ++i) {
                sample.motor_commands[i] = motors_[i].payload();
                sample.armed |= motors_[i].armed();
            }
        }
        control_servos(command, command_available);
        control_lights(command, command_available);
        if (control_observer_) {
            sample.timestamp_us = esp_timer_get_time();
            sample.command = command;
            sample.rc_usable = command_available;
            sample.steering = steering_terms_;
            sample.tuning = yaw_rate_controller_.config();
            sample.tuning_revision = applied_tuning_revision_;
            const auto gyro = imu::IMU::instance().samples().gyro;
            sample.gyro_valid = gyro.valid && std::isfinite(gyro.z_rads) && gyro.timestamp_us <= sample.timestamp_us;
            if (sample.gyro_valid) {
                sample.gyro_z_rads = gyro.z_rads;
                sample.gyro_age_us = std::min<uint64_t>(sample.timestamp_us - gyro.timestamp_us, UINT32_MAX);
            }
            control_observer_(control_observer_context_, sample);
        }
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(14));
    }
    close_outputs();
    xSemaphoreGive(stopped_);
    vTaskDelete(nullptr);
}

void Controller::close_outputs() {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    motors_started_ = false;
    ESP_ERROR_CHECK(detail::stop_passthrough(&passthrough_));
    driver_.stop();
    for (auto& motor : motors_) {
        motor.link_.end();
        motor.suspend();
    }
    usb_was_connected_ = false;
    yaw_rate_controller_.reset();
    servos_.stop();
    lights_.stop();
}

void Controller::control_motors(const VehicleCommand& command, bool command_available) {
    if (!apply_motor_output_restrictions(command, command_available)) return;
    for (auto& motor : motors_) {
        motor.arm();
        if (command.brake > 0.0f) {
            motor.brake(command.brake, command.direction);
        }
        else motor.drive(command.throttle, command.direction);
    }
}

bool Controller::apply_motor_output_restrictions(const VehicleCommand& command, bool command_available) {
    if (!passthrough_owner_.owns_motor_pins() && !passthrough_owner_.motor_fault_latched() &&
        driver_.last_error() != ESP_OK) {
        passthrough_owner_.close_motor_links_after_fault();
    }
    if (passthrough_owner_.owns_motor_pins() || passthrough_owner_.motor_fault_latched()) {
        for (auto& motor : motors_) motor.suspend();
        return false;
    }

    if (passthrough_owner_.rearming() &&
        esp_timer_get_time() >= passthrough_owner_.rearm_deadline_us() &&
        command_available && command.throttle == 0.0f && command.brake == 0.0f) {
        passthrough_owner_.finish_rearming();
    }
    if (passthrough_owner_.rearming() || !command_available || !command.arm) {
        for (auto& motor : motors_) motor.disarm();
        return false;
    }
    return true;
}

} // namespace vehicle
