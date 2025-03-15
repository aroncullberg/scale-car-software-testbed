#include "vdc.h"

#include <sys/stat.h>
#include <cinttypes>

#include "sensor_types.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task_wdt.h"
#include "config_manager.h"

VehicleDynamicsController::VehicleDynamicsController(const Config& config)
    : config_(config),
      steering_servo_(config.steering_servo) {

    callback_ = [this] { this->updateFromConfig(); };
    ConfigManager::instance().registerCallback(callback_);
}

VehicleDynamicsController::~VehicleDynamicsController() {
    stop();
}

esp_err_t VehicleDynamicsController::init() {
    esp_err_t err = steering_servo_.setPosition(sensor::Servo::NEUTRAL_POSITION);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize steering servo");

    err = esc_driver_.init(config_.esc_config);
    ESP_RETURN_ON_ERROR(err, TAG, "Failed to initialize ESC driver");

    return ESP_OK;
}

esp_err_t VehicleDynamicsController::start() {
    if (is_running_) {
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        controllerTask,
        "veh_dynamics",
        config_.task_stack_size,
        this,
        config_.task_priority,
        &task_handle_,
        1
    );

    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "Failed to create task");

    is_running_ = true;
    return ESP_OK;
}

esp_err_t VehicleDynamicsController::stop() {
    if (!is_running_) {
        return ESP_ERR_INVALID_STATE;
    }

    esc_driver_.set_all_throttles(0);

    // Clean up task
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

void VehicleDynamicsController::updateFromConfig() {
    ESP_LOGI(TAG, "Updating VDC configuration from ConfigManager");

    test_value_ = ConfigManager::instance().getInt("vdc/cmd", test_value_);
    test_delay_ = ConfigManager::instance().getInt("vdc/delay", test_delay_);
    test_repeat_ = ConfigManager::instance().getInt("vdc/repeat", test_repeat_);

    // TODO: make this update stuff for pid ctrlr too
}

namespace {
    float mapChannelToRange(uint16_t channel_value, float min_val, float max_val) {
        if (channel_value > 2000) channel_value = 2000;

        // Map to 0.0-1.0 range
        float normalized = (static_cast<float>(channel_value)) / 2000.0f;

        // Map to target range
        return min_val + normalized * (max_val - min_val);
    }
}

void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    const VehicleData &vehicle_data = VehicleData::instance();

    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    constexpr auto pid_state = static_cast<size_t>(sensor::SbusChannel::AUX3);
    constexpr auto toggle_pidloop = static_cast<size_t>(sensor::SbusChannel::AUX4);
    constexpr auto arm_switch = static_cast<size_t>(sensor::SbusChannel::AUX8);
    constexpr auto ch_debug = static_cast<size_t>(sensor::SbusChannel::AUX9);
    constexpr auto ch_arm = static_cast<size_t>(sensor::SbusChannel::AUX10);

    while(true) {
        const sensor::SbusData& sbus_data = vehicle_data.getSbus();
        const sensor::ImuData& imu_data = vehicle_data.getImu();

        if (!sbus_data.quality.valid_signal) {
            ESP_LOGW(TAG, "Invalid SBUS signal");
            controller->steering_servo_.setPosition(sensor::Servo::FAILSAFE_POSITION);
            controller->esc_driver_.set_all_throttles(sensor::Motor::FAILSAFE_THROTTLE); // TODO: Change this to be a realfailsafe where the rmt driver shuts off so esc just shuts down (i hope they do atelast)
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        bool pid_enabled = sbus_data.channels_scaled[toggle_pidloop] > 1900;
        if (pid_enabled && controller->pid_state_ == PidState::DISABLED) {
            controller->pid_state_ = PidState::ACTIVE;
            ESP_LOGI(TAG, "PID loop enabled (switch)");
        } else if (!pid_enabled && (controller->pid_state_ == PidState::ACTIVE ||
                   controller->pid_state_ == PidState::SUSPENDED)) {
            controller->pid_state_ = PidState::DISABLED;
            ESP_LOGI(TAG, "PID loop disabled (switch)");
        }

        if (sbus_data.channels_scaled[pid_state] > 800 && sbus_data.channels_scaled[pid_state] < 1200 && controller->pid_state_ != PidState::GYROFF) {
            controller->pid_state_ = PidState::GYROFF;
            ESP_LOGI(TAG, "PID state set to GYROFF");
        } else if (sbus_data.channels_scaled[pid_state] > 1900 && sbus_data.channels_scaled[pid_state] < 2100 && controller->pid_state_ != PidState::QUATMODE) {
            controller->pid_state_ = PidState::QUATMODE;
            ESP_LOGI(TAG, "PID state set to QUATMODE");
        } else if (sbus_data.channels_scaled[pid_state] < 800 && controller->pid_state_ != PidState::RATE) {
            controller->pid_state_ = PidState::RATE;
            ESP_LOGI(TAG, "PID state set to RATE");
        }

        // const bool throttleActive = sbus_data.channels_scaled[0] > 10;
        // const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        // if (throttleActive) {
        //     controller->pid_last_throttle_active_time = currentTime;
        //     if (controller->pid_state_ == PidState::SUSPENDED) {
        //         controller->pid_state_ = PidState::ACTIVE;
        //         ESP_LOGI(TAG, "PID reactivated due to throttle application");
        //     }
        // } else {
        //     // If throttle has been inactive for the timeout period, reset reference
        //     if ((currentTime - controller->pid_last_throttle_active_time) > controller->pid_reset_timeout_ms_ && controller->pid_state_ == PidState::ACTIVE) {
        //         controller->pid_state_ = PidState::SUSPENDED;
        //         ESP_LOGI(TAG, "PID suspended due to throttle cut for %ldms", controller->pid_reset_timeout_ms_);
        //     }
        // }

        controller->updateSteering(sbus_data.channels_scaled[ch_steering]);

        if (sbus_data.channels_scaled[ch_debug] > 1900) {
            ESP_LOGI(TAG, "Debug command received");
            controller->esc_driver_.debug(
                controller->test_value_,
                controller->test_delay_,
                controller->test_repeat_
            );
            vTaskDelay(pdMS_TO_TICKS(350));
        }

        if (sbus_data.channels_scaled[ch_arm] > 1900) {
            ESP_LOGI(TAG, "Arm command received");
            controller->esc_driver_.arm_all();
            vTaskDelay(pdMS_TO_TICKS(350));
        }


        if (sbus_data.channels_scaled[arm_switch] > 1900 && !controller->armed_) {
            controller->armed_ = true;
            ESP_LOGI(TAG, "Armed!");
        } else if (sbus_data.channels_scaled[arm_switch] < 1900 && controller->armed_) {
            controller->armed_ = false;
            ESP_LOGI(TAG, "Disarmed (or is it?)");
            controller->esc_driver_.set_all_throttles(sensor::Motor::FAILSAFE_THROTTLE);
        }

        if (controller->armed_) {
            controller->updateThrottle(sbus_data.channels_scaled[ch_throttle]);
        }

        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateThrottle(sensor::channel_t throttle_value) {
    return esc_driver_.set_all_throttles(throttle_value);
}

esp_err_t VehicleDynamicsController::updateSteering(sensor::channel_t steering_value) {
            static uint32_t count = 0;
    // Get IMU data
    const sensor::ImuData& imu_data = VehicleData::instance().getImu();

    // Direct steering if PID is not active
    if (pid_state_ == PidState::SUSPENDED || pid_state_ == PidState::DISABLED) {
        return steering_servo_.setPosition(steering_value);
    }

    // Calculate current heading from quaternion
    float current_heading = extractHeadingFromQuaternion(imu_data);

    // Initialize output to neutral position
    float output = 1000.0f;

    // Calculate steering output based on current mode
    switch (pid_state_) {
        case PidState::QUATMODE: {
            // Update reference heading based on steering input
            heading_reference_ += mapSteeringToRate(steering_value) * 0.02f; // Assuming 50Hz (20ms)

            // Calculate heading error
            float heading_error = calculateHeadingError(current_heading, heading_reference_);

            // Simple P control
            output = 1000 + kp_heading_ * heading_error * 1000.0f; // Scale appropriately

            if (count++ % 10 == 0) {
                ESP_LOGI(TAG, "QUAT - Ref: %.2f, Curr: %.2f, Err: %.2f, Out: %.2f",
                      heading_reference_, current_heading, heading_error, output);
            }
            break;
        }

        case PidState::GYROFF: {
            // Similar to QUATMODE but with gyro feed forward
            heading_reference_ += mapSteeringToRate(steering_value) * 0.02f;
            float heading_error = calculateHeadingError(current_heading, heading_reference_);

            // P control with feed forward
            float pid_term = kp_heading_ * heading_error * 1000.0f;
            float ff_term = ff_gain_ * imu_data.gyro_z;
            output = 1000 + pid_term + ff_term;
            if (count++ % 10 == 0) {
                ESP_LOGI(TAG, "GYROFF - Err: %.2f, PID: %.2f, FF: %.2f, Out: %.2f",
                         heading_error, pid_term, ff_term, output);
            }

            break;
        }

        case PidState::RATE: {
            // Pure rate control mode
            float target_rate = mapSteeringToRate(steering_value);
            float current_rate = static_cast<float>(imu_data.gyro_z) / 64.0f; // Convert to dps
            float rate_error = target_rate - current_rate;

            // Simple P control on rate
            output = 1000 + kp_heading_ * rate_error * 1000.0f;

            if (count++ % 10 == 0) {
                ESP_LOGI(TAG, "RATE - Target: %.2f dps, Current: %.2f dps, Error: %.2f, Out: %.2f",
                         target_rate, current_rate, rate_error, output);
            }

            break;
        }

        default:
            // Default to direct steering
            output = steering_value;
            break;
    }

    // Clamp output to valid range
    output = std::clamp(output, 0.0f, 2000.0f);

    return steering_servo_.setPosition(static_cast<uint16_t>(output));
}

float VehicleDynamicsController::extractHeadingFromQuaternion(const sensor::ImuData& imu_data) {
    // Convert from fixed-point to floating point
    constexpr float Q30_TO_FLOAT = 1.0f / (1 << 30);
    float qx = imu_data.quat9_x * Q30_TO_FLOAT;
    float qy = imu_data.quat9_y * Q30_TO_FLOAT;
    float qz = imu_data.quat9_z * Q30_TO_FLOAT;

    // Calculate w (assuming quaternion is normalized)
    float w_squared = 1.0f - (qx*qx + qy*qy + qz*qz);
    float w = (w_squared > 0.0f) ? sqrtf(w_squared) : 0.0f;

    // Extract yaw (heading) angle from quaternion
    return atan2f(2.0f * (w * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

float VehicleDynamicsController::calculateHeadingError(float current_heading, float reference_heading) {
    float error = reference_heading - current_heading;

    // Normalize error to [-π, π] range
    if (error > M_PI) error -= 2.0f * M_PI;
    if (error < -M_PI) error += 2.0f * M_PI;

    return error;
}

float VehicleDynamicsController::mapSteeringToRate(sensor::channel_t steering_value) {
    // Map 0-2000 to rate (centered at 1000)
    float normalized = (static_cast<float>(steering_value) - 1000.0f) / 1000.0f;

    // Scale to reasonable rate range (±π/2 rad/s ≈ ±90°/s)
    return normalized * (M_PI / 2.0f);
}