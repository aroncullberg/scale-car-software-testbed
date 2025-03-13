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
    constexpr auto toggle_pidloop = static_cast<size_t>(sensor::SbusChannel::AUX7);
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

        const bool throttleActive = sbus_data.channels_scaled[0] > 10;
        const uint32_t currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (throttleActive) {
            controller->pid_last_throttle_active_time = currentTime;
            if (controller->pid_state_ == PidState::SUSPENDED) {
                controller->pid_state_ = PidState::ACTIVE;
                ESP_LOGI(TAG, "PID reactivated due to throttle application");
            }
        } else {
            // If throttle has been inactive for the timeout period, reset reference
            if ((currentTime - controller->pid_last_throttle_active_time) > controller->pid_reset_timeout_ms_ && controller->pid_state_ == PidState::ACTIVE) {
                controller->pid_state_ = PidState::SUSPENDED;
                ESP_LOGI(TAG, "PID suspended due to throttle cut for %ldms", controller->pid_reset_timeout_ms_);
            }
        }

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

esp_err_t VehicleDynamicsController::updateSteering(uint16_t steering_value) {
    return steering_servo_.setPosition(steering_value);
}