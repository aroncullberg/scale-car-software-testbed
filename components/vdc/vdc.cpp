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

    esc_driver_.set_all_throttles(sensor::Motor::FAILSAFE_THROTTLE);

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

    float new_rate_p_gain = ConfigManager::instance().getFloat("vdc/kp", rate_p_gain_);
    if (new_rate_p_gain != rate_p_gain_) {
        ESP_LOGI(TAG, "Rate P gain changed: %.2f -> %.2f", rate_p_gain_, new_rate_p_gain);
        rate_p_gain_ = new_rate_p_gain;
    }
    float new_rate_i_gain_ = ConfigManager::instance().getFloat("vdc/ki", rate_i_gain_);
    if (new_rate_i_gain_ != rate_i_gain_) {
        ESP_LOGI(TAG, "Rate I gain changed: %.2f -> %.2f", rate_i_gain_, new_rate_i_gain_);
        rate_i_gain_ = new_rate_i_gain_;
    }
    float new_rate_d_gain_ = ConfigManager::instance().getFloat("vdc/kd", rate_d_gain_);
    if (new_rate_d_gain_ != rate_d_gain_) {
        ESP_LOGI(TAG, "Rate D gain changed: %.2f -> %.2f", rate_d_gain_, new_rate_d_gain_);
        rate_d_gain_ = new_rate_d_gain_;
    }
    float new_anti_windup_limit_ = ConfigManager::instance().getFloat("vdc/anti_windup", anti_windup_limit_);
    if (new_anti_windup_limit_ != anti_windup_limit_) {
        ESP_LOGI(TAG, "Anti-windup limit changed: %.2f -> %.2f", anti_windup_limit_, new_anti_windup_limit_);
        anti_windup_limit_ = new_anti_windup_limit_;
    }
    float new_max_turn_rate_ = ConfigManager::instance().getFloat("vdc/turnrate", max_turn_rate_);
    if (new_max_turn_rate_ != max_turn_rate_) {
        ESP_LOGI(TAG, "Max turn rate changed: %.2f -> %.2f", max_turn_rate_, new_max_turn_rate_);
        max_turn_rate_ = new_max_turn_rate_;
    }
}

void VehicleDynamicsController::resetPidController() {
    integral_sum_ = 0.0f;
    previous_error_ = 0.0f;
    previous_time_ = esp_timer_get_time();
}


void VehicleDynamicsController::controllerTask(void* arg) {
    auto* controller = static_cast<VehicleDynamicsController*>(arg);
    TickType_t last_wake_time = xTaskGetTickCount();

    const VehicleData &vehicle_data = VehicleData::instance();

    constexpr auto ch_throttle = static_cast<size_t>(sensor::SbusChannel::THROTTLE);
    constexpr auto ch_steering = static_cast<size_t>(sensor::SbusChannel::STEERING);
    constexpr auto pid_state = static_cast<size_t>(sensor::SbusChannel::AUX3);
    constexpr auto arm_switch = static_cast<size_t>(sensor::SbusChannel::AUX8);
    // constexpr auto ch_debug = static_cast<size_t>(sensor::SbusChannel::AUX9);

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
        if (sbus_data.channels_scaled[pid_state] > 1900 && controller->pid_state_ != PidState::DISABLED) {
            controller->pid_state_ = PidState::DISABLED;
            controller->resetPidController();
            ESP_LOGI(TAG, "P(ID) loop state set to DISABLED");
        } else if (sbus_data.channels_scaled[pid_state] < 1900 && controller->pid_state_ != PidState::ACTIVE) {
            controller->pid_state_ = PidState::ACTIVE;
            ESP_LOGI(TAG, "P(ID) loop state set to ACTIVE");
        }

        // if (sbus_data.channels_scaled[ch_debug] > 1900) {
        //     ESP_LOGI(TAG, "Debug command received");
        //     controller->esc_driver_.debug(
        //         controller->test_value_,
        //         controller->test_delay_,
        //         controller->test_repeat_
        //     );
        //     vTaskDelay(pdMS_TO_TICKS(350));
        // }

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

        controller->updateSteering(sbus_data.channels_scaled[ch_steering], imu_data);


        vTaskDelayUntil(&last_wake_time, controller->config_.task_period);
    }
}

esp_err_t VehicleDynamicsController::updateThrottle(sensor::channel_t throttle_value) {
    return esc_driver_.set_all_throttles(throttle_value);
}

esp_err_t VehicleDynamicsController::updateSteering(sensor::channel_t steering_value, const sensor::ImuData& imu_data) {
    if (!is_running_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pid_state_ == PidState::DISABLED) {
        return steering_servo_.setPosition(steering_value);
    }

    uint64_t current_time = esp_timer_get_time();
    float dt = (current_time - previous_time_) / 1000000.0f; // μs -> s
    if (dt > 0.1f) dt = 0.02f;
    previous_time_ = current_time;

    const float norm_steering = (static_cast<float>(steering_value) - 1000.0f) / 1000.0f;
    const float desired_rate = norm_steering * max_turn_rate_;
    const float current_rate = imu_data.gyro_z * sensor::ImuData::GYRO_TO_DPS;

    const float error = desired_rate - current_rate;

    // P - TERM
    float p_output = error * rate_p_gain_;

    // I - TERM
    integral_sum_ += error * dt;
    integral_sum_ = std::clamp(integral_sum_, -anti_windup_limit_, anti_windup_limit_);
    const float i_output = integral_sum_ * rate_i_gain_;

    // D - TERM
    // TODO: Add low-pass filter to gyro rate
    const float error_rate = (error - previous_error_) / dt;
    previous_error_ = error;
    const float d_output = error_rate * rate_d_gain_;

    const float pid_output = p_output + i_output + d_output;

    int16_t output = sensor::Servo::NEUTRAL_POSITION + static_cast<int16_t>(pid_output);

    output = std::ranges::clamp(output, sensor::Servo::MIN_POSITION, sensor::Servo::MAX_POSITION);

    static uint32_t log_counter = 0;
    if (log_counter++ % 10 == 0) {
        ESP_LOGI(TAG, "RATE: desired=%.1f, current=%.1f, error=%.1f, output=%u",
                 desired_rate, current_rate, error, output);
    }

    return steering_servo_.setPosition(output);
}
