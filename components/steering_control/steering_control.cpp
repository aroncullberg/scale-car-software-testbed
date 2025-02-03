#include "steering_control.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include <cmath>

namespace control {

SteeringControl::SteeringControl(const Config& config) 
    : config_(config) {
}

SteeringControl::~SteeringControl() {
    stop();
}

esp_err_t SteeringControl::init() {
    // Configure LEDC timer for servo control
    ledc_timer_.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer_.duty_resolution = LEDC_TIMER_14_BIT;
    ledc_timer_.timer_num = LEDC_TIMER_0;
    ledc_timer_.freq_hz = 50; // Standard servo frequency
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_));

    // Configure LEDC channel
    ledc_channel_.channel = LEDC_CHANNEL_0;
    ledc_channel_.duty = 0;
    ledc_channel_.gpio_num = config_.servo_pin;
    ledc_channel_.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel_.timer_sel = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_));

    return ESP_OK;
}

esp_err_t SteeringControl::start() {
    if (is_running_) {
        return ESP_OK;
    }

    // Create steering control task
    BaseType_t ret = xTaskCreatePinnedToCore(
        steeringTask,
        "steering_ctrl",
        4096,
        this,
        config_.task_priority,
        &task_handle_,
        config_.task_core_id
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create steering control task");
        return ESP_FAIL;
    }

    is_running_ = true;
    return ESP_OK;
}

esp_err_t SteeringControl::stop() {
    if (!is_running_) {
        return ESP_OK;
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    is_running_ = false;
    return ESP_OK;
}

float SteeringControl::calculateDynamicSteeringRatio(float speed) const {
    if (speed <= config_.min_speed_for_dynamic_ratio) {
        return config_.max_steering_ratio;
    }
    if (speed >= config_.max_speed_for_dynamic_ratio) {
        return config_.min_steering_ratio;
    }

    // Linear interpolation between max and min ratio based on speed
    float speed_factor = (speed - config_.min_speed_for_dynamic_ratio) / 
                        (config_.max_speed_for_dynamic_ratio - config_.min_speed_for_dynamic_ratio);
    
    return config_.max_steering_ratio + 
           (config_.min_steering_ratio - config_.max_steering_ratio) * speed_factor;
}

float SteeringControl::calculateServoAngle(float steering_input, float speed) const {
    // Get dynamic steering ratio based on speed
    float dynamic_ratio = calculateDynamicSteeringRatio(speed);
    
    // Calculate steering angle with dynamic ratio
    float steering_angle = steering_input * config_.max_steering_angle * dynamic_ratio;
    
    // Clamp to maximum steering angle
    return std::clamp(steering_angle, -config_.max_steering_angle, config_.max_steering_angle);
}

esp_err_t SteeringControl::outputServoCommand(float angle) {
    // Convert angle to servo duty cycle (assuming standard servo range -90 to +90 degrees)
    // For 14-bit resolution (0-16383), typical servo range is ~820 to ~2250
    constexpr float SERVO_MIN_DUTY = 820.0f;
    constexpr float SERVO_MAX_DUTY = 2250.0f;
    constexpr float SERVO_RANGE_DEGREES = 180.0f;

    // Map angle to duty cycle
    float normalized_angle = (angle + 90.0f) / SERVO_RANGE_DEGREES;
    uint32_t duty = static_cast<uint32_t>(
        SERVO_MIN_DUTY + normalized_angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)
    );

    return ledc_set_duty_and_update(ledc_channel_.speed_mode, 
                                   ledc_channel_.channel, 
                                   duty, 
                                   0);
}

void SteeringControl::steeringTask(void* parameters) {
    auto* steering = static_cast<SteeringControl*>(parameters);
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // Get latest data from the vehicle data pool
        const auto sbus_data = VehicleData::instance().getSbus();
        const auto gps_data = VehicleData::instance().getGPS();

        // Calculate speed in m/s from GPS data
        float speed_mps = static_cast<float>(gps_data.speed_mmps) / 1000.0f;

        // Get steering input from SBUS (-1 to 1 range)
        float steering_input = sbus_data.channels[static_cast<uint8_t>(sensor::SbusChannel::STEERING)];

        // Calculate desired steering angle
        float steering_angle = steering->calculateServoAngle(steering_input, speed_mps);

        // Output to servo
        ESP_ERROR_CHECK(steering->outputServoCommand(steering_angle));

        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, steering->config_.task_period);
    }
}

} // namespace control