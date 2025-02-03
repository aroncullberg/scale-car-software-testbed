#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"

namespace control {

class SteeringControl {
public:
    struct Config {
        // Task configuration
        TickType_t task_period{pdMS_TO_TICKS(10)}; // 100Hz default
        uint8_t task_priority{5};
        uint8_t task_core_id{1}; // Run on core 1 by default

        // Steering configuration
        gpio_num_t servo_pin{GPIO_NUM_NC};
        float max_steering_angle{30.0f}; // Maximum steering angle in degrees
        float min_speed_for_dynamic_ratio{2.0f}; // m/s
        float max_speed_for_dynamic_ratio{10.0f}; // m/s
        float min_steering_ratio{1.0f}; // At high speed
        float max_steering_ratio{2.0f}; // At low speed
    };

    explicit SteeringControl(const Config& config);
    ~SteeringControl();

    // Delete copy constructor and assignment
    SteeringControl(const SteeringControl&) = delete;
    SteeringControl& operator=(const SteeringControl&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

private:
    static constexpr const char* TAG = "SteeringControl";
    
    // Task related
    static void steeringTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};
    
    // Helper methods
    float calculateDynamicSteeringRatio(float speed) const;
    float calculateServoAngle(float steering_input, float speed) const;
    esp_err_t outputServoCommand(float angle);

    Config config_;
    bool is_running_{false};

    // Ledc configuration for servo control
    ledc_timer_config_t ledc_timer_{};
    ledc_channel_config_t ledc_channel_{};
};

} // namespace control