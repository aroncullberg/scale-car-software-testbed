#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"

namespace control {

class VehicleDynamicsController {
public:
    struct Config {
        // Task configuration
        TickType_t task_period{pdMS_TO_TICKS(5)}; // 200Hz default
        uint8_t task_priority{5};
        uint8_t task_core_id{1}; // Run on core 1 by default

        // Power distribution configuration
        float default_front_rear_split{0.5f}; // 50-50 by default
        uint8_t front_rear_split_channel{2}; // AUX1 by default for F/R split
    };

    struct MotorThrottles {
        float front_left{0.0f};
        float front_right{0.0f};
        float rear_left{0.0f};
        float rear_right{0.0f};
    };

    explicit VehicleDynamicsController(const Config& config);
    ~VehicleDynamicsController();

    // Delete copy constructor and assignment
    VehicleDynamicsController(const VehicleDynamicsController&) = delete;
    VehicleDynamicsController& operator=(const VehicleDynamicsController&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

private:
    static constexpr const char* TAG = "VDC";
    
    // Task related
    static void controllerTask(void* parameters);
    TaskHandle_t task_handle_{nullptr};
    
    // Helper methods
    MotorThrottles calculateMotorThrottles(float base_throttle, float front_rear_split) const;

    Config config_;
    bool is_running_{false};

    // Current state
    MotorThrottles current_throttles_{};
};

} // namespace control