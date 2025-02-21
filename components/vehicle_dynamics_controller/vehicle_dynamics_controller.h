#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "servo.h"
#include "data_pool.h"
#include "esc_driver.h"
#include <memory>

class VehicleDynamicsController {
public:
    struct Config {
        // Servo configuration
        Servo::Config steering_servo;

        // ESC driver configuration
        EscDriver::Config esc_config;
        
        // Task configuration
        uint32_t task_stack_size{4096};
        uint8_t task_priority{5};
        TickType_t task_period{pdMS_TO_TICKS(20)}; // 50Hz default
    };

    explicit VehicleDynamicsController(const Config& config);
    ~VehicleDynamicsController();

    // Delete copy constructor and assignment operator
    VehicleDynamicsController(const VehicleDynamicsController&) = delete;
    VehicleDynamicsController& operator=(const VehicleDynamicsController&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

private:
    static constexpr const char* TAG = "VehicleDynamics";
    
    static void controllerTask(void* arg);
    esp_err_t updateSteering(uint16_t steering_position);
    esp_err_t updateThrottle(uint16_t throttle_value);


    Config config_;
    Servo steering_servo_;
    EscDriver esc_driver_;
    TaskHandle_t task_handle_{nullptr};
    bool is_running_{false};
};