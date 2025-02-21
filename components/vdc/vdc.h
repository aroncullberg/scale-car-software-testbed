#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "servo.h"
#include "data_pool.h"
#include "esc_driver.h"
#include "quaternion.h"
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

    Quaternion referenceOrientation_;
    float targetHeading_{0.0f};
    uint32_t lastThrottleActiveTime_{0};

    struct {
        float kP{2.0f};                      // Proportional gain
        float kI{0.05f};                     // Integral gain
        float kD{0.1f};                      // Derivative gain
        float integral{0.0f};                // Integral accumulator
        float previousError{0.0f};           // Previous error for derivative
    } headingPid_;

    // Gyro assistance settings
    struct {
        bool enabled{true};                  // Enable/disable gyro assistance
        float strength{0.5f};                // Gyro effect strength (0.0-1.0)
        uint32_t resetTimeoutMs{500};        // Time after throttle cut to reset reference
    } gyroConfig_;

    // Helper methods
    Quaternion getCurrentOrientation();
    float calculateHeadingError();
    float computePID(float error, float deltaTime);
    void updateReferenceOrientation();
    void updateGyroAssistance(float deltaTime);
    

    Config config_;
    Servo steering_servo_;
    EscDriver esc_driver_;
    TaskHandle_t task_handle_{nullptr};
    bool is_running_{false};
};