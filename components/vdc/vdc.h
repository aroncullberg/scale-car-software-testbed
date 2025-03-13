#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "servo.h"
#include "data_pool.h"
#include "esc_driver.h"
#include "quaternion.h"
#include "config_manager.h"

#include <memory>
#include <functional>

class VehicleDynamicsController {
public:
    struct Config {
        Servo::Config steering_servo;

        EscDriver::Config esc_config;

        uint32_t task_stack_size{4096};
        uint8_t task_priority{5};
        TickType_t task_period{pdMS_TO_TICKS(20)}; // 50Hz default
    };

    explicit VehicleDynamicsController(const Config& config);
    ~VehicleDynamicsController();

    VehicleDynamicsController(const VehicleDynamicsController&) = delete;
    VehicleDynamicsController& operator=(const VehicleDynamicsController&) = delete;

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

    void enablePID(bool enable);

    void updateFromConfig();

private:
    static constexpr auto TAG = "VehicleDynamics";

    static void controllerTask(void* arg);
    esp_err_t updateSteering(sensor::channel_t steering_value);
    esp_err_t updateThrottle(sensor::channel_t throttle_value);

    Config config_;
    Servo steering_servo_;
    EscDriver esc_driver_;
    TaskHandle_t task_handle_{nullptr};

    enum class PidState {
        ACTIVE,
        SUSPENDED,
        DISABLED
    };

    float pid_turnrate_{2.0f}; // radians per second
    PidState pid_state_{PidState::DISABLED};

    Quaternion referenceOrientation_;
    float targetHeading_{0.0f};
    uint32_t lastThrottleActiveTime_{0};

    struct {
        float kP{1.5f};                      // Proportional gain
        float kI{0.05f};                     // Integral gain (bad for drift?)
        float kD{0.2f};                      // Derivative gain
        float integral{0.0f};                // Integral accumulator
        float previousError{0.0f};           // Previous error for derivative
    } headingPid_;

    // Gyro assistance settings
    struct {
        bool enabled{true};                  // Enable/disable gyro assistance
        float strength{0.75f};                // Gyro effect strength (0.0-1.0)
        uint32_t resetTimeoutMs{800};        // Time after throttle cut to reset reference
    } gyroConfig_;

    // Helper methods
    static Quaternion getCurrentOrientation(const sensor::ImuData& imu_data);
    float calculateHeadingError(const sensor::ImuData& imu_data) const;
    float computePID(float error, float deltaTime);
    void updateReferenceOrientation(const sensor::SbusData& sbus_data,
                                                         const sensor::ImuData& imu_data);
    void updateGyroAssistance(float deltaTime, const sensor::SbusData& sbus_data,
                                                         const sensor::ImuData& imu_data);
    void logOrientationData(const char* prefix, const Quaternion& orientation);

    bool is_running_{false};
    bool armed_{false};

    int test_value_{2};
    int test_delay_{500};
    int test_repeat_{1};

    std::function<void()> callback_;
};