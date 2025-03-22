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
    esp_err_t updateSteering(sensor::channel_t steering_value, const sensor::ImuData& imu_data);
    esp_err_t updateThrottle(sensor::channel_t throttle_value);

    Config config_;
    Servo steering_servo_;
    EscDriver esc_driver_;
    TaskHandle_t task_handle_{nullptr};

    enum class PidState {
        ACTIVE,
        DISABLED
    };


    PidState pid_state_{PidState::ACTIVE};

    // New methods for control modes
    esp_err_t processRateControl(sensor::channel_t steering_value, const sensor::ImuData& imu_data);
    esp_err_t processGyroFeedForward(sensor::channel_t steering_value, const sensor::ImuData& imu_data);
    esp_err_t processCombinedControl(sensor::channel_t steering_value, const sensor::ImuData& imu_data);

    void resetPidController();

    float rate_p_gain_{10.0f};          // P gain for rate control
    float rate_d_gain_{1.0f};
    float rate_i_gain_{0.05f};
    float max_turn_rate_{110.0f};        // deg/s
    float integral_sum_{0.0f};          // Accumulated error for I term
    float previous_error_{0.0f};        // Previous error for D term
    uint64_t previous_time_{0};
    float anti_windup_limit_{10.0f};    // Limit to prevent integral windup

    bool pid_debug_{false};

    bool is_running_{false};
    bool armed_{false};

    int test_value_{2};
    int test_delay_{500};
    int test_repeat_{1};

    bool norxw_continous_{true};
    bool rx_warned_{false};

    std::function<void()> callback_;
};