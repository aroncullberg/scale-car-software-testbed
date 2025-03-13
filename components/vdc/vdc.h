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

    PidState pid_state_{PidState::DISABLED};

    uint32_t pid_last_throttle_active_time{0};
    uint32_t pid_reset_timeout_ms_{1000}; // 1 second default

    bool is_running_{false};
    bool armed_{false};

    int test_value_{2};
    int test_delay_{500};
    int test_repeat_{1};

    std::function<void()> callback_;
};