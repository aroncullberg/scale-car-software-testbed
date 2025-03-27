#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "servo.h"
#include "data_pool.h"
#include "pid.h"
#include "quaternion.h"
#include "config_manager.h"

#include "DShotRMT.h"

#include <memory>
#include <functional>

class VehicleDynamicsController {
public:
    struct Config {
        Servo::Config servo_config{};

        struct MotorsConfig {
            gpio_num_t front_right_pin{GPIO_NUM_NC};
            gpio_num_t front_left_pin{GPIO_NUM_NC};
            gpio_num_t rear_left_pin{GPIO_NUM_NC};
            gpio_num_t rear_right_pin{GPIO_NUM_NC};
            dshot_mode_t dshot_mode{DSHOT150}; // NOTE: bluejay which were using does not support DSHOT150 only DSHOT300 and up
        } motors_config;

        uint16_t steering_pid_hz{60};


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

    void updateFromConfig();

private:
    static constexpr auto TAG = "VehicleDynamics";

    static void controllerTask(void* arg);
    static void steeringTask(void* arg);
    static void motorTask(void* arg);

    esp_err_t updateThrottle(sensor::channel_t throttle_value);

    Config config_;
    Servo steering_servo_;
    TaskHandle_t task_handle_{nullptr};
    TaskHandle_t steeringtask_handle_{nullptr};
    TaskHandle_t motortask_handle_{nullptr};



    void updateRPMTelemetry(sensor::channel_t throttle_value);

    DShotRMT motor_fr_; // Front right
    DShotRMT motor_fl_; // Front left
    DShotRMT motor_rl_; // Rear left
    DShotRMT motor_rr_; // Rear right


    PIDController pid_steering_;
    PIDController pid_motor_fr_;
    PIDController pid_motor_fl_;
    PIDController pid_motor_rl_;
    PIDController pid_motor_rr_;


    float max_turn_rate_{60.0f};
    uint32_t throttle_to_rpm_throttle_{29};
    // 2300kv * 3.8 * 6 = 57960 rpm at 100% throttle
    // 57960 / 2000 = 28.98 ~> 29

    float gyro_deadband_{0.1f};

    bool log_erp_{false};
    bool erp_verbose{true};
    bool is_running_{false};
    bool bidirectional_{false};
    bool bypass_pid_{false};
    bool failsafe_{true};
    bool armed_{false};

    std::function<void()> callback_;
};