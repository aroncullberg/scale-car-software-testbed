#pragma once

#include <array>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "channel_types.h"
#include "servo.h"
#include "DShotRMT.h"
// #include "waypoint_storage.h"

namespace vehicle {

class Controller {
public:
    struct Config {
        // Hardware components (references, owned by main)
        Servo& steering_servo;
        DShotRMT& motor_fl;
        DShotRMT& motor_fr;
        DShotRMT& motor_rl;
        DShotRMT& motor_rr;

        // Motor configuration
        int motor_poles{14};
    };

    explicit Controller(const Config& config);
    ~Controller();

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    esp_err_t init();
    esp_err_t start();
    void stop();

private:
    static void control_task_entry(void* arg);
    [[noreturn]] void control_task();

    // Core control methods
    void process_rc_input(const rclink::RcChannels& channels);
    void control_steering(rclink::channel_value_t steering_input);
    void control_motors(rclink::channel_value_t throttle_input, bool reverse);

    // Safety
    void handle_rc_loss();
    void emergency_stop();

    // Navigation math helpers
    float calculate_bearing(int32_t from_lat_e7, int32_t from_lon_e7,
                           int32_t to_lat_e7, int32_t to_lon_e7);
    float calculate_distance(int32_t from_lat_e7, int32_t from_lon_e7,
                            int32_t to_lat_e7, int32_t to_lon_e7);
    float normalize_angle(float angle);

    // State
    Config config_;
    bool armed_{false};
    bool reverse_{false};

    // Steering calibration (stored when disarmed)
    int steering_range_{65};
    int steering_center_{1500};

    // Task handle
    TaskHandle_t control_task_handle_{nullptr};

    // Motor state (4 motors: FL, FR, RL, RR)
    struct MotorState {
        uint32_t current_erpm{0};
    };
    std::array<MotorState, 4> motor_states_;

    // Telemetry counter (send every N control cycles)
    uint32_t control_cycle_count_{0};

    // RC switch debouncing (previous state)
    bool prev_record_switch_{false};
    bool prev_reset_switch_{false};
    bool prev_next_switch_{false};
    bool prev_prev_switch_{false};
    bool prev_preset_switch_{false};

    static constexpr const char* TAG = "VehicleController";
};

} // namespace vehicle
