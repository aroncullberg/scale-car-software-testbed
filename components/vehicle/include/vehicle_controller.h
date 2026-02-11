#pragma once

#include <array>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "channel_types.h"
#include "servo.h"
#include "DShotRMT.h"

namespace vehicle {

class Controller {
public:
    struct Config {
        Servo& steering_servo;
        DShotRMT& motor_fl;
        DShotRMT& motor_fr;
        DShotRMT& motor_rl;
        DShotRMT& motor_rr;
        int motor_poles{14};
    };

    explicit Controller(const Config& config);
    ~Controller();

    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    esp_err_t start();
    void stop();

private:
    static void control_task_entry(void* arg);
    [[noreturn]] void control_task();

    void process_rc_input(const rclink::RcChannels& channels);
    void control_steering(rclink::channel_value_t steering_input);
    void control_motors(const rclink::RcChannels& channels,
                        rclink::channel_value_t throttle_input, bool reverse);
    void handle_rc_loss();
    void countersteer();

    Config config_;
    bool armed_{false};
    bool reverse_{false};
    bool diff_on_{false};
    int steering_range_{65};
    int steering_center_{1500};
    TaskHandle_t control_task_handle_{nullptr};
    std::array<uint32_t, 4> motor_erpm_{};

    static constexpr const char* TAG = "VehicleController";
};

} // namespace vehicle
