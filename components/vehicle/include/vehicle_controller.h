#pragma once

#include <array>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "channel_types.h"
#include "servo.h"
#include "motor.h"

namespace vehicle {

class Controller {
public:
    struct Config {
        Servo& steering_servo;
        Servo& aero_servo;
        Servo& aux_servo1;
        Servo& aux_servo2;
        motor::Motor& motor_fl;
        motor::Motor& motor_fr;
        motor::Motor& motor_rl;
        motor::Motor& motor_rr;
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
    void control_motors(const rclink::RcChannels& channels);
    void handle_rc_loss();
    void countersteer();
    void update_leds();
    void refresh_headlights(uint8_t r, uint8_t g, uint8_t b);
    void refresh_rear_strip(const uint8_t pixels[34][3]);
    float get_brightness_multiplier();

    Config config_;
    bool armed_{false};
    bool diff_on_{false};
    int steering_range_{65};
    int steering_center_{1500};
    TaskHandle_t control_task_handle_{nullptr};

    // LED Control state
    uint32_t led_update_counter_{0};
    uint8_t current_led_r_{120};
    uint8_t current_led_g_{42};
    uint8_t current_led_b_{0};
    bool red_flash_state_{false};
    uint32_t red_flash_counter_{0};

    // New non-blocking animation states
    bool last_armed_state_{false};
    bool arming_animation_active_{false};
    uint32_t arming_animation_step_{0};
    int seeker_pos_{0};
    int seeker_dir_{1};
    float seeker_trails_[34]{0.0f};

    static constexpr uint16_t THROTTLE_DEADBAND = 40;

    static constexpr const char* TAG = "VehicleController";
};

} // namespace vehicle
