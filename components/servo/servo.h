#pragma once

#include <bits/stl_pair.h>

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "channel_types.h"

class Servo {
public:
    struct Config {
        gpio_num_t gpio_num{GPIO_NUM_NC};     // GPIO pin number
        int min_pulse_width_us{1000};         // Minimum pulse width in microseconds
        int max_pulse_width_us{2000};        // Maximum pulse width in microseconds
        uint32_t freq_hz{50};                // PWM frequency in Hz (standard servo is 50Hz)
        rclink::channel_value_t failsafe_position{1000}; // Failsafe position (center)
    };

    explicit Servo(const Config& config);
    ~Servo();

    // Delete copy operations
    Servo(const Servo&) = delete;
    Servo& operator=(const Servo&) = delete;

    // Primary control method
    esp_err_t setPosition(rclink::channel_value_t position);

    // Runtime configuration methods
    esp_err_t setCenterPoint(uint16_t center_us);
    esp_err_t setLimits(uint16_t min_us, uint16_t max_us);
    esp_err_t setRange(int range_percent);
    esp_err_t setOffset(int offset_us);
    esp_err_t setInvert(bool invert);
    esp_err_t setFailsafe();

    // Getters
    uint16_t getCenterPoint() const { return center_pulse_width_us_; }
    std::pair<uint16_t, uint16_t> getLimits() const { return {config_.min_pulse_width_us, config_.max_pulse_width_us}; }
    int getRange() const { return range_; }
    int getOffset() const { return offset_; }
    bool getInvert() const { return invert_steering_; }

    // Safety methods
    esp_err_t enable();
    esp_err_t disable();
    bool isEnabled() const { return enabled_; }

private:
    esp_err_t init();
    uint32_t calculateCompareValue(rclink::channel_value_t position);

    Config config_;
    mcpwm_timer_handle_t timer_{nullptr};
    mcpwm_oper_handle_t operator_{nullptr};
    mcpwm_cmpr_handle_t comparator_{nullptr};
    mcpwm_gen_handle_t generator_{nullptr};

    int center_pulse_width_us_{1500};
    int offset_{0};
    int range_{65};
    bool invert_steering_{true};
    bool enabled_{true};

    static constexpr uint32_t TIMEBASE_RESOLUTION_HZ = 1000000;  // 1MHz, 1us per tick
    static constexpr const char* TAG = "Servo";
};
