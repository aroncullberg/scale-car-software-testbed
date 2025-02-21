// servo.cpp
#include "servo.h"
#include <algorithm>

Servo::Servo(const Config& config) : config_(config) {
    init();
}

Servo::~Servo() {
    if (generator_) {
        mcpwm_del_generator(generator_);
    }
    if (comparator_) {
        mcpwm_del_comparator(comparator_);
    }
    if (operator_) {
        mcpwm_del_operator(operator_);
    }
    if (timer_) {
        mcpwm_del_timer(timer_);
    }
}

esp_err_t Servo::init() {
    // Calculate period based on frequency
    uint32_t period_ticks = TIMEBASE_RESOLUTION_HZ / config_.freq_hz;

    // Create timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = period_ticks,
    };
    esp_err_t err = mcpwm_new_timer(&timer_config, &timer_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create timer: %d", err);
        return err;
    }

    // Create operator
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    err = mcpwm_new_operator(&operator_config, &operator_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create operator: %d", err);
        return err;
    }

    // Connect timer and operator
    err = mcpwm_operator_connect_timer(operator_, timer_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to connect timer to operator: %d", err);
        return err;
    }

    // Create comparator
    mcpwm_comparator_config_t comparator_config = {
        .flags = {.update_cmp_on_tez = true}
    };
    err = mcpwm_new_comparator(operator_, &comparator_config, &comparator_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create comparator: %d", err);
        return err;
    }

    // Create generator
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = config_.gpio_num,
    };
    err = mcpwm_new_generator(operator_, &generator_config, &generator_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create generator: %d", err);
        return err;
    }

    // Set generator actions
    err = mcpwm_generator_set_action_on_timer_event(
        generator_,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    );
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to set generator action on timer event: %d", err);
        return err;
    }

    err = mcpwm_generator_set_action_on_compare_event(
        generator_,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_, MCPWM_GEN_ACTION_LOW)
    );
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to set generator action on compare event: %d", err);
        return err;
    }

    // Enable and start timer
    err = mcpwm_timer_enable(timer_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to enable timer: %d", err);
        return err;
    }

    err = mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to start timer: %d", err);
        return err;
    }

    return ESP_OK;
}

// uint32_t Servo::calculateCompareValue(uint16_t position) const {
//     position = std::max(uint16_t(1000), std::min(uint16_t(2000), position));
    
//     return 3000 - position;
// }

uint32_t Servo::calculateCompareValue(uint16_t position) const {
    // Clamp the position to the valid range
    position = std::max(uint16_t(1000), std::min(uint16_t(2000), position));

    // Scale position from [1000, 2000] to [500, 2500]
    uint32_t scaled = CONFIG_SERVO_MIN_PULSE_WIDTH_US + ((position - 1000) * (CONFIG_SERVO_MAX_PULSE_WIDTH_US - CONFIG_SERVO_MIN_PULSE_WIDTH_US)) / (2000 - 1000);

    // Invert within the range [500, 2500]
    return 3000 - scaled;
}


esp_err_t Servo::setPosition(uint16_t position) {
    // return ESP_OK;
    return mcpwm_comparator_set_compare_value(comparator_, calculateCompareValue(position));
}
