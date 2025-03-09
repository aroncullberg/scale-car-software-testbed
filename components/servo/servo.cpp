// servo.cpp
#include "servo.h"
#include <algorithm>

// TODO: figure out what should be in constuctor(?) and what should be in init
Servo::Servo(const Config& config) : config_(config) {
    // TODO: Clamp this value to a resnoable range
    min_pulse_width_us_ = ConfigManager::instance().getInt("servo/min_steer", min_pulse_width_us_);
    max_pulse_width_us_ = ConfigManager::instance().getInt("servo/max_steer", max_pulse_width_us_);
    // center_pulse_width_us_ = ConfigManager::instance().getInt("servo/center", center_pulse_width_us_);
    invert_steering_ = ConfigManager::instance().getBool("servo/inv_steer", invert_steering_);
    offset_ = ConfigManager::instance().getInt("servo/offset", offset_);

    callback_ = [this] { this->updateFromConfig(); };
    ConfigManager::instance().registerCallback(callback_);

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
    ConfigManager::instance().unregisterCallback(&callback_);
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

void Servo::updateFromConfig() {
    ESP_LOGI(TAG, "Updating Servo configuration from ConfigManager");

    int new_min_pulse_width_us_ = ConfigManager::instance().getInt("servo/min_steer", min_pulse_width_us_);
    if (new_min_pulse_width_us_ != min_pulse_width_us_) {
        ESP_LOGI(TAG, "Min pulse width changed: %d -> %d",
                 min_pulse_width_us_, new_min_pulse_width_us_);
        min_pulse_width_us_ = new_min_pulse_width_us_;
    }
    int new_max_pulse_width_us_ = ConfigManager::instance().getInt("servo/max_steer", max_pulse_width_us_);
    if (new_max_pulse_width_us_ != max_pulse_width_us_) {
        ESP_LOGI(TAG, "Max pulse width changed: %d -> %d",
                 max_pulse_width_us_, new_max_pulse_width_us_);
        max_pulse_width_us_ = new_max_pulse_width_us_;
    }
    // center_pulse_width_us_ = ConfigManager::instance().getInt("servo/center", center_pulse_width_us_);

    bool new_invert_steering_ = ConfigManager::instance().getBool("servo/inv_steer", invert_steering_);
    if (new_invert_steering_ != invert_steering_) {
        ESP_LOGI(TAG, "Invert steering changed: %d -> %d",
                 invert_steering_, new_invert_steering_);
        invert_steering_ = new_invert_steering_;
    }
    int new_offset_ = ConfigManager::instance().getInt("servo/offset", offset_);
    if (new_offset_ != offset_) {
        ESP_LOGI(TAG, "Offset changed: %d -> %d",
                 offset_, new_offset_);
        offset_ = new_offset_;
    }
}



uint32_t Servo::calculateCompareValue(uint16_t position) {
    position = std::clamp(position,
                         static_cast<uint16_t>(min_pulse_width_us_),
                         static_cast<uint16_t>(max_pulse_width_us_));

    uint32_t pulse_width;

    // TODO: Make more readable
    if (position < center_pulse_width_us_) {
        float ratio = static_cast<float>(position - min_pulse_width_us_) /
                     (center_pulse_width_us_ - min_pulse_width_us_);
        pulse_width = min_pulse_width_us_ +
                     ratio * (center_pulse_width_us_ - min_pulse_width_us_);
    } else {
        float ratio = static_cast<float>(position - center_pulse_width_us_) /
                     (max_pulse_width_us_ - center_pulse_width_us_);
        pulse_width = center_pulse_width_us_ +
                     ratio * (max_pulse_width_us_ - center_pulse_width_us_);
    }

    pulse_width = std::clamp(pulse_width + offset_,
                             static_cast<uint32_t>(min_pulse_width_us_),
                             static_cast<uint32_t>(max_pulse_width_us_));

    if (invert_steering_) {
        pulse_width = min_pulse_width_us_ + max_pulse_width_us_ - pulse_width;
    }

    return pulse_width;
}

esp_err_t Servo::setPosition(const uint16_t position) {
    return mcpwm_comparator_set_compare_value(comparator_, calculateCompareValue(position));
}
