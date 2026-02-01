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

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    err = mcpwm_new_operator(&operator_config, &operator_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create operator: %d", err);
        return err;
    }

    err = mcpwm_operator_connect_timer(operator_, timer_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to connect timer to operator: %d", err);
        return err;
    }

    mcpwm_comparator_config_t comparator_config = {
        .flags = {.update_cmp_on_tez = true}
    };
    err = mcpwm_new_comparator(operator_, &comparator_config, &comparator_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create comparator: %d", err);
        return err;
    }

    mcpwm_comparator_set_compare_value(comparator_, center_pulse_width_us_);

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = config_.gpio_num,
    };
    err = mcpwm_new_generator(operator_, &generator_config, &generator_);
    if (err != ESP_OK) {
        ESP_LOGE("servo", "Failed to create generator: %d", err);
        return err;
    }

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




uint32_t Servo::calculateCompareValue(const rclink::channel_value_t position) {
    if (position > 2000) {
        ESP_LOGW(TAG, "Position out of range: %u setting servo to failsafe position", position);
        return std::clamp(static_cast<uint32_t>(config_.failsafe_position) + static_cast<uint32_t>(offset_),
                          static_cast<uint32_t>(config_.min_pulse_width_us),
                          static_cast<uint32_t>(config_.max_pulse_width_us));
    }

    // if (range_ < 0 || range_ > 75) {
    //     ESP_LOGW(TAG, "Range out of bounds: %d, setting to default 20%%", range_);
    //     range_ = std::clamp(range_, 0, 75);
    // }

    constexpr int32_t symmetric_offset = (2000 - 0) / 2; // RC channel range is 0-2000

    const int32_t normalized_position = static_cast<int32_t>(position) - symmetric_offset;

    const int32_t scaled_position = (normalized_position - 100)/ 2 * range_ / 100;

    uint32_t pulse_width = center_pulse_width_us_ + offset_ + scaled_position; // Offset doesn't do anything anymore, but it's there for legacy (read: im lazy)

    if (invert_steering_) {
        pulse_width = config_.min_pulse_width_us + config_.max_pulse_width_us - pulse_width;
    }

    // ESP_LOGI(TAG, "Pulse width: %lu", pulse_width);

    return std::clamp(pulse_width, static_cast<uint32_t>(config_.min_pulse_width_us), static_cast<uint32_t>(config_.max_pulse_width_us));
}

esp_err_t Servo::setPosition(const rclink::channel_value_t position)  {
    if (!enabled_) {
        return ESP_ERR_INVALID_STATE;
    }
    // ESP_LOGI(TAG, "Setting servo position: %d", calculateCompareValue(position));
    return mcpwm_comparator_set_compare_value(comparator_, calculateCompareValue(position));
}

// Runtime configuration methods
esp_err_t Servo::setCenterPoint(uint16_t center_us) {
    if (center_us < config_.min_pulse_width_us || center_us > config_.max_pulse_width_us) {
        ESP_LOGW(TAG, "Center point %u out of range [%d, %d]", center_us, config_.min_pulse_width_us, config_.max_pulse_width_us);
        return ESP_ERR_INVALID_ARG;
    }
    if (center_us != center_pulse_width_us_) {
        center_pulse_width_us_ = center_us;
        ESP_LOGI(TAG, "Center point updated to %u us", center_us);
    }
    return ESP_OK;
}

esp_err_t Servo::setLimits(uint16_t min_us, uint16_t max_us) {
    if (min_us >= max_us) {
        ESP_LOGW(TAG, "Invalid limits: min %u >= max %u", min_us, max_us);
        return ESP_ERR_INVALID_ARG;
    }
    config_.min_pulse_width_us = min_us;
    config_.max_pulse_width_us = max_us;
    ESP_LOGI(TAG, "Limits updated to [%u, %u] us", min_us, max_us);
    return ESP_OK;
}

esp_err_t Servo::setRange(int range_percent) {
    if (range_percent < 0 || range_percent > 100) {
        ESP_LOGW(TAG, "Range %d%% out of bounds [0, 100]", range_percent);
        return ESP_ERR_INVALID_ARG;
    }
    if (range_percent != range_) {
        range_ = range_percent;
        ESP_LOGI(TAG, "Range updated to %d%%", range_percent);
    }
    return ESP_OK;
}

esp_err_t Servo::setOffset(int offset_us) {
    offset_ = offset_us;
    ESP_LOGI(TAG, "Offset updated to %d us", offset_us);
    return ESP_OK;
}

esp_err_t Servo::setInvert(bool invert) {
    invert_steering_ = invert;
    ESP_LOGI(TAG, "Invert updated to %s", invert ? "true" : "false");
    return ESP_OK;
}

esp_err_t Servo::setFailsafe() {
    return setPosition(config_.failsafe_position);
}

// Safety methods
esp_err_t Servo::enable() {
    enabled_ = true;
    ESP_LOGI(TAG, "Servo enabled");
    return ESP_OK;
}

esp_err_t Servo::disable() {
    enabled_ = false;
    // Set to failsafe position when disabled
    esp_err_t ret = mcpwm_comparator_set_compare_value(comparator_, calculateCompareValue(config_.failsafe_position));
    ESP_LOGI(TAG, "Servo disabled, set to failsafe position");
    return ret;
}
