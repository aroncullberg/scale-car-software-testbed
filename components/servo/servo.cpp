#include "servo/controller.hpp"

#include <algorithm>

#include "driver/gpio.h"

namespace servo {
namespace {
constexpr ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t timer = LEDC_TIMER_0;
constexpr ledc_timer_bit_t resolution = LEDC_TIMER_14_BIT;
}

Controller::~Controller() {
    stop();
}

esp_err_t Controller::start(const OutputConfig *outputs, size_t count, uint32_t frequency_hz) {
    if (timer_started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!outputs || count == 0 || count > max_outputs || frequency_hz < 40 || frequency_hz > 333) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < count; ++i) {
        const auto &output = outputs[i];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(output.gpio) || output.minimum_us == 0 ||
            output.minimum_us >= output.center_us || output.center_us >= output.maximum_us ||
            duty_for_pulse(output.maximum_us, frequency_hz, resolution) >= (1u << resolution)) {
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = 0; j < i; ++j) {
            if (outputs[j].gpio == output.gpio) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = mode;
    timer_config.duty_resolution = resolution;
    timer_config.timer_num = timer;
    timer_config.freq_hz = frequency_hz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t result = ledc_timer_config(&timer_config);
    if (result != ESP_OK) {
        return result;
    }
    timer_started_ = true;
    frequency_hz_ = ledc_get_freq(mode, timer);
    if (frequency_hz_ == 0) {
        stop();
        return ESP_FAIL;
    }

    for (size_t i = 0; i < count; ++i) {
        outputs_[i] = outputs[i];
        ledc_channel_config_t channel_config{};
        channel_config.gpio_num = outputs[i].gpio;
        channel_config.speed_mode = mode;
        channel_config.channel = static_cast<ledc_channel_t>(i);
        channel_config.timer_sel = timer;
        channel_config.duty = duty_for_pulse(outputs[i].center_us, frequency_hz_, resolution);
        result = ledc_channel_config(&channel_config);
        if (result != ESP_OK) {
            stop();
            return result;
        }
        pulses_[i] = outputs[i].center_us;
        ++count_;
    }
    return ESP_OK;
}

esp_err_t Controller::write(size_t index, uint16_t position) {
    if (index >= count_) {
        return timer_started_ ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_STATE;
    }
    return write_pulse_us(index, pulse_for_position(outputs_[index], position));
}

esp_err_t Controller::write_pulse_us(size_t index, uint16_t pulse_us) {
    if (index >= count_) {
        return timer_started_ ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_STATE;
    }
    const auto &output = outputs_[index];
    const uint16_t clamped = std::clamp(pulse_us, output.minimum_us, output.maximum_us);
    if (clamped == pulses_[index]) {
        return ESP_OK;
    }
    // The app owns these channels in one task, so the ordinary LEDC duty API suffices.
    esp_err_t result = ledc_set_duty(mode, static_cast<ledc_channel_t>(index),
                                   duty_for_pulse(clamped, frequency_hz_, resolution));
    if (result == ESP_OK) {
        result = ledc_update_duty(mode, static_cast<ledc_channel_t>(index));
    }
    if (result == ESP_OK) {
        pulses_[index] = clamped;
    }
    return result;
}

esp_err_t Controller::center() {
    if (!timer_started_) {
        return ESP_ERR_INVALID_STATE;
    }
    for (size_t i = 0; i < count_; ++i) {
        const esp_err_t result = write_pulse_us(i, outputs_[i].center_us);
        if (result != ESP_OK) {
            return result;
        }
    }
    return ESP_OK;
}

void Controller::stop() {
    for (size_t i = 0; i < count_; ++i) {
        const auto channel = static_cast<ledc_channel_t>(i);
        ledc_stop(mode, channel, 0);
        ledc_channel_config_t config{};
        config.speed_mode = mode;
        config.channel = channel;
        config.deconfigure = true;
        ledc_channel_config(&config);
        gpio_reset_pin(static_cast<gpio_num_t>(outputs_[i].gpio));
    }
    count_ = 0;
    if (timer_started_) {
        ledc_timer_pause(mode, timer);
        ledc_timer_config_t config{};
        config.speed_mode = mode;
        config.timer_num = timer;
        config.deconfigure = true;
        ledc_timer_config(&config);
    }
    timer_started_ = false;
    frequency_hz_ = 0;
    pulses_.fill(0);
}

uint16_t Controller::pulse_us(size_t index) const {
    return index < count_ ? pulses_[index] : 0;
}

} // namespace servo
