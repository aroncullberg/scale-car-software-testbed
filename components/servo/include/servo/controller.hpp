#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/ledc.h"
#include "esp_err.h"
#include "servo/pulse.hpp"

namespace servo {

// Owns low-speed LEDC timer 0 and channels 0..count-1. Use from one task.
class Controller {
  public:
    static constexpr size_t max_outputs = LEDC_CHANNEL_MAX;

    Controller() = default;
    ~Controller();
    Controller(const Controller &) = delete;
    Controller &operator=(const Controller &) = delete;

    // Starts every output at its configured center pulse.
    esp_err_t start(const OutputConfig *outputs, size_t count, uint32_t frequency_hz = 50);
    esp_err_t write(size_t index, uint16_t position);
    // Direct pulse writes are clamped to the output's endpoints; reversal applies to write().
    esp_err_t write_pulse_us(size_t index, uint16_t pulse_us);
    esp_err_t center();
    void stop();
    uint16_t pulse_us(size_t index) const;

  private:
    std::array<OutputConfig, max_outputs> outputs_{};
    std::array<uint16_t, max_outputs> pulses_{};
    size_t count_{0};
    uint32_t frequency_hz_{0};
    bool timer_started_{false};
};

} // namespace servo
