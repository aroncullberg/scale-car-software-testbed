#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "led_strip.h"

namespace led {

/**
 * @brief Self-initializing RGB LED singleton
 *
 * Auto-initializes on first use with GPIO48 and SPI3 backend.
 * Thread-safe for concurrent access from multiple tasks.
 * Last write wins - no priority system.
 *
 * Usage:
 * @code
 * led::Rgb::instance().set_color(255, 0, 0);  // Red
 * led::Rgb::instance().set_color(0, 255, 0);  // Green
 * led::Rgb::instance().off();
 * @endcode
 */
class Rgb {
public:
    static Rgb& instance();

    /**
     * @brief Set LED color
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    void set_color(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Turn off LED
     */
    void off();

    // Delete copy/move
    Rgb(const Rgb&) = delete;
    Rgb& operator=(const Rgb&) = delete;
    Rgb(Rgb&&) = delete;
    Rgb& operator=(Rgb&&) = delete;

private:
    Rgb() = default;
    ~Rgb();

    void ensure_initialized();

    bool initialized_ = false;
    bool init_failed_ = false;
    led_strip_handle_t strip_ = nullptr;

    uint8_t current_r_ = 0;
    uint8_t current_g_ = 0;
    uint8_t current_b_ = 0;

    SemaphoreHandle_t mutex_ = nullptr;
};

} // namespace led
