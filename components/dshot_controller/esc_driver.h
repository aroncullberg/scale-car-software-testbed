// dshot_esc.hpp
#pragma once
#include <cstdint>
#include <vector>
#include <stdexcept>
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include <esp_log.h>

class EscDriver {
public:
    struct Config {
        uint32_t resolution_hz = 40000000;   // 40MHz resolution (ESP32-S3 typical)
        uint32_t baud_rate = 300000;         // DSHOT300
        uint32_t post_delay_us = 50;         // Post-frame delay
        std::vector<gpio_num_t> gpio_nums;   // GPIO pins for ESCs
        size_t mem_block_symbols = 48;       // WARNING: If this is >48 then we can not have 4 esc's | RMT memory symbols per channel
    };

    EscDriver() = default;
    ~EscDriver();

    // Core functionality methods
    esp_err_t initialize(const Config& config);
    esp_err_t start();
    esp_err_t arm_motor(size_t motor_idx, bool arm);
    esp_err_t set_throttle(size_t motor_idx, uint16_t throttle, bool telemetry = false);

    // Status checking
    bool is_armed(size_t motor_idx) const {
        if (motor_idx >= motors_.size()) {
            ESP_LOGE(TAG, "Invalid motor index %zu (max %zu)", motor_idx, motors_.size());
            return false; // or use abort() for critical failures
        }
        return motors_[motor_idx].armed;
    }
    
private:
    struct MotorControl {
        rmt_channel_handle_t channel = nullptr;
        rmt_encoder_handle_t encoder = nullptr;
        bool armed = false;
        uint16_t current_throttle = 0;
    };

    static constexpr const char* TAG = "ESC Driver";

    Config config_;
    std::vector<MotorControl> motors_;
    bool initialized_ = false;
    bool started_ = false;

    // Internal initialization helpers
    esp_err_t create_rmt_channel(gpio_num_t gpio_num, rmt_channel_handle_t* channel);
    esp_err_t create_encoder(rmt_encoder_handle_t* encoder);
};
